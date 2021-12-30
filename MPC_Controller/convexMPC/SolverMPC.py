import numpy as np
import quaternion
import cvxopt
from convexMPC.RobotState import RobotState

K_NUM_LEGS = 4
K_MAX_GAIT_SEGMENTS = 36


class ProblemSetup:
    def __init__(self) -> None: 
        self.dt = 0.0
        self.mu = 0.0
        self.f_max = 0.0
        self.horizon = 0

class UpdateData:
    def __init__(self) -> None:
        self.p = [0.0 for _ in range(3)]
        self.v = [0.0 for _ in range(3)]
        self.q = [0.0 for _ in range(4)]
        self.w = [0.0 for _ in range(3)]
        self.r = [0.0 for _ in range(12)]
        self.weights = [0.0 for _ in range(12)]
        self.traj = [0.0 for _ in range(12*K_MAX_GAIT_SEGMENTS)]
        self.traj = ""
        self.gait = ""
        self.yaw = 0.0
        self.alpha = 0.0
        self.rho = 0.0
        self.sigma = 0.0
        self.solver_alpha = 0.0
        self.terminate = 0.0
        self.x_drag = 0.0
        self.max_iterations = 0


DTYPE = np.float32
BIG_NUMBER = 5e10

rs = RobotState()
Adt = np.zeros((13,13), dtype=DTYPE)
Bdt = np.zeros((13,12), dtype=DTYPE)
ABc = np.zeros((25,25), dtype=DTYPE)
expmm = np.zeros((25,25), dtype=DTYPE)
x_0 = np.zeros((13,1), dtype=DTYPE)
I_world = np.zeros((3,3), dtype=DTYPE)
A_ct = np.zeros((13,13), dtype=DTYPE)
B_ct_r = np.zeros((13,12), dtype=DTYPE)

A_qp = np.empty([])
B_qp = np.empty([])
S = np.empty([])
X_d = np.empty([])
U_b = np.empty([])
fmat = np.empty([])
qH = np.empty([])
qg = np.empty([])
eye_12h = np.empty([])

q_soln = 0.0

def near_zero(a:float):
    return (a < 0.01 and a > -.01)

def near_one(a:float):
    return near_zero(a-1)

def cross_mat(I_inv:np.ndarray, r:np.ndarray):
    cm = np.array([[0.0, -r[2], r[1]],
                  [r[2], 0.0, -r[0]],
                  [-r[1], r[0], 0.0]])

    return I_inv * cm

def quat_to_rpy(q, rpy:np.ndarray):
    as_ = np.min(-2.*(q.x*q.z-q.w*q.y),.99999)
    rpy[0] = np.atan2(2.*(q.x*q.y+q.w*q.z),q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z)
    rpy[1] = np.asin(as_)
    rpy[2] = np.atan2(2.*(q.y*q.z+q.w*q.x),q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)

# continuous time state space matrices.  
def ct_ss_mats(I_world:np.ndarray, m:float, r_feet:np.ndarray, R_yaw:np.ndarray, A:np.ndarray, B:np.ndarray, x_drag:float):
    A.fill(0)
    A[3,9] = 1.0
    A[11,9] = x_drag
    A[4,10] = 1.0
    A[5,11] = 1.0

    A[11,12] = 1.0
    A[0:3, 6:9] = R_yaw.T

    B.fill(0)
    I_inv = np.linalg.inv(I_world)

    for b in range(4):
        B[6:9, b*3:b*3+3] = cross_mat(I_inv,r_feet[:, b])
        B[9:12, b*3:b*3+3] = np.identity(3) / m

def resize_qp_mats(horizon:int):
    global A_qp, B_qp, S, X_d, U_b, fmat, qH, qg, eye_12h
    A_qp = np.zeros((13*horizon, 13), dtype=DTYPE)
    B_qp = np.zeros((13*horizon, 12*horizon), dtype=DTYPE)
    S = np.zeros((13*horizon, 13*horizon), dtype=DTYPE)
    X_d = np.zeros((13*horizon, 1), dtype=DTYPE)
    U_b = np.zeros((20*horizon, 1), dtype=DTYPE)
    fmat = np.zeros((20*horizon, 12*horizon), dtype=DTYPE)
    qH = np.zeros((12*horizon, 12*horizon), dtype=DTYPE)
    qg = np.zeros((12*horizon, 1), dtype=DTYPE)
    eye_12h = np.identity(12*horizon, dtype=DTYPE)

def c2qp(Ac:np.ndarray, Bc:np.ndarray, dt:float, horizon:int):
    global ABc
    ABc.fill(0)
    ABc[0:13,0:13] = Ac
    ABc[0:13,13:25] = Bc
    ABc = dt * ABc
    expmm = np.exp(ABc)
    Adt = expmm[0:13,0:13]
    Bdt = expmm[0:13,13:25]
    if horizon > 19:
        raise "horizon is too long!"

    powerMats = [np.zeros((13,13), dtype=DTYPE) for _ in range(20)]
    powerMats[0] = np.identity(13, dtype=DTYPE)
    for i in range(1, horizon+1):
        powerMats[i] = Adt * powerMats[i-1]
    
    for r in range(horizon):
        A_qp[13*r:13*r+13, 0:13] = powerMats[r+1]
        for c in range(horizon):
            if r>=c:
                a_num = r-c
                B_qp[13*r:13*r+13, 12*c:12*c+12] = powerMats[a_num] * Bdt

def solve_mpc(update:UpdateData, setup:ProblemSetup):
    global A_qp, B_qp, S, X_d, U_b, fmat, qH, qg, eye_12h
    global rs, x_0, I_world, A_ct, B_ct_r, q_soln
    rs.set(update.p, update.v, update.q, update.w, update.r, update.yaw)

    # roll pitch yaw
    rpy = np.zeros((3,1), dtype=DTYPE)
    quat_to_rpy(rs.q, rpy)

    # initial state (13 state representation)
    x_0 = np.array([rpy[2], rpy[1], rpy[0], rs.p, rs.w, rs.v, -9.81])
    I_world = rs.R_yaw*rs.I_body*rs.R_yaw.T
    # state space models
    ct_ss_mats(I_world, rs.m, rs.r_feet,rs.R_yaw,A_ct, B_ct_r, update.x_drag)
    # QP matrices
    c2qp(A_ct, B_ct_r, setup.dt, setup.horizon)

    full_weight = np.asarray(update.weights, dtype=DTYPE)
    full_weight[12] = 0.0
    np.fill_diagonal(S, np.tile(full_weight,(setup.horizon,1)))

    # trajectory
    for i in range(setup.horizon):
        for j in range(12):
            X_d[13*i+j,0] = update.traj[12*i+j]
    
    k = 0
    for i in range(setup.horizon):
        for j in range(4):
            U_b[5*k + 0] = BIG_NUMBER
            U_b[5*k + 1] = BIG_NUMBER
            U_b[5*k + 2] = BIG_NUMBER
            U_b[5*k + 3] = BIG_NUMBER
            U_b[5*k + 4] = update.gait[i*4 + j] * setup.f_max
            k += 1

    mu = 1.0/setup.mu
    f_block = np.array([[mu, 0,  1.0,
                        -mu, 0,  1.0,
                        0,  mu, 1.0,
                        0, -mu, 1.0,
                        0,   0, 1.0]], 
                        dtype=DTYPE)
    for i in range(setup.horizon*4):
        fmat[i*5:i*5+5, i*3:i*3+3] = f_block

    qH = 2*(B_qp.T*S*B_qp + update.alpha*eye_12h)
    qg = 2*B_qp.T*S*(A_qp*x_0 - X_d)
    
    # TODO solve this QP using cvxopt
    q_soln = cvxopt.solvers.qp(qH, qg, fmat, U_b, solver=None) # "mosek"

def get_q_soln():
    return q_soln