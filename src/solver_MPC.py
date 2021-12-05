import numpy as np
import quaternion
import cvxopt
from robot_state import RobotState

DTYPE = np.float32
K_MAX_GAIT_SEGMENTS = 36
BIG_NUMBER = 5e10

class ProblemSetup:
    def __init__(self, dt:float, horizon:int, mu:float, fmax:float):
        self.dt = dt
        self.mu = mu
        self.f_max = fmax
        self.horizon = horizon

class UpdateData:
    p = [0.0 for _ in range(3)]
    v = [0.0 for _ in range(3)]
    q = [0.0 for _ in range(4)]
    w = [0.0 for _ in range(3)]
    r = [0.0 for _ in range(12)]
    weights = [0.0 for _ in range(12)]
    traj = [0.0 for _ in range(12*K_MAX_GAIT_SEGMENTS)]
    traj = ""
    gait = ""
    yaw = 0.0
    alpha = 0.0
    rho = 0.0
    sigma = 0.0
    solver_alpha = 0.0
    terminate = 0.0
    x_drag = 0.0
    max_iterations = 0

def near_zero(a:float):
    return (a < 0.01 and a > -.01)

def near_one(a:float):
    return near_zero(a-1)

def cross_mat(I_inv, r):
    cm = np.array([[0.0, -r[2], r[1]],
                  [r[2], 0.0, -r[0]],
                  [-r[1], r[0], 0.0]])

    return I_inv * cm

def quat_to_rpy(q, rpy):
    as_ = np.min(-2.*(q.x*q.z-q.w*q.y),.99999)
    rpy[0] = np.atan2(2.*(q.x*q.y+q.w*q.z),q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z)
    rpy[1] = np.asin(as_)
    rpy[2] = np.atan2(2.*(q.y*q.z+q.w*q.x),q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)

def setup_problem(dt, horizon, mu, fmax):
    return ProblemSetup (dt, horizon, mu, fmax)

class SolverMPC():

    def __init__(self, setup:ProblemSetup):
        self.setup = setup
        self.rs = RobotState()
        horizon = setup.horizon
        DTYPE = np.float32
        self.A_qp = np.zeros((13*horizon, 13), dtype=DTYPE)
        self.B_qp = np.zeros((13*horizon, 12*horizon), dtype=DTYPE)
        self.S = np.zeros((13*horizon, 13*horizon), dtype=DTYPE)
        self.X_d = np.zeros((13*horizon, 1), dtype=DTYPE)
        self.U_b = np.zeros((20*horizon, 1), dtype=DTYPE)
        self.fmat = np.zeros((20*horizon, 12*horizon), dtype=DTYPE)
        self.qH = np.zeros((12*horizon, 12*horizon), dtype=DTYPE)
        self.qg = np.zeros((12*horizon, 1), dtype=DTYPE)
        self.eye_12h = np.identity(12*horizon, dtype=DTYPE)
        self.Adt = np.zeros((13,13), dtype=DTYPE)
        self.Bdt = np.zeros((13,12), dtype=DTYPE)
        self.ABc = np.zeros((25,25), dtype=DTYPE)
        self.expmm = np.zeros((25,25), dtype=DTYPE)
        self.x_0 = np.zeros((13,1), dtype=DTYPE)
        self.I_world = np.zeros((3,3), dtype=DTYPE)
        self.A_ct = np.zeros((13,13), dtype=DTYPE)
        self.B_ct_r = np.zeros((13,12), dtype=DTYPE)

    # continuous time state space matrices.  
    def ct_ss_mats(self, m:float, r_feet, R_yaw, x_drag:float):
        self.A_ct[3,9] = 1.0
        self.A_ct[11,9] = x_drag
        self.A_ct[4,10] = 1.0
        self.A_ct[5,11] = 1.0

        self.A_ct[11,12] = 1.0
        self.A_ct[0:3, 6:9] = R_yaw.T

        I_inv = np.linalg.inv(self.I_world)

        for b in range(4):
            self.B_ct_r[6:9, b*3:b*3+3] = cross_mat(I_inv,r_feet[:, b])
            self.B_ct_r[9:12, b*3:b*3+3] = np.identity(3) / m

    def c2qp(self, dt:float,horizon:int):
        self.ABc[0:13,0:13] = self.A_ct
        self.ABc[0:13,13:25] = self.B_ct_r
        ABc = dt * self.ABc
        expmm = ABc.exp()
        Adt = expmm[0:13,0:13]
        Bdt = expmm[0:13,13:25]
        if horizon > 19:
            raise "horizon is too long!"

        powerMats = [np.zeros((13,13), dtype=DTYPE) for x in range(20)]
        powerMats[0] = np.identity(13, dtype=DTYPE)
        for i in range(1, horizon+1):
            powerMats[i] = Adt * powerMats[i-1]
        
        for r in range(horizon):
            self.A_qp[13*r:13*r+13, 0:13] = powerMats[r+1]
            for c in range(horizon):
                if r>=c:
                    a_num = r-c
                    self.B_qp[13*r:13*r+13, 12*c:12*c+12] = powerMats[a_num] * Bdt

    def solve_mpc(self, update:UpdateData, setup:ProblemSetup):
        self.rs.set(update.p, update.v, update.q, update.w, update.r, update.yaw)

        # roll pitch yaw
        rpy = np.zeros((3,1), dtype=DTYPE)
        quat_to_rpy(self.rs.q, rpy)

        # initial state (13 state representation)
        self.x_0 = np.array([rpy[2], rpy[1], rpy[0], self.rs.p, self.rs.w, self.rs.v, -9.81])
        self.I_world = self.rs.R_yaw*self.rs.I_body*self.rs.R_yaw.T
        # state space models
        self.ct_ss_mats(self.rs.m,self.rs.r_feet,self.rs.R_yaw,update.x_drag)
        # QP matrices
        self.c2qp(setup.dt,setup.horizon)

        full_weight = np.asarray(update.weights, dtype=DTYPE)
        full_weight[12] = 0.0
        np.fill_diagonal(self.S, np.tile(full_weight,(setup.horizon,1)))

        # trajectory
        for i in range(setup.horizon):
            for j in range(12):
                self.X_d[13*i+j,0] = update.traj[12*i+j]
        
        k = 0
        for i in range(setup.horizon):
            for j in range(4):
                self.U_b[5*k + 0] = BIG_NUMBER
                self.U_b[5*k + 1] = BIG_NUMBER
                self.U_b[5*k + 2] = BIG_NUMBER
                self.U_b[5*k + 3] = BIG_NUMBER
                self.U_b[5*k + 4] = update.gait[i*4 + j] * setup.f_max
                k += 1

        mu = 1.0/setup.mu
        f_block = np.array([[mu, 0,  1.0,
                            -mu, 0,  1.0,
                            0,  mu, 1.0,
                            0, -mu, 1.0,
                            0,   0, 1.0]], 
                            dtype=DTYPE)
        for i in range(setup.horizon*4):
            self.fmat[i*5:i*5+5, i*3:i*3+3] = f_block

        qH = 2*(self.B_qp.T*self.S*self.B_qp + update.alpha*self.eye_12h)
        qg = 2*self.B_qp.T*self.S*(self.A_qp*self.x_0 - self.X_d)
        
        # TODO solve this QP using cvxopt
        