import numpy as np
from numpy.core.fromnumeric import resize
from SolverMPC import *

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

problem_configuration = ProblemSetup()
update = UpdateData()
gait_data = np.zeros(K_MAX_GAIT_SEGMENTS, dtype=np.uint8)
first_run = 1
has_solved = 0

def setup_problem(dt:float, horizon:int, mu:float, fmax:float):
    problem_configuration.dt = dt
    problem_configuration.horizon = horizon
    problem_configuration.mu = mu
    problem_configuration.f_max = fmax
    resize_qp_mats(horizon)

def update_problem_data(p:np.ndarray, v:np.ndarray, q, w:np.ndarray, r:list, yaw:float, weights:list, state_trajectory:list, alpha:float, gait:list):
    global has_solved
    update.p = p
    update.v = v
    update.q = q
    update.w = w
    update.r = r
    update.yaw = yaw
    update.weights = weights
    update.traj = state_trajectory[12*problem_configuration.horizon]
    update.alpha = alpha
    update.gait = gait[4*problem_configuration.horizon]
    solve_mpc(update, problem_configuration)
    has_solved = 1

# def update_solver_settings(max_iter:int, rho:float, sigma:float, solver_alpha:float, terminate:float):
#     """
#     This is for jcqp only, which means useless
#     """
#     update.max_iterations = max_iter
#     update.rho = rho
#     update.sigma = sigma
#     update.solver_alpha = solver_alpha
#     update.terminate = terminate

def update_x_drag(x_drag:float):
    update.x_drag = x_drag

def get_solution(index:int):
    if not has_solved:
        return 0.0

    qs = get_q_soln()
    return qs[index]