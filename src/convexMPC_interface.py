import numpy as np
from numpy.core.fromnumeric import resize

from solver_MPC import *


K_NUM_LEGS = 4
K_MAX_GAIT_SEGMENTS = 36

class ProblemSetup:
    dt = 0.0
    mu = 0.0
    f_max = 0.0
    horizon = 0

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

problem_configuration = ProblemSetup()
update = UpdateData()
gait_data = np.zeros(K_MAX_GAIT_SEGMENTS, dtype=np.uint8)
first_run = 1
has_solved = 0

def setup_problem(dt, horizon, mu, fmax):
    problem_configuration.dt = dt
    problem_configuration.horizon = horizon
    problem_configuration.mu = mu
    problem_configuration.f_max = fmax
    resize_qp_mats(horizon)

def update_problem_data(p, v, q, w, r, yaw:float, weights, state_trajectory, alpha:float, gait):
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

def update_solver_settings(max_iter:int, rho:float, sigma:float, solver_alpha:float, terminate:float):
    update.max_iterations = max_iter
    update.rho = rho
    update.sigma = sigma
    update.solver_alpha = solver_alpha
    update.terminate = terminate

def update_x_drag(x_drag:float):
    update.x_drag = x_drag

def get_solution(index:int):
    if not has_solved:
        return 0.0

    qs = get_q_soln()
    return qs[index]