import numpy as np
from copy import deepcopy, copy
from MPC_Controller.convexMPC.SolverMPC import *

CASTING = "same_kind"

problem_configuration = ProblemSetup()
update = UpdateData()
has_solved = 0

def setup_problem(dt:float, horizon:int, mu:float, fmax:float):
    problem_configuration.dt = dt
    problem_configuration.horizon = horizon
    problem_configuration.mu = mu
    problem_configuration.f_max = fmax
    resize_qp_mats(horizon)

def update_problem_data(p:np.ndarray, v:np.ndarray, q:np.quaternion, w:np.ndarray, 
                        r_feet:np.ndarray, yaw:float, weights:np.ndarray, 
                        state_trajectory:list, alpha:float, gait:list):
    global has_solved
    np.copyto(update.p, p, casting=CASTING)
    np.copyto(update.v, v, casting=CASTING)
    update.q = copy(q)
    np.copyto(update.w, w, casting=CASTING)
    # update.r_feet = copy(r_feet)
    np.copyto(update.r_feet, r_feet)
    update.yaw = yaw
    np.copyto(update.weights, weights, casting=CASTING)
    update.traj = copy(state_trajectory)
    update.alpha = alpha
    update.gait = copy(gait)
    solve_mpc(update, problem_configuration)
    has_solved = 1

def update_x_drag(x_drag:float):
    update.x_drag = x_drag

def get_solution(index:int):
    if not has_solved:
        return 0.0

    qs = get_q_soln()
    return qs[index]