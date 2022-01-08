from typing import cast
import numpy as np
from copy import deepcopy, copy
from MPC_Controller.convexMPC.SolverMPC import *

CASTING = "same_kind"

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

def update_problem_data(p:np.ndarray, v:np.ndarray, q:np.quaternion, w:np.ndarray, 
                        r:list, yaw:float, weights:np.ndarray, 
                        state_trajectory:list, alpha:float, gait:list):
    global has_solved
    np.copyto(update.p, p, casting=CASTING)
    np.copyto(update.v, v, casting=CASTING)
    update.q = copy(q)
    np.copyto(update.w, w, casting=CASTING)
    update.r = copy(r)
    update.yaw = yaw
    np.copyto(update.weights, weights, casting=CASTING)
    update.traj = copy(state_trajectory)
    update.alpha = alpha
    update.gait = copy(gait)
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