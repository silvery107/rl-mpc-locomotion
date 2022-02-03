import numpy as np
from copy import deepcopy, copy
from MPC_Controller.utils import Quaternion
import MPC_Controller.convex_MPC.SolverMPC as solver
from MPC_Controller.utils import CASTING


problem_configuration = solver.ProblemSetup()
update = solver.UpdateData()
has_solved = 0

def setup_problem(dt:float, horizon:int, mu:float, fmax:float):
    problem_configuration.dt = dt
    problem_configuration.horizon = horizon
    problem_configuration.mu = mu
    problem_configuration.f_max = fmax
    solver.resize_qp_mats(horizon)

def update_problem_data(p:np.ndarray, v:np.ndarray, q:Quaternion, w:np.ndarray, 
                        rpy:np.ndarray, r_feet:np.ndarray, yaw:float, weights:np.ndarray, 
                        state_trajectory:list, alpha:float, gait:list):
    global has_solved
    np.copyto(update.p, p, casting=CASTING)
    np.copyto(update.v, v, casting=CASTING)
    # update.p = p
    # update.v = v
    update.q.w = q.w
    update.q.x = q.x
    update.q.y = q.y
    update.q.z = q.z
    np.copyto(update.w, w, casting=CASTING)
    np.copyto(update.rpy, rpy, casting=CASTING)
    np.copyto(update.r_feet, r_feet, casting=CASTING)
    # update.w = w
    # update.rpy = rpy
    # update.r_feet = r_feet
    update.yaw = yaw
    np.copyto(update.weights, weights, casting=CASTING)
    # update.weights = weights
    update.traj = state_trajectory
    update.alpha = alpha
    update.gait = gait
    solver.solve_mpc(update, problem_configuration)
    has_solved = 1

def update_x_drag(x_drag:float):
    update.x_drag = x_drag

def get_solution(index:int):
    if not has_solved:
        return 0.0

    qs = solver.get_q_soln()
    return qs[index]