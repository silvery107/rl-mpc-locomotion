import numpy as np
import quaternion
import cvxopt

from robot_state import RobotState

rs = RobotState()


class ProblemSetup:
    dt = 0.0
    mu = 0.0
    f_max = 0.0
    horizon = 0

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

# continuous time state space matrices.  
def ct_ss_mats(I_world, m:float, r_feet, R_yaw, x_drag:float):
    A = np.zeros((13,13), dtype=np.float32)
    A[3,9] = 1.0
    A[11,9] = x_drag
    A[4,10] = 1.0
    A[5,11] = 1.0

    A[11,12] = 1.0
    A[0:3, 6:9] = R_yaw.T

    B = np.zeros((13,12), dtype=np.float32)
    I_inv = np.linalg.inv(I_world)

    for b in range(4):
        B[6:9, b*3:b*3+3] = cross_mat(I_inv,r_feet[:, b])
        B[9:12, b*3:b*3+3] = np.identity(3) / m
    return A, B

def resize_qp_mats(horizon:int):
    pass

def c2qp(Ac, Bc, dt:float, horizon:int):
    pass

def solve_mpc():
    pass