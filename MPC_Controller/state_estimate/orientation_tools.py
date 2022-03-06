import numpy as np
from math import sin, cos


def get_rot_from_normals(world_normal, ground_normal):
    """
    get rotation matrix from two plane normals
    """
    k = np.cross(world_normal, ground_normal)
    theta = np.arccos(world_normal.dot(ground_normal))
    return axis_angle_to_rot(k, theta)

def axis_angle_to_rot(k, theta):
    c_t = cos(theta)
    s_t = sin(theta)
    v_t = 1 - c_t

    R_axis_angle = np.array([
        k[0]*k[0]*v_t + c_t, k[0]*k[1]*v_t - k[2]*s_t, k[0]*k[2]*v_t + k[1]*s_t,
        k[0]*k[1]*v_t + k[2]*s_t, k[1]*k[1]*v_t + c_t, k[1]*k[2]*v_t - k[0]*s_t,
        k[0]*k[2]*v_t - k[1]*s_t, k[1]*k[2]*v_t + k[0]*s_t, k[1]*k[1]*v_t + c_t
        ]).reshape((3,3))
    
    return R_axis_angle.T