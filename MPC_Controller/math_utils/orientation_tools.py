import math
import numpy as np
from math import sin, cos
from MPC_Controller.utils import DTYPE, CoordinateAxis, Quaternion


# Orientation tools
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
        k[0]*k[0]*v_t + c_t,        k[0]*k[1]*v_t - k[2]*s_t,   k[0]*k[2]*v_t + k[1]*s_t,
        k[0]*k[1]*v_t + k[2]*s_t,   k[1]*k[1]*v_t + c_t,        k[1]*k[2]*v_t - k[0]*s_t,
        k[0]*k[2]*v_t - k[1]*s_t,   k[1]*k[2]*v_t + k[0]*s_t,   k[1]*k[1]*v_t + c_t
        ], dtype=DTYPE).reshape((3,3))
    
    return R_axis_angle.T

def coordinateRotation(axis:CoordinateAxis, theta:float) -> np.ndarray:
    s = sin(float(theta))
    c = cos(float(theta))
    R:np.ndarray = None
    if axis is CoordinateAxis.X:
        R = np.array([1, 0, 0, 0, c, s, 0, -s, c], dtype=DTYPE).reshape((3,3))
    elif axis is CoordinateAxis.Y:
        R = np.array([c, 0, -s, 0, 1, 0, s, 0, c], dtype=DTYPE).reshape((3,3))
    elif axis is CoordinateAxis.Z:
        R = np.array([c, s, 0, -s, c, 0, 0, 0, 1], dtype=DTYPE).reshape((3,3))

    return R

def quat_to_rpy(q:Quaternion) -> np.ndarray:
    """
    * Convert a quaternion to RPY. Return
    * angles in (roll, pitch, yaw).
    """
    rpy = np.zeros((3,1), dtype=DTYPE)
    as_ = np.min([-2.*(q.x*q.z-q.w*q.y),.99999])
    # roll
    rpy[0] = np.arctan2(2.*(q.y*q.z+q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)
    # pitch
    rpy[1] = np.arcsin(as_)
    # yaw
    rpy[2] = np.arctan2(2.*(q.x*q.y+q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z)
    return rpy

def quat_to_rot(q:Quaternion) -> np.ndarray:
    """
    * Convert a quaternion to a rotation matrix.  This matrix represents a
    * coordinate transformation into the frame which has the orientation specified
    * by the quaternion
    """
    e0 = q.w
    e1 = q.x
    e2 = q.y
    e3 = q.z
    R = np.array([1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e0 * e3),
                  2 * (e1 * e3 + e0 * e2), 2 * (e1 * e2 + e0 * e3),
                  1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e0 * e1),
                  2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1),
                  1 - 2 * (e1 * e1 + e2 * e2)], 
                  dtype=DTYPE).reshape((3,3))
    return R.T

def rpy_to_rot(rpy):
    """
    convert RPY to a rotation matrix
    """
    R = coordinateRotation(CoordinateAxis.X, rpy[0]) @\
        coordinateRotation(CoordinateAxis.Y, rpy[1]) @\
        coordinateRotation(CoordinateAxis.Z, rpy[2])
    return R

def rot_to_quat(rot:np.ndarray):
    """
    * Convert a coordinate transformation matrix to an orientation quaternion.
    """
    q = Quaternion()
    r = rot.T.copy() # important
    tr = np.trace(r)
    if tr>0.0:
        S = math.sqrt(tr + 1.0) * 2.0
        q.w = 0.25 * S
        q.x = (r[2,1] - r[1,2])/S
        q.y = (r[0,2] - r[2,0])/S
        q.z = (r[1,0] - r[0,1])/S

    elif (r[0, 0] > r[1, 1]) and (r[0, 0] > r[2, 2]):
        S = math.sqrt(1.0 + r[0,0] - r[1,1] - r[2,2]) * 2.0
        q.w = (r[2,1] - r[1,2])/S
        q.x = 0.25 * S
        q.y = (r[0,1] + r[1,0])/S
        q.z = (r[0,2] + r[2,0])/S

    elif r[1,1]>r[2,2]:
        S = math.sqrt(1.0 + r[1,1] -r[0,0] -r[2,2]) * 2.0
        q.w = (r[0,2] - r[2,0])/S
        q.x = (r[0,1] + r[1,0])/S
        q.y = 0.25 * S
        q.z = (r[1,2] + r[2,1])/S
        
    else:
        S = math.sqrt(1.0 + r[2,2] - r[0,0] - r[1,1]) * 2.0
        q.w = (r[1,0] - r[0,1])/S
        q.x = (r[0,2] + r[2,0])/S
        q.y = (r[1,2] + r[2,1])/S
        q.z = 0.25 * S
    
    return q
    
def rot_to_rpy(rot:np.ndarray):
    return quat_to_rpy(rot_to_quat(rot))

def deg2rad(deg:float):
    return deg*math.pi/180

def rad2deg(rad:float):
    return rad*180/math.pi