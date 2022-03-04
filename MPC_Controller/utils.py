import math
import numpy as np
from enum import Enum, auto
from math import sin, cos

DTYPE = np.float32
CASTING = "same_kind"
# SIDE_SIGN = [-1, 1, -1, 1]
SIDE_SIGN = [1, -1, 1, -1]
K_MAX_GAIT_SEGMENTS = 40

class CoordinateAxis(Enum):
    X = auto()
    Y = auto()
    Z = auto()

class GaitType(Enum):
    TROT = 0
    TROTRUN = 7
    # GALLOP = 5
    WALK = 6
    PACE = 3
    # PRONK = 2
    BOUND = 1
    STAND = 4

class FSM_StateName(Enum):
    INVALID = 99
    PASSIVE = 0
    LOCOMOTION = 4
    RECOVERY_STAND = 6

class FSM_OperatingMode(Enum):
    TEST = 0
    NORMAL = 1
    TRANSITIONING = auto()
    # ESTOP = auto()
    # EDAMP = auto()

def coordinateRotation(axis:CoordinateAxis, theta:float) -> np.ndarray:
    s = sin(float(theta))
    c = cos(float(theta))
    R:np.ndarray = None
    if axis == CoordinateAxis.X:
        R = np.array([1, 0, 0, 0, c, s, 0, -s, c], dtype=DTYPE).reshape((3,3))
    elif axis == CoordinateAxis.Y:
        R = np.array([c, 0, -s, 0, 1, 0, s, 0, c], dtype=DTYPE).reshape((3,3))
    elif axis == CoordinateAxis.Z:
        R = np.array([c, s, 0, -s, c, 0, 0, 0, 1], dtype=DTYPE).reshape((3,3))

    return R

class Quaternion:
    def __init__(self, w:float, x:float, y:float, z:float):
        self.w = float(w)
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        
    def toNumpy(self):
        return np.array([self.w,self.x,self.y,self.z], dtype=DTYPE).reshape((4,1))

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

def deg2rad(deg:float):
    return deg*math.pi/180

def rad2deg(rad:float):
    return rad*180/math.pi

# Interpolation
def cubicBezier(y0:np.ndarray, yf:np.ndarray, x:float):
    """
    Cubic bezier interpolation between y0 and yf.  x is between 0 and 1
    """
    assert x >= 0 and x <= 1
    yDiff = yf - y0
    bezier = x * x * x + 3.0 * (x * x * (1.0 - x))
    return y0 + bezier * yDiff

def cubicBezierFirstDerivative(y0:np.ndarray, yf:np.ndarray, x:float):
    """
    Cubic bezier interpolation derivative between y0 and yf.  x is between 0 and 1
    """
    assert x >= 0 and x <= 1
    yDiff = yf - y0
    bezier = 6.0 * x * (1.0 - x)
    return bezier * yDiff

def cubicBezierSecondDerivative(y0:np.ndarray, yf:np.ndarray, x:float):
    """Cubic bezier interpolation derivative between y0 and yf.  x is between 0 and 1"""
    assert x >= 0 and x <= 1
    yDiff = yf - y0
    bezier = 6.0 - 12.0 * x
    return bezier * yDiff

def getSideSign(leg:int):
    """
    Get if the i-th leg is on the left (+) or right (-) of the robot
    """
    assert leg >= 0 and leg < 4
    return SIDE_SIGN[leg]