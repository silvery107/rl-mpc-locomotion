import math
import numpy as np
from enum import Enum, auto
from math import sin, cos

# Constants
DTYPE = np.float32
CASTING = "same_kind"
SIDE_SIGN = [1, -1, 1, -1]
K_MAX_GAIT_SEGMENTS = 40
NUM_LEGS = 4

# Enumerate classes
class CoordinateAxis(Enum):
    X = auto()
    Y = auto()
    Z = auto()

class GaitType(Enum):
    TROT = 0
    TROTRUN = 7
    GALLOP = 5
    # WALK = 6
    PACE = 3
    # PRONK = 2
    BOUND = 1
    # STAND = 4

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

# Orientation tools
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
    def __init__(self, w:float=1, x:float=0, y:float=0, z:float=0):
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

# Others
def getSideSign(leg:int):
    """
    Get if the i-th leg is on the left (+) or right (-) of the robot
    """
    assert leg >= 0 and leg < 4
    return SIDE_SIGN[leg]