import numpy as np
from enum import Enum, auto

# Constants
DTYPE = np.float16
CASTING = "same_kind"
SIDE_SIGN = [1, -1, 1, -1]
K_MAX_GAIT_SEGMENTS = 40
NUM_LEGS = 4

# Enumerate classes
class ControllerType(Enum):
    FSM = auto()
    MIN = auto()
    POLICY = auto()

class CoordinateAxis(Enum):
    X = auto()
    Y = auto()
    Z = auto()

class GaitType(Enum):
    TROT = 0
    TROTRUN = 7
    GALLOP = 5
    WALK = 6
    PACE = 3
    # PRONK = 2
    # BOUND = 1
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

class Quaternion:
    def __init__(self, w:float=1, x:float=0, y:float=0, z:float=0):
        self.w = float(w)
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self._norm = np.sqrt(self.w*self.w+self.x*self.x+self.y*self.y+self.z*self.z, dtype=DTYPE)

    def toNumpy(self):
        """convert to an (4,1) numpy array"""
        return np.array([self.w,self.x,self.y,self.z], dtype=DTYPE).reshape((4,1))
    
    def unit(self):
        """return the unit quaternion"""
        return Quaternion(self.w/self._norm,self.x/self._norm,self.y/self._norm,self.z/self._norm)

    def conjugate(self):
        return Quaternion(self.w, -self.x, -self.y, -self.z)
    
    def reverse(self):
        """return the reverse rotation representation as the same as the transpose op of rotation matrix"""
        return Quaternion(-self.w,self.x,self.y,self.z)

    def inverse(self):
        return Quaternion(self.w/(self._norm*self._norm),-self.x/(self._norm*self._norm),-self.y/(self._norm*self._norm),-self.z/(self._norm*self._norm))
    
    def __str__(self) -> str:
        return '['+str(self.w)+', '+str(self.x)+', '+str(self.y)+', '+str(self.z)+']'

# Others
def getSideSign(leg:int):
    """
    Get if the i-th leg is on the left (+) or right (-) of the robot
    """
    assert leg >= 0 and leg < 4
    return SIDE_SIGN[leg]