import numpy as np
from enum import Enum, auto

# Constants
DTYPE = np.float32
CASTING = "same_kind"
SIDE_SIGN = [1, -1, 1, -1]
K_MAX_GAIT_SEGMENTS = 40
NUM_LEGS = 4

# Enumerate classes
class ControllerType(Enum):
    FSM = auto()
    MIN = auto()
    POLICY = auto()

class GaitType(Enum):
    TROT = 0
    # TROTRUN = 7
    # GALLOP = 5
    WALK = 6
    # PACE = 3
    # PRONK = 2
    BOUND = 1

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

# Others
def getSideSign(leg:int):
    """
    Get if the i-th leg is on the left (+) or right (-) of the robot
    """
    assert leg >= 0 and leg < 4
    return SIDE_SIGN[leg]