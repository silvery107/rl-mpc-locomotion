from enum import Enum, auto
import numpy as np

DTYPE = np.float32

# Data structure containing parameters for quadruped robot
class RobotType(Enum):
    ALIENGO = auto()
    MINI_CHEETAH = auto()
    XIAOTIAN = auto()
    ANYMAL = auto()

class Quadruped:

    # num_act_joint = 12
    # num_q = 19
    # dim_config = 18
    # num_leg = 4
    # num_leg_joint = 3

    # Link indices for cheetah-shaped robots
    # FR = 9   # Front Right Foot
    # FL = 11  # Front Left Foot
    # HR = 13  # Hind Right Foot
    # HL = 15  # Hind Left Foot

    # FR_abd = 2  # Front Right Abduction
    # FL_abd = 0  # Front Left Abduction
    # HR_abd = 3  # Hind Right Abduction
    # HL_abd = 1  # Hind Left Abduction
    
    # _robotType = RobotType.ALIENGO

    def __init__(self, robotype:RobotType):

        if robotype == RobotType.ALIENGO:
            self._abadLinkLength = 0.062
            self._hipLinkLength = 0.209
            self._kneeLinkLength = 0.195
            self._kneeLinkY_offset = 0.004
            self._abadLocation = np.array([0.19, 0.049, 0], dtype=DTYPE).reshape((3,1))
            self.bodyName = "trunk"

        elif robotype == RobotType.MINI_CHEETAH:
            self._abadLinkLength = 0.062
            self._hipLinkLength = 0.209
            self._kneeLinkLength = 0.195
            self._kneeLinkY_offset = 0.004
            self._abadLocation = np.array([0.19, 0.049, 0], dtype=DTYPE).reshape((3,1))
            self.bodyName = "body"
            
        else:
            raise "Invalid RobotType"
            
        self._robotType = robotype

    def getSideSign(self, leg:int):
        """
        Get if the i-th leg is on the left (+) or right (-) of the robot
        """
        sideSigns= [-1, 1, -1, 1]
        assert leg >= 0 and leg < 4
        return sideSigns[leg]

    def getHipLocation(self, leg:int):
        """
        Get location of the hip for the given leg in robot frame
        """
        assert leg >= 0 and leg < 4
        pHip = np.array([
            self._abadLocation[0] if (leg == 0 or leg == 1) else -self._abadLocation[0],
            self._abadLocation[1] if (leg == 1 or leg == 3) else -self._abadLocation[1],
            self._abadLocation[2]
            ], dtype=DTYPE).reshape((3,1))

        return pHip

    