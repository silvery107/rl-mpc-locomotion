from enum import Enum, auto
import numpy as np
from MPC_Controller.utils import DTYPE

# Data structure containing parameters for quadruped robot
class RobotType(Enum):
    ALIENGO = auto()
    MINI_CHEETAH = auto()
    XIAOTIAN = auto()
    ANYMAL = auto()
    A1 = auto()

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
            self._abadLinkLength = 0.0418
            self._hipLinkLength = 0.25
            self._kneeLinkLength = 0.25
            self._kneeLinkY_offset = 0.0
            self._abadLocation = np.array([0.2399, 0.051, 0], dtype=DTYPE).reshape((3,1))
            self._bodyName = "trunk"
            self._bodyMass = 9.041*3
            self._bodyInertia = np.array([0.033260231, 0, 0, 
                                      0, 0.16117211, 0, 
                                      0, 0, 0.17460442])*10
            self._bodyHeight = 0.4
            self._mpc_weights = [1., 1., 0, 0, 0, 10, 0., 0., .1, .1, .1, .0, 0]

        elif robotype == RobotType.A1:
            self._abadLinkLength = 0.04
            self._hipLinkLength = 0.2
            self._kneeLinkLength = 0.2
            self._kneeLinkY_offset = 0.0
            self._abadLocation = np.array([0.183, 0.047, 0], dtype=DTYPE).reshape((3,1))
            self._bodyName = "trunk"
            self._bodyMass = 110 / 9.8
            self._bodyInertia = np.array([0.017, 0, 0, 
                                      0, 0.057, 0, 
                                      0, 0, 0.064])*10
            self._bodyHeight = 0.26
            self._mpc_weights = [1., 1., 0, 0, 0, 10, 0., 0., .1, .1, .1, .0, 0]

        elif robotype == RobotType.MINI_CHEETAH:
            self._abadLinkLength = 0.062
            self._hipLinkLength = 0.209
            self._kneeLinkLength = 0.195
            self._kneeLinkY_offset = 0.004
            self._abadLocation = np.array([0.19, 0.049, 0], dtype=DTYPE).reshape((3,1))
            self._bodyName = "body"
            self._bodyMass = 9.0
            self._bodyInertia = np.array([0.011253, 0, 0, 
                                      0, 0.036203, 0, 
                                      0, 0, 0.042673]) * 10
            self._bodyHeight = 0.29
            self._mpc_weights = [0.25, 0.25, 10, 2, 2, 50, 0, 0, 0.3, 0.2, 0.2, 0.1, 0]
        
        else:
            raise "Invalid RobotType"
            
        self._robotType = robotype

    def getHipLocation(self, leg:int):
        """
        Get location of the hip for the given leg in robot frame
        """
        assert leg >= 0 and leg < 4
        pHip = np.array([
            self._abadLocation[0] if (leg == 0 or leg == 1) else -self._abadLocation[0],
            self._abadLocation[1] if (leg == 0 or leg == 2) else -self._abadLocation[1],
            self._abadLocation[2]
            ], dtype=DTYPE).reshape((3,1))

        return pHip

    