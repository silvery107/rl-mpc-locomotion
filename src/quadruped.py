# Data structure containing parameters for quadruped robot

class Quadruped:

    num_act_joint = 12
    num_q = 19
    dim_config = 18
    num_leg = 4
    num_leg_joint = 3

    # Link indices for cheetah-shaped robots
    FR = 9   # Front Right Foot
    FL = 11  # Front Left Foot
    HR = 13  # Hind Right Foot
    HL = 15  # Hind Left Foot

    FR_abd = 2  # Front Right Abduction
    FL_abd = 0  # Front Left Abduction
    HR_abd = 3  # Hind Right Abduction
    HL_abd = 1  # Hind Left Abduction
    
    def __init__(self) -> None:
        # mini cheetah
        self._abadLinkLength = 0.062
        self._hipLinkLength = 0.209
        self._kneeLinkLength = 0.195
        self._kneeLinkY_offset = 0.004

    def getSideSign(self, leg):
        sideSigns= [-1, 1, -1, 1]
        assert leg >= 0 and leg < 4
        return sideSigns[leg]

    