import numpy as np
from MPC_Controller.utils import DTYPE

class TransitionData:
    """
    Struct of relevant data that can be used during transition to pass
    data between states.
    """

    def __init__(self) -> None:
        # Flag to mark when transition is done
        self.done = False
        
        # Timing parameters
        self.t0 = 0.0
        self.tCurrent = 0.0
        self.tDuration = 0.0

        # Robot state at the beginning of transition
        self.comState0 = np.zeros((12,1), dtype=DTYPE)
        self.qJoints0 = np.zeros((12,1), dtype=DTYPE)
        self.pFoot0 = np.zeros((3,4), dtype=DTYPE)

        # Current robot state
        self.comState = np.zeros((12,1), dtype=DTYPE)
        self.qJoints = np.zeros((12,1), dtype=DTYPE)
        self.pFoot = np.zeros((3,4), dtype=DTYPE)

    def zero(self):
        # Flag to mark when transition is done
        self.done = False
        
        # Timing parameters
        self.t0 = 0.0
        self.tCurrent = 0.0
        self.tDuration = 0.0

        # Robot state at the beginning of transition
        self.comState0.fill(0)
        self.qJoints0.fill(0)
        self.pFoot0.fill(0)

        # Current robot state
        self.comState.fill(0)
        self.qJoints.fill(0)
        self.pFoot.fill(0)
