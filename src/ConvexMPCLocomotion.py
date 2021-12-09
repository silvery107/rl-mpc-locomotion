import numpy as np
import LegController as legctl

DTYPE = np.float32

class CMPC_Result:
    commands = [legctl.LegController() for _ in range(4)]
    contactPhase = np.zeros((4,1), dtype=DTYPE)

class ConvexMPCLocomotion:
    # TODO finish this fucking class
    pass
    