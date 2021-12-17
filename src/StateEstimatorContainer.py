import numpy as np
import quaternion

DTYPE = np.float32


class StateEstimate:
    orientation = np.quaternion(0.0, 0.0, 0.0, 0.0)
    position = np.zeros((3,1), dtype=DTYPE)
    omegaBody = np.zeros((3,1), dtype=DTYPE)
    vBody = np.zeros((3,1), dtype=DTYPE)
    aBody = np.zeros((3,1), dtype=DTYPE)

    contactEstimate = np.zeros((4,1), dtype=DTYPE)
    rBody = np.zeros((3,3), dtype=DTYPE)
    rpy = np.zeros((3,1), dtype=DTYPE)
    omegaWorld = np.zeros((3,1), dtype=DTYPE)
    vWorld = np.zeros((3,1), dtype=DTYPE)
    aWorld = np.zeros((3,1), dtype=DTYPE)



class StateEstimatorContainer:

    def __init__(self, stateEstimate:StateEstimate):
        self.result = stateEstimate
        self._phase = np.zeros((4,1), dtype=DTYPE)
        self.contactPhase = self._phase

    def setContactPhase(self, phase:np.ndarray):
        self.contactPhase = phase

    def getResult(self):
        return self.result

