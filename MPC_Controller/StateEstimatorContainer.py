import numpy as np
import quaternion

DTYPE = np.float32


class StateEstimate:
    def __init__(self) -> None:
        self.orientation = np.quaternion(0.0, 0.0, 0.0, 0.0)
        self.position = np.zeros((3,1), dtype=DTYPE)
        self.omegaBody = np.zeros((3,1), dtype=DTYPE)
        self.vBody = np.zeros((3,1), dtype=DTYPE)
        self.aBody = np.zeros((3,1), dtype=DTYPE)

        self.contactEstimate = np.zeros((4,1), dtype=DTYPE)
        self.rBody = np.zeros((3,3), dtype=DTYPE)
        self.rpy = np.zeros((3,1), dtype=DTYPE)
        self.omegaWorld = np.zeros((3,1), dtype=DTYPE)
        self.vWorld = np.zeros((3,1), dtype=DTYPE)
        self.aWorld = np.zeros((3,1), dtype=DTYPE)



class StateEstimatorContainer:

    def __init__(self, stateEstimate:StateEstimate):
        self.result = stateEstimate
        self._phase = np.zeros((4,1), dtype=DTYPE)
        self.contactPhase = self._phase

    def setContactPhase(self, phase:np.ndarray):
        self.contactPhase = phase

    def getResult(self):
        return self.result

