# Compute foot swing trajectory with a bezier curve
# phase : How far along we are in the swing (0 to 1)
# swingTime : How long the swing should take (seconds)
import numpy as np
import interpolation as interp

class FootSwingTrajectory:
    def __init__(self):
        # vec3 (3,1)
        self._p0 = np.zeros((3,1), dtype=np.float32)
        self._pf = np.zeros((3,1), dtype=np.float32)
        self._p = np.zeros((3,1), dtype=np.float32)
        self._v = np.zeros((3,1), dtype=np.float32)
        self._a = np.zeros((3,1), dtype=np.float32)
        # float or int
        self._height = 0.0

    def setInitialPosition(self, p0):
        self._p0 = p0
    
    def setFinalPosition(self, pf):
        self.pf = pf

    def setHeight(self, h):
        self._height = h

    def getPosition(self):
        return self._p
    
    def getVelocity(self):
        return self._v
    
    def getAcceleration(self):
        return self._a

    def computeSwingTrajectoryBezier(self, phase:float, swingTime:float):
        self._p = interp.cubicBezier(self._p0, self._pf, phase)
        self._v = interp.cubicBezierFirstDerivative(self._p0, self._pf, phase) / swingTime
        self._a = interp.cubicBezierSecondDerivative(self._p0, self._pf, phase) / (swingTime * swingTime)

        if phase < float(0.5):
            zp = interp.cubicBezier(self._p0[2], self._p0[2] + self._height, phase * 2)
            zv = interp.cubicBezierFirstDerivative(self._p0[2], self._p0[2] + self._height, phase * 2) * 2 / swingTime
            za = interp.cubicBezierSecondDerivative(self._p0[2], self._p0[2] + self._height, phase * 2) * 4 / (swingTime * swingTime)
        else:
            zp = interp.cubicBezier(self._p0[2] + self._height, self._pf[2], phase * 2 - 1)
            zv = interp.cubicBezierFirstDerivative(self._p0[2] + self._height, self._pf[2], phase * 2 - 1) * 2 / swingTime
            za = interp.cubicBezierSecondDerivative(self._p0[2] + self._height, self._pf[2], phase * 2 - 1) * 4 / (swingTime * swingTime)

        self._p[2] = zp
        self._v[2] = zv
        self._a[2] = za
