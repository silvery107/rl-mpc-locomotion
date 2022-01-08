import numpy as np
from copy import copy, deepcopy
from math import sin, cos
import quaternion
CASTING = "same_kind"
DTYPE = np.float32

# ! update I_body val
class RobotState:

    def __init__(self) -> None:
        self.p = np.zeros((3, 1), dtype=DTYPE)
        self.v = np.zeros((3, 1), dtype=DTYPE)
        self.w = np.zeros((3, 1), dtype=DTYPE)
        self.r_feet = np.zeros((3, 4), dtype=DTYPE)
        self.R = np.zeros((3, 3), dtype=DTYPE)
        self.R_yaw = np.zeros((3, 3), dtype=DTYPE)
        self.I_body = np.zeros((3, 3))
        self.q = np.quaternion(1, 0, 0, 0)
        self.yaw = 0.0
        self.m = 9.0

    def set(self, p_, v_, q_, w_, r_, yaw_:float):
        
        np.copyto(self.p, p_, casting=CASTING)
        np.copyto(self.v, v_, casting=CASTING)
        np.copyto(self.w, w_, casting=CASTING)
        self.q = copy(q_)
        self.yaw = yaw_
        for rs in range(3):
            for c in range(4):
                self.r_feet[rs, c] = r_[rs*4+c]

        self.R = quaternion.as_rotation_matrix(self.q)
        yc = cos(yaw_)
        ys = sin(yaw_)
        self.R_yaw = np.array([[yc,  -ys,   0],
                               [ys,  yc,   0],
                               [0,   0,   1]])
        # ! update I_body
        np.fill_diagonal(self.I_body, [0.07, 0.26, 0.242])


    def printState(self):
        print("Robot State:\n" +
            "Position\n" + self.p.T +
            "\nVelocity\n" + self.v.T +
            "\nAngular Velocity\n" + self.w.T +
            "\nRotation\n" + self.R + 
            "\nYaw Rotation\n" + self.R_yaw +
            "\nFoot Locations\n"+ self.r_feet +
            "\nInertia\n" + self.I_body + "\n")