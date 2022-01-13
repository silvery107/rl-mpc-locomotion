import numpy as np
from copy import copy, deepcopy
from math import sin, cos
import quaternion
CASTING = "same_kind"
DTYPE = np.float32

class RobotState:

    def __init__(self):
        self.p:np.ndarray = None # (3,1)
        self.v:np.ndarray = None # (3,1)
        self.w:np.ndarray = None # (3,1)
        self.r_feet:np.ndarray = None # (3,4)
        self.R:np.ndarray = None # (3,3)
        self.R_yaw:np.ndarray = None # (3,3)
        self.I_body:np.ndarray = np.zeros((3, 3))
        self.q:np.quaternion = None
        self.yaw = 0.0
        self.m = 9.0

    def set(self, p_, v_, q_, w_, r_feet_, yaw_:float):
        self.p = p_
        self.v = v_
        self.q = q_
        self.w = w_
        self.yaw = yaw_
        self.r_feet = r_feet_

        self.R = quaternion.as_rotation_matrix(self.q)
        yc = cos(yaw_)
        ys = sin(yaw_)
        self.R_yaw = np.array([[yc,  -ys,   0],
                               [ys,  yc,   0],
                               [0,   0,   1]], dtype=DTYPE)
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