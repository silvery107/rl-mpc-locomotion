import numpy as np
import quaternion

class RobotState:

    def __init__(self) -> None:
        self.p = np.zeros((3, 1), dtype=np.float32)
        self.v = np.zeros((3, 1), dtype=np.float32)
        self.w = np.zeros((3, 1), dtype=np.float32)
        self.r_feet = np.zeros((3, 4), dtype=np.float32)
        self.R = np.zeros((3, 3), dtype=np.float32)
        self.R_yaw = np.zeros((3, 3), dtype=np.float32)
        # self.I_body = np.zeros((3, 3))
        self.q = np.quaternion(0, 0, 0, 0)
        self.yaw = 0.0
        self.m = 9.0

    def set(self, p_, v_, q_, w_, r_, yaw_):
        
        self.p = np.copy(p_)
        self.v = np.copy(v_)
        self.w = np.copy(w_)
        self.q.w = q_[0]
        self.q.x = q_[1]
        self.q.y = q_[2]
        self.q.z = q_[3]
        self.yaw = yaw_
        for rs in range(4):
            for c in range(5):
                self.r_feet[rs, c] = r_[rs*4+c]

        self.R = quaternion.as_rotation_matrix(self.q)
        yc = np.cos(yaw_)
        ys = np.sin(yaw_)
        self.R_yaw = np.array([[yc,  -ys,   0],
                               [ys,  yc,   0],
                               [0,   0,   1]])
        # np.fill_diagonal(self.I_body, [0.07, 0.26, 0.242])


    def printState(self):
        print("Robot State:\n" +
            "Position\n" + self.p.T +
            "\nVelocity\n" + self.v.T +
            "\nAngular Velocity\n" + self.w.T +
            "\nRotation\n" + self.R + 
            "\nYaw Rotation\n" + self.R_yaw +
            "\nFoot Locations\n"+ self.r_feet +
            "\nInertia\n" + self.I_body + "\n")