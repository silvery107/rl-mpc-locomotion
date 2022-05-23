import math
import numpy as np
from math import sin, cos
from MPC_Controller.Parameters import Parameters
from MPC_Controller.common.Quadruped import Quadruped
from MPC_Controller.utils import DTYPE, getSideSign

class LegControllerCommand:
    def __init__(self):
        self.tauFeedForward = np.zeros((3,1), dtype=DTYPE)
        self.forceFeedForward = np.zeros((3,1), dtype=DTYPE)

        self.qDes = np.zeros((3,1), dtype=DTYPE)
        self.qdDes = np.zeros((3,1), dtype=DTYPE)
        self.pDes = np.zeros((3,1), dtype=DTYPE)
        self.vDes = np.zeros((3,1), dtype=DTYPE)

        self.kpCartesian = np.zeros((3,3), dtype=DTYPE)
        self.kdCartesian = np.zeros((3,3), dtype=DTYPE)
        self.kpJoint = np.zeros((3,3), dtype=DTYPE)
        self.kdJoint = np.zeros((3,3), dtype=DTYPE)

    def zero(self):
        """
        Zero the leg command so the leg will not output torque
        """
        self.tauFeedForward.fill(0)
        self.forceFeedForward.fill(0)

        self.qDes.fill(0)
        self.qdDes.fill(0)
        self.pDes.fill(0)
        self.vDes.fill(0)

        self.kpCartesian.fill(0)
        self.kdCartesian.fill(0)
        self.kpJoint.fill(0)
        self.kdJoint.fill(0)

class LegControllerData:
    def __init__(self):
        self.q = np.zeros((3,1), dtype=DTYPE)
        self.qd = np.zeros((3,1), dtype=DTYPE)

        self.p = np.zeros((3,1), dtype=DTYPE)
        self.v = np.zeros((3,1), dtype=DTYPE)

        self.J = np.zeros((3,3), dtype=DTYPE)
        # self.tauEstimate = np.zeros((3,1), dtype=DTYPE)

    def zero(self):

        self.q.fill(0)
        self.qd.fill(0)

        self.p.fill(0)
        self.v.fill(0)

        self.J.fill(0)
        # self.tauEstimate.fill(0)

    def setQuadruped(self, quad:Quadruped):
        self.quadruped = quad

class LegController:

    def __init__(self, quad:Quadruped):
        self.commands = [LegControllerCommand() for _ in range(4)]
        self.datas = [LegControllerData() for _ in range(4)]
        # self._legsEnabled = False
        self._maxTorque = 0.0

        self._quadruped = quad
        for data in self.datas:
            data.setQuadruped(self._quadruped)

    def zeroCommand(self):
        """
        Zero all leg commands.  This should be run *before* any control code, so if
        the control code is confused and doesn't change the leg command, the legs
        won't remember the last command.
        """
        for cmd in self.commands:
            cmd.zero()

    def setMaxTorque(self, tau:float):
        self._maxTorque = tau     

    def updateData(self, dof_states):
        """
        update leg data from simulator
        """
        # ! update q, qd, J, p and v here
        for leg in range(4):
            # q and qd
            if Parameters.bridge_MPC_to_RL:
                self.datas[leg].q[:, 0] = dof_states[3*leg:3*(leg+1), 0]
                self.datas[leg].qd[:, 0] = dof_states[3*leg:3*(leg+1), 1]
            else:
                self.datas[leg].q[:, 0] = dof_states["pos"][3*leg:3*(leg+1)]
                self.datas[leg].qd[:, 0] = dof_states["vel"][3*leg:3*(leg+1)]
            
            # J and p
            self.computeLegJacobianAndPosition(leg)
            # v
            self.datas[leg].v = self.datas[leg].J @ self.datas[leg].qd

    def updateCommand(self): # , gym, env, actor):
        """
        update leg commands for simulator
        """
        # ! update joint PD gain, leg enable, feedforward torque and estimate torque
        legTorques = np.zeros(12, dtype=DTYPE)

        for leg in range(4):
            # MPC -> f_ff -R^T-> forceFeedForward
            # force feedforward + cartesian PD
            footForce = self.commands[leg].forceFeedForward \
                        + self.commands[leg].kpCartesian @ (self.commands[leg].pDes - self.datas[leg].p) \
                        + self.commands[leg].kdCartesian @ (self.commands[leg].vDes - self.datas[leg].v)

            # tau feedforward + torque
            legTorque = self.commands[leg].tauFeedForward + self.datas[leg].J.T @ footForce

            # joint PD control
            legTorque += self.commands[leg].kpJoint @ (self.commands[leg].qDes - self.datas[leg].q)
            legTorque += self.commands[leg].kdJoint @ (self.commands[leg].qdDes - self.datas[leg].qd)

            legTorques[leg*3:(leg+1)*3] = legTorque.flatten()
        
        # print("leg 0 effort %.3f %.3f %.3f"%(legTorques[0], legTorques[1], legTorques[2]))
        return legTorques


    def computeLegJacobianAndPosition(self, leg:int):
        """
        return J and p
        """
        l1 = self._quadruped._abadLinkLength
        l2 = self._quadruped._hipLinkLength
        l3 = self._quadruped._kneeLinkLength
        l4 = self._quadruped._kneeLinkY_offset
        sideSign = getSideSign(leg)

        q = self.datas[leg].q

        s1 = sin(q[0])
        s2 = sin(q[1])
        s3 = sin(q[2])

        c1 = cos(q[0])
        c2 = cos(q[1])
        c3 = cos(q[2])

        c23 = c2 * c3 - s2 * s3
        s23 = s2 * c3 + c2 * s3

        self.datas[leg].J[0, 0] = 0.0
        self.datas[leg].J[0, 1] = l3 * c23 + l2 * c2
        self.datas[leg].J[0, 2] = l3 * c23
        self.datas[leg].J[1, 0] = l3 * c1 * c23 + l2 * c1 * c2 - (l1 + l4) * sideSign * s1
        self.datas[leg].J[1, 1] = -l3 * s1 * s23 - l2 * s1 * s2
        self.datas[leg].J[1, 2] = -l3 * s1 * s23
        self.datas[leg].J[2, 0] = l3 * s1 * c23 + l2 * c2 * s1 + (l1 + l4) * sideSign * c1
        self.datas[leg].J[2, 1] = l3 * c1 * s23 + l2 * c1 * s2
        self.datas[leg].J[2, 2] = l3 * c1 * s23

        self.datas[leg].p[0] = l3 * s23 + l2 * s2
        self.datas[leg].p[1] = (l1 + l4) * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1
        self.datas[leg].p[2] = (l1 + l4) * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2
