import numpy as np
from Quadruped import Quadruped, RobotType
from SolverMPC import DTYPE


# q = np.zeros((3,1))
DTYPE = np.float64

class LegControllerCommand:
    def __init__(self):
        self.zero()

    def zero(self):
        """
        Zero the leg command so the leg will not output torque
        """
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

class LegControllerData:
    def __init__(self):
        self.zero()

    def zero(self):
        self.q = np.zeros((3,1), dtype=DTYPE)
        self.qd = np.zeros((3,1), dtype=DTYPE)
        self.p = np.zeros((3,1), dtype=DTYPE)
        self.v = np.zeros((3,1), dtype=DTYPE)
        self.J = np.zeros((3,3), dtype=DTYPE)
        self.tauEstimate = np.zeros((3,1), dtype=DTYPE)

    def setQuadruped(self, quad:Quadruped):
        self.quadruped = quad

class LegController:

    def __init__(self, quad:Quadruped):
        self.commands = [LegControllerCommand() for _ in range(4)]
        self.datas = [LegControllerData() for _ in range(4)]
        self._legsEnabled = False
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
        self._legsEnabled = False

    def setMaxTorque(self, tau:float):
        self._maxTorque = tau
    
    def edampCommand(self, robot:RobotType, gain:float):
        """
        emergency damp command
        """
        # TODO assign gain to kdCartesian or kdJoint ?

        self.zeroCommand()
        if robot == RobotType.ALIENGO:
            for leg in range(4):
                for axis in range(3):
                    # self.commands[leg].kdCartesian[axis, axis] = gain
                    # self.commands[leg].kdJoint[axis, axis] = gain
                    pass
        

    def updateData(self, data):
        """
        update the leg data
        """
        # TODO update q qd J p and v here
        pass

    def updateCommand(self, command):
        """
        update the leg command
        """
        # TODO update PD feedforward torque and force
        pass


def computeLegJacobianAndPosition(quad:Quadruped, q:np.ndarray, leg:int):
    """
    return J and p
    """
    l1 = quad._abadLinkLength
    l2 = quad._hipLinkLength
    l3 = quad._kneeLinkLength
    l4 = quad._kneeLinkY_offset
    sideSign = quad.getSideSign(leg)

    s1 = np.sin(q[0])
    s2 = np.sin(q[1])
    s3 = np.sin(q[2])

    c1 = np.cos(q[0])
    c2 = np.cos(q[1])
    c3 = np.cos(q[2])

    c23 = c2 * c3 - s2 * s3
    s23 = s2 * c3 + c2 * s3

    J = np.zeros((3,3), dtype=np.double)
    J[0, 0] = 0
    J[0, 1] = l3 * c23 + l2 * c2
    J[0, 2] = l3 * c23
    J[1, 0] = l3 * c1 * c23 + l2 * c1 * c2 - (l1+l4) * sideSign * s1
    J[1, 1] = -l3 * s1 * s23 - l2 * s1 * s2
    J[1, 2] = -l3 * s1 * s23
    J[2, 0] = l3 * s1 * c23 + l2 * c2 * s1 + (l1+l4) * sideSign * c1
    J[2, 1] = l3 * c1 * s23 + l2 * c1 * s2
    J[2, 2] = l3 * c1 * s23

    p = np.zeros((3,1), dtype=np.double)
    p[0] = l3 * s23 + l2 * s2
    p[1] = (l1+l4) * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1
    p[2] = (l1+l4) * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2
    
    return J, p
