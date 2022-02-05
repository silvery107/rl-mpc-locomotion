import sys
sys.path.append("..")
import numpy as np
from MPC_Controller.convex_MPC.ConvexMPCLocomotion import ConvexMPCLocomotion
from MPC_Controller.FSM_states.ControlFSMData import ControlFSMData
from MPC_Controller.Parameters import Parameters
from MPC_Controller.common.Quadruped import RobotType
from MPC_Controller.FSM_states.FSM_State import FSM_State, FSM_StateName
from MPC_Controller.utils import DTYPE

class FSM_State_Locomotion(FSM_State):
    def __init__(self, _controlFSMData:ControlFSMData):
        super().__init__(_controlFSMData, FSM_StateName.LOCOMOTION, "LOCOMOTION")

        if _controlFSMData._quadruped._robotType == RobotType.MINI_CHEETAH:
            self.cMPC = ConvexMPCLocomotion(Parameters.controller_dt,
                27/(1000.0*Parameters.controller_dt))
        elif _controlFSMData._quadruped._robotType == RobotType.ALIENGO:
            self.cMPC = ConvexMPCLocomotion(Parameters.controller_dt,
                27/(1000.0*Parameters.controller_dt))
        else:
            raise "Invalid RobotType"
        

        self.iter = 0
        self.checkPDesFoot = False
        self.footFeedForwardForces = np.zeros((3,4), dtype=DTYPE)
        self.footstepLocations = np.zeros((3,4), dtype=DTYPE)
        
    def onEnter(self):
        self.cMPC.initialize()

    def run(self):
        self.LocomotionControlStep()

    def onExit(self):
        self.iter = 0

    def LocomotionControlStep(self):
        self.cMPC.run(self._data)
