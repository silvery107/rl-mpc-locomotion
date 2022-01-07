import sys
sys.path.append("..")
import numpy as np
from MPC_Controller.convexMPC.ConvexMPCLocomotion import *
from MPC_Controller.FSM_States.ControlFSMData import ControlFSMData
from MPC_Controller.common.Quadruped import RobotType
from MPC_Controller.FSM_States.FSM_State import FSM_State, FSM_StateName

DTYPE = np.float32

class FSM_State_Locomotion(FSM_State):
    def __init__(self, _controlFSMData:ControlFSMData):
        super().__init__(_controlFSMData, FSM_StateName.LOCOMOTION, "LOCOMOTION")

        if _controlFSMData._quadruped._robotType == RobotType.MINI_CHEETAH:
            self.cMPC = ConvexMPCLocomotion(_controlFSMData.userParameters.controller_dt,
                27/(1000.0*_controlFSMData.userParameters.controller_dt),
                _controlFSMData.userParameters)
        elif _controlFSMData._quadruped._robotType == RobotType.ALIENGO:
            self.cMPC = ConvexMPCLocomotion(_controlFSMData.userParameters.controller_dt,
                27/(1000.0*_controlFSMData.userParameters.controller_dt),
                _controlFSMData.userParameters)
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
