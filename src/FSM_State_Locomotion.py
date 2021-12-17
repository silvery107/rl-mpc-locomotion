import numpy as np
from ConvexMPCLocomotion import *
from ControlFSMData import ControlFSMData
from Quadruped import RobotType
from RobotParameters import RobotControlParameters

DTYPE = np.float32

class FSM_State_Locomotion:

    def __init__(self, _controlFSMData:ControlFSMData):
        if _controlFSMData._quadruped._robotType == RobotType.MINI_CHEETAH:
            self.cMPC = ConvexMPCLocomotion(_controlFSMData.controlParameters.controller_dt,
                27/(1000.0*_controlFSMData.controlParameters.controller_dt),
                _controlFSMData.userParameters)
        elif _controlFSMData._quadruped._robotType == RobotType.ALIENGO:
            self.cMPC = ConvexMPCLocomotion(_controlFSMData.controlParameters.controller_dt,
                27/(1000.0*_controlFSMData.controlParameters.controller_dt),
                _controlFSMData.userParameters)
        else:
            raise "Invalid RobotType"
        
        self._data = ControlFSMData()
        
    def onEnter(self):
        self.cMPC.initialize()

    def run(self):
        self.LocomotionControlStep()

    def LocomotionControlStep(self):
        self.cMPC.run(self._data)
