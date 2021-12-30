import sys
sys.path.append("..")

from FSM_States.FSM_State import FSM_State
from Quadruped import Quadruped
from StateEstimatorContainer import StateEstimatorContainer
from LegController import LegController
from RobotParameters import RobotControlParameters
from MIT_UserParameters import MIT_UserParameters
from FSM_States.ControlFSMData import ControlFSMData
from FSM_States.FSM_State_Locomotion import FSM_State_Locomotion
from enum import Enum, auto

class FSM_OperatingMode(Enum):
    NORMAL = auto()
    TRANSITIONING = auto() 
    ESTOP = auto()
    EDAMP = auto()


class FSM_StatesList:
    def __init__(self) -> None:
        self.invalid:FSM_State = None
        self.locomotion:FSM_State_Locomotion = None

class ControlFSM:
    def __init__(self,
                 _quadruped:Quadruped,
                 _stateEstimator:StateEstimatorContainer,
                 _legController:LegController,
                 controlParameters:RobotControlParameters,
                 userParameters:MIT_UserParameters):
        self.data = ControlFSMData()
        self.data._quadruped = _quadruped
        self.data._stateEstimator = _stateEstimator
        self.data._legController = _legController
        self.data.controlParameters = controlParameters
        self.data.userParameters = userParameters

        self.statesList = FSM_StatesList()
        self.statesList.invalid = None
        self.statesList.locomotion = FSM_State_Locomotion(self.data)

        self.iter = 0

        # ! need a SafetyChecker
        # self.safetyChecker = 

        self.initialize()

    def initialize(self):
        # Initialize a new FSM State with the control data
        self.currentState = self.statesList.locomotion
        # Enter the new current state cleanly
        self.currentState.onEnter()
        # Initialize to not be in transition
        self.nextState = self.currentState
        # Initialize FSM mode to normal operation
        self.operatingMode = FSM_OperatingMode.NORMAL

    def runFSM(self):
        if self.operatingMode == FSM_OperatingMode.NORMAL:
            self.currentState.run()
            
        self.iter += 1
