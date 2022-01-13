import sys
sys.path.append("..")
from enum import Enum, auto
from MPC_Controller.FSM_states.ControlFSMData import ControlFSMData
from MPC_Controller.FSM_states.TransitionData import TransitionData
from abc import abstractclassmethod

class FSM_StateName(Enum):
    INVALID = auto()
    PASSIVE = auto()
    JOINT_PD = auto()
    IMPEDANCE_CONTROL = auto()
    STAND_UP = auto()
    BALANCE_STAND = auto()
    LOCOMOTION = auto()
    RECOVERY_STAND = auto()
    VISION = auto()
    BACKFLIP = auto()
    FRONTJUMP = auto()

class FSM_State:
    def __init__(self, 
                 _controlFSMData:ControlFSMData,
                 stateNameIn:FSM_StateName,
                 stateStringIn:str):
        self._data = _controlFSMData
        self.stateName = stateNameIn
        self.stateString = stateStringIn
        self.transitionData = TransitionData()
        print("[FSM_State] Initialized FSM state:", self.stateString)

    @abstractclassmethod
    def onEnter(self):
        pass

    @abstractclassmethod
    def run(self):
        pass
    @abstractclassmethod
    def onExit(self):
        pass

    def checkTransition(self):
        return FSM_StateName.INVALID

    def transition(self):
        return self.transitionData