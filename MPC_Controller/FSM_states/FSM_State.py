import numpy as np
from MPC_Controller.Parameters import Parameters
from MPC_Controller.utils import DTYPE, CASTING
from MPC_Controller.FSM_states.ControlFSMData import ControlFSMData
from MPC_Controller.utils import FSM_StateName

class FSM_State:
    """
    * Constructor for the FSM State class.
    *
    * @param _controlFSMData holds all of the relevant control data
    * @param stateNameIn the enumerated state name
    * @param stateStringIn the string name of the current FSM state

    """
    def __init__(self, 
                 _controlFSMData:ControlFSMData,
                 stateNameIn:FSM_StateName,
                 stateStringIn:str):
        self._data = _controlFSMData
        self.stateName = stateNameIn
        self.nextStateName:FSM_StateName = None
        self.stateString = stateStringIn
        self.transitionDone = False
        if Parameters.FSM_print_info:
            print("[FSM_State] Initialized FSM state:", self.stateString)

    def onEnter(self):
        raise NotImplementedError
        

    # @abstractmethod
    def run(self):
        raise NotImplementedError
    
    # @abstractmethod
    def onExit(self):
        raise NotImplementedError

    def checkTransition(self):
        return FSM_StateName.INVALID

    def transition(self):
        return self.transitionDone

    def jointPDControl(self, leg:int, qDes:np.ndarray, qdDes:np.ndarray, kpMat=None, kdMat=None):
        """
        * Cartesian impedance control for a given leg.
        *
        * @param leg the leg number to control
        * @param qDes desired joint position
        * @param dqDes desired joint velocity
        """
        if kpMat is None:
            kpMat = np.array([80, 0, 0, 0, 80, 0, 0, 0, 80], dtype=DTYPE).reshape((3,3))
        if kdMat is None:
            kdMat = np.array([1, 0, 0, 0, 1, 0, 0, 0, 1], dtype=DTYPE).reshape((3,3))

        self._data._legController.commands[leg].kpJoint = kpMat
        self._data._legController.commands[leg].kdJoint = kdMat
        # np.copyto(self._data._legController.commands[leg].kpJoint, kpMat, casting=CASTING)
        # np.copyto(self._data._legController.commands[leg].kdJoint, kdMat, casting=CASTING)

        self._data._legController.commands[leg].qDes = qDes
        self._data._legController.commands[leg].qdDes = qdDes

    def turnOnAllSafetyChecks(self):
        # Pre controls safety checks
        self.checkSafeOrientation = True  # check roll and pitch

        # Post control safety checks
        self.checkPDesFoot = True          # do not command footsetps too far
        self.checkForceFeedForward = True  # do not command huge forces
        self.checkLegSingularity = True    # do not let leg

    def turnOffAllSafetyChecks(self):
        # Pre controls safety checks
        self.checkSafeOrientation = False  # check roll and pitch

        # Post control safety checks
        self.checkPDesFoot = False          # do not command footsetps too far
        self.checkForceFeedForward = False  # do not command huge forces
        self.checkLegSingularity = False    # do not let leg