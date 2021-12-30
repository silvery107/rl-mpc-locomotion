from FSM_States.ControlFSM import ControlFSM
from Quadruped import Quadruped
from StateEstimatorContainer import StateEstimatorContainer
from LegController import LegController
from RobotParameters import RobotControlParameters
from MIT_UserParameters import MIT_UserParameters

class MIT_Controller:
    def __init__(self, ) -> None:
        pass

    def initializeController(self,
                             _quadruped:Quadruped,
                             _stateEstimator:StateEstimatorContainer,
                             _legController:LegController,
                             controlParameters:RobotControlParameters,
                             userParameters:MIT_UserParameters):
        """Initializes the Control FSM"""

        # self._gaitScheduler = GaitScheduler()

        self._controlFSM = ControlFSM(_quadruped, 
                                      _stateEstimator, 
                                      _legController, 
                                      controlParameters, 
                                      userParameters)

    def runController(self):
        """
        Calculate the commands for the leg controllers using the ControlFSM logic
        """
        # _gaitScheduler.step()

        # _desiredStateCommand.convertToStateCommands()

        # Run the Control FSM code
        self._controlFSM.runFSM()


