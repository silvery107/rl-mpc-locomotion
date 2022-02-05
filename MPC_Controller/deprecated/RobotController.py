from MPC_Controller.common.Quadruped import Quadruped
from MPC_Controller.common.LegController import LegController
from MPC_Controller.FSM_states.ControlFSM import ControlFSM
# from MPC_Controller.Parameters import Parameters
from MPC_Controller.StateEstimatorContainer import StateEstimatorContainer
from MPC_Controller.DesiredStateCommand import DesiredStateCommand

class RobotController:
    def __init__(self, ) -> None:
        pass

    def initializeController(self,
                             _quadruped:Quadruped,
                             _stateEstimator:StateEstimatorContainer,
                             _legController:LegController,
                             _desiredStateCommand:DesiredStateCommand):
        """Initializes the Control FSM"""
        # self._gaitScheduler = GaitScheduler()

        self._controlFSM = ControlFSM(_quadruped, 
                                      _stateEstimator, 
                                      _legController,
                                      _desiredStateCommand)

    def runController(self):
        """
        Calculate the commands for the leg controllers using the ControlFSM logic
        """
        # _gaitScheduler.step()

        # Run the Control FSM code
        self._controlFSM.runFSM()

    # def getUserControlParameters(self):
    #     return self.userParameters
