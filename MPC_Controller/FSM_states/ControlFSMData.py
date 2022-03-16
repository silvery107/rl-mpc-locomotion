from MPC_Controller.common.Quadruped import Quadruped
from MPC_Controller.common.LegController import LegController
# from MPC_Controller.Parameters import Parameters
from MPC_Controller.StateEstimator import StateEstimator
from MPC_Controller.DesiredStateCommand import DesiredStateCommand

class ControlFSMData:
    def __init__(self):
        self._quadruped:Quadruped = None
        self._stateEstimator:StateEstimator = None
        self._legController:LegController = None
        self._desiredStateCommand:DesiredStateCommand = None
        # self._gaitScheduler:GaitScheduler = None
        # self.userParameters:Parameters = None

