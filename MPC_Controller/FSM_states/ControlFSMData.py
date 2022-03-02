import sys
sys.path.append("..")
from MPC_Controller.common.Quadruped import Quadruped
from MPC_Controller.common.LegController import LegController
# from MPC_Controller.Parameters import Parameters
from MPC_Controller.StateEstimatorContainer import StateEstimatorContainer
from MPC_Controller.DesiredStateCommand import DesiredStateCommand

class ControlFSMData:
    def __init__(self):
        self._quadruped:Quadruped = None
        self._stateEstimator:StateEstimatorContainer = None
        self._legController:LegController = None
        # self._gaitScheduler:GaitScheduler = None
        # self._desiredStateCommand:DesiredStateCommand = None
        # self.userParameters:Parameters = None

