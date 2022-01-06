import sys
sys.path.append("..")
from MPC_Controller.common.Quadruped import Quadruped
from MPC_Controller.common.LegController import LegController
from MIT_UserParameters import MIT_UserParameters
from StateEstimatorContainer import StateEstimatorContainer
from RobotParameters import RobotControlParameters
from DesiredStateCommand import DesiredStateCommand

class ControlFSMData:
    def __init__(self):
        self._quadruped:Quadruped = None
        self._stateEstimator:StateEstimatorContainer = None
        self._legController:LegController = None
        # self._gaitScheduler:GaitScheduler = None
        self._desiredStateCommand:DesiredStateCommand = None
        self.controlParameters:RobotControlParameters = None
        self.userParameters:MIT_UserParameters = None

