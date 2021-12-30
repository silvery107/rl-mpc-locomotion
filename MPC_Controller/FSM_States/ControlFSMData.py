import sys
sys.path.append("..")
from Quadruped import Quadruped
from LegController import LegController
from MIT_UserParameters import MIT_UserParameters
from StateEstimatorContainer import StateEstimatorContainer
from RobotParameters import RobotControlParameters

class ControlFSMData:
    def __init__(self):
        self._quadruped:Quadruped = None
        self._legController:LegController = None
        self._stateEstimator:StateEstimatorContainer = None
        self.controlParameters:RobotControlParameters = None
        self.userParameters:MIT_UserParameters = None

