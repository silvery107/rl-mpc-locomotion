from Quadruped import Quadruped
from LegController import LegController
from MIT_UserParameters import MIT_UserParameters
from StateEstimatorContainer import StateEstimatorContainer
from RobotParameters import RobotControlParameters

class ControlFSMData:
    _quadruped:Quadruped
    _legController:LegController
    _stateEstimator:StateEstimatorContainer
    controlParameters:RobotControlParameters
    userParameters:MIT_UserParameters


