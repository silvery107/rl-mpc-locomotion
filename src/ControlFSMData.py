from Quadruped import Quadruped
from LegController import LegController
from MIT_UserParameters import MIT_UserParameters
from StateEstimatorContainer import StateEstimatorContainer


class ControlFSMData:
    _quadruped = Quadruped()
    _legController = LegController()
    _stateEstimator = StateEstimatorContainer()
    _gaitScheduler = None
    controlParameters = None
    userParameters = MIT_UserParameters()


