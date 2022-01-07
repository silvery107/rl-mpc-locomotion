from MPC_Controller.MIT_Controller import MIT_Controller
from MPC_Controller.common.Quadruped import Quadruped, RobotType
from MPC_Controller.common.LegController import LegController
from MPC_Controller.StateEstimatorContainer import StateEstimatorContainer
from MPC_Controller.DesiredStateCommand import DesiredStateCommand
from MPC_Controller.Parameters import Parameters
import numpy as np


class RobotRunner:
    def __init__(self, robot_ctrl:MIT_Controller):
        self._robot_ctrl = robot_ctrl
        self._iterations = 0


    def init(self, robotType:RobotType):
        """
        Initializes the robot model, state estimator, leg controller,
        robot data, and any control logic specific data.
        """
        self.robotType = robotType

        print("[RobotRunner] initialize")

        # init quadruped
        if self.robotType == RobotType.MINI_CHEETAH:
            # initial directly instead of buildMiniCheetah()
            self._quadruped = Quadruped(RobotType.MINI_CHEETAH)

        elif self.robotType == RobotType.ALIENGO:
            self._quadruped = Quadruped(RobotType.ALIENGO)
            
        else:
            raise "Invalid RobotType"

        # init leg controller
        self._legController = LegController(self._quadruped)

        # init state estimator
        self._stateEstimator = StateEstimatorContainer()

        # init desired state command
        self._desiredStateCommand = DesiredStateCommand()

        # init control and user params
        self.userControlParameters = Parameters()
        
        # Controller initializations
        self._robot_ctrl.initializeController(self._quadruped, 
                                              self._stateEstimator, 
                                              self._legController, 
                                              self._desiredStateCommand,
                                              self.userControlParameters)


    def run(self):
        """
        Runs the overall robot control system by calling each of the major components
        to run each of their respective steps.
        """
        # Update the data from the robot
        self._legController.updateData()
        self._legController.zeroCommand()
        self._legController.setEnable(True)
        self._legController.setMaxTorque(100)
        
        # Run Control user code
        self._robot_ctrl.runController()

        # Sets the leg controller commands for the robot
        self._legController.updateCommand()
        self._iterations += 1

