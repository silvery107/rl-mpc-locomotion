from MPC_Controller.FSM_states.ControlFSM import ControlFSM
from MPC_Controller.common.Quadruped import Quadruped, RobotType
from MPC_Controller.common.LegController import LegController
from MPC_Controller.StateEstimatorContainer import StateEstimatorContainer
from MPC_Controller.DesiredStateCommand import DesiredStateCommand
import numpy as np


class RobotRunner:
    def __init__(self):
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
        
        # Controller initializations
        self._controlFSM = ControlFSM(self._quadruped, 
                                      self._stateEstimator, 
                                      self._legController,
                                      self._desiredStateCommand)


    def run(self, gym, env, actor):
        """
        Runs the overall robot control system by calling each of the major components
        to run each of their respective steps.
        """
        # Update the joint states
        self._legController.updateData(gym, env, actor)
        self._legController.zeroCommand()
        self._legController.setEnable(True)
        self._legController.setMaxTorque(100)

        # update robot states
        self._stateEstimator.update(gym, env, actor, self._quadruped.bodyName)
        
        # Run the Control FSM code
        self._controlFSM.runFSM()

        # Sets the leg controller commands for the robot
        self._legController.updateCommand(gym, env, actor)
        self._iterations += 1

