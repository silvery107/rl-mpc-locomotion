from MPC_Controller.common.DesiredStateCommand import DesiredStateCommand
from MPC_Controller.FSM_states.ControlFSM import ControlFSM
from MPC_Controller.common.Quadruped import Quadruped, RobotType
from MPC_Controller.common.LegController import LegController
from MPC_Controller.common.StateEstimator import StateEstimator

class RobotRunnerFSM:
    def __init__(self):
        pass
        # self._iterations = 0


    def init(self, robotType:RobotType):
        """
        Initializes the robot model, state estimator, leg controller,
        robot data, and any control logic specific data.
        """
        self.robotType = robotType

        # print("[RobotRunner] initialize")

        # init quadruped
        if self.robotType in RobotType:
            self._quadruped = Quadruped(self.robotType)
        else:
            raise Exception("Invalid RobotType")

        # init leg controller
        self._legController = LegController(self._quadruped)

        # init state estimator
        self._stateEstimator = StateEstimator(self._quadruped)

        # init desired state command
        self._desiredStateCommand = DesiredStateCommand()
        
        # Controller initializations
        self._controlFSM = ControlFSM(self._quadruped, 
                                      self._stateEstimator, 
                                      self._legController,
                                      self._desiredStateCommand)

    def reset(self):
        self._controlFSM.initialize()

    def run(self, dof_states, body_states, commands):
        """
        Runs the overall robot control system by calling each of the major components
        to run each of their respective steps.
        """
        # Update desired commands
        self._desiredStateCommand.updateCommand(commands)

        # Update the joint states
        self._legController.updateData(dof_states)
        self._legController.zeroCommand()
        # self._legController.setEnable(True)

        # Update robot states
        self._stateEstimator.update(body_states)
        
        # Run the Control FSM code
        self._controlFSM.runFSM()

        # Sets the leg controller commands for the robot
        legTorques = self._legController.updateCommand()

        # self._iterations += 1

        return legTorques # numpy (12,) float32

