from MPC_Controller.DesiredStateCommand import DesiredStateCommand
from MPC_Controller.FSM_states.ControlFSMData import ControlFSMData
from MPC_Controller.Parameters import Parameters
from MPC_Controller.common.Quadruped import Quadruped, RobotType
from MPC_Controller.common.LegController import LegController
from MPC_Controller.StateEstimator import StateEstimator
from MPC_Controller.convex_MPC.ConvexMPCLocomotion import ConvexMPCLocomotion

class RobotRunnerMin:
    def __init__(self):
        pass


    def init(self, robotType:RobotType):
        """
        Initializes the robot model, state estimator, leg controller,
        robot data, and any control logic specific data.
        """
        self.robotType = robotType

        if self.robotType is RobotType.MINI_CHEETAH:
            self.cMPC = ConvexMPCLocomotion(Parameters.controller_dt,
                27/(1000.0*Parameters.controller_dt))
        elif self.robotType is RobotType.ALIENGO:
            self.cMPC = ConvexMPCLocomotion(Parameters.controller_dt,
                27/(1000.0*Parameters.controller_dt))
        elif self.robotType is RobotType.A1:
            self.cMPC = ConvexMPCLocomotion(Parameters.controller_dt,
                29/(1000.0*Parameters.controller_dt))
        else:
            raise Exception("Invalid RobotType")

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
        
        # init controller data
        self.data = ControlFSMData()
        self.data._quadruped = self._quadruped
        self.data._stateEstimator = self._stateEstimator
        self.data._legController = self._legController
        self.data._desiredStateCommand = self._desiredStateCommand

        # init convex MPC controller
        self.cMPC.initialize(self.data)

    def reset(self):
        self.cMPC.initialize(self.data)
        self._desiredStateCommand.reset()
        self._stateEstimator.reset()

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
        self.cMPC.run(self.data)

        # Sets the leg controller commands for the robot
        legTorques = self._legController.updateCommand()

        return legTorques # numpy (12,) float32