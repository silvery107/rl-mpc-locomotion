from MPC_Controller.FSM_states.ControlFSM import ControlFSM
from MPC_Controller.common.Quadruped import Quadruped, RobotType
from MPC_Controller.common.LegController import LegController
from MPC_Controller.state_estimate.StateEstimatorContainer import StateEstimatorContainer

# from isaacgym import gymapi
import numpy as np


class RobotRunner:
    def __init__(self):
        pass
        # self._iterations = 0


    def init(self, robotType:RobotType):
        """
        Initializes the robot model, state estimator, leg controller,
        robot data, and any control logic specific data.
        """
        self.robotType = robotType

        print("[RobotRunner] initialize")

        # init quadruped
        if self.robotType in RobotType:
            self._quadruped = Quadruped(self.robotType)
        else:
            raise "Invalid RobotType"

        # init leg controller
        self._legController = LegController(self._quadruped)

        # init state estimator
        self._stateEstimator = StateEstimatorContainer(self._quadruped)

        # init desired state command
        # self._desiredStateCommand = DesiredStateCommand()
        
        # Controller initializations
        self._controlFSM = ControlFSM(self._quadruped, 
                                      self._stateEstimator, 
                                      self._legController)#,
                                    #   self._desiredStateCommand)


    def run(self, dof_states, body_states): # gym, env, actor):
        """
        Runs the overall robot control system by calling each of the major components
        to run each of their respective steps.
        """
        # Update the joint states
        # dof_states = gym.get_actor_dof_states(env, actor, gymapi.STATE_ALL)
        self._legController.updateData(dof_states) # gym, env, actor)
        self._legController.zeroCommand()
        self._legController.setEnable(True)
        # self._legController.setMaxTorque(100)

        # update robot states
        # body_idx = gym.find_actor_rigid_body_index(env, actor, self._quadruped._bodyName, gymapi.DOMAIN_ACTOR)
        # body_states = gym.get_actor_rigid_body_states(env, actor, gymapi.STATE_ALL)[body_idx]
        self._stateEstimator.update(body_states) # gym, env, actor, self._quadruped._bodyName)
        
        # Run the Control FSM code
        self._controlFSM.runFSM()

        # Sets the leg controller commands for the robot
        legTorques = self._legController.updateCommand() # gym, env, actor)
        # gym.apply_actor_dof_efforts(env, actor, legTorques / (Parameters.controller_dt*100))

        # self._iterations += 1

        return legTorques

