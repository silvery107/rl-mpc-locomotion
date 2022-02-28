import sys
sys.path.append("..")
import numpy as np
from math import fabs
from numpy.linalg import norm
from MPC_Controller.convex_MPC.ConvexMPCLocomotion import ConvexMPCLocomotion
from MPC_Controller.FSM_states.ControlFSMData import ControlFSMData
from MPC_Controller.Parameters import Parameters
from MPC_Controller.common.Quadruped import RobotType
from MPC_Controller.FSM_states.FSM_State import K_LOCOMOTION, K_PASSIVE, K_RECOVERY_STAND, FSM_State, FSM_StateName
from MPC_Controller.utils import DTYPE, deg2rad, rad2deg

class FSM_State_Locomotion(FSM_State):
    """
    * FSM State for robot locomotion. Manages the contact specific logic
    * and handles calling the interfaces to the controllers. This state
    * should be independent of controller, gait, and desired trajectory.
    """
    def __init__(self, _controlFSMData:ControlFSMData):
        super().__init__(_controlFSMData, FSM_StateName.LOCOMOTION, "LOCOMOTION")

        if _controlFSMData._quadruped._robotType == RobotType.MINI_CHEETAH:
            self.cMPC = ConvexMPCLocomotion(Parameters.controller_dt,
                27/(1000.0*Parameters.controller_dt))
        elif _controlFSMData._quadruped._robotType == RobotType.ALIENGO:
            self.cMPC = ConvexMPCLocomotion(Parameters.controller_dt,
                27/(1000.0*Parameters.controller_dt))
        elif _controlFSMData._quadruped._robotType == RobotType.A1:
            self.cMPC = ConvexMPCLocomotion(Parameters.controller_dt,
                27/(1000.0*Parameters.controller_dt))
        else:
            raise Exception("Invalid RobotType")
        
        self.turnOnAllSafetyChecks()

        # Turn off Foot pos command since it is set in WBC as operational task
        # self.checkPDesFoot = False
        # Initialize GRF and footstep locations to 0s
        # self.footFeedForwardForces = np.zeros((3,4), dtype=DTYPE)
        # self.footstepLocations = np.zeros((3,4), dtype=DTYPE)

        self.iter = 0
        
    def onEnter(self):
        # Default is to not transition
        self.nextStateName = self.stateName
        # Reset the transition data
        self.transitionData.zero()

        self.cMPC.initialize()
        print("[FSM LOCOMOTION] On Enter")

    def run(self):
        """
        * Calls the functions to be executed on each control loop iteration.
        """
        # # Call the locomotion control logic for this iteration
        self.LocomotionControlStep()

    def onExit(self):
        self.iter = 0
    
    def checkTransition(self):
        """
        * Manages which states can be transitioned into either by the user
        * commands or state event triggers.
        *
        * @return the enumerated FSM state name to transition into
        """
        self.iter += 1
        if self.locomotionSafe():
            if Parameters.control_mode == K_LOCOMOTION:
                pass

            elif Parameters.control_mode == K_PASSIVE:
                # Requested change to PASSIVE
                self.nextStateName = FSM_StateName.PASSIVE
                # Transition time is immediate
                self.transitionDuration = 0.0

            elif Parameters.control_mode == K_RECOVERY_STAND:
                self.nextStateName = FSM_StateName.RECOVERY_STAND
                self.transitionDuration = 0.0
            
            else:
                print("[CONTROL FSM] Bad Request: Cannot transition from "
                      + K_LOCOMOTION + " to "
                      + Parameters.control_mode)
        else:
            self.nextStateName = FSM_StateName.RECOVERY_STAND
            self.transitionDuration = 0.0
        
        return self.nextStateName

    def transition(self):
        """
        * Handles the actual transition for the robot between states.
        * Returns true when the transition is completed.
        *
        * @return true if transition is complete
        """
        if self.nextStateName == FSM_StateName.PASSIVE:
            self.turnOffAllSafetyChecks()
            self.transitionData.done = True

        elif self.nextStateName == FSM_StateName.RECOVERY_STAND:
            self.transitionData.done = True
        
        else:
            print("[CONTROL FSM] Something went wrong in transition")

        return self.transitionData

    def locomotionSafe(self):
        seResult = self._data._stateEstimator.getResult()

        max_roll = 40
        max_pitch = 40

        if fabs(seResult.rpy[0]>deg2rad(max_roll)):
            print("Unsafe locomotion: roll is %.3f degrees (max %.3f)"%(rad2deg(seResult.rpy[0]), max_roll))
            return False
        
        if fabs(seResult.rpy[1]) > deg2rad(max_pitch):
            print("Unsafe locomotion: pitch is %.3f degrees (max %.3f)\n"% (rad2deg(seResult.rpy[1]), max_pitch))
            return False
        
        for leg in range(4):
            p_leg = self._data._legController.datas[leg].p
            if p_leg[2]>0:
                print("Unsafe locomotion: leg %d is above hip (%.3f m)"%(leg, p_leg[2]))
                return False
            
            if fabs(p_leg[1]>0.18):
                print("Unsafe locomotion: leg %d's y-position is bad (%.3f m)"%(leg, p_leg[1]))
                return False
            
            v_leg = norm(self._data._legController.datas[leg].v)
            if fabs(v_leg) > 9.0:
                print("Unsafe locomotion: leg %d is moving too quickly (%.3f m/s)"%(leg, v_leg))
                return False
        
        return True

    def LocomotionControlStep(self):
        self.cMPC.run(self._data)
