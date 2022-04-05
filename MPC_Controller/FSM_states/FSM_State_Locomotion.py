from math import fabs
from numpy.linalg import norm
# from MPC_Controller.deprecated.ConvexMPCLocomotion_copy import ConvexMPCLocomotion
from MPC_Controller.Parameters import Parameters
from MPC_Controller.common.Quadruped import RobotType
from MPC_Controller.FSM_states.ControlFSMData import ControlFSMData
from MPC_Controller.FSM_states.FSM_State import FSM_State, FSM_StateName
from MPC_Controller.convex_MPC.ConvexMPCLocomotion import ConvexMPCLocomotion
from MPC_Controller.math_utils.orientation_tools import deg2rad, rad2deg

class FSM_State_Locomotion(FSM_State):
    """
    * FSM State for robot locomotion. Manages the contact specific logic
    * and handles calling the interfaces to the controllers. This state
    * should be independent of controller, gait, and desired trajectory.
    """
    def __init__(self, _controlFSMData:ControlFSMData):
        super().__init__(_controlFSMData, FSM_StateName.LOCOMOTION, "LOCOMOTION")

        if _controlFSMData._quadruped._robotType is RobotType.MINI_CHEETAH:
            self.cMPC = ConvexMPCLocomotion(Parameters.controller_dt,
                27/(1000.0*Parameters.controller_dt))
        elif _controlFSMData._quadruped._robotType is RobotType.ALIENGO:
            self.cMPC = ConvexMPCLocomotion(Parameters.controller_dt,
                27/(1000.0*Parameters.controller_dt))
        elif _controlFSMData._quadruped._robotType is RobotType.A1:
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
        self.transitionDone = False

        self.cMPC.initialize(self._data)
        self._data._desiredStateCommand.reset()
        self._data._stateEstimator.reset()
        if Parameters.FSM_print_info:
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
            if Parameters.control_mode is FSM_StateName.LOCOMOTION:
                pass

            elif Parameters.control_mode is FSM_StateName.PASSIVE:
                # Requested change to PASSIVE
                self.nextStateName = FSM_StateName.PASSIVE

            elif Parameters.control_mode is FSM_StateName.RECOVERY_STAND:
                self.nextStateName = FSM_StateName.RECOVERY_STAND
            
            else:
                print("[CONTROL FSM] Bad Request: Cannot transition from "
                    + self.stateName.name
                    + " to "
                    + Parameters.control_mode.name)
        else:
            self.nextStateName = FSM_StateName.RECOVERY_STAND
            # TODO change to an individual indicator may be better
            Parameters.locomotionUnsafe = True
        
        return self.nextStateName

    def transition(self):
        """
        * Handles the actual transition for the robot between states.
        * Returns true when the transition is completed.
        *
        * @return true if transition is complete
        """
        if self.nextStateName is FSM_StateName.PASSIVE:
            self.turnOffAllSafetyChecks()
            self.transitionDone = True

        elif self.nextStateName is FSM_StateName.RECOVERY_STAND:
            self.transitionDone = True
        
        else:
            print("[CONTROL FSM] Something went wrong in transition")

        return self.transitionDone

    def locomotionSafe(self):
        if not Parameters.FSM_check_safety:
            return True
        
        seResult = self._data._stateEstimator.getResult()

        max_roll = 40
        max_pitch = 40

        if fabs(seResult.rpy[0]>deg2rad(max_roll)):
            print("[FSM LOCOMOTION] Unsafe locomotion: roll is %.3f degrees (max %.3f)"%(rad2deg(seResult.rpy[0]), max_roll))
            return False
        
        if fabs(seResult.rpy[1]) > deg2rad(max_pitch):
            print("[FSM LOCOMOTION] Unsafe locomotion: pitch is %.3f degrees (max %.3f)\n"% (rad2deg(seResult.rpy[1]), max_pitch))
            return False
        
        for leg in range(4):
            p_leg = self._data._legController.datas[leg].p
            if p_leg[2]>0:
                print("[FSM LOCOMOTION] Unsafe locomotion: leg %d is above hip (%.3f m)"%(leg, p_leg[2]))
                return False
            
            if fabs(p_leg[1]>0.18):
                print("[FSM LOCOMOTION] Unsafe locomotion: leg %d's y-position is bad (%.3f m)"%(leg, p_leg[1]))
                return False
            
            # v_leg = norm(self._data._legController.datas[leg].v)
            # if fabs(v_leg) > 11.0:
            #     print("[FSM LOCOMOTION] Unsafe locomotion: leg %d is moving too quickly (%.3f m/s)"%(leg, v_leg))
            #     return False
        
        return True

    def LocomotionControlStep(self):
        self.cMPC.run(self._data)
