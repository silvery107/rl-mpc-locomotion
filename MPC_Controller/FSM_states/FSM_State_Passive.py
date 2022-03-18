import numpy as np
from MPC_Controller.FSM_states.ControlFSMData import ControlFSMData
from MPC_Controller.FSM_states.FSM_State import FSM_State, FSM_StateName
from MPC_Controller.Parameters import Parameters
from MPC_Controller.utils import DTYPE

class FSM_State_Passive(FSM_State):
    """
    * FSM State that calls no controls. Meant to be a safe state where the
    * robot should not do anything as all commands will be set to 0.
    """
    def __init__(self, _controlFSMData: ControlFSMData):
        super().__init__(_controlFSMData, FSM_StateName.PASSIVE, "PASSIVE")

        # Do nothing
        
        # Set the pre controls safety checks
        self.checkSafeOrientation = False
        # Post control safety checks
        self.checkPDesFoot = False
        self.checkForceFeedForward = False

        self.iter = 0
    
    def onEnter(self):
        # Default is to not transition
        self.nextStateName = self.stateName

        # Reset the transition data
        self.transitionDone = False

    def run(self):
        """
         * Calls the functions to be executed on each control loop iteration.
        """
        if self.iter<10:
            qDes = np.array([0.0, 0.01, 0.01], dtype=DTYPE).reshape((3,1))
            qdDes = np.zeros((3,1), dtype=DTYPE)
            for leg in range(4):
                self.jointPDControl(leg, qDes, qdDes)

        else:
            pass
    
    def onExit(self):
        """
         * Cleans up the state information on exiting the state.
        """
        # Nothing to clean up when exiting
        pass

    def checkTransition(self):
        """
        * Manages which states can be transitioned into either by the user
        * commands or state event triggers.
        *
        * @return the enumerated FSM state name to transition into
        """
        self.nextStateName = self.stateName
        self.iter += 1

        # Switch FSM control mode
        if Parameters.control_mode is FSM_StateName.PASSIVE:
            pass

        elif Parameters.control_mode is FSM_StateName.RECOVERY_STAND:
            self.nextStateName = FSM_StateName.RECOVERY_STAND
            
        else:
            print("[CONTROL FSM] Bad Request: Cannot transition from "
                + self.stateName.name
                + " to "
                + Parameters.control_mode.name)
        return self.nextStateName

    def transition(self):
        """
        * Handles the actual transition for the robot between states.
        * Returns true when the transition is completed.
        *
        * @return true if transition is complete
        """
        # Finish Transition
        self.transitionDone = True
        # Return the transition data to the FSM
        return self.transitionDone