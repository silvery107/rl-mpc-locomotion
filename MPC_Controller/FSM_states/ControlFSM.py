"""
* The Finite State Machine that manages the robot's controls. Handles
* calls to the FSM State functions and manages transitions between all
* of the states.
"""

from MPC_Controller.FSM_states.FSM_State_Passive import FSM_State_Passive
from MPC_Controller.FSM_states.FSM_State_RecoveryStand import FSM_State_RecoveyrStand

from MPC_Controller.FSM_states.FSM_State import FSM_State, FSM_StateName
from MPC_Controller.common.Quadruped import Quadruped
from MPC_Controller.common.StateEstimator import StateEstimator
from MPC_Controller.common.LegController import LegController
from MPC_Controller.Parameters import Parameters
from MPC_Controller.FSM_states.ControlFSMData import ControlFSMData
from MPC_Controller.FSM_states.FSM_State_Locomotion import FSM_State_Locomotion
from MPC_Controller.common.DesiredStateCommand import DesiredStateCommand
from MPC_Controller.utils import FSM_OperatingMode

class FSM_StatesList:
    def __init__(self) -> None:
        self.invalid:FSM_State = None
        self.passive:FSM_State_Passive = None
        self.recoveryStand:FSM_State_RecoveyrStand = None
        self.locomotion:FSM_State_Locomotion = None

class ControlFSM:
    def __init__(self,
                 _quadruped:Quadruped,
                 _stateEstimator:StateEstimator,
                 _legController:LegController,
                 _desiredStateCommand:DesiredStateCommand):
        self.data = ControlFSMData()
        self.data._quadruped = _quadruped
        self.data._stateEstimator = _stateEstimator
        self.data._legController = _legController
        self.data._desiredStateCommand = _desiredStateCommand

        self.statesList = FSM_StatesList()
        self.statesList.invalid = None
        self.statesList.passive = FSM_State_Passive(self.data)
        self.statesList.locomotion = FSM_State_Locomotion(self.data)
        self.statesList.recoveryStand = FSM_State_RecoveyrStand(self.data)

        # FSM state information
        self.currentState:FSM_State = None
        self.nextState:FSM_State = None
        self.nextStateName:FSM_StateName = None
        
        self.transitionDone = False
        self.printIter = 0
        self.printNum = int(1000/(Parameters.controller_dt*100)) # N*(0.01s) in simulation time
        self.iter = 0

        # ! may need a SafetyChecker
        # self.safetyChecker = 

        self.initialize()

    def initialize(self):
        # Initialize a new FSM State with the control data
        if Parameters.control_mode is FSM_StateName.LOCOMOTION:
            self.currentState = self.statesList.locomotion
        elif Parameters.control_mode is FSM_StateName.PASSIVE:
            self.currentState = self.statesList.passive
        elif Parameters.control_mode is FSM_StateName.RECOVERY_STAND:
            self.currentState = self.statesList.recoveryStand
        else:
            raise Exception("Invalid Initial FSM State!")
            
        # Enter the new current state cleanly
        self.currentState.onEnter()
        # Initialize to not be in transition
        self.nextState = self.currentState
        # Initialize FSM mode to normal operation
        self.operatingMode = Parameters.operatingMode

    def runFSM(self):
        # Check the robot state for safe operation
        # operatingMode = safetyPreCheck();

        if self.operatingMode is FSM_OperatingMode.TEST:
            self.currentState.run()

        # Run normal controls if no transition is detected
        elif self.operatingMode is FSM_OperatingMode.NORMAL:
            # Check the current state for any transition
            nextStateName = self.currentState.checkTransition()

            # Detect a commanded transition
            if nextStateName != self.currentState.stateName:
                # Set the FSM operating mode to transitioning
                self.operatingMode = FSM_OperatingMode.TRANSITIONING
                # Get the next FSM State by name
                self.nextState = self.getNextState(nextStateName)
                # Print transition initialized info
                self.printInfo(1)
            else:
                # Run the iteration for the current state normally
                self.currentState.run()

        # Run the transition code while transition is occuring
        elif self.operatingMode is FSM_OperatingMode.TRANSITIONING:
            self.transitionDone = self.currentState.transition()

            # TODO Check the robot state for safe operation
            # safetyPostCheck()

            # Run the state transition
            if self.transitionDone:
                # Exit the current state cleanly
                self.currentState.onExit()

                # Print finalizing transition info
                self.printInfo(2)

                # Complete the transition
                self.currentState = self.nextState

                # Enter the new current state cleanly
                self.currentState.onEnter()

                # Return the FSM to normal operation mode
                self.operatingMode = FSM_OperatingMode.NORMAL
        else:
            raise NotImplementedError
            # TODO Check the robot state for safe operation
            # safetyPostCheck()
        

        # TODO if ESTOP
        # self.currentState = self.statesList.passive
        # self.currentState.onEnter()
        # nextStateName = self.currentState.stateName

        # Print the current state of the FSM
        self.printInfo(0) 
        self.iter += 1


    def getNextState(self, stateName:FSM_StateName):
        """
        * Returns the approptiate next FSM State when commanded.
        *
        * @param  next commanded enumerated state name
        * @return next FSM state
        """
        if stateName is FSM_StateName.INVALID:
            return self.statesList.invalid
        elif stateName is FSM_StateName.PASSIVE:
            return self.statesList.passive
        elif stateName is FSM_StateName.LOCOMOTION:
            return self.statesList.locomotion
        elif stateName is FSM_StateName.RECOVERY_STAND:
            return self.statesList.recoveryStand
        else:
            return self.statesList.invalid


    def printInfo(self, opt:int):
        """
        * Prints Control FSM info at regular intervals and on important events
        * such as transition initializations and finalizations. Separate function
        * to not clutter the actual code.
        *
        * @param printing mode option for regular or an event
        """
        if not Parameters.FSM_print_info:
            return
            
        if opt == 0:
            self.printIter += 1
            if self.printIter == self.printNum:
                print("---------------------------------------------------------")
                print("[CONTROL FSM] Printing FSM Info...")
                print("Iteration: " + str(self.iter))
                if self.operatingMode is FSM_OperatingMode.NORMAL:
                    print("Operating Mode:: NORMAL in "+self.currentState.stateString)
                elif self.operatingMode is FSM_OperatingMode.TRANSITIONING:
                    print("Operating Mode: TRANSITIONING from "+self.currentState.stateString+" to "+self.nextState.stateString)

                self.printIter = 0
        
        # Initializing FSM State transition
        elif opt == 1:
            print("[CONTROL FSM] Transition initialized from "+
                  self.currentState.stateString + " to " +
                  self.nextState.stateString)

        # Finalizing FSM State transition
        elif opt == 2:
            print("[CONTROL FSM] Transition finalizing from "+
                  self.currentState.stateString + " to " +
                  self.nextState.stateString)


