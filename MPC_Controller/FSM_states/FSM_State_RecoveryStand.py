from math import floor
import numpy as np
from MPC_Controller.FSM_states.ControlFSMData import ControlFSMData
from MPC_Controller.FSM_states.FSM_State import FSM_State, FSM_StateName
from MPC_Controller.Parameters import Parameters
from MPC_Controller.utils import DTYPE

StandUp = 0
FoldLegs = 1
RollOver = 2

class FSM_State_RecoveyrStand(FSM_State):
    def __init__(self, _controlFSMData: ControlFSMData):
        super().__init__(_controlFSMData, FSM_StateName.RECOVERY_STAND, "RECOVERY_STAND")


        # Set the pre controls safety checks
        self.checkSafeOrientation = False
        # Post control safety checks
        self.checkPDesFoot = False
        self.checkForceFeedForward = False

        self.iter = 0
        self._state_iter = 0
        self._motion_start_iter = 0
        self._flag = FoldLegs

        self.zero_vec3 = np.zeros((3,1), dtype=DTYPE)

        # goal configuration
        
        # Folding
        # * test passed
        self.fold_ramp_iter = int(45 / (Parameters.controller_dt*100))
        self.fold_settle_iter = int(75 / (Parameters.controller_dt*100))
        self.fold_jpos = np.array([0.0, 1.4, -2.7,
                                   -0.0, 1.4, -2.7,
                                   0.0, 1.4, -2.7,
                                   -0.0, 1.4, -2.7],
                                   dtype=DTYPE).reshape((4,3,1))

        # Stand Up 
        # * test passed
        self.standup_ramp_iter = int(30 / (Parameters.controller_dt*100))
        self.standup_settle_iter = int(30 / (Parameters.controller_dt*100))
        self.stand_jpos = np.array([0., 0.8, -1.6,
                                    0., 0.8, -1.6,
                                    0., 0.8, -1.6,
                                    0., 0.8, -1.6],
                                    dtype=DTYPE).reshape((4,3,1))

        # Rolling
        # * test passed
        self.rollover_ramp_iter = int(13 / (Parameters.controller_dt*100))
        self.rollover_settle_iter = int(15 / (Parameters.controller_dt*100))
        self.rolling_jpos = np.array([1.3, 3.1, -2.77,
                                      0.0, 1.6, -2.77,
                                      1.3, 3.1, -2.77,
                                      0.0, 1.6, -2.77],
                                     dtype=DTYPE).reshape((4,3,1))

        self.initial_jpos = np.zeros((4,3,1), dtype=DTYPE)


    def onEnter(self):
        # Default is to not transition
        self.nextStateName = self.stateName

        # Reset the transition data
        self.transitionDone = False

        # Reset iteration counter
        self.iter = 0
        self._state_iter = 0

        # initial configuration, position
        for leg in range(4):
            self.initial_jpos[leg] = self._data._legController.datas[leg].q

        body_height = self._data._stateEstimator.getResult().position[2]

        self._flag = FoldLegs
        if not self._UpsideDown():
            if 0.2<body_height and body_height<0.45:
                print("[Recovery Balance] body height is %.3f; Stand Up"%body_height)
                self._flag = StandUp
            else:
                print("[Recovery Balance] body height is %.3f; Folding legs"%body_height)
        else:
            print("[Recovery Balance] UpsideDown (%d)"%self._UpsideDown())
        
        self._motion_start_iter = 0

    def run(self):
        """
         * Calls the functions to be executed on each control loop iteration.
        """
        if self._flag == StandUp:
            self._StandUp(self._state_iter - self._motion_start_iter)
        elif self._flag == FoldLegs:
            self._FoldLegs(self._state_iter - self._motion_start_iter)
        elif self._flag == RollOver:
            self._RollOver(self._state_iter - self._motion_start_iter)

        self._state_iter += 1

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
        if Parameters.control_mode is FSM_StateName.RECOVERY_STAND:
            pass

        elif Parameters.control_mode is FSM_StateName.LOCOMOTION:
            self.nextStateName = FSM_StateName.LOCOMOTION

        elif Parameters.control_mode is FSM_StateName.PASSIVE:
            self.nextStateName = FSM_StateName.PASSIVE
            
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

        if self.nextStateName==FSM_StateName.PASSIVE:
            self.transitionDone = True

        elif self.nextStateName==FSM_StateName.LOCOMOTION:
            self.transitionDone = True

        else:
            print("[CONTROL FSM] Something went wrong in transition")

        # Return the transition data to the FSM
        return self.transitionDone

    def _UpsideDown(self):
        if self._data._stateEstimator.getResult().rBody[2,2]<0:
            return True

        return False

    def _SetJPosInterPts(self, 
                         curr_iter:int, max_iter:int, leg:int, 
                         ini:np.ndarray, fin:np.ndarray):
        a = 0.0
        b = 1.0

        # if we're done interpolating
        if curr_iter <= max_iter:
            b = float(curr_iter) / max_iter
            a = 1.0 - b
        
        # compute setpoints
        inter_pos = a * ini + b * fin
        # do control
        self.jointPDControl(leg, inter_pos, self.zero_vec3)


    def _StandUp(self, curr_iter:int):
        body_height = self._data._quadruped._bodyHeight #self._data._stateEstimator.getResult().position[2]
        something_wrong = False

        if self._UpsideDown() or body_height<0.1:
            something_wrong = True

        if curr_iter > floor(self.standup_ramp_iter*0.7) and something_wrong:
            # If body height is too low because of some reason 
            # even after the stand up motion is almost over 
            for leg in range(4):
                self.initial_jpos[leg] = self._data._legController.datas[leg].q
            self._flag = FoldLegs
            self._motion_start_iter = self._state_iter + 1

            print("[Recovery Balance - Warning] body height is still too low (%f) or UpsideDown (%d); Folding legs"
                %(body_height, self._UpsideDown()))
        else:
            for leg in range(4):
                self._SetJPosInterPts(curr_iter, self.standup_ramp_iter,
                                      leg, self.initial_jpos[leg], self.stand_jpos[leg])
        # setContactPhase = np.array([0.5,0.5,0.5,0.5], dtype=DTYPE).reshape((4,1))
        # self._data._stateEstimator.setContactPhase(setContactPhase)

    def _FoldLegs(self, curr_iter:int):
        for leg in range(4):
            self._SetJPosInterPts(curr_iter, self.rollover_ramp_iter, leg,
                                  self.initial_jpos[leg], self.fold_jpos[leg])
        if curr_iter >= self.fold_ramp_iter + self.fold_settle_iter:
            if self._UpsideDown():
                self._flag = RollOver
                for leg in range(4):
                    self.initial_jpos[leg] = self.fold_jpos[leg]
            else:
                self._flag = StandUp
                for leg in range(4):
                    self.initial_jpos[leg] = self.fold_jpos[leg]

            self._motion_start_iter = self._state_iter + 1

    def _RollOver(self, curr_iter:int):
        for leg in range(4):
            self._SetJPosInterPts(curr_iter, self.rollover_ramp_iter, leg,
                                  self.initial_jpos[leg], self.rolling_jpos[leg])
        
        if curr_iter > self.rollover_ramp_iter + self.rollover_settle_iter:
            self._flag = FoldLegs
            for leg in range(4):
                self.initial_jpos[leg] = self.rolling_jpos[leg]
            
            self._motion_start_iter = self._state_iter + 1
