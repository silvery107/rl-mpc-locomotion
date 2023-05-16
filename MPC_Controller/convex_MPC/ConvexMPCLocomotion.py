# import math
import time
import sys

import numpy as np
# import MPC_Controller.convex_MPC.mpc_osqp as mpc
from MPC_Controller.common.Quadruped import RobotType
from MPC_Controller.Parameters import Parameters
from MPC_Controller.convex_MPC.Gait import OffsetDurationGait
from MPC_Controller.common.DesiredStateCommand import DesiredStateCommand
from MPC_Controller.FSM_states.ControlFSMData import ControlFSMData
from MPC_Controller.common.FootSwingTrajectory import FootSwingTrajectory
from MPC_Controller.utils import CASTING, NUM_LEGS, DTYPE, getSideSign
from MPC_Controller.math_utils.orientation_tools import coordinateRotation, CoordinateAxis
from MPC_Controller.Logger import Logger

try:
    import mpc_osqp as mpc
except:
    print("You need to install 'rl-mpc-locomotion'")
    print("Run 'pip install -e .' in this repo")
    sys.exit()

class ConvexMPCLocomotion:
    def __init__(self, _dt:float, _iterationsBetweenMPC:int):
        self.iterationsBetweenMPC = int(_iterationsBetweenMPC)
        self.horizonLength = 10 # a fixed number for all mpc gait
        self.dt = _dt
        
        self.trotting = OffsetDurationGait(10, 
                            np.array([0, 5, 5, 0], dtype=DTYPE), 
                            np.array([5, 5, 5, 5], dtype=DTYPE), "Trotting")
        
        self.bounding = OffsetDurationGait(10,
                            np.array([5, 5, 0, 0], dtype=DTYPE), 
                            np.array([4, 4, 4, 4], dtype=DTYPE), "Bounding")
        
        self.pronking = OffsetDurationGait(10,
                            np.array([0, 0, 0, 0], dtype=DTYPE), 
                            np.array([4, 4, 4, 4], dtype=DTYPE), "Pronking")

        self.pacing = OffsetDurationGait(10,
                            np.array([5, 0, 5, 0], dtype=DTYPE), 
                            np.array([5, 5, 5, 5], dtype=DTYPE), "Pacing")

        self.galloping = OffsetDurationGait(10,
                            np.array([0, 2, 7, 9], dtype=DTYPE), 
                            np.array([4, 4, 4, 4], dtype=DTYPE), "Galloping")

        self.walking = OffsetDurationGait(10,
                            np.array([0, 3, 5, 8], dtype=DTYPE), 
                            np.array([5, 5, 5, 5], dtype=DTYPE), "Walking")

        self.trotRunning = OffsetDurationGait(10,
                            np.array([0, 5, 5, 0], dtype=DTYPE), 
                            np.array([4, 4, 4, 4], dtype=DTYPE), "Trot Running")

        self.dtMPC = self.dt * self.iterationsBetweenMPC
        self.default_iterations_between_mpc = self.iterationsBetweenMPC
        print("[Convex MPC] dt: %.3f iterations: %d, dtMPC: %.3f" % (self.dt, self.iterationsBetweenMPC, self.dtMPC))
        
        self.firstSwing:list = None
        self.firstRun = True
        self.iterationCounter = 0
        self.pFoot = np.zeros((4,3,1), dtype=DTYPE)
        self.f_ff = np.zeros((4,3,1), dtype=DTYPE)

        self.foot_positions = np.zeros((4,3,1), dtype=DTYPE)

        self.current_gait = 0
        self._x_vel_des = 0.0
        self._y_vel_des = 0.0
        self._yaw_turn_rate = 0.0

        self._roll_des = 0.0
        self._pitch_des = 0.0

        self.footSwingTrajectories = [FootSwingTrajectory() for _ in range(4)]
        self.swingTimes = np.zeros((4,1), dtype=DTYPE)
        self.swingTimeRemaining = [0.0 for _ in range(4)]

        self.Kp = np.array([700, 0, 0, 0, 700, 0, 0, 0, 150], dtype=DTYPE).reshape((3,3))
        self.Kd = np.array([7, 0, 0, 0, 7, 0, 0, 0, 7], dtype=DTYPE).reshape((3,3))
        self.Kp_stance = np.zeros_like(self.Kp)
        self.Kd_stance = self.Kd

        self.logger = Logger("logs/")
 
    def initialize(self, data:ControlFSMData):
        if Parameters.cmpc_alpha > 1e-4:
            print("Alpha was set too high (" + str(Parameters.cmpc_alpha) + ") adjust to 1e-5\n")
            Parameters.cmpc_alpha = 1e-5

        if Parameters.cmpc_enable_log:
            # flush last log
            if not self.logger.is_empty():
                self.logger.flush_logging()
            # start new logs
            self.logger.start_logging()

        self.iterationCounter = 0
        self._cpp_mpc = mpc.ConvexMpc(data._quadruped._bodyMass, 
                            list(data._quadruped._bodyInertia),
                            NUM_LEGS,
                            self.horizonLength,
                            self.dtMPC,
                            Parameters.cmpc_alpha,
                            mpc.QPOASES)

        self._x_vel_des = 0.0
        self._y_vel_des = 0.0
        self._yaw_turn_rate = 0.0
        self.firstSwing = [True for _ in range(4)]
        self.firstRun = True

    def recomputerTiming(self, iterations_per_mpc:int):
        self.iterationsBetweenMPC = iterations_per_mpc
        self.dtMPC = self.dt*iterations_per_mpc

    def __SetupCommand(self, data:ControlFSMData):

        self._body_height = data._quadruped._bodyHeight
        self._x_vel_des = data._desiredStateCommand.x_vel_cmd
        self._y_vel_des = data._desiredStateCommand.y_vel_cmd

        self._yaw_turn_rate = data._desiredStateCommand.yaw_turn_rate

    def solveDenseMPC(self, mpcTable:list, data:ControlFSMData):
        seResult = data._stateEstimator.getResult()
        
        # *MPC Weights
        if data._desiredStateCommand.mpc_weights is None:
            mpc_weight = data._quadruped._mpc_weights
        else:
            mpc_weight = data._desiredStateCommand.mpc_weights

        timer = time.time()

        # *Normal Vector of ground
        if Parameters.flat_ground:
            gravity_projection_vec = np.array([0, 0, 1],dtype=DTYPE)
        else:
            gravity_projection_vec = seResult.ground_normal_yaw
        
        # *Google's way of states
        com_roll_pitch_yaw = seResult.rpyBody.flatten()
        # com_roll_pitch_yaw = np.array([seResult.rpyBody[0], seResult.rpyBody[1], 0], dtype=DTYPE)
        com_position = seResult.position.flatten()
        com_angular_velocity = seResult.omegaBody.flatten()
        com_velocity = seResult.vBody.flatten()

        desired_com_position = np.array([0., 0., self._body_height], dtype=DTYPE)
        desired_com_velocity = np.array([self._x_vel_des, self._y_vel_des, 0], dtype=DTYPE)
        desired_com_roll_pitch_yaw = np.zeros(3, dtype=DTYPE) # walk parallel to the ground
        desired_com_angular_velocity = np.array([0, 0, self._yaw_turn_rate], dtype=DTYPE)

        if Parameters.cmpc_print_states:
            print("------------------------------------------")
            print("COM RPY: {: .4f}, {: .4f}, {: .4f}".format(*np.rad2deg(com_roll_pitch_yaw)))
            print("COM Pos: {: .4f}, {: .4f}, {: .4f}".format(*com_position))
            print("COM Ang: {: .4f}, {: .4f}, {: .4f}".format(*com_angular_velocity))
            print("COM Vel: {: .4f}, {: .4f}, {: .4f}".format(*com_velocity))
            # print("------------------------------------------")
            # print("DES RPY: {: .4f}, {: .4f}, {: .4f}".format(*np.rad2deg(desired_com_roll_pitch_yaw)))
            # print("DES Pos: {: .4f}, {: .4f}, {: .4f}".format(*desired_com_position))
            # print("DES Ang: {: .4f}, {: .4f}, {: .4f}".format(*desired_com_angular_velocity))
            # print("DES Vel: {: .4f}, {: .4f}, {: .4f}".format(*desired_com_velocity))
            print("------------------------------------------")
            print("GND Vec: {: .4f}, {: .4f}, {: .4f}".format(*gravity_projection_vec))

        predicted_contact_forces = self._cpp_mpc.compute_contact_forces(
            mpc_weight, # mpc weights list(12,)
            com_position, # com_position (set x y to 0.0)
            com_velocity, # com_velocity
            com_roll_pitch_yaw, # com_roll_pitch_yaw (set yaw to 0.0)
            gravity_projection_vec,  # Normal Vector of ground
            com_angular_velocity, # com_angular_velocity
            np.asarray(mpcTable, dtype=DTYPE),  # Foot contact states
            np.array(self.foot_positions.flatten(), dtype=DTYPE),  # foot_positions_base_frame
            data._quadruped._friction_coeffs,  # foot_friction_coeffs
            desired_com_position,  # desired_com_position
            desired_com_velocity,  # desired_com_velocity
            desired_com_roll_pitch_yaw,  # desired_com_roll_pitch_yaw
            desired_com_angular_velocity  # desired_com_angular_velocity
            )
        for leg in range(4):
            self.f_ff[leg] = np.array(predicted_contact_forces[leg*3: (leg+1)*3],dtype=DTYPE).reshape((3,1))

        if Parameters.cmpc_print_update_time:
            print("MPC Update Time %.3f s\n"%(time.time()-timer))
        
        if Parameters.cmpc_enable_log:
            mpc_state_loss = (com_roll_pitch_yaw - desired_com_roll_pitch_yaw).dot(mpc_weight[0:3]) + \
                            (com_position - desired_com_position).dot(mpc_weight[3:6]) + \
                            (com_angular_velocity - desired_com_velocity).dot(mpc_weight[6:9]) + \
                            (com_velocity - desired_com_velocity).dot(mpc_weight[9:12])
                        
            mpc_torque_loss = Parameters.cmpc_alpha * np.sum(predicted_contact_forces[:12])


            log_data_frame = dict(
                COM_RPY = com_roll_pitch_yaw, # COM_RPY
                COM_POS = com_position, # COM_POS
                COM_ANG = com_angular_velocity, # COM_ANG
                COM_VEL = com_velocity, # COM_VEL
                DES_RPY = desired_com_roll_pitch_yaw, # DES_RPY
                DES_POS = desired_com_position, # DES_POS
                DES_ANG = desired_com_angular_velocity, # DES_ANG
                DES_VEL = desired_com_velocity, # DES_VEL
                MPC_GRF = predicted_contact_forces[:12], # MPC_GRF
                MPC_LOS = mpc_state_loss+mpc_torque_loss, # MPC_LOS
                MPC_WEI = mpc_weight, # MPC_WEI
                TIM_STA = self.iterationCounter # TIM_STA
            )
            self.logger.update_logging(log_data_frame)

    def updateMPCIfNeeded(self, mpcTable:list, data:ControlFSMData):
        # self.solveDenseMPC(mpcTable, data)
        if(self.iterationCounter%self.iterationsBetweenMPC)==0:
            self.solveDenseMPC(mpcTable, data)

    def run(self, data:ControlFSMData):
        # Command Setup
        self.__SetupCommand(data)
        gaitNumber = Parameters.cmpc_gait.value
        seResult = data._stateEstimator.getResult()

        # pick gait
        gait = self.trotting
        if gaitNumber == 1:
            gait = self.bounding
        elif gaitNumber == 2:
            gait = self.pronking
        elif gaitNumber == 3:
            gait = self.pacing
        elif gaitNumber == 5:
            gait = self.galloping
        elif gaitNumber == 6:
            gait = self.walking
        elif gaitNumber == 7:
            gait = self.trotRunning

        self.current_gait = gaitNumber
        gait.setIterations(self.iterationsBetweenMPC, self.iterationCounter)

        self.recomputerTiming(self.default_iterations_between_mpc)

        for i in range(4):
            self.foot_positions[i] = data._quadruped.getHipLocation(i) + data._legController.datas[i].p
            self.pFoot[i] = self.foot_positions[i] + seResult.position
            # np.copyto(self.pFoot[i], seResult.position + \
                                        # (data._quadruped.getHipLocation(i)+
                                        # data._legController.datas[i].p))
        # self.foot_positions = np.array([self.pFoot[i] - seResult.position for i in range(4)], dtype=DTYPE).reshape((4,3,1))

        # * first time initialization
        if self.firstRun:
            self.firstRun = False
            data._stateEstimator._init_contact_history(self.foot_positions)
            for i in range(4):
                self.footSwingTrajectories[i].setHeight(0.05)
                self.footSwingTrajectories[i].setInitialPosition(self.pFoot[i])
                self.footSwingTrajectories[i].setFinalPosition(self.pFoot[i])

        if Parameters.flat_ground:
            data._stateEstimator._update_com_position_ground_frame(self.foot_positions)
        else:
            data._stateEstimator._compute_ground_normal_and_com_position(self.foot_positions)
        

        # * foot placement
        for l in range(4):
            self.swingTimes[l] = gait.getCurrentSwingTime(self.dtMPC, l)

        v_des_robot = np.array([self._x_vel_des, self._y_vel_des, 0], dtype=DTYPE).reshape((3,1))
        # interleave_y = [0.08, -0.08, -0.02, 0.02]
        # interleave_gain = -0.2
        # v_abs = math.fabs(v_des_robot[0])

        for i in range(4):
            if self.firstSwing[i]:
                self.swingTimeRemaining[i] = self.swingTimes[i].item()
            else:
                self.swingTimeRemaining[i] -= self.dt

            # self.footSwingTrajectories[i].setHeight(0.2)
            self.footSwingTrajectories[i].setHeight(self._body_height/3)
            
            offset = np.array([0, getSideSign(i)*data._quadruped._abadLinkLength, 0], dtype=DTYPE).reshape((3,1))
            pRobotFrame = data._quadruped.getHipLocation(i) + offset
            # pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain
            stance_time = gait.getCurrentStanceTime(self.dtMPC, i)
            pYawCorrected = coordinateRotation(CoordinateAxis.Z, -self._yaw_turn_rate*stance_time/2) @ pRobotFrame

            Pf = seResult.position + (pYawCorrected + v_des_robot * self.swingTimeRemaining[i])

            p_rel_max = 0.3
            pfx_rel = seResult.vBody[0] * (0.5 + Parameters.cmpc_bonus_swing) * stance_time + \
                      0.03 * (seResult.vBody[0] - v_des_robot[0]) + \
                      (0.5 * seResult.position[2] / 9.81) * (seResult.vBody[1] * self._yaw_turn_rate)
            
            pfy_rel = seResult.vBody[1] * 0.5 * stance_time * self.dtMPC + \
                      0.03 * (seResult.vBody[1] - v_des_robot[1]) + \
                      (0.5 * seResult.position[2] / 9.81) * (-seResult.vBody[0] * self._yaw_turn_rate)
            
            pfx_rel = min(max(pfx_rel, -p_rel_max), p_rel_max)
            pfy_rel = min(max(pfy_rel, -p_rel_max), p_rel_max)
            Pf[0] += pfx_rel
            Pf[1] += pfy_rel
            Pf[2] = -0.003
            self.footSwingTrajectories[i].setFinalPosition(Pf)

        # calc gait
        self.iterationCounter += 1

        # gait
        contactStates = gait.getContactState()
        swingStates = gait.getSwingState()
        mpcTable = gait.getMpcTable()

        # * update MPC
        self.updateMPCIfNeeded(mpcTable, data)

        se_contactState = np.array([0,0,0,0], dtype=DTYPE).reshape((4,1))

        for foot in range(4):
            contactState = contactStates[foot]
            swingState = swingStates[foot]
            if swingState > 0: #* foot is in swing
                if self.firstSwing[foot]:
                    self.firstSwing[foot] = False
                    self.footSwingTrajectories[foot].setInitialPosition(self.pFoot[foot])

                self.footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, self.swingTimes[foot].item())
                pDesFoot = self.footSwingTrajectories[foot].getPosition()
                vDesFoot = self.footSwingTrajectories[foot].getVelocity()

                pDesLeg = (pDesFoot - seResult.position) \
                          - data._quadruped.getHipLocation(foot)
                vDesLeg = (vDesFoot - seResult.vBody)

                # data._legController.commands[foot].pDes = pDesLeg
                # data._legController.commands[foot].vDes = vDesLeg
                # data._legController.commands[foot].kpCartesian = self.Kp
                # data._legController.commands[foot].kdCartesian = self.Kd

                np.copyto(data._legController.commands[foot].pDes, pDesLeg, casting=CASTING)
                np.copyto(data._legController.commands[foot].vDes, vDesLeg, casting=CASTING)
                np.copyto(data._legController.commands[foot].kpCartesian, self.Kp, casting=CASTING)
                np.copyto(data._legController.commands[foot].kdCartesian, self.Kd, casting=CASTING)

            else: #* foot is in stance
                self.firstSwing[foot] = True
                pDesFoot = self.footSwingTrajectories[foot].getPosition()
                vDesFoot = self.footSwingTrajectories[foot].getVelocity()

                pDesLeg = (pDesFoot - seResult.position) \
                          - data._quadruped.getHipLocation(foot)
                vDesLeg = (vDesFoot - seResult.vBody)
                
                # data._legController.commands[foot].pDes = pDesLeg
                # data._legController.commands[foot].vDes = vDesLeg
                # data._legController.commands[foot].kpCartesian = self.Kp_stance
                # data._legController.commands[foot].kdCartesian = self.Kd_stance

                # data._legController.commands[foot].forceFeedForward = self.f_ff[foot]
                # data._legController.commands[foot].kdJoint = np.identity(3, dtype=DTYPE)*0.2

                np.copyto(data._legController.commands[foot].pDes, pDesLeg, casting=CASTING)
                np.copyto(data._legController.commands[foot].vDes, vDesLeg, casting=CASTING)
                np.copyto(data._legController.commands[foot].kpCartesian, self.Kp_stance, casting=CASTING)
                np.copyto(data._legController.commands[foot].kdCartesian, self.Kd_stance, casting=CASTING)
                np.copyto(data._legController.commands[foot].forceFeedForward, self.f_ff[foot], casting=CASTING)
                np.copyto(data._legController.commands[foot].kdJoint, np.identity(3, dtype=DTYPE)*0.2, casting=CASTING)

                se_contactState[foot] = contactState

        data._stateEstimator.setContactPhase(se_contactState)
