import math
import sys
import time

sys.path.append("..")

from MPC_Controller.DesiredStateCommand import DesiredStateCommand
import numpy as np
from MPC_Controller.convex_MPC.Gait import OffsetDurationGait
from MPC_Controller.FSM_states.ControlFSMData import ControlFSMData
from MPC_Controller.common.FootSwingTrajectory import FootSwingTrajectory
from MPC_Controller.utils import K_MAX_GAIT_SEGMENTS, coordinateRotation, CoordinateAxis, DTYPE, getSideSign

from MPC_Controller.Parameters import Parameters
import MPC_Controller.convex_MPC.mpc_osqp as mpc


num_legs = 4

class ConvexMPCLocomotion:
    def __init__(self, _dt:float, _iterationsBetweenMPC:int):
        self.iterationsBetweenMPC = int(_iterationsBetweenMPC)
        self.horizonLength = 10 # a fixed number for all mpc gait
        self.dt = _dt
        self.trotting = OffsetDurationGait(self.horizonLength, 
                            np.array([0, 5, 5, 0], dtype=DTYPE), 
                            np.array([5, 5, 5, 5], dtype=DTYPE), "Trotting")
        
        self.bounding = OffsetDurationGait(self.horizonLength,
                            np.array([5, 5, 0, 0], dtype=DTYPE), 
                            np.array([4, 4, 4, 4], dtype=DTYPE), "Bounding")
        
        self.pronking = OffsetDurationGait(self.horizonLength,
                            np.array([0, 0, 0, 0], dtype=DTYPE), 
                            np.array([4, 4, 4, 4], dtype=DTYPE), "Pronking")

        self.pacing = OffsetDurationGait(self.horizonLength,
                            np.array([5, 0, 5, 0], dtype=DTYPE), 
                            np.array([5, 5, 5, 5], dtype=DTYPE), "Pacing")

        self.galloping = OffsetDurationGait(self.horizonLength,
                            np.array([0, 2, 7, 9], dtype=DTYPE), 
                            np.array([4, 4, 4, 4], dtype=DTYPE), "Galloping")

        self.walking = OffsetDurationGait(self.horizonLength,
                            np.array([0, 3, 5, 8], dtype=DTYPE), 
                            np.array([5, 5, 5, 5], dtype=DTYPE), "Walking")

        self.trotRunning = OffsetDurationGait(self.horizonLength,
                            np.array([0, 5, 5, 0], dtype=DTYPE), 
                            np.array([4, 4, 4, 4], dtype=DTYPE), "Trot Running")

        self.dtMPC = self.dt * self.iterationsBetweenMPC
        self.default_iterations_between_mpc = self.iterationsBetweenMPC
        print("[Convex MPC] dt: %.3f iterations: %d, dtMPC: %.3f" % (self.dt, self.iterationsBetweenMPC, self.dtMPC))
        
        # self.rpy_comp = np.zeros((3,1),dtype=DTYPE)
        # self.rpy_int = np.zeros((3,1),dtype=DTYPE)
        self.firstSwing:list = None
        

        self.firstRun = True
        self.pFoot = [np.zeros((3,1)) for _ in range(4)]
        # self.x_comp_integral = 0.0
        # self.trajAll = [0.0 for _ in range(12*K_MAX_GAIT_SEGMENTS)]
        # force feedforward
        self.f_ff = [np.zeros((3,1), dtype=DTYPE) for _ in range(4)]
        self.iterationCounter = 0
        self._x_vel_des = 0.0
        self._y_vel_des = 0.0
        self.current_gait = 0
        self._roll_des = 0.0
        self._pitch_des = 0.0

        # self.stand_traj = [0.0 for _ in range(6)]
        # self.world_position_desired = np.zeros((3,1), dtype=DTYPE)
        self._yaw_des = 0.0
        self._yaw_turn_rate = 0.0
        self.footSwingTrajectories = [FootSwingTrajectory() for _ in range(4)]
        self.swingTimes = np.zeros((4,1), dtype=DTYPE)
        self.swingTimeRemaining = [0.0 for _ in range(4)]
        self.Kp:np.ndarray = None
        self.Kp_stance:np.ndarray = None
        self.Kd:np.ndarray = None
        self.Kd_stance:np.ndarray = None

    def initialize(self, data:ControlFSMData):
        if Parameters.cmpc_alpha > 1e-4:
            print("Alpha was set too high (" + str(Parameters.cmpc_alpha) + ") adjust to 1e-5\n")
            Parameters.cmpc_alpha = 1e-5

        self._cpp_mpc = mpc.ConvexMpc(data._quadruped._bodyMass, 
                            list(data._quadruped._bodyInertia),
                            num_legs,
                            self.horizonLength,
                            self.dtMPC, 
                            data._quadruped._mpc_weights, 
                            Parameters.cmpc_alpha,
                            mpc.QPOASES)

        self._x_vel_des = 0.0
        self._y_vel_des = 0.0
        self._yaw_turn_rate = 0.0
        self.firstSwing = [True for _ in range(4)]
        self.firstRun = True

    def recomputer_timing(self, iterations_per_mpc:int):
        self.iterationsBetweenMPC = iterations_per_mpc
        self.dtMPC = self.dt*iterations_per_mpc

    def __SetupCommand(self, data:ControlFSMData):

        self.__body_height = data._quadruped._bodyHeight
        filter = 0.1

        x_vel_cmd = DesiredStateCommand.x_vel_cmd
        y_vel_cmd = DesiredStateCommand.y_vel_cmd
        self._yaw_turn_rate = DesiredStateCommand.yaw_turn_rate

        self._x_vel_des = self._x_vel_des*(1-filter) + x_vel_cmd*filter
        self._y_vel_des = self._y_vel_des*(1-filter) + y_vel_cmd*filter

        # self.__yaw_des = data._stateEstimator.getResult().rpy[2].item() + self.dt*self._yaw_turn_rate

    def solveDenseMPC(self, mpcTable:list, data:ControlFSMData):
        seResult = data._stateEstimator.getResult()
        r_feet = np.array([self.pFoot[i] - seResult.position for i in range(4)], dtype=DTYPE).reshape((3,4))

        self.dtMPC = self.dt*self.iterationsBetweenMPC

        timer = time.time()

        # *Normal Vector of ground
        gravity_projection_vec = np.array([0, 0, 1],dtype=DTYPE)
        # gravity_projection_vec = seResult.ground_normal
        
        # *Google's way of states
        com_roll_pitch_yaw = np.array([seResult.rpyBody[0], seResult.rpyBody[1], 0], dtype=DTYPE)
        com_position = np.array([0, 0, seResult.position[2]], dtype=DTYPE)
        com_angular_velocity = seResult.omegaBody.flatten()
        com_velocity = seResult.vBody.flatten()

        desired_com_position = np.array([0., 0., self.__body_height], dtype=DTYPE)
        desired_com_velocity = np.array([self._x_vel_des, self._y_vel_des, 0], dtype=DTYPE)
        desired_com_roll_pitch_yaw = np.zeros(3, dtype=DTYPE) # walk parallel to the ground
        desired_com_angular_velocity = np.array([0, 0, self._yaw_turn_rate], dtype=DTYPE)

        if Parameters.cmpc_print_states:
            print("------------------------------------------")
            print("Com Pos: {: .4f}, {: .4f}, {: .4f}".format(*com_position))
            print("Com Vel: {: .4f}, {: .4f}, {: .4f}".format(*com_velocity))
            print("Com Ang: {: .4f}, {: .4f}, {: .4f}".format(*com_angular_velocity))
            print("Com RPY: {: .4f}, {: .4f}, {: .4f}".format(*np.rad2deg(com_roll_pitch_yaw)))
            print("------------------------------------------")
            print("Des Pos: {: .4f}, {: .4f}, {: .4f}".format(*desired_com_position))
            print("Des Vel: {: .4f}, {: .4f}, {: .4f}".format(*desired_com_velocity))
            print("Des Ang: {: .4f}, {: .4f}, {: .4f}".format(*desired_com_angular_velocity))
            print("Des RPY: {: .4f}, {: .4f}, {: .4f}".format(*np.rad2deg(desired_com_roll_pitch_yaw)))

        predicted_contact_forces = self._cpp_mpc.compute_contact_forces(
            com_position, # com_position (set x y to 0.0)
            com_velocity, # com_velocity
            com_roll_pitch_yaw, # com_roll_pitch_yaw (set yaw to 0.0)
            gravity_projection_vec,  # Normal Vector of ground
            com_angular_velocity, # com_angular_velocity
            np.asarray(mpcTable, dtype=DTYPE),  # Foot contact states
            np.array(r_feet.flatten(), dtype=DTYPE),  # foot_positions_base_frame
            data._quadruped._friction_coeffs,  # foot_friction_coeffs
            desired_com_position,  # desired_com_position
            desired_com_velocity,  # desired_com_velocity
            desired_com_roll_pitch_yaw,  # desired_com_roll_pitch_yaw
            desired_com_angular_velocity  # desired_com_angular_velocity
            )

        for leg in range(4):
            self.f_ff[leg] = np.array(predicted_contact_forces[leg*3: (leg+1)*3],dtype=DTYPE).reshape((3,1))

        if Parameters.cmpc_print_total_time:
            print("MPC Update Time %.3f s\n"%(time.time()-timer))

    def updateMPCIfNeeded(self, mpcTable:list, data:ControlFSMData):
        
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

        self.recomputer_timing(self.default_iterations_between_mpc)

        # integrate position setpoint
        # *v_des_robot = np.array([self.__x_vel_des, self.__y_vel_des, 0], dtype=DTYPE).reshape((3,1))
        # v_des_world = seResult.rBody.T @ v_des_robot
        # v_robot = seResult.vWorld
        v_des_robot = np.array([self._x_vel_des, self._y_vel_des, 0], dtype=DTYPE).reshape((3,1))
        v_des_world = v_des_robot

        for i in range(4):
            # *self.pFoot[i] = seResult.position + \
            #                 seResult.rBody.T @ (data._quadruped.getHipLocation(i)+
            #                 data._legController.datas[i].p)
            self.pFoot[i] = seResult.position + \
                            (data._quadruped.getHipLocation(i)+
                            data._legController.datas[i].p)
            # np.copyto(self.pFoot[i], seResult.position + \
            #                          seResult.rBody.T @ (data._quadruped.getHipLocation(i)+
            #                          data._legController.datas[i].p))
        

        # first time initialization
        if self.firstRun:
            self.firstRun = False
            for i in range(4):
                self.footSwingTrajectories[i].setHeight(0.05)
                self.footSwingTrajectories[i].setInitialPosition(self.pFoot[i])
                self.footSwingTrajectories[i].setFinalPosition(self.pFoot[i])


        # * foot placement
        for l in range(4):
            self.swingTimes[l] = gait.getCurrentSwingTime(self.dtMPC, l)

        interleave_y = [0.08, -0.08, -0.02, 0.02] # 这里调过了
        interleave_gain = -0.2
        v_abs = math.fabs(v_des_robot[0])

        for i in range(4):
            if self.firstSwing[i]:
                self.swingTimeRemaining[i] = self.swingTimes[i].item()
            else:
                self.swingTimeRemaining[i] -= self.dt

            self.footSwingTrajectories[i].setHeight(0.1)
            
            offset = np.array([0, getSideSign(i)*0.065, 0], dtype=DTYPE).reshape((3,1))
            pRobotFrame = data._quadruped.getHipLocation(i) + offset
            pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain
            stance_time = gait.getCurrentStanceTime(self.dtMPC, i)
            pYawCorrected = coordinateRotation(CoordinateAxis.Z, -self._yaw_turn_rate*stance_time/2) @ pRobotFrame

            des_vel = np.array([self._x_vel_des, self._y_vel_des, 0.0], dtype=DTYPE).reshape((3,1))

            # *Pf = seResult.position + seResult.rBody.T @ (pYawCorrected + des_vel * self.swingTimeRemaining[i])
            Pf = seResult.position + (pYawCorrected + des_vel * self.swingTimeRemaining[i])

            p_rel_max = 0.3
            pfx_rel = seResult.vWorld[0] * (0.5 + Parameters.cmpc_bonus_swing) * stance_time + \
                      0.03 * (seResult.vWorld[0] - v_des_world[0]) + \
                      (0.5 * seResult.position[2] / 9.81) * (seResult.vWorld[1] * self._yaw_turn_rate)
            
            pfy_rel = seResult.vWorld[1] * 0.5 * stance_time * self.dtMPC + \
                      0.03 * (seResult.vWorld[1] - v_des_world[1]) + \
                      (0.5 * seResult.position[2] / 9.81) * (-seResult.vWorld[0] * self._yaw_turn_rate)
            
            pfx_rel = min(max(pfx_rel, -p_rel_max), p_rel_max)
            pfy_rel = min(max(pfy_rel, -p_rel_max), p_rel_max)
            Pf[0] += pfx_rel
            Pf[1] += pfy_rel
            Pf[2] = -0.003
            self.footSwingTrajectories[i].setFinalPosition(Pf)

        # calc gait
        self.iterationCounter += 1

        self.Kp = np.array([700, 0, 0, 0, 700, 0, 0, 0, 150], dtype=DTYPE).reshape((3,3))
        self.Kp_stance = np.zeros_like(self.Kp)

        self.Kd = np.array([7, 0, 0, 0, 7, 0, 0, 0, 7], dtype=DTYPE).reshape((3,3))
        self.Kd_stance = self.Kd

        # gait
        contactStates = gait.getContactState()
        swingStates = gait.getSwingState()
        mpcTable = gait.getMpcTable()
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
                pDesFootWorld = self.footSwingTrajectories[foot].getPosition()
                vDesFootWorld = self.footSwingTrajectories[foot].getVelocity()
                # *pDesLeg = seResult.rBody @ (pDesFootWorld - seResult.position) \
                #           - data._quadruped.getHipLocation(foot)
                # vDesLeg = seResult.rBody @ (vDesFootWorld - seResult.vWorld)

                pDesLeg = (pDesFootWorld - seResult.position) \
                          - data._quadruped.getHipLocation(foot)
                vDesLeg = (vDesFootWorld - seResult.vWorld)

                data._legController.commands[foot].pDes = pDesLeg
                data._legController.commands[foot].vDes = vDesLeg
                data._legController.commands[foot].kpCartesian = self.Kp
                data._legController.commands[foot].kdCartesian = self.Kd
                # np.copyto(data._legController.commands[foot].pDes, pDesLeg, casting=CASTING)
                # np.copyto(data._legController.commands[foot].vDes, vDesLeg, casting=CASTING)
                # np.copyto(data._legController.commands[foot].kpCartesian, self.Kp, casting=CASTING)
                # np.copyto(data._legController.commands[foot].kdCartesian, self.Kd, casting=CASTING)

            else: #* foot is in stance
                self.firstSwing[foot] = True
                pDesFootWorld = self.footSwingTrajectories[foot].getPosition()
                vDesFootWorld = self.footSwingTrajectories[foot].getVelocity()
                # *pDesLeg = seResult.rBody @ (pDesFootWorld - seResult.position) \
                #           - data._quadruped.getHipLocation(foot)
                # vDesLeg = seResult.rBody @ (vDesFootWorld - seResult.vWorld)

                pDesLeg = (pDesFootWorld - seResult.position) \
                          - data._quadruped.getHipLocation(foot)
                vDesLeg = (vDesFootWorld - seResult.vWorld)
                
                data._legController.commands[foot].pDes = pDesLeg
                data._legController.commands[foot].vDes = vDesLeg
                data._legController.commands[foot].kpCartesian = self.Kp_stance
                data._legController.commands[foot].kdCartesian = self.Kd_stance

                data._legController.commands[foot].forceFeedForward = self.f_ff[foot]
                data._legController.commands[foot].kdJoint = np.identity(3, dtype=DTYPE)*0.2

                # np.copyto(data._legController.commands[foot].pDes, pDesLeg, casting=CASTING)
                # np.copyto(data._legController.commands[foot].vDes, vDesLeg, casting=CASTING)
                # np.copyto(data._legController.commands[foot].kpCartesian, self.Kp_stance, casting=CASTING)
                # np.copyto(data._legController.commands[foot].kdCartesian, self.Kd_stance, casting=CASTING)
                # np.copyto(data._legController.commands[foot].forceFeedForward, self.f_ff[foot], casting=CASTING)

                se_contactState[foot] = contactState

        # data._stateEstimator.setContactPhase(se_contactState)