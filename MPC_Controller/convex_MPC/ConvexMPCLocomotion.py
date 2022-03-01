import math
import sys
import time
sys.path.append("..")
import numpy as np
from MPC_Controller.convex_MPC.Gait import OffsetDurationGait
from MPC_Controller.FSM_states.ControlFSMData import ControlFSMData
from MPC_Controller.common.Quadruped import RobotType
from MPC_Controller.common.FootSwingTrajectory import FootSwingTrajectory
from MPC_Controller.utils import K_MAX_GAIT_SEGMENTS, coordinateRotation, CoordinateAxis, DTYPE, CASTING, getSideSign

from MPC_Controller.Parameters import Parameters
if Parameters.cmpc_solver_type == 1:
    import MPC_Controller.convex_MPC.C_SolverMPC as mpc
elif Parameters.cmpc_solver_type == 0:
    import MPC_Controller.convex_MPC.convexMPC_interface as mpc
elif Parameters.cmpc_solver_type == 2:
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

        self.standing = OffsetDurationGait(self.horizonLength, 
                            np.array([0, 0, 0, 0], dtype=DTYPE), 
                            np.array([10, 10, 10, 10], dtype=DTYPE), "Standing")
        
        self.bounding = OffsetDurationGait(self.horizonLength,
                            np.array([5, 5, 0, 0], dtype=DTYPE), 
                            np.array([4, 4, 4, 4], dtype=DTYPE), "Bounding")
        
        self.pronking = OffsetDurationGait(self.horizonLength,
                            np.array([0, 0, 0, 0], dtype=DTYPE), 
                            np.array([4, 4, 4, 4], dtype=DTYPE), "Pronking")

        self.pacing = OffsetDurationGait(self.horizonLength,
                            np.array([5, 0, 5, 0], dtype=DTYPE), 
                            np.array([5, 5, 5, 5], dtype=DTYPE), "Pacing")

        self.dtMPC = self.dt * self.iterationsBetweenMPC
        self.default_iterations_between_mpc = self.iterationsBetweenMPC
        print("[Convex MPC] dt: %.3f iterations: %d, dtMPC: %.3f" % (self.dt, self.iterationsBetweenMPC, self.dtMPC))
        
        if Parameters.cmpc_solver_type != 2:
            mpc.setup_problem(self.dtMPC, self.horizonLength, mu=0.4, fmax=120)
        # else:
        #     self._cpp_mpc = mpc.ConvexMpc(body_mass, body_inertia_list,
        #                                     num_legs,
        #                                     self.horizonLength,
        #                                     self.dtMPC, Parameters.yuxiang_weights, 1e-5,
        #                                     mpc.QPOASES)

        self.rpy_comp = np.zeros((3,1),dtype=DTYPE)
        self.rpy_int = np.zeros((3,1),dtype=DTYPE)
        self.firstSwing:list = None
        

        self.firstRun = True
        self.pFoot = [np.zeros((3,1)) for _ in range(4)]
        self.x_comp_integral = 0.0
        self.trajAll = [0.0 for _ in range(12*K_MAX_GAIT_SEGMENTS)]
        # force feedforward
        self.f_ff = [np.zeros((3,1), dtype=DTYPE) for _ in range(4)]
        self.iterationCounter = 0
        self.__x_vel_des = 0.0
        self.__y_vel_des = 0.0
        self.current_gait = 0
        self.__roll_des = 0.0
        self.__pitch_des = 0.0

        self.stand_traj = [0.0 for _ in range(6)]
        self.world_position_desired = np.zeros((3,1), dtype=DTYPE)
        self.__yaw_des = 0.0
        self.__yaw_turn_rate = 0.0
        self.footSwingTrajectories = [FootSwingTrajectory() for _ in range(4)]
        self.swingTimes = np.zeros((4,1), dtype=DTYPE)
        self.swingTimeRemaining = [0.0 for _ in range(4)]
        self.Kp:np.ndarray = None
        self.Kp_stance:np.ndarray = None
        self.Kd:np.ndarray = None
        self.Kd_stance:np.ndarray = None

    def initialize(self):
        self.firstSwing = [True for _ in range(4)]
        self.firstRun = True

    def recomputer_timing(self, iterations_per_mpc:int):
        self.iterationsBetweenMPC = iterations_per_mpc
        self.dtMPC = self.dt*iterations_per_mpc

    def __SetupCommand(self, data:ControlFSMData):

        self.__body_height = data._quadruped._bodyHeight
        x_vel_cmd = 0.0
        y_vel_cmd = 0.0
        filter = 0.1

        self.__yaw_turn_rate = data._desiredStateCommand.yaw_turn_rate
        y_vel_cmd = data._desiredStateCommand.y_vel_cmd
        x_vel_cmd = data._desiredStateCommand.x_vel_cmd

        self.__x_vel_des = self.__x_vel_des*(1-filter) + x_vel_cmd*filter
        self.__y_vel_des = self.__y_vel_des*(1-filter) + y_vel_cmd*filter

        self.__yaw_des = data._stateEstimator.getResult().rpy[2].item() + self.dt*self.__yaw_turn_rate
        self.__roll_des = 0.0
        self.__pitch_des = 0.0

    def solveDenseMPC(self, mpcTable:list, data:ControlFSMData):
        seResult = data._stateEstimator.getResult()
        
        Q = data._quadruped._mpc_weights[:12]
        alpha = Parameters.cmpc_alpha

        p = seResult.position
        v = seResult.vWorld
        w = seResult.omegaWorld
        q = seResult.orientation
        rpy = seResult.rpy

        # r = [self.pFoot[i%4][int(i/4)] - seResult.position[int(i/4)] for i in range(12)]
        r_feet = np.array([self.pFoot[i] - seResult.position for i in range(4)], dtype=DTYPE).reshape((3,4))
        yaw = float(seResult.rpy[2])
        weights = np.asarray(Q, dtype=DTYPE).reshape((12,1))

        if alpha > 1e-4:
            print("Alpha was set too high (" + str(alpha) + ") adjust to 1e-5\n")
            alpha = 1e-5
        
        pz_err = p[2] - self.__body_height
        vxy = np.array([seResult.vWorld[0], seResult.vWorld[1], 0], dtype=DTYPE).reshape((3,1))
        self.dtMPC = self.dt*self.iterationsBetweenMPC
        if Parameters.cmpc_solver_type == 2:
            self._cpp_mpc = mpc.ConvexMpc(data._quadruped._bodyMass, 
                                          list(data._quadruped._bodyInertia),
                                          num_legs,
                                          self.horizonLength,
                                          self.dtMPC, 
                                          data._quadruped._mpc_weights, 
                                          1e-5,
                                          mpc.QPOASES)
        else:
            mpc.setup_problem(self.dtMPC, self.horizonLength, mu=0.4, fmax=120)
            mpc.update_x_drag(self.x_comp_integral)

        if vxy[0]>0.3 or vxy[0]<-0.3:
            self.x_comp_integral += Parameters.cmpc_x_drag * pz_err * self.dtMPC / vxy[0]

        timer = time.time()
        if Parameters.cmpc_solver_type==1:
            mpc.update_problem_data(
                p, v, q.toNumpy(), w, 
                r_feet, yaw, weights, 
                np.array(self.trajAll, dtype=DTYPE).reshape((12*K_MAX_GAIT_SEGMENTS,1)),
                alpha, 
                np.array(mpcTable, dtype=DTYPE).reshape((K_MAX_GAIT_SEGMENTS,1)))

        elif Parameters.cmpc_solver_type==0:
            mpc.update_problem_data(p, v, q, w, rpy, r_feet, yaw, weights, self.trajAll, alpha, mpcTable)
        
        elif Parameters.cmpc_solver_type==2:
            predicted_contact_forces = self._cpp_mpc.compute_contact_forces(
                p.flatten(), #! com_position (set x y to 0.0)
                v.flatten(), #com_velocity
                rpy.flatten(), #! com_roll_pitch_yaw (set yaw to 0.0)
                np.array([0,0,1],dtype=DTYPE),  # Normal Vector of ground
                w.flatten(), #com_angular_velocity
                np.asarray(mpcTable, dtype=DTYPE),  # Foot contact states
                np.array(r_feet.flatten(), dtype=DTYPE),  #foot_positions_base_frame
                data._quadruped._friction_coeffs,  #foot_friction_coeffs
                np.array([0., 0., self.__body_height], dtype=DTYPE),  #desired_com_position
                np.array([self.__x_vel_des, self.__y_vel_des, 0], dtype=DTYPE),  #desired_com_velocity
                np.zeros(3, dtype=DTYPE),  #desired_com_roll_pitch_yaw
                np.array([0, 0, self.__yaw_turn_rate], dtype=DTYPE)  #desired_com_angular_velocity
            )

        for leg in range(4):
            if Parameters.cmpc_solver_type==2:
                self.f_ff[leg] = np.array(predicted_contact_forces[leg*3: (leg+1)*3],dtype=DTYPE).reshape((3,1))

            else:
                f = np.zeros((3,1), dtype=DTYPE)
                for axis in range(3):
                    f[axis] = mpc.get_solution(leg * 3 + axis)

                self.f_ff[leg] = - seResult.rBody @ f

        if Parameters.cmpc_print_total_time:
            print("MPC Update Time %.3f s\n"%(time.time()-timer))

    def updateMPCIfNeeded(self, mpcTable:list, data:ControlFSMData):
        
        if((self.iterationCounter%self.iterationsBetweenMPC)==0):
            seResult = data._stateEstimator.getResult()
            p = seResult.position
            v_des_robot = np.array([self.__x_vel_des, self.__y_vel_des, 0], dtype=DTYPE).reshape((3,1))
            v_des_world = seResult.rBody.T @ v_des_robot
            
            if self.current_gait==4: # stand gait
                trajInitial = [self.__roll_des,
                               self.__pitch_des,
                               self.stand_traj[5],
                               self.stand_traj[0],
                               self.stand_traj[1],
                               self.__body_height,
                               0,0,0,0,0,0]

                for i in range(self.horizonLength):
                    for j in range(12):
                        self.trajAll[12*i+j] = trajInitial[j]
            
            else: # other gait
                max_pos_error = 0.1
                xStart = self.world_position_desired[0].copy()
                yStart = self.world_position_desired[1].copy()

                if xStart-p[0] > max_pos_error:
                    xStart = p[0] + max_pos_error
                
                if p[0]-xStart > max_pos_error:
                    xStart = p[0] - max_pos_error

                if yStart-p[1] > max_pos_error:
                    yStart = p[1] + max_pos_error
                
                if p[1]-yStart > max_pos_error:
                    yStart = p[1] - max_pos_error

                self.world_position_desired[0] = xStart
                self.world_position_desired[1] = yStart

                trajInitial = [
                    self.rpy_comp[0],
                    self.rpy_comp[1],
                    self.__yaw_des,
                    xStart,
                    yStart,
                    self.__body_height,
                    0,
                    0,
                    self.__yaw_turn_rate,
                    v_des_world[0],
                    v_des_world[1],
                    0
                ]

                for i in range(self.horizonLength):
                    for j in range(12):
                        self.trajAll[12*i + j] = trajInitial[j]
                    
                    if i == 0: # start at current position
                        self.trajAll[2] = seResult.rpy[2]
                    else:
                        self.trajAll[12*i + 3] = self.trajAll[12 * (i - 1) + 3] + self.dtMPC * v_des_world[0]
                        self.trajAll[12*i + 4] = self.trajAll[12 * (i - 1) + 4] + self.dtMPC * v_des_world[1]
                        self.trajAll[12*i + 2] = self.trajAll[12 * (i - 1) + 2] + self.dtMPC * self.__yaw_turn_rate

            self.solveDenseMPC(mpcTable, data)



    def run(self, data:ControlFSMData):
        # Command Setup
        self.__SetupCommand(data)
        gaitNumber = Parameters.cmpc_gait.value
        seResult = data._stateEstimator.getResult()

        # Check if transition to standing
        if (gaitNumber==4 and self.current_gait!=4) or self.firstRun:
            self.stand_traj[0] = seResult.position[0]
            self.stand_traj[1] = seResult.position[1]
            self.stand_traj[2] = 0.21
            self.stand_traj[3] = 0
            self.stand_traj[4] = 0
            self.stand_traj[5] = seResult.rpy[2]
            self.world_position_desired[0] = self.stand_traj[0]
            self.world_position_desired[1] = self.stand_traj[1]

        # pick gait
        gait = self.trotting
        if gaitNumber == 1:
            gait = self.bounding

        elif gaitNumber == 2:
            gait = self.pronking

        elif gaitNumber == 3:
            gait = self.pacing

        elif gaitNumber == 4:
            gait = self.standing

        self.current_gait = gaitNumber
        gait.setIterations(self.iterationsBetweenMPC, self.iterationCounter)

        self.recomputer_timing(self.default_iterations_between_mpc)

        if self.__body_height < 0.02:
            self.__body_height = 0.29
        
        # integrate position setpoint
        v_des_robot = np.array([self.__x_vel_des, self.__y_vel_des, 0], dtype=DTYPE).reshape((3,1))
        v_des_world = seResult.rBody.T @ v_des_robot
        v_robot = seResult.vWorld
        
        # Integral-esque pitch and roll compensation
        if np.abs(v_robot[0]>0.2): # avoid dividing by zero
            self.rpy_int[1] += self.dt * (self.__pitch_des - seResult.rpy[1]) / v_robot[0]

        if np.abs(v_robot[1]>0.1): # avoid dividing by zero
            self.rpy_int[0] += self.dt * (self.__roll_des - seResult.rpy[0]) / v_robot[1]

        self.rpy_int[0] = min(max(self.rpy_int[0], -0.25), 0.25)
        self.rpy_int[1] = min(max(self.rpy_int[1], -0.25), 0.25)

        self.rpy_comp[0] = v_robot[1]*self.rpy_int[0]
        self.rpy_comp[1] = v_robot[0]*self.rpy_int[1]

        for i in range(4):
            self.pFoot[i] = seResult.position + \
                            seResult.rBody.T @ (data._quadruped.getHipLocation(i)+
                            data._legController.datas[i].p)
            # np.copyto(self.pFoot[i], seResult.position + \
            #                          seResult.rBody.T @ (data._quadruped.getHipLocation(i)+
            #                          data._legController.datas[i].p))
        
        if gait is not self.standing:
            self.world_position_desired += self.dt*np.array([v_des_world[0], v_des_world[1], 0.0], dtype=DTYPE).reshape((3,1))
        
        # some first time initialization
        if self.firstRun:
            self.world_position_desired[0] = seResult.position[0]
            self.world_position_desired[1] = seResult.position[1]
            self.world_position_desired[2] = seResult.rpy[2]

            for i in range(4):
                self.footSwingTrajectories[i].setHeight(0.05)
                self.footSwingTrajectories[i].setInitialPosition(self.pFoot[i])
                self.footSwingTrajectories[i].setFinalPosition(self.pFoot[i])

            self.firstRun = False

        # foot placement
        for l in range(4):
            self.swingTimes[l] = gait.getCurrentSwingTime(self.dtMPC, l)

        interleave_y = [-0.08, 0.08, 0.02, -0.02]
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
            pYawCorrected = coordinateRotation(CoordinateAxis.Z, -self.__yaw_turn_rate*stance_time/2) @ pRobotFrame

            des_vel = np.array([self.__x_vel_des, self.__y_vel_des, 0.0], dtype=DTYPE).reshape((3,1))

            Pf = seResult.position + seResult.rBody.T @ (pYawCorrected + des_vel * self.swingTimeRemaining[i])

            p_rel_max = 0.3
            pfx_rel = seResult.vWorld[0] * (0.5 + Parameters.cmpc_bonus_swing) * stance_time + \
                      0.03 * (seResult.vWorld[0] - v_des_world[0]) + \
                      (0.5 * seResult.position[2] / 9.81) * (seResult.vWorld[1] * self.__yaw_turn_rate)
            
            pfy_rel = seResult.vWorld[1] * 0.5 * stance_time * self.dtMPC + \
                      0.03 * (seResult.vWorld[1] - v_des_world[1]) + \
                      (0.5 * seResult.position[2] / 9.81) * (-seResult.vWorld[0] * self.__yaw_turn_rate)
            
            pfx_rel = min(max(pfx_rel, -p_rel_max), p_rel_max)
            pfy_rel = min(max(pfy_rel, -p_rel_max), p_rel_max)
            Pf[0] += pfx_rel
            Pf[1] += pfy_rel
            Pf[2] = -0.003
            self.footSwingTrajectories[i].setFinalPosition(Pf)

        # calc gait
        self.iterationCounter += 1

        self.Kp = np.array([700, 0, 0, 0, 700, 0, 0, 0, 150], dtype=DTYPE).reshape((3,3))
        self.Kp_stance = 0 * self.Kp

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
            # swingState = 0
            if swingState > 0: #* foot is in swing
                if self.firstSwing[foot]:
                    self.firstSwing[foot] = False
                    self.footSwingTrajectories[foot].setInitialPosition(self.pFoot[foot])

                self.footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, self.swingTimes[foot].item())
                pDesFootWorld = self.footSwingTrajectories[foot].getPosition()
                vDesFootWorld = self.footSwingTrajectories[foot].getVelocity()
                pDesLeg = seResult.rBody @ (pDesFootWorld - seResult.position) \
                          - data._quadruped.getHipLocation(foot)
                vDesLeg = seResult.rBody @ (vDesFootWorld - seResult.vWorld)

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
                pDesLeg = seResult.rBody @ (pDesFootWorld - seResult.position) \
                          - data._quadruped.getHipLocation(foot)
                vDesLeg = seResult.rBody @ (vDesFootWorld - seResult.vWorld)
                
                data._legController.commands[foot].pDes = pDesLeg
                data._legController.commands[foot].vDes = vDesLeg
                data._legController.commands[foot].kpCartesian = self.Kp_stance
                data._legController.commands[foot].kdCartesian = self.Kd_stance
                data._legController.commands[foot].forceFeedForward = self.f_ff[foot]
                # data._legController.commands[foot].kdJoint = np.identity(3, dtype=DTYPE)*0.2

                # np.copyto(data._legController.commands[foot].pDes, pDesLeg, casting=CASTING)
                # np.copyto(data._legController.commands[foot].vDes, vDesLeg, casting=CASTING)
                # np.copyto(data._legController.commands[foot].kpCartesian, self.Kp_stance, casting=CASTING)
                # np.copyto(data._legController.commands[foot].kdCartesian, self.Kd_stance, casting=CASTING)
                # np.copyto(data._legController.commands[foot].forceFeedForward, self.f_ff[foot], casting=CASTING)

                se_contactState[foot] = contactState

        # data._stateEstimator.setContactPhase(se_contactState)