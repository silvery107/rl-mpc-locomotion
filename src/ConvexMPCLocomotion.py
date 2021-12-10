import numpy as np
import LegController as legctl
import Gait
import convexMPC_interface as cin

DTYPE = np.float32

class CMPC_Result:
    commands = [legctl.LegController() for _ in range(4)]
    contactPhase = np.zeros((4,1), dtype=DTYPE)

class ConvexMPCLocomotion:
    def __init__(self, _dt:float, _iterationsBetweenMPC:int, parameters):
        self.iterationsBetweenMPC = _iterationsBetweenMPC
        self.horizonLength = 10
        self.dt = _dt
        self.trotting = Gait.OffsetDurationGait(self.horizonLength, np.array([0,5,5,0]), np.array([5,5,5,5]), "Trotting")
        self.standing = Gait.OffsetDurationGait(self.horizonLength, np.array([0,0,0,0]), np.array([10,10,10,10]), "Standing")
        self.__parameters = parameters
        self.dtMPC = self.dt*self.iterationsBetweenMPC
        print("[Convex MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n"% (self.dt, self.iterationsBetweenMPC, self.dtMPC))
        cin.setup_problem(self.dtMPC, self.horizonLength, mu=0.4, fmax=120)


        self.rpy_comp = np.zeros((3,1),dtype=DTYPE)
        self.rpy_int = np.zeros((3,1),dtype=DTYPE)
        self.firstSwing = [True for _ in range(4)]
        
        # self.initSparseMPC()

        self.pBody_des = np.zeros((3,1), dtype=DTYPE)
        self.vBody_des = np.zeros((3,1), dtype=DTYPE)
        self.aBody_des = np.zeros((3,1), dtype=DTYPE)
        self.firstRun = True


    def initialize(self):
        for i in range(4):
            self.firstSwing[i] = True
        self.firstRun = True

    def recomputer_timing(self, iterations_per_mpc:int):
        self.iterationsBetweenMPC = iterations_per_mpc
        self.dtMPC = self.dt*iterations_per_mpc
    
    def __SetupCommand():
        pass

    def solveDenseMPC():
        pass

    def updateMPCIfNeeded():
        pass

    def run():
        pass