import numpy as np
from enum import Enum, auto
from MPC_Controller.RobotRunner import RobotRunner
from MPC_Controller.common.Quadruped import RobotType
from MPC_Controller.RobotController import RobotController

class SimulatorMode(Enum):
    RUN_CONTROL_PARAMETERS = auto()
    RUN_CONTROLLER = auto()
    DO_NOTHING = auto()
    EXIT = auto()

# Setup MPC Controller
robotType = RobotType.ALIENGO
robotController = RobotController()
robotRunner = RobotRunner(robotController)

iterations = 0
firstControllerRun = True
simMode = SimulatorMode.RUN_CONTROLLER

while True:
    # let the simulator tells us which mode to run in
    if simMode == SimulatorMode.RUN_CONTROL_PARAMETERS:
        # handle control parameters
        pass
    elif simMode == SimulatorMode.RUN_CONTROLLER:
        iterations += 1
        if firstControllerRun:
            firstControllerRun = False
            print("[Simulator Driver] First run of robot controller...")
            robotRunner.init(robotType)

        robotRunner.run()
    elif simMode == SimulatorMode.DO_NOTHING:
        pass
    elif simMode == SimulatorMode.EXIT:
        print("[Simulation Driver] Transitioned to exit mode")
        break
    else:
        raise "Invalid SimulatorMode"