from MPC_Controller.RobotRunner import RobotRunner
from MPC_Controller.common.Quadruped import RobotType
from MPC_Controller.RobotController import RobotController
from enum import Enum, auto

class SimulatorMode(Enum):
    RUN_CONTROL_PARAMETERS = auto()
    RUN_CONTROLLER = auto()
    DO_NOTHING = auto()
    EXIT = auto()

class SimulationBridge:
    def __init__(self, robot:RobotType, robot_ctrl:RobotController) -> None:
        self._robot = robot
        self._robotRunner = RobotRunner(robot_ctrl)
        self._userParams = robot_ctrl.getUserControlParameters()
        self._iterations = 0
        self._firstControllerRun = True
        self._simMode = SimulatorMode.DO_NOTHING

    def setSimMode(self, simMode:SimulatorMode):
        self._simMode = simMode

    def run(self):
        # self._simMode = SimulatorMode.RUN_CONTROLLER

        while True:
            # let the simulator tells us which mode to run in
            if self._simMode == SimulatorMode.RUN_CONTROL_PARAMETERS:
                # handle control parameters
                pass
            elif self._simMode == SimulatorMode.RUN_CONTROLLER:
                self._iterations += 1
                self.runRobotControl()
            elif self._simMode == SimulatorMode.DO_NOTHING:
                pass
            elif self._simMode == SimulatorMode.EXIT:
                print("[Simulation Driver] Transitioned to exit mode")
                break
            else:
                raise "Invalid SimulatorMode"


    def runRobotControl(self):
        if self._firstControllerRun:
            self._firstControllerRun = False
            print("[Simulator Driver] First run of robot controller...")
            self._robotRunner.init(self._robot)

        self._robotRunner.run()


