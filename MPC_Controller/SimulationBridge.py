from MPC_Controller.RobotRunner import RobotRunner
from MPC_Controller.common.Quadruped import RobotType
from MIT_Controller import MIT_Controller
from enum import Enum, auto

class SimulatorMode(Enum):
    RUN_CONTROL_PARAMETERS = auto()
    RUN_CONTROLLER = auto()
    DO_NOTHING = auto()
    EXIT = auto()


class SimulationBridge:
    def __init__(self, robot:RobotType, robot_ctrl:MIT_Controller) -> None:
        self._robot = robot
        self._robotRunner = RobotRunner(robot_ctrl)
        self._userParams = robot_ctrl.getUserControlParameters()
        self._iterations = 0
        self._firstControllerRun = True

    def run(self):
        self._simMode = SimulatorMode.RUN_CONTROLLER

        while True:
            # ! update sim mode from user input
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


