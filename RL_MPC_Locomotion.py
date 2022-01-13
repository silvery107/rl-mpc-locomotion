import numpy as np
from enum import Enum, auto

from MPC_Controller.RobotRunner import RobotRunner
from MPC_Controller.common.Quadruped import RobotType
from MPC_Controller.RobotController import RobotController

from isaacgym import gymapi
from RL_Simulator.util_isaac import *

class SimulatorMode(Enum):
    RUN_CONTROL_PARAMETERS = auto()
    RUN_CONTROLLER = auto()
    DO_NOTHING = auto()
    EXIT = auto()

robot = RobotType.ALIENGO
gym = gymapi.acquire_gym()
sim = start_sim(gym)
asset = load_asset(gym, sim, robot=robot, fix_base_link=True)

# set up the env grid
num_envs = 1
envs_per_row = 1
env_spacing = 1.0
# one actor per env 
envs, actor_handles = create_envs(gym, sim, asset, num_envs, envs_per_row, env_spacing)
# force_sensors = add_force_sensor(gym, num_envs, envs, actor_handles)
cam_pos = gymapi.Vec3(1, 1, 1) # w.r.t target env
viewer = add_viewer(gym, sim, envs[0], cam_pos)

# configure the joints for effort control mode (once)
for idx in range(num_envs):
    props = gym.get_actor_dof_properties(envs[idx], actor_handles[idx])
    props["driveMode"].fill(gymapi.DOF_MODE_EFFORT)
    props["stiffness"].fill(0.0)
    props["damping"].fill(0.0)
    gym.set_actor_dof_properties(envs[idx], actor_handles[idx], props)



# Setup MPC Controller
robotController = RobotController()
robotRunner = RobotRunner(robotController)

iterations = 0
firstControllerRun = True
simMode = SimulatorMode.RUN_CONTROLLER

# basic simulation loop
while not gym.query_viewer_has_closed(viewer):

    # step the physics
    gym.simulate(sim)
    gym.fetch_results(sim, True)

    # let the simulator tells us which mode to run in
    if simMode == SimulatorMode.RUN_CONTROL_PARAMETERS:
        # handle control parameters
        pass
    elif simMode == SimulatorMode.RUN_CONTROLLER:
        iterations += 1
        if firstControllerRun:
            firstControllerRun = False
            print("[Simulator Driver] First run of robot controller...")
            robotRunner.init(robot)

        robotRunner.run()
    elif simMode == SimulatorMode.DO_NOTHING:
        pass
    elif simMode == SimulatorMode.EXIT:
        print("[Simulation Driver] Transitioned to exit mode")
        break
    else:
        raise "Invalid SimulatorMode"


    # t = gym.get_sim_time(sim)
    # if t>1:
    #     # read joint states
    #     body_states = gym.get_actor_dof_states(envs[0], actor_handles[0], gymapi.STATE_POS)
    #     ps = []


    #     # ! pos targets 有问题 p 是足端笛卡尔坐标
    #     pos_targets = np.asarray(ps).reshape(-1).astype(np.float32)
    #     gym.set_actor_dof_position_targets(envs[0], actor_handles[0], pos_targets)

    # update the viewer
    gym.step_graphics(sim);
    gym.draw_viewer(viewer, sim, True)
    
    # Wait for dt to elapse in real time.
    # This synchronizes the physics simulation with the rendering rate.
    gym.sync_frame_time(sim)

gym.destroy_viewer(viewer)
gym.destroy_sim(sim)
