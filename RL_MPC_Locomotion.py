import numpy as np
from enum import Enum, auto

from MPC_Controller.RobotRunner import RobotRunner
from MPC_Controller.RobotController import RobotController
from MPC_Controller.common.Quadruped import Quadruped, RobotType
from MPC_Controller.common.LegController import LegController
from MPC_Controller.StateEstimatorContainer import StateEstimatorContainer
from MPC_Controller.DesiredStateCommand import DesiredStateCommand
from MPC_Controller.Parameters import Parameters

from isaacgym import gymapi
from RL_Simulator.utils import *

class SimulatorMode(Enum):
    RUN_CONTROL_PARAMETERS = auto()
    RUN_CONTROLLER = auto()
    DO_NOTHING = auto()
    EXIT = auto()

robot = RobotType.ALIENGO
dt = 1/60
gym = gymapi.acquire_gym()
sim = acquire_sim(gym, dt)
asset = load_asset(gym, sim, robot=robot, fix_base_link=True)

# set up the env grid
num_envs = 1
envs_per_row = 1
env_spacing = 1.0
# one actor per env 
envs, actors = create_envs(gym, sim, asset, num_envs, envs_per_row, env_spacing)
# force_sensors = add_force_sensor(gym, num_envs, envs, actor_handles)
cam_pos = gymapi.Vec3(1, 1, 1) # w.r.t target env
viewer = add_viewer(gym, sim, envs[0], cam_pos)

# configure the joints for effort control mode (once)
for idx in range(num_envs):
    props = gym.get_actor_dof_properties(envs[idx], actors[idx])
    props["driveMode"].fill(gymapi.DOF_MODE_EFFORT)
    props["stiffness"].fill(0.0)
    props["damping"].fill(0.0)
    gym.set_actor_dof_properties(envs[idx], actors[idx], props)


# Setup MPC Controller
robotController = RobotController()
robotRunner = RobotRunner(robotController)
print("[Simulator Driver] First run of robot controller...")
robotRunner.init(robot)

# simulation loop
while not gym.query_viewer_has_closed(viewer):

    # step the physics
    gym.simulate(sim)
    gym.fetch_results(sim, True)

    # run controller
    robotRunner.run(gym, envs[0], actors[0])

    # update the viewer
    gym.step_graphics(sim);
    gym.draw_viewer(viewer, sim, True)
    
    # Wait for dt to elapse in real time.
    # This synchronizes the physics simulation with the rendering rate.
    # like time.sleep()
    gym.sync_frame_time(sim)

gym.destroy_viewer(viewer)
gym.destroy_sim(sim)
