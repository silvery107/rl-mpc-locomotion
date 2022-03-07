import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
os.sys.path.insert(0, parentdir)

from MPC_Controller.DesiredStateCommand import DesiredStateCommand
from MPC_Controller.Parameters import Parameters
from MPC_Controller.RobotRunner import RobotRunner
from MPC_Controller.common.Quadruped import RobotType
from RL_Simulator import gamepad_reader
from isaacgym import gymapi
from RL_Simulator.utils import *

use_gamepad = True
# robot = RobotType.A1
dt =  Parameters.controller_dt
gym = gymapi.acquire_gym()
sim = acquire_sim(gym, dt)
add_ground(gym, sim)
add_terrain(gym, sim, "slope")
add_terrain(gym, sim, "stair", 3.95, True)

robots = [RobotType.ALIENGO, RobotType.ALIENGO,
          RobotType.A1, RobotType.A1,
          RobotType.MINI_CHEETAH, RobotType.MINI_CHEETAH]

mini_cheetah = load_asset(gym, sim, RobotType.MINI_CHEETAH, False)
a1 = load_asset(gym, sim, RobotType.A1, False)
aliengo = load_asset(gym, sim, RobotType.ALIENGO, False)

assets = [aliengo, aliengo,
          a1, a1,
          mini_cheetah, mini_cheetah]


# set up the env grid
num_envs = 6
envs_per_row = 2
env_spacing = 0.5

env_lower = gymapi.Vec3(-env_spacing, -env_spacing, 0.0)
env_upper = gymapi.Vec3(env_spacing*2, env_spacing, env_spacing)

# cache some common handles for later use
envs = []
actors = []
height = 0.5

# create and populate the environments
for asset, i in zip(assets,range(num_envs)):
    env = gym.create_env(sim, env_lower, env_upper, envs_per_row)
    envs.append(env)

    pose = gymapi.Transform()
    pose.p = gymapi.Vec3(0.0, 0.0, height)

    # pose.r = gymapi.Quat(-0.707107, 0.0, 0.0, 0.707107) # rotate -90deg about x
    # pose.r = gymapi.Quat.from_axis_angle(gymapi.Vec3(1, 0, 0), -0.5*math.pi)

    actor_handle = gym.create_actor(env, asset, pose, "MyActor", group=i, filter=1)
    actors.append(actor_handle)


cam_pos = gymapi.Vec3(8, 2.5, 1.5) # w.r.t target env
viewer = add_viewer(gym, sim, envs[0], cam_pos)

controllers = []
for robot,idx in zip(robots,range(num_envs)):
    # configure the joints for effort control mode (once)
    props = gym.get_actor_dof_properties(envs[idx], actors[idx])
    props["driveMode"].fill(gymapi.DOF_MODE_EFFORT)
    props["stiffness"].fill(0.0)
    props["damping"].fill(0.0)
    gym.set_actor_dof_properties(envs[idx], actors[idx], props)

    # Setup MPC Controller
    robotRunner = RobotRunner()
    robotRunner.init(robot)
    controllers.append(robotRunner)

# Setup MPC Controller
if use_gamepad:
    gamepad = gamepad_reader.Gamepad(vel_scale_x=2, vel_scale_y=2, vel_scale_rot=2)

print("[Simulator Driver] First run of robot controller...")

count = 0
render_fps = 30
render_count = int(1/render_fps/Parameters.controller_dt)

# simulation loop
while not gym.query_viewer_has_closed(viewer):
    # step the physics
    gym.simulate(sim)
    gym.fetch_results(sim, True)

    # current_time = gym.get_sim_time(sim)

    if use_gamepad:
        lin_speed, ang_speed, e_stop = gamepad.get_command()
        Parameters.cmpc_gait = gamepad.get_gait()
        Parameters.control_mode = gamepad.get_mode()

        if not e_stop:
            DesiredStateCommand.x_vel_cmd = lin_speed[0]
            DesiredStateCommand.y_vel_cmd = lin_speed[1]
            DesiredStateCommand.yaw_turn_rate = ang_speed

    # run controller
    for i in range(num_envs):
        controllers[i].run(gym, envs[i], actors[i])

    if Parameters.locomotionUnsafe:
        gamepad.fake_event(ev_type='Key',code='BTN_TR',value=0)
        Parameters.locomotionUnsafe = False

    if count % render_count == 0:
        # update the viewer
        gym.step_graphics(sim)
        gym.draw_viewer(viewer, sim, True)
        
        # Wait for dt to elapse in real time.
        gym.sync_frame_time(sim)
        count = 0

    count += 1

if use_gamepad:
    gamepad.stop()
gym.destroy_viewer(viewer)
gym.destroy_sim(sim)