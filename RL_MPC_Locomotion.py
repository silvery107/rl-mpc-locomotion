import numpy as np
from MPC_Controller.DesiredStateCommand import DesiredStateCommand
from MPC_Controller.Parameters import Parameters
from MPC_Controller.RobotRunner import RobotRunner
from MPC_Controller.common.Quadruped import RobotType
from RL_Simulator import gamepad_reader

from isaacgym import gymapi
from RL_Simulator.utils import acquire_sim, create_envs, add_viewer, add_force_sensor

use_gamepad = True
robot = RobotType.A1
dt =  Parameters.controller_dt
gym = gymapi.acquire_gym()
sim = acquire_sim(gym, dt)

# set up the env grid
num_envs = 4
envs_per_row = 2
env_spacing = 0.5
# one actor per env 
envs, actors = create_envs(gym, sim, robot, num_envs, envs_per_row, env_spacing)
# force_sensors = add_force_sensor(gym, num_envs, envs, actor_handles)
cam_pos = gymapi.Vec3(0.5, 0.6, 0.7) # w.r.t target env
viewer = add_viewer(gym, sim, envs[0], cam_pos)

controllers = []
for idx in range(num_envs):
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
# robotRunner = RobotRunner()
# robotRunner.init(robot)
if use_gamepad:
    gamepad = gamepad_reader.Gamepad(vel_scale_x=2.0, vel_scale_y=1., vel_scale_rot=1.)

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

    # robotRunner.run(gym, envs[0], actors[0])

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
