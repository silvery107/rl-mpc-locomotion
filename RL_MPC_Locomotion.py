from MPC_Controller.DesiredStateCommand import DesiredStateCommand
from MPC_Controller.Parameters import Parameters
from MPC_Controller.RobotRunner import RobotRunner
from MPC_Controller.common.Quadruped import RobotType
from RL_Environment import gamepad_reader
from isaacgym import gymapi
from RL_Environment.sim_utils import acquire_sim, add_ground, add_terrain, add_uneven_terrains, create_envs, add_viewer, get_force_sensor

use_gamepad = True
robot = RobotType.A1
dt =  Parameters.controller_dt
gym = gymapi.acquire_gym()
sim = acquire_sim(gym, dt)
add_ground(gym, sim)
# add_terrain(gym, sim, "slope")
# add_terrain(gym, sim, "stair", 3.95, True)
# add_uneven_terrains(gym, sim)

# set up the env grid
num_envs = 4
envs_per_row = 2
env_spacing = 0.5
# one actor per env 
envs, actors = create_envs(gym, sim, robot, num_envs, envs_per_row, env_spacing)
# force_sensors = get_force_sensor(gym, envs, actors)
cam_pos = gymapi.Vec3(2,2,2) # w.r.t target env
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
if use_gamepad:
    gamepad = gamepad_reader.Gamepad(vel_scale_x=2, vel_scale_y=1.5, vel_scale_rot=2)

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
        dof_states = gym.get_actor_dof_states(envs[i], actors[i], gymapi.STATE_ALL)
        body_idx = gym.find_actor_rigid_body_index(envs[i], actors[i], controllers[i]._quadruped._bodyName, gymapi.DOMAIN_ACTOR)
        body_states = gym.get_actor_rigid_body_states(envs[i], actors[i], gymapi.STATE_ALL)[body_idx]
        legTorques = controllers[i].run(dof_states, body_states) # gym, envs[i], actors[i])
        gym.apply_actor_dof_efforts(envs[i], actors[i], legTorques / (Parameters.controller_dt*100))

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