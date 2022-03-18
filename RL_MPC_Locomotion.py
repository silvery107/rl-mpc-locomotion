import math
from MPC_Controller.Parameters import Parameters
from MPC_Controller.RobotRunnerFSM import RobotRunnerFSM
from MPC_Controller.RobotRunnerMin import RobotRunnerMin
from MPC_Controller.RobotRunnerPolicy import RobotRunnerPolicy
from MPC_Controller.common.Quadruped import RobotType
from MPC_Controller.utils import DTYPE, ControllerType
from RL_Environment import gamepad_reader
from isaacgym import gymapi
from RL_Environment.sim_utils import *
from argparse import ArgumentParser

parser = ArgumentParser(prog="RL_MPC_LOCOMOTION")

parser.add_argument("--robot", default="Aliengo", choices=[name.title() for name in RobotType.__members__.keys()], help="robot types")
parser.add_argument("--mode", default="FSM", choices=[name.title() for name in ControllerType.__members__.keys()], help="controller types")
parser.add_argument("--num_envs", default=1, type=int, help="the number of robots")
parser.add_argument("--render_fps", type=int, default=30, help="render fps")
parser.add_argument('--disable_gamepad', action='store_true')

args = parser.parse_args()

use_gamepad = not args.disable_gamepad
debug_vis = False # draw ground normal vector


def main():
    robot = RobotType[args.robot.upper()]
    dt =  Parameters.controller_dt
    gym = gymapi.acquire_gym()
    sim = acquire_sim(gym, dt)
    add_ground(gym, sim)
    # add_random_uniform_terrain(gym, sim)
    add_terrain(gym, sim, "slope")
    add_terrain(gym, sim, "stair", 3.95, True)
    # add_uneven_terrains(gym, sim)

    # set up the env grid
    num_envs = args.num_envs
    envs_per_row = int(math.sqrt(args.num_envs))
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
        controller_type = ControllerType[args.mode.upper()]
        if controller_type is ControllerType.FSM:
            robotRunner = RobotRunnerFSM()
        elif controller_type is ControllerType.MIN:
            robotRunner = RobotRunnerMin()
        elif controller_type is ControllerType.POLICY:
            robotRunner = RobotRunnerPolicy()
        else:
            raise Exception("Invalid ControllerType!")

        robotRunner.init(robot)
        controllers.append(robotRunner)

    # Setup MPC Controller
    if use_gamepad:
        gamepad = gamepad_reader.Gamepad(vel_scale_x=3, vel_scale_y=1.5, vel_scale_rot=5)

    count = 0
    render_fps = args.render_fps
    render_count = int(1/render_fps/Parameters.controller_dt)

    # simulation loop
    while not gym.query_viewer_has_closed(viewer):
        # step the physics
        gym.simulate(sim)
        gym.fetch_results(sim, True)

        # current_time = gym.get_sim_time(sim)
        commands = np.zeros(3, dtype=DTYPE)
        if use_gamepad:
            lin_speed, ang_speed, e_stop = gamepad.get_command()
            Parameters.cmpc_gait = gamepad.get_gait()
            Parameters.control_mode = gamepad.get_mode()
            if not e_stop:
                commands = np.array([lin_speed[0], lin_speed[1], ang_speed], dtype=DTYPE)

        # run controllers
        for idx, (env, actor, controller) in enumerate(zip(envs, actors, controllers)):
            dof_states = gym.get_actor_dof_states(env, actor, gymapi.STATE_ALL)
            body_idx = gym.find_actor_rigid_body_index(env, actor, controller._quadruped._bodyName, gymapi.DOMAIN_ACTOR)
            body_states = gym.get_actor_rigid_body_states(env, actor, gymapi.STATE_ALL)[body_idx]
            legTorques = controller.run(dof_states, body_states, commands)
            gym.apply_actor_dof_efforts(env, actor, legTorques / (Parameters.controller_dt*100))

        if Parameters.locomotionUnsafe:
            gamepad.fake_event(ev_type='Key',code='BTN_TR',value=0)
            Parameters.locomotionUnsafe = False

        if debug_vis:
            pos_np = np.asarray([p for p in body_states["pose"]["p"]], dtype=np.float32)
            gym.add_lines(viewer, envs[0], 1, 
                [pos_np, pos_np + controllers[0]._stateEstimator.result.ground_normal_world], 
                [[255,0,0]])

        if count % render_count == 0:
            # update the viewer
            gym.step_graphics(sim)
            gym.draw_viewer(viewer, sim, True)
            gym.clear_lines(viewer)
            # Wait for dt to elapse in real time.
            gym.sync_frame_time(sim)
            count = 0

        count += 1

    if use_gamepad:
        gamepad.stop()
        # gamepad.read_thread.join()
        # print("Gamepad read thread killed!") # too slow

    gym.destroy_viewer(viewer)
    gym.destroy_sim(sim)

if __name__=="__main__":
    main()