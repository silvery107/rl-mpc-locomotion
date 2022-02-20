import numpy as np
from MPC_Controller.Parameters import Parameters
from MPC_Controller.RobotRunner import RobotRunner
from MPC_Controller.common.Quadruped import RobotType

from isaacgym import gymapi
from RL_Simulator.utils import acquire_sim, create_envs, add_viewer, add_force_sensor

robot = RobotType.MINI_CHEETAH
dt =  0.01
gym = gymapi.acquire_gym()
sim = acquire_sim(gym, dt)

# set up the env grid
num_envs = 4
envs_per_row = 2
env_spacing = 1.0
# one actor per env 
envs, actors = create_envs(gym, sim, robot, num_envs, envs_per_row, env_spacing)
# force_sensors = add_force_sensor(gym, num_envs, envs, actor_handles)
cam_pos = gymapi.Vec3(0.5, 0.6, 0.7) # w.r.t target env
viewer = add_viewer(gym, sim, envs[0], cam_pos)
gym.subscribe_viewer_keyboard_event(viewer, gymapi.KEY_4, "stand")
gym.subscribe_viewer_keyboard_event(viewer, gymapi.KEY_9, "trot")

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

print("[Simulator Driver] First run of robot controller...")
# simulation loop
while not gym.query_viewer_has_closed(viewer):

    # step the physics
    gym.simulate(sim)
    gym.fetch_results(sim, True)

    t = gym.get_sim_time(sim)
    for evt in gym.query_viewer_action_events(viewer):
        if evt.action == "stand" and evt.value > 0:
            print("[FSM LOCOMOTION] MPC Stand triggled")
            Parameters.cmpc_gait = 4
        elif evt.action == "trot" and evt.value > 0:
            print("[FSM LOCOMOTION] MPC Trot triggled")
            Parameters.cmpc_gait = 9

    # run controller
    for i in range(num_envs):
        controllers[i].run(gym, envs[i], actors[i])
    # robotRunner.run(gym, envs[0], actors[0])

    # update the viewer
    gym.step_graphics(sim)
    gym.draw_viewer(viewer, sim, True)
    
    # Wait for dt to elapse in real time.
    # This synchronizes the physics simulation with the rendering rate.
    # like time.sleep()
    gym.sync_frame_time(sim)

gym.destroy_viewer(viewer)
gym.destroy_sim(sim)
