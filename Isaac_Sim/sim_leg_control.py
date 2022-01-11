import sys
sys.path.append("..")
from isaacgym import gymapi
import numpy as np
from MPC_Controller.common.Quadruped import Quadruped, RobotType
from MPC_Controller.common.LegController import computeLegJacobianAndPosition

quad = Quadruped(RobotType.ALIENGO)

if __name__ == "__main__":
    from Isaac_Sim.util_isaac import *
    XIAOTIAN = "urdf/Xiaotian-ROS/urdf/xiaotian_description.urdf"
    ALIENGO = "urdf/aliengo_description/xacro/aliengo.urdf"
    ANYMAL = "urdf/anymal_c/urdf/anymal.urdf"

    gym = gymapi.acquire_gym()
    sim = start_sim(gym)
    asset = load_asset(gym, sim, robot=ALIENGO, fix_base_link=True)

    # set up the env grid
    num_envs = 4
    envs_per_row = 2
    env_spacing = 1.0
    envs, actor_handles = create_envs(gym, sim, asset, num_envs, envs_per_row, env_spacing)
    force_sensors = add_force_sensor(gym, num_envs, envs, actor_handles)
    cam_pos = gymapi.Vec3(1, 1, 1) # w.r.t target env
    viewer = add_viewer(gym, sim, envs[0], cam_pos)

    # set actor properties
    for idx in range(num_envs):
        props = gym.get_actor_dof_properties(envs[idx], actor_handles[idx])
        props["driveMode"].fill(gymapi.DOF_MODE_POS)
        props["stiffness"].fill(1000.0)
        props["damping"].fill(100.0)
        gym.set_actor_dof_properties(envs[idx], actor_handles[idx], props)

    # basic simulation loop
    while not gym.query_viewer_has_closed(viewer):

        # step the physics
        gym.simulate(sim)
        gym.fetch_results(sim, True)


        t = gym.get_sim_time(sim)
        if t>1:
            # read joint states
            body_states = gym.get_actor_dof_states(envs[0], actor_handles[0], gymapi.STATE_POS)
            ps = []
            for idx in range(4):
                q = np.asarray(body_states["pos"][3*idx:3*idx+3], dtype=np.float32)
                J, p = computeLegJacobianAndPosition(quad, q, idx)
                ps.append(p)

            # ! pos targets 有问题 p 是足端笛卡尔坐标
            pos_targets = np.asarray(ps).reshape(-1).astype(np.float32)
            gym.set_actor_dof_position_targets(envs[0], actor_handles[0], pos_targets)

        # update the viewer
        gym.step_graphics(sim);
        gym.draw_viewer(viewer, sim, True)
        
        # Wait for dt to elapse in real time.
        # This synchronizes the physics simulation with the rendering rate.
        gym.sync_frame_time(sim)

    gym.destroy_viewer(viewer)
    gym.destroy_sim(sim)
