from isaacgym import gymapi
import math
from MPC_Controller.common.Quadruped import RobotType

ASSET_ROOT = "/home/silvery/isaacgym/assets"
MINI_CHEETAH = "urdf/mini_cheetah/mini_cheetah.urdf"
XIAOTIAN = "urdf/Xiaotian-ROS/urdf/xiaotian_description.urdf"
ALIENGO = "urdf/aliengo_description/xacro/aliengo.urdf"
ANYMAL = "urdf/anymal_c/urdf/anymal.urdf"

fix_base_link = True
init_height = 0.55

def acquire_sim(gym, dt):
    # get default set of parameters
    sim_params = gymapi.SimParams()

    # set common parameters
    sim_params.dt = dt # control timestep
    sim_params.substeps = 1 # physics simulation timestep
    sim_params.up_axis = gymapi.UP_AXIS_Z
    sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.8)

    # set PhysX-specific parameters
    sim_params.physx.use_gpu = True
    sim_params.physx.solver_type = 1
    sim_params.physx.num_position_iterations = 6
    sim_params.physx.num_velocity_iterations = 1
    sim_params.physx.contact_offset = 0.01
    sim_params.physx.rest_offset = 0.0

    # create sim with these parameters
    sim = gym.create_sim(compute_device=0, graphics_device=0, type=gymapi.SIM_PHYSX, params=sim_params)

    # configure the ground plane
    plane_params = gymapi.PlaneParams()
    plane_params.normal = gymapi.Vec3(0, 0, 1)  # z-up!
    plane_params.distance = 0
    plane_params.static_friction = 1
    plane_params.dynamic_friction = 1
    plane_params.restitution = 0    # control the elasticity of collisions (amount of bounce)
    # create the ground plane
    gym.add_ground(sim, plane_params)
    return sim

def load_asset(gym, sim, robot, fix_base_link):
    if robot == RobotType.ALIENGO:
        asset_file = ALIENGO
    elif robot == RobotType.MINI_CHEETAH:
        asset_file = MINI_CHEETAH
    elif robot == RobotType.XIAOTIAN:
        asset_file = XIAOTIAN
    elif robot == RobotType.ANYMAL:
        asset_file = ANYMAL
    
    asset_options = gymapi.AssetOptions()
    asset_options.fix_base_link = fix_base_link
    asset_options.use_mesh_materials = True
    asset_options.flip_visual_attachments = False if asset_file==XIAOTIAN or asset_file==MINI_CHEETAH else True
    asset_options.armature = 0.01   # added to the diagonal elements of inertia tensors
                                    # for all of the asset’s rigid bodies/links. 
                                    # Could improve simulation stability
    print("Loading asset '%s' from '%s'" % (asset_file, ASSET_ROOT))
    asset = gym.load_asset(sim, ASSET_ROOT, asset_file, asset_options) # or load_asset_urdf
    return asset

def add_viewer(gym, sim, env, cam_pos):
    # add viewer
    cam_props = gymapi.CameraProperties()
    viewer = gym.create_viewer(sim, cam_props)
    # Look at the env
    # cam_pos = gymapi.Vec3(1.5, 1, 3)
    cam_target = gymapi.Vec3(0, 0, 0)
    gym.viewer_camera_look_at(viewer, env, cam_pos, cam_target)
    return viewer

def create_envs(gym, sim, robot, num_envs, envs_per_row, env_spacing):
    # load robot from urdf
    asset = load_asset(gym, sim, robot, fix_base_link)
    # set up the env grid
    env_lower = gymapi.Vec3(-env_spacing, -env_spacing, 0.0)
    env_upper = gymapi.Vec3(env_spacing, env_spacing, env_spacing)

    # cache some common handles for later use
    envs = []
    actor_handles = []
    height = init_height

    # create and populate the environments
    for i in range(num_envs):
        env = gym.create_env(sim, env_lower, env_upper, envs_per_row)
        envs.append(env)

        pose = gymapi.Transform()
        pose.p = gymapi.Vec3(0.0, 0.0, height)

        # pose.r = gymapi.Quat(-0.707107, 0.0, 0.0, 0.707107) # rotate -90deg about x
        # pose.r = gymapi.Quat.from_axis_angle(gymapi.Vec3(1, 0, 0), -0.5*math.pi)

        actor_handle = gym.create_actor(env, asset, pose, "MyActor", group=i, filter=1)
        actor_handles.append(actor_handle)
    
    return envs, actor_handles

def add_force_sensor(gym, num_envs, envs, actor_handles):
    # add force sensors
    sensor_pose = gymapi.Transform(gymapi.Vec3(0.0, 0.0, 0.0))
    # ! attention here, foot ids are not static for each robot
    foot_ids = [5, 9, 13, 17]
    # cache some common handles for later use
    force_sensors = []
    # create and populate the environments
    for idx in range(num_envs):
        for id in foot_ids:
            foot_handle = gym.get_actor_rigid_body_handle(envs[idx], actor_handles[idx], id)
            sensor = gym.create_force_sensor(envs[idx], foot_handle, sensor_pose)
            force_sensors.append(sensor)
    return force_sensors

