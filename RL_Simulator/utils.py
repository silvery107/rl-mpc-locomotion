from re import A
from isaacgym import gymapi
import math
from MPC_Controller.common.Quadruped import RobotType

ASSET_ROOT = "assets"
MINI_CHEETAH = "mini_cheetah/mini_cheetah.urdf"
ALIENGO = "aliengo_description/xacro/aliengo.urdf"
A1 = "a1_description/a1.urdf"
XIAOTIAN = "Xiaotian-ROS/urdf/xiaotian_description.urdf"

FOOT_IDX = [4, 8, 12, 16]

fix_base_link = False
init_height = 0.5

def acquire_sim(gym, dt):
    # get default set of parameters
    sim_params = gymapi.SimParams()

    # set common parameters
    sim_params.dt = dt # control timestep
    sim_params.substeps = 2 # physics simulation timestep
    sim_params.up_axis = gymapi.UP_AXIS_Z
    sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.81)
    # sim_params.use_gpu_pipeline = True

    # set PhysX-specific parameters
    sim_params.physx.use_gpu = True
    sim_params.physx.solver_type = 1 # TGS
    sim_params.physx.num_position_iterations = 6 #4 improve solver convergence
    sim_params.physx.num_velocity_iterations = 1 # keep default
    # shapes whose distance is less than the sum of their contactOffset values will generate contacts
    sim_params.physx.contact_offset = 0.01 # 0.02
    # two shapes will come to rest at a distance equal to the sum of their restOffset values
    sim_params.physx.rest_offset = 0.0
    # A contact with a relative velocity below this will not bounce.
    sim_params.physx.bounce_threshold_velocity = 0.2
    # The maximum velocity permitted to be introduced by the solver to correct for penetrations in contacts.
    sim_params.physx.max_depenetration_velocity = 100.0
    # sim_params.physx.default_buffer_size_multiplier = 5.0
    # sim_params.physx.max_gpu_contact_pairs = 8388608 # 8*1024*1024

    # create sim with these parameters
    sim = gym.create_sim(compute_device=0, graphics_device=0, type=gymapi.SIM_PHYSX, params=sim_params)
    # gym.prepare_sim(sim)
    # configure the ground plane
    plane_params = gymapi.PlaneParams()
    plane_params.normal = gymapi.Vec3(0, 0, 1)  # z-up!
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
    elif robot == RobotType.A1:
        asset_file = A1
    else:
        raise Exception("Invalid RobotType")
    
    asset_options = gymapi.AssetOptions()
    asset_options.fix_base_link = fix_base_link
    asset_options.use_mesh_materials = True
    asset_options.flip_visual_attachments = False if asset_file==XIAOTIAN or asset_file==MINI_CHEETAH or asset_file==A1 else True
    # asset_options.density = 0.001
    # asset_options.angular_damping = 0.0
    # asset_options.linear_damping = 0.0
    # asset_options.thickness = 0.01
    # asset_options.disable_gravity = False
    asset_options.armature = 0.01   # added to the diagonal elements of inertia tensors
                                    # for all of the asset’s rigid bodies/links. 
                                    # Could improve simulation stability
    print("Loading asset '%s' from '%s'" % (asset_file, ASSET_ROOT))
    asset = gym.load_asset(sim, ASSET_ROOT, asset_file, asset_options) # or load_asset_urdf
    return asset


def create_envs(gym, sim, robot, num_envs, envs_per_row, env_spacing):
    # load robot from urdf
    asset = load_asset(gym, sim, robot, fix_base_link)

    # attach force sensor on robot foot
    create_asset_force_sensor(gym, asset)

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

def create_asset_force_sensor(gym, asset):
    sensor_pose = gymapi.Transform(gymapi.Vec3(0.0, 0.0, 0.0))
    
    sensor_props = gymapi.ForceSensorProperties()
    sensor_props.enable_forward_dynamics_forces = True
    sensor_props.enable_constraint_solver_forces = True
    sensor_props.use_world_frame = False

    for id in FOOT_IDX:
        gym.create_asset_force_sensor(asset, id, sensor_pose, sensor_props)

def get_force_sensor(gym, envs, actor_handles):
    sensors = []
    for env, actor_handle in zip(envs, actor_handles):
        num_sensors = gym.get_actor_force_sensor_count(env, actor_handle)
        for i in range(num_sensors):
            sensor = gym.get_actor_force_sensor(env, actor_handle, i)
            sensors.append(sensor)

    return sensors

def add_viewer(gym, sim, env, cam_pos):
    # add viewer
    cam_props = gymapi.CameraProperties()
    viewer = gym.create_viewer(sim, cam_props)
    # Look at the env
    cam_target = gymapi.Vec3(0, 0, 0.0)
    gym.viewer_camera_look_at(viewer, env, cam_pos, cam_target)
    return viewer