from isaacgym import gymapi
from MPC_Controller.common.Quadruped import RobotType
from isaacgym.terrain_utils import *

ASSET_ROOT = "assets"
ALIENGO = "aliengo_description/urdf/aliengo.urdf"
A1 = "a1_description/urdf/a1.urdf"
GO1 = "go1_description/urdf/go1.urdf"
# MINI_CHEETAH = "mini_cheetah/mini_cheetah.urdf"

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

    # set PhysX-specific parameters
    sim_params.physx.use_gpu = True
    sim_params.physx.solver_type = 1 # TGS
    sim_params.physx.num_position_iterations = 6 #4 improve solver convergence
    sim_params.physx.num_velocity_iterations = 1 # keep default
    # shapes whose distance is less than the sum of their contactOffset values will generate contacts
    sim_params.physx.contact_offset = 0.02 # 0.02
    # two shapes will come to rest at a distance equal to the sum of their restOffset values
    sim_params.physx.rest_offset = 0.0
    # A contact with a relative velocity below this will not bounce.
    sim_params.physx.bounce_threshold_velocity = 0.2
    # The maximum velocity permitted to be introduced by the solver to correct for penetrations in contacts.
    sim_params.physx.max_depenetration_velocity = 100.0

    # create sim with these parameters
    sim = gym.create_sim(compute_device=0, graphics_device=0, type=gymapi.SIM_PHYSX, params=sim_params)
    
    return sim

def load_asset(gym, sim, robot, fix_base_link):
    if robot is RobotType.ALIENGO:
        asset_file = ALIENGO
    # elif robot is RobotType.MINI_CHEETAH:
    #     asset_file = MINI_CHEETAH
    elif robot is RobotType.A1:
        asset_file = A1
    elif robot is RobotType.GO1:
        asset_file = GO1
    else:
        raise Exception("Invalid RobotType")
    
    asset_options = gymapi.AssetOptions()
    asset_options.fix_base_link = fix_base_link
    asset_options.use_mesh_materials = True
    asset_options.flip_visual_attachments = True
    asset_options.angular_damping = 0.0
    asset_options.linear_damping = 0.0
    # asset_options.density = 0.001
    # asset_options.thickness = 0.01
    # asset_options.disable_gravity = False
    asset_options.armature = 0.01   # added to the diagonal elements of inertia tensors
                                    # for all of the asset’s rigid bodies/links. 
                                    # Could improve simulation stability
    print("Loading asset '%s' from '%s'" % (asset_file, ASSET_ROOT))
    asset = gym.load_urdf(sim, ASSET_ROOT, asset_file, asset_options) # or load_asset_urdf
    return asset


def create_envs(gym, sim, robot, num_envs, envs_per_row, env_spacing):
    # load robot from urdf
    asset = load_asset(gym, sim, robot, fix_base_link)

    # attach force sensor on robot foot
    # create_asset_force_sensor(gym, asset)

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

def add_viewer(gym, sim, env, cam_pos, _cam_target=[0.0, 0.0, 0.0]):
    # add viewer
    cam_props = gymapi.CameraProperties()
    viewer = gym.create_viewer(sim, cam_props)
    # Look at the env
    cam_target = gymapi.Vec3(*_cam_target)
    gym.viewer_camera_look_at(viewer, env, cam_pos, cam_target)
    return viewer

def add_ground(gym, sim):
    # configure the ground plane
    plane_params = gymapi.PlaneParams()
    plane_params.normal = gymapi.Vec3(0, 0, 1)  # z-up!
    plane_params.static_friction = 1
    plane_params.dynamic_friction = 1
    plane_params.restitution = 0    # control the elasticity of collisions (amount of bounce)
    # create the ground plane
    gym.add_ground(sim, plane_params)

def add_terrain(gym, sim, name="slope", x_offset=2., invert=False, width=2.8):
    # terrains
    num_terrains = 1
    terrain_width = 2.
    terrain_length = width
    horizontal_scale = 0.05  # [m] resolution in x
    vertical_scale = 0.005  # [m] resolution in z
    num_rows = int(terrain_width/horizontal_scale)
    num_cols = int(terrain_length/horizontal_scale)
    heightfield = np.zeros((num_terrains*num_rows, num_cols), dtype=np.int16)

    step_height = 0.07
    step_width = 0.3
    num_steps = terrain_width / step_width
    height = step_height * num_steps
    slope = height / terrain_width
    # num_stairs = height / step_height
    # step_width = terrain_length / num_stairs
    
    def new_sub_terrain(): return SubTerrain(width=num_rows, length=num_cols, vertical_scale=vertical_scale, horizontal_scale=horizontal_scale)
    if name=="slope":
        heightfield[0: num_rows, :] = sloped_terrain(new_sub_terrain(), slope=slope).height_field_raw
    elif name=="stair":
        heightfield[0: num_rows, :] = stairs_terrain(new_sub_terrain(), step_width=step_width, step_height=step_height).height_field_raw
    elif name=="pyramid":
        heightfield[0: num_rows, :] = pyramid_stairs_terrain(new_sub_terrain(), step_width=step_width, step_height=step_height).height_field_raw
    else:
        raise NotImplementedError("Not support terrains!")

    if invert:
        heightfield[0: num_rows, :] = heightfield[0: num_rows, :][::-1]

    # add the terrain as a triangle mesh
    vertices, triangles = convert_heightfield_to_trimesh(heightfield, horizontal_scale=horizontal_scale, vertical_scale=vertical_scale, slope_threshold=1.5)
    tm_params = gymapi.TriangleMeshParams()
    tm_params.nb_vertices = vertices.shape[0]
    tm_params.nb_triangles = triangles.shape[0]
    tm_params.transform.p.x = x_offset
    tm_params.transform.p.y = -1
    if name=="stair":
        tm_params.transform.p.z = -0.09
    elif name=="pyramid":
        tm_params.transform.p.z = 0.01

    gym.add_triangle_mesh(sim, vertices.flatten(), triangles.flatten(), tm_params)

def add_random_uniform_terrain(gym, sim):
    num_terrains = 1
    terrain_width = 50.
    terrain_length = 50.
    horizontal_scale = 0.1  # [m] resolution in x
    vertical_scale = 0.005  # [m] resolution in z
    num_rows = int(terrain_width/horizontal_scale)
    num_cols = int(terrain_length/horizontal_scale)
    heightfield = np.zeros((num_terrains*num_rows, num_cols), dtype=np.int16)
    
    def new_sub_terrain(): return SubTerrain(width=num_rows, length=num_cols, vertical_scale=vertical_scale, horizontal_scale=horizontal_scale)

    heightfield[0:1*num_rows, :] = random_uniform_terrain(new_sub_terrain(), min_height=-0.2, max_height=0.0, step=0.05, downsampled_scale=0.3).height_field_raw

    vertices, triangles = convert_heightfield_to_trimesh(heightfield, horizontal_scale=horizontal_scale, vertical_scale=vertical_scale, slope_threshold=1.5)
    tm_params = gymapi.TriangleMeshParams()
    tm_params.nb_vertices = vertices.shape[0]
    tm_params.nb_triangles = triangles.shape[0]
    tm_params.transform.p.x = -terrain_width/3
    tm_params.transform.p.y = -terrain_length/3
    gym.add_triangle_mesh(sim, vertices.flatten(), triangles.flatten(), tm_params)

def add_uneven_terrains(gym, sim):
    # terrains
    num_terrains = 4
    terrain_width = 12.
    terrain_length = 12.
    horizontal_scale = 0.25  # [m] resolution in x
    vertical_scale = 0.005  # [m] resolution in z
    num_rows = int(terrain_width/horizontal_scale)
    num_cols = int(terrain_length/horizontal_scale)
    heightfield = np.zeros((num_terrains*num_rows, num_cols), dtype=np.int16)
    
    def new_sub_terrain(): return SubTerrain(width=num_rows, length=num_cols, vertical_scale=vertical_scale, horizontal_scale=horizontal_scale)

    heightfield[0:1*num_rows, :] = random_uniform_terrain(new_sub_terrain(), min_height=-0.1, max_height=0.1, step=0.2, downsampled_scale=0.5).height_field_raw
    heightfield[1*num_rows:2*num_rows, :] = sloped_terrain(new_sub_terrain(), slope=-0.5).height_field_raw
    heightfield[2*num_rows:3*num_rows, :] = stairs_terrain(new_sub_terrain(), step_width=0.75, step_height=-0.35).height_field_raw
    heightfield[2*num_rows:3*num_rows, :] = heightfield[2*num_rows:3*num_rows, :][::-1]
    heightfield[3*num_rows:4*num_rows, :] = pyramid_stairs_terrain(new_sub_terrain(), step_width=0.75, step_height=-0.5).height_field_raw

    # add the terrain as a triangle mesh
    vertices, triangles = convert_heightfield_to_trimesh(heightfield, horizontal_scale=horizontal_scale, vertical_scale=vertical_scale, slope_threshold=1.5)
    tm_params = gymapi.TriangleMeshParams()
    tm_params.nb_vertices = vertices.shape[0]
    tm_params.nb_triangles = triangles.shape[0]
    tm_params.transform.p.x = -1.
    tm_params.transform.p.y = -terrain_width/2 - 1.
    gym.add_triangle_mesh(sim, vertices.flatten(), triangles.flatten(), tm_params)
