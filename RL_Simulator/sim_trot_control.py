from isaacgym import gymapi
import numpy as np
import math

gym = gymapi.acquire_gym()

# get default set of parameters
sim_params = gymapi.SimParams()

# set common parameters
sim_params.dt = 1 / 60
sim_params.substeps = 2
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

#* load asset
xiaotian = "urdf/Xiaotian-ROS/urdf/xiaotian_description.urdf"
aliengo = "urdf/aliengo_description/xacro/aliengo.urdf"
anymal = "urdf/anymal_c/urdf/anymal.urdf"
asset_root = "/home/silvery/isaacgym/assets"
asset_file = aliengo
asset_options = gymapi.AssetOptions()
asset_options.fix_base_link = False
asset_options.use_mesh_materials = True
asset_options.flip_visual_attachments = False if asset_file==xiaotian else True
asset_options.armature = 0.01   # added to the diagonal elements of inertia tensors
                                # for all of the asset’s rigid bodies/links. 
                                # Could improve simulation stability
print("Loading asset '%s' from '%s'" % (asset_file, asset_root))
asset = gym.load_asset(sim, asset_root, asset_file, asset_options) # or load_asset_urdf

# set up the env grid
num_envs = 4
envs_per_row = 2
env_spacing = 1.0
env_lower = gymapi.Vec3(-env_spacing, -env_spacing, 0.0)
env_upper = gymapi.Vec3(env_spacing, env_spacing, env_spacing)

#* add force sensors
sensor_pose = gymapi.Transform(gymapi.Vec3(0.0, 0.0, 0.0))
foot_ids = [5, 9, 13, 17]

# cache some common handles for later use
envs = []
actor_handles = []
force_sensors = []

# create and populate the environments
for i in range(num_envs):
    env = gym.create_env(sim, env_lower, env_upper, envs_per_row)
    envs.append(env)

    height = 0.45
    pose = gymapi.Transform()
    pose.p = gymapi.Vec3(0.0, 0.0, height)
    # pose.r = gymapi.Quat(-0.707107, 0.0, 0.0, 0.707107) # rotate -90deg about x
    # pose.r = gymapi.Quat.from_axis_angle(gymapi.Vec3(0, 0, 1), -0.5*math.pi)

    actor_handle = gym.create_actor(env, asset, pose, "MyActor", group=i, filter=1)
    for id in foot_ids:
        foot_handle = gym.get_actor_rigid_body_handle(env, actor_handle, id)
        sensor = gym.create_force_sensor(env, foot_handle, sensor_pose)
        force_sensors.append(sensor)

    actor_handles.append(actor_handle)

# add viewer
cam_props = gymapi.CameraProperties()
viewer = gym.create_viewer(sim, cam_props)
# Look at the first env
cam_pos = gymapi.Vec3(1.5, 1, 1)
cam_target = gymapi.Vec3(0, 1, 1)
gym.viewer_camera_look_at(viewer, envs[1], cam_pos, cam_target)


T = 0.2
Ah = 40
Ak = 80
t = 0

leg_const = 26
FL_hip_pos = 0
FL_thigh_pos = 0+leg_const
FL_calf_pos = 0+leg_const
FR_hip_pos = 0
FR_thigh_pos = 0+leg_const
FR_calf_pos = 0+leg_const
RL_hip_pos = 0
RL_thigh_pos = 0+leg_const
RL_calf_pos = 0+leg_const
RR_hip_pos = 0
RR_thigh_pos = 0+leg_const
RR_calf_pos = 0+leg_const

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

    # update the viewer
    gym.step_graphics(sim)
    gym.draw_viewer(viewer, sim, True)

    # trot control
    t = gym.get_sim_time(sim) # /1000 
    # hip -> 0
    # thigh -> hip
    # calf -> knee
    hip_pos = 10
    FL_hip_pos = hip_pos
    FR_hip_pos = -hip_pos
    RL_hip_pos = hip_pos
    RR_hip_pos = -hip_pos
    if(t>1):
        FL_thigh_pos = Ah *np.sin(2 *np.pi / T * t +np.pi / 2) +leg_const
        FL_calf_pos = Ak *np.sin(2 *np.pi / T * t -np.pi)+leg_const
        FR_thigh_pos = Ah *np.sin(2 *np.pi / T * t -np.pi / 2)+leg_const
        FR_calf_pos = Ak *np.sin(2 *np.pi / T * t +np.pi) +leg_const
        RL_thigh_pos =  Ah *np.sin(2 *np.pi / T * t -np.pi / 2)+leg_const
        RL_calf_pos = Ak *np.sin(2 *np.pi / T * t +np.pi) +leg_const
        RR_thigh_pos =  Ah *np.sin(2 *np.pi / T * t +np.pi / 2)+leg_const
        RR_calf_pos = Ak *np.sin(2 *np.pi / T * t -np.pi) +leg_const

    # pos control
    for idx in range(num_envs):
        pos_targets = np.deg2rad(
            np.array([FL_hip_pos, FL_thigh_pos, FL_calf_pos,
                      FR_hip_pos, FR_thigh_pos, FR_calf_pos,
                      RL_hip_pos, RL_thigh_pos, RL_calf_pos,
                      RR_hip_pos, RR_thigh_pos, RR_calf_pos], dtype=np.float32))
        gym.set_actor_dof_position_targets(envs[idx], actor_handles[idx], pos_targets)

    # Wait for dt to elapse in real time.
    # This synchronizes the physics simulation with the rendering rate.
    gym.sync_frame_time(sim)

gym.destroy_viewer(viewer)
gym.destroy_sim(sim)