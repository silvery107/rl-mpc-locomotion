from unittest import result
import numpy as np
from isaacgym import gymapi
from MPC_Controller.state_estimate.moving_window_filter import MovingWindowFilter
from MPC_Controller.utils import quat_to_rot, quat_to_rpy, Quaternion, DTYPE, rot_to_rpy, rpy_to_rot

class StateEstimate:
    def __init__(self):
        self.position = np.zeros((3,1), dtype=DTYPE)
        self.vWorld = np.zeros((3,1), dtype=DTYPE)
        self.omegaWorld = np.zeros((3,1), dtype=DTYPE)
        self.orientation = Quaternion(1, 0, 0, 0)

        self.rBody = np.zeros((3,3), dtype=DTYPE)
        self.rpy = np.zeros((3,1), dtype=DTYPE)
        self.rpyBody = np.zeros((3,1), dtype=DTYPE)

        self.ground_normal = np.zeros(3, dtype=DTYPE)

        self.vBody = np.zeros((3,1), dtype=DTYPE)
        self.omegaBody = np.zeros((3,1), dtype=DTYPE)

        # self.aBody = np.zeros((3,1), dtype=DTYPE)
        # self.aWorld = np.zeros((3,1), dtype=DTYPE)
        # self.contactEstimate = np.zeros((4,1), dtype=DTYPE)


class StateEstimatorContainer:

    def __init__(self):
        self.result = StateEstimate()
        self._phase = np.zeros((4,1), dtype=DTYPE)
        self._ground_normal_filter = MovingWindowFilter(window_size=10)
        self._contactPhase = self._phase
        self._foot_contact_history:np.ndarray = None

    def setContactPhase(self, phase:np.ndarray):
        self._contactPhase = phase

    def getResult(self):
        return self.result

    def update(self, gym, env, actor, body_name):
        body_idx = gym.find_actor_rigid_body_index(env, actor, body_name, gymapi.DOMAIN_ACTOR)
        body_states = gym.get_actor_rigid_body_states(env, actor, gymapi.STATE_ALL)[body_idx]

        for idx in range(3):
            self.result.position[idx] = body_states["pose"]["p"][idx] # positions (Vec3: x, y, z)
            self.result.vWorld[idx] = body_states["vel"]["linear"][idx] # linear velocities (Vec3: x, y, z)
            self.result.omegaWorld[idx] = body_states["vel"]["angular"][idx] # angular velocities (Vec3: x, y, z)

        self.result.orientation.w = body_states["pose"]["r"]["w"] # orientations (Quat: x, y, z, w)
        self.result.orientation.x = body_states["pose"]["r"]["x"]
        self.result.orientation.y = body_states["pose"]["r"]["y"]
        self.result.orientation.z = body_states["pose"]["r"]["z"]

        # all good here
        self.result.rBody = quat_to_rot(self.result.orientation)
        self.result.vBody = self.result.rBody @ self.result.vWorld
        self.result.omegaBody = self.result.rBody @ self.result.omegaWorld

        #  RPY of body in the world frame
        self.result.rpy = quat_to_rpy(self.result.orientation)

        base_R_world = rpy_to_rot([0,0,self.result.rpy[2]])
        body_R_base = self.result.rBody @ base_R_world.T

        # RPY of body in yaw aligned base frame
        self.result.rpyBody = rot_to_rpy(body_R_base)

        # change position to base frame
        self.result.position = np.array([0, 0, self.result.position[2]], dtype=DTYPE).reshape((3,1))

    def _compute_ground_normal(self, foot_positions:np.ndarray):
        """
        Computes the surface orientation in robot frame based on foot positions.
        Solves a least squares problem, see the following paper for details:
        https://ieeexplore.ieee.org/document/7354099
        """
        self._update_contact_history(foot_positions)
        contact_foot_positions = self._foot_contact_history.reshape((4,3)) # reshape from (4,3,1) to (4,3)
        normal_vec = np.linalg.lstsq(contact_foot_positions, np.ones(4))[0]
        normal_vec /= np.linalg.norm(normal_vec)
        if normal_vec[2] < 0:
            normal_vec = -normal_vec

        _ground_normal = self._ground_normal_filter.calculate_average(normal_vec)
        _ground_normal /= np.linalg.norm(_ground_normal)
        self.result.ground_normal = _ground_normal

    def _init_contact_history(self, foot_positions:np.ndarray, height:float):
        self._foot_contact_history = foot_positions.copy()
        self._foot_contact_history[:, 2] = - height

    def _update_contact_history(self, foot_positions:np.ndarray):
        foot_contacts = self._contactPhase.flatten().copy()
        foot_positions_ = foot_positions.copy()
        for leg_id in range(4):
            if foot_contacts[leg_id]:
                self._foot_contact_history[leg_id] = foot_positions_[leg_id]
