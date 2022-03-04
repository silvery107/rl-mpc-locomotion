import numpy as np
from isaacgym import gymapi
from MPC_Controller.state_estimate.moving_window_filter import MovingWindowFilter
from MPC_Controller.utils import quat_to_rot, quat_to_rpy, Quaternion, DTYPE

class StateEstimate:
    def __init__(self):
        self.position = np.zeros((3,1), dtype=DTYPE)
        self.vWorld = np.zeros((3,1), dtype=DTYPE)
        self.omegaWorld = np.zeros((3,1), dtype=DTYPE)
        self.orientation = Quaternion(1, 0.0, 0.0, 0.0)

        self.rBody = np.zeros((3,3), dtype=DTYPE)
        self.rpy = np.zeros((3,1), dtype=DTYPE)

        self._ground_normal = np.zeros(3, dtype=DTYPE)

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
    #     self.contactPhase = self._phase

    # def setContactPhase(self, phase:np.ndarray):
    #     self.contactPhase = phase

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

        self.result.rpy = quat_to_rpy(self.result.orientation)
        self.result.rBody = quat_to_rot(self.result.orientation)

        self.result.vBody = self.result.rBody @ self.result.vWorld
        self.result.omegaBody = self.result.rBody @ self.result.omegaWorld

        # np.copyto(self.result.rpy, quat_to_rpy(self.result.orientation))
        # np.copyto(self.result.rBody, quat_to_rot(self.result.orientation))

    def _compute_ground_normal(self, contact_foot_positions):
        """Computes the surface orientation in robot frame based on foot positions.
        Solves a least squares problem, see the following paper for details:
        https://ieeexplore.ieee.org/document/7354099
        """
        contact_foot_positions = np.array(contact_foot_positions)
        normal_vec = np.linalg.lstsq(contact_foot_positions, np.ones(4))[0]
        normal_vec /= np.linalg.norm(normal_vec)
        if normal_vec[2] < 0:
            normal_vec = -normal_vec

        _ground_normal = self._ground_normal_filter.calculate_average(normal_vec)
        _ground_normal /= np.linalg.norm(_ground_normal)
        self.result._ground_normal = _ground_normal