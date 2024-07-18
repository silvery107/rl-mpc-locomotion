import isaacgym
import os
import inspect

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
os.sys.path.insert(0, parentdir)

import torch
import time
import numpy as np
from omegaconf import OmegaConf
from hydra import compose, initialize
from hydra.utils import to_absolute_path
from MPC_Controller.Parameters import Parameters
from MPC_Controller.utils import DTYPE
from MPC_Controller.common.StateEstimator import StateEstimate

from RL_Environment.utils.utils import set_seed
from RL_Environment.utils.rsl_rl_utils import update_cfg_from_args, class_to_dict, get_load_path
from RL_Environment.tasks.legged_config_ppo import LeggedCfgPPO

from rsl_rl.modules import ActorCritic

## OmegaConf & Hydra Config
OmegaConf.register_new_resolver('eq', lambda x, y: x.lower()==y.lower())
OmegaConf.register_new_resolver('contains', lambda x, y: x.lower() in y.lower())
OmegaConf.register_new_resolver('if', lambda pred, a, b: a if pred else b)
OmegaConf.register_new_resolver('resolve_default', lambda default, arg: default if arg=='' else arg)

ROOT_DIR = os.path.dirname(os.path.realpath(__file__)) # Under <RL_Environment>

class WeightPolicy:
    def __init__(self, 
                 task="Aliengo", 
                 checkpoint="runs/Aliengo/nn/Aliengo.pth",
                 num_envs=1):

        self.num_actions = 12
        self.num_obs = 48
        self.device = "cuda" 
        self.is_determenistic = True
        self.clip_actions = True

        # hydra global initialization
        initialize(config_path="./cfg")
        cfg = compose(config_name="config", 
                      overrides=["checkpoint="+checkpoint, 
                                 "task="+task, 
                                 "num_envs="+str(num_envs)])

        self.lin_vel_scale = cfg["task"]["env"]["learn"]["linearVelocityScale"]
        self.ang_vel_scale = cfg["task"]["env"]["learn"]["angularVelocityScale"]
        self.dof_pos_scale = cfg["task"]["env"]["learn"]["dofPositionScale"]
        self.dof_vel_scale = cfg["task"]["env"]["learn"]["dofVelocityScale"]
        # cfg_dict = omegaconf_to_dict(cfg)
        # print_dict(cfg_dict)

        cfg.seed = set_seed(cfg.seed, torch_deterministic=cfg.torch_deterministic)
        # ensure checkpoints can be specified as relative paths
        if cfg.checkpoint:
            cfg.checkpoint = to_absolute_path(cfg.checkpoint)

        train_cfg = LeggedCfgPPO()
        train_cfg = update_cfg_from_args(train_cfg, cfg)
        train_cfg_dict = class_to_dict(train_cfg)
        policy_cfg = train_cfg_dict["policy"]
        self.actor_critic = ActorCritic(self.num_obs,
                                        self.num_obs,
                                        self.num_actions,
                                        **policy_cfg).to(self.device)

        # load checkpoint
        try:
            print(f"Loading model from: {cfg.checkpoint}")
            loaded_dict = torch.load(checkpoint)
            self.actor_critic.load_state_dict(loaded_dict['model_state_dict'])
        except:
            print("Failed...")
            log_root = os.path.join(ROOT_DIR, 'runs', cfg.task_name)
            fallback_path = get_load_path(log_root)
            print(f"Loading model from the latest run: {fallback_path}")
            loaded_dict = torch.load(fallback_path)
            self.actor_critic.load_state_dict(loaded_dict['model_state_dict'])

        self.actor_critic.eval()
        self.actor_critic.to(self.device)
        self.policy = self.actor_critic.act_inference

        self.num_agents = 1
        self.obs = torch.ones([self.num_agents, self.num_obs], 
                              requires_grad=False, dtype=torch.float, device=self.device)

    def step(self):
        obs = self._preproc_obs(self.obs)
        # get action

        t_start = time.time()
        with torch.no_grad():
            current_action = self.policy(obs.detach())
        if Parameters.policy_print_time:
            print("Model Inference Time: {:.5f}".format(time.time()-t_start))

        # clip actions to (-1, 1)
        if self.clip_actions:
            current_action = self._rescale_actions(
                -torch.ones_like(current_action, requires_grad=False, device=self.device), 
                torch.ones_like(current_action, requires_grad=False, device=self.device), 
                torch.clamp(current_action, -1.0, 1.0))

        # * [-1, 1] -> [a, b] => [-1, 1] * (b-a)/2 + (b+a)/2
        actions_rescale = torch.mul(current_action, 
                                    torch.tensor(
                                    Parameters.MPC_param_scale,
                                    dtype=torch.float,
                                    device=self.device)).add(
                                    torch.tensor(
                                    Parameters.MPC_param_const,
                                    dtype=torch.float,
                                    device=self.device))

        # weights = torch.nn.functional.pad(actions_rescale, (0, 1),mode="constant", value=0)
        return actions_rescale.detach().cpu().numpy()[0] # shape (12,)

    def compute_observations(self, dof_states, se_result:StateEstimate, _commands, _actions):
        base_lin_vel = se_result.vBody.flatten() * self.lin_vel_scale
        base_ang_vel = se_result.omegaBody.flatten() * self.ang_vel_scale

        # TODO check gravity direction
        projected_gravity = - se_result.ground_normal_yaw
        commands = _commands * np.array([self.lin_vel_scale, 
                                         self.lin_vel_scale, 
                                         self.ang_vel_scale], 
                                         dtype=DTYPE)
        dof_pos = dof_states["pos"] * self.dof_pos_scale
        dof_vel = dof_states["vel"] * self.dof_vel_scale
        observations = np.concatenate((base_lin_vel, 
                                       base_ang_vel, 
                                       projected_gravity, 
                                       commands, 
                                       dof_pos, 
                                       dof_vel, 
                                       _actions))
        obs_pad = np.expand_dims(observations, axis=0)
        self.obs = torch.from_numpy(obs_pad.astype(np.float32)).to(self.device)

    def _preproc_obs(self, obs_batch):
        if type(obs_batch) is dict:
            for k, v in obs_batch.items():
                obs_batch[k] = self._preproc_obs(v)
        else:
            if obs_batch.dtype == torch.uint8:
                obs_batch = obs_batch.float() / 255.0

        return obs_batch

    def _rescale_actions(self, low, high, action):
        d = (high - low) / 2.0
        m = (high + low) / 2.0
        scaled_action =  action * d + m
        return scaled_action
