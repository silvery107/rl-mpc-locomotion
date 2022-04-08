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
from MPC_Controller.StateEstimator import StateEstimate

from RL_Environment.utils.reformat import omegaconf_to_dict, print_dict

from rl_games.algos_torch.model_builder import ModelBuilder
from rl_games.algos_torch.running_mean_std import RunningMeanStd
from rl_games.algos_torch import torch_ext

## OmegaConf & Hydra Config
OmegaConf.register_new_resolver('eq', lambda x, y: x.lower()==y.lower())
OmegaConf.register_new_resolver('contains', lambda x, y: x.lower() in y.lower())
OmegaConf.register_new_resolver('if', lambda pred, a, b: a if pred else b)
OmegaConf.register_new_resolver('resolve_default', lambda default, arg: default if arg=='' else arg)

class WeightPolicy:
    def __init__(self, 
                 task="Aliengo", 
                 checkpoint="runs/Aliengo/nn/Aliengo.pth",
                 num_envs=1):

        self.num_actions = 12
        self.num_obs = 48
        self.device = "cuda" 
        self.is_determenistic = True

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

        # ensure checkpoints can be specified as relative paths
        if cfg.checkpoint:
            cfg.checkpoint = to_absolute_path(cfg.checkpoint)

        rlg_config_dict = omegaconf_to_dict(cfg.train)
        
        # prepare config and params dict
        params = rlg_config_dict['params']
        config = params['config']
        model_builder = ModelBuilder()
        config['network'] = model_builder.load(params)
    
        print('Found checkpoint:')
        print(params['load_path'])
        load_path = params['load_path']
        num_agents = config['num_actors']

        obs_shape = (self.num_obs,)

        # use model directly
        checkpoint = torch_ext.load_checkpoint(load_path)
        # load model
        self.model = config['network'].build({
                'actions_num' : self.num_actions,
                'input_shape' : obs_shape,
                'num_seqs' : num_agents
            })
        self.model.to(self.device)
        self.model.eval()
        self.model.load_state_dict(checkpoint['model'])
        # load obs normalizer
        self.running_mean_std = RunningMeanStd(obs_shape).to(self.device)
        self.running_mean_std.eval()
        self.running_mean_std.load_state_dict(checkpoint['running_mean_std'])

        self.num_agents = num_agents
        self.obs = torch.ones([self.num_agents, self.num_obs], requires_grad=False, dtype=torch.float, device=self.device)

    def step(self):
        obs = self._preproc_obs(self.obs)
        # get action
        input_dict = {
            'is_train': False,
            'prev_actions': None, 
            'obs' : obs,
            'rnn_states' : None
        }

        t_start = time.time()
        with torch.no_grad():
            res_dict = self.model(input_dict)
        if Parameters.policy_print_time:
            print("Model Inference Time: {:.5f}".format(time.time()-t_start))

        if self.is_determenistic:
            # determenistic action
            action = res_dict['mus']
        else:
            # non-determenistic action
            action = res_dict['actions']

        # clip actions to (-1, 1)
        action_clip = self._rescale_actions(-torch.ones_like(action, requires_grad=False, device=self.device), 
                                        torch.ones_like(action, requires_grad=False, device=self.device), 
                                        torch.clamp(action, -1.0, 1.0))
        # * [-1, 1] -> [a, b] => [-1, 1] * (b-a)/2 + (b+a)/2
        actions_rescale = torch.mul(action_clip, 
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
        commands = _commands * np.array([self.lin_vel_scale, self.lin_vel_scale, self.ang_vel_scale], dtype=DTYPE)
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
                
        # normalize obs
        obs_batch = self.running_mean_std(obs_batch)
        return obs_batch

    def _rescale_actions(self, low, high, action):
        d = (high - low) / 2.0
        m = (high + low) / 2.0
        scaled_action =  action * d + m
        return scaled_action