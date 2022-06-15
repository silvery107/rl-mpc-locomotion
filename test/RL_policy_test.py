import os
import inspect

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
os.sys.path.insert(0, parentdir)
import isaacgym

from time import time
import hydra
import numpy as np
from omegaconf import DictConfig, OmegaConf
from hydra.utils import to_absolute_path

import torch

from MPC_Controller.Parameters import Parameters
from RL_Environment.utils.reformat import omegaconf_to_dict, print_dict
from RL_Environment.utils.utils import set_np_formatting

from rl_games.algos_torch.model_builder import ModelBuilder
from rl_games.algos_torch.running_mean_std import RunningMeanStd
from rl_games.algos_torch import torch_ext

## OmegaConf & Hydra Config
# Resolvers used in hydra configs (see https://omegaconf.readthedocs.io/en/2.1_branch/usage.html#resolvers)
OmegaConf.register_new_resolver('eq', lambda x, y: x.lower()==y.lower())
OmegaConf.register_new_resolver('contains', lambda x, y: x.lower() in y.lower())
OmegaConf.register_new_resolver('if', lambda pred, a, b: a if pred else b)
# allows us to resolve default arguments which are copied in multiple places in the config. used primarily for
# num_ensv
OmegaConf.register_new_resolver('resolve_default', lambda default, arg: default if arg=='' else arg)

@hydra.main(config_name="config", config_path="../RL_Environment/cfg")
def launch_rlg_hydra(cfg: DictConfig):

    def rescale_actions(low, high, action):
        d = (high - low) / 2.0
        m = (high + low) / 2.0
        scaled_action =  action * d + m
        return scaled_action

    def _preproc_obs(obs_batch):
        if type(obs_batch) is dict:
            for k, v in obs_batch.items():
                obs_batch[k] = _preproc_obs(v)
        else:
            if obs_batch.dtype == torch.uint8:
                obs_batch = obs_batch.float() / 255.0
                
        # normalize obs
        obs_batch = running_mean_std(obs_batch)
        return obs_batch

    # ensure checkpoints can be specified as relative paths
    # if cfg.checkpoint:
    #     cfg.checkpoint = to_absolute_path(cfg.checkpoint)
    cfg.num_envs = 4
    cfg.checkpoint = "RL_Environment/runs/Aliengo/nn/Aliengo.pth"
    # set numpy formatting for printing only
    set_np_formatting()
    
    cfg_dict = omegaconf_to_dict(cfg)
    print_dict(cfg_dict)
    
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
    num_actions = 12
    num_obs = 48
    obs_shape = (num_obs,)
    device = "cuda" 
    is_determenistic = True

    # use model directly
    checkpoint = torch_ext.load_checkpoint(load_path)
    # load model
    model = config['network'].build({
            'actions_num' : num_actions,
            'input_shape' : obs_shape,
            'num_seqs' : num_agents
        } )
    model.to(device)
    model.eval()
    model.load_state_dict(checkpoint['model'])
    # load obs normalizer
    running_mean_std = RunningMeanStd(obs_shape).to(device)
    running_mean_std.eval()
    running_mean_std.load_state_dict(checkpoint['running_mean_std'])

    obs = torch.ones([num_agents, num_obs], requires_grad=False, device=device)
    obs = _preproc_obs(obs)

    # get action
    input_dict = {
        'is_train': False,
        'prev_actions': None, 
        'obs' : obs,
        'rnn_states' : None
    }
    t_start = time()
    torch.cuda.synchronize()
    with torch.no_grad():
        res_dict = model(input_dict)
    if is_determenistic:
        # determenistic action
        action = res_dict['mus']
    else:
        # non-determenistic action
        action = res_dict['actions']
    print(time()-t_start)
    # clip actions to (-1, 1)
    action_clip = rescale_actions(-torch.ones_like(action, requires_grad=False, device=device), 
                                    torch.ones_like(action, requires_grad=False, device=device), 
                                    torch.clamp(action, -1.0, 1.0))
    # * [-1, 1] -> [a, b] => [-1, 1] * (b-a)/2 + (b+a)/2
    actions_rescale = torch.mul(action_clip, 
                                torch.tensor(
                                Parameters.MPC_param_scale,
                                dtype=torch.float,
                                device=device)).add(
                                torch.tensor(
                                Parameters.MPC_param_const,
                                dtype=torch.float,
                                device=device))
    print(actions_rescale.cpu().numpy())

if __name__ == "__main__":
    launch_rlg_hydra()