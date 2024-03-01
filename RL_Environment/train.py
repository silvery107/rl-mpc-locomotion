import isaacgym
from datetime import datetime

from extern.rsl_rl.rsl_rl.runners import OnPolicyRunner

import os
import hydra
from omegaconf import DictConfig, OmegaConf
from hydra.utils import to_absolute_path

from utils.reformat import omegaconf_to_dict, print_dict
from utils.utils import set_np_formatting, set_seed
from utils.rsl_rl_utils import update_cfg_from_args, class_to_dict, get_load_path

from tasks import isaacgym_task_map

ROOT_DIR = os.path.dirname(os.path.realpath(__file__)) # Under <RL_Environment>

## OmegaConf & Hydra Config

# Resolvers used in hydra configs
OmegaConf.register_new_resolver('eq', lambda x, y: x.lower()==y.lower())
OmegaConf.register_new_resolver('contains', lambda x, y: x.lower() in y.lower())
OmegaConf.register_new_resolver('if', lambda pred, a, b: a if pred else b)
# allows us to resolve default arguments which are copied in multiple places in the config. 
# used primarily for num_ensv
OmegaConf.register_new_resolver('resolve_default', lambda default, arg: default if arg=='' else arg)

@hydra.main(config_name="config", config_path="./cfg")
def launch_hydra(cfg: DictConfig):

    # ensure checkpoints can be specified as relative paths
    if cfg.checkpoint:
        cfg.checkpoint = to_absolute_path(cfg.checkpoint)

    cfg_dict = omegaconf_to_dict(cfg)
    print_dict(cfg_dict)

    # set numpy formatting for printing only
    set_np_formatting()

    # sets seed. if seed is -1 will pick a random one
    cfg.seed = set_seed(cfg.seed, torch_deterministic=cfg.torch_deterministic)

    # create native task and pass custom config
    env = isaacgym_task_map[cfg.task_name](
        cfg=omegaconf_to_dict(cfg.task),
        sim_device=cfg.sim_device,
        graphics_device_id=cfg.graphics_device_id,
        headless=cfg.headless
    )

    train_cfg = isaacgym_task_map["ConfigPPO"]
    train_cfg = update_cfg_from_args(train_cfg, cfg)

    log_root = os.path.join(ROOT_DIR, 'runs', cfg.task_name)
    log_dir = os.path.join(log_root, datetime.now().strftime('%b%d_%H-%M-%S'))

    train_cfg_dict = class_to_dict(train_cfg)
    ppo_runner = OnPolicyRunner(env, train_cfg_dict, log_dir, cfg.rl_device)

    if cfg.test or cfg.checkpoint:
        # load previously trained model
        try:
            print(f"Loading model from: {cfg.checkpoint}")
            ppo_runner.load(cfg.checkpoint)
        except:
            print("Failed...")
            resume_path = get_load_path(log_root)
            print(f"Loading model from the latest run: {resume_path}")
            ppo_runner.load(resume_path)

    # dump config dict
    experiment_dir = log_dir
    os.makedirs(experiment_dir, exist_ok=True)
    with open(os.path.join(experiment_dir, 'config.yaml'), 'w') as f:
        f.write(OmegaConf.to_yaml(cfg))

    if not cfg.test:
        ppo_runner.learn(num_learning_iterations=train_cfg.runner.max_iterations, init_at_random_ep_len=False)
    else:
        policy = ppo_runner.get_inference_policy(device=env.device)
        obs = env.get_observations()

        for i in range(10*int(env.max_episode_length)):
            actions = policy(obs.detach())
            obs, _, rews, dones, infos = env.step(actions.detach())


if __name__ == '__main__':
    launch_hydra()
