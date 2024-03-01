from .aliengo import Aliengo
from .go1 import Go1
from .legged_config_ppo import LeggedCfgPPO

# Mappings from strings to environments
isaacgym_task_map = {
    "Aliengo": Aliengo,
    "Go1": Go1,
    "ConfigPPO": LeggedCfgPPO,
}
