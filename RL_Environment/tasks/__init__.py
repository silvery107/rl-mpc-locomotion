from .aliengo import Aliengo
from .go1 import Go1
from .a1 import A1Task
from .legged_config_ppo import LeggedCfgPPO

# Mappings from strings to environments
isaacgym_task_map = {
    "Aliengo": Aliengo,
    "Go1": Go1,
    "A1": A1Task,
    "ConfigPPO": LeggedCfgPPO,
}
