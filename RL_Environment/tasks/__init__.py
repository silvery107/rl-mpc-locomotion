from tasks.anymal import Anymal
from tasks.anymal_terrain import AnymalTerrain
from tasks.aliengo import Aliengo
from tasks.go1 import Go1

# Mappings from strings to environments
isaacgym_task_map = {
    "Anymal": Anymal,
    "AnymalTerrain": AnymalTerrain,
    "Aliengo":Aliengo,
    "Go1":Go1,
}