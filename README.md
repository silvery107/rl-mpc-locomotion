# RL MPC Locomotion

<img src="images/Isaac.png" width=300>

## Control Blocks

<img src="images/controller_blocks.png" width=700>

## Development Log
- python模仿结构体必须放在__init__()里面，否则无法实例化
- class可以声明确定类型的值为None成员变量
- `*` 对mat做矩阵乘法, 对ndarray做点乘 
- `@` 矩阵乘法
- `.dot` 矩阵乘法 
- `np.mutiply()` 点乘
- `self._quadruped:Quadruped = None` 类内类型声明
- Successfully bridge Isaac Gym and MPC Controller on 1.7.2022, it took me 1 month coding blindly.
- 矩阵和列表的等号赋值是不安全的
- solver exp存在指数爆炸 1.8
- `np.copyto()` numpy ndarray copy
- `copy.copy()` for shallow copy
- `copied_list[:] = original_list` shallow copy for list with out a new `id`
- `scipy.linalg.expm` Compute the matrix exponential using Pade approximation.
- 目前CPU跑满了,可能还有不少优化空间 1.9

### Architecture
```c
main() ->
    Quadruped,
    RobotRunner ->
        LegController,
        DesiredStateCommand,
        StateEstimatorContainer,
        MIT_UserParameters,
        MIT_Controller ->
            ControlFSM ->
                FSM_State_Locomotion ->
                    ConvexMPCLocomotion ->
                        convexMPC_interface ->
                            SolverMPC
```

### Partially implemented:
- SimulationBridge
- RobotRunner
- StateEstimatorContainer
- DesiredStateCommand
- LegController
- ControlFSM
- FSM_State_Locomotion

### Fully implemented:
- ConvexMPCLocomotion
- convexMPC_interface
- SolverMPC

## User Notes

- [Import URDF model in ISAAC](docs/0-model_import.md)
- [Install MIT Cheetah Software](docs/1-MIT_cheetah_installation.md)
- [Development Log](docs/2-development_log.md)

## Gallery

<img src="images/aliengo_static.png" width=500>
<img src="images/aliengo_train.png" width=500>
