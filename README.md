# RL MPC Locomotion

<img src="images/Isaac.png" width=200>

## Control Blocks

<img src="images/controller_blocks.png" width=700>

## Development Log
- python模仿结构体必须放在__init__()里面, 否则无法实例化
- class可以声明确定类型的值为None成员变量
- `*` 对mat做矩阵乘法, 对ndarray做点乘 
- `@` 矩阵乘法
- `ndarray.dot` 矩阵乘法 
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
- 控制器无延时死循环导致cpu跑满 1.11
- 在ISAAC中控制器频率受到仿真器限制(软实时), 500Hz只消耗20%CPU 1.12
- 改写了legController Commands的更新, 全部换成`np.copyto`
- `ndarray.item()`
- 腿部控制器需要按照`SpineBoard.cpp`改写`legController.command`到12自由度的torque
- bridged legController, stateEstimator and simulator 1.13
- 关节零点待修正, 控制器基本work 1.14
- 10 horizon MPC average solved time: 0.1 s
- MPC solver needed to be check, like result order etc... 1.17
- joint zero pos and conventions !!! 摆动相和站立相永远差一个pi..... 1.18
- 优化了调试参数传递方式, 目前打算重新检查一遍翻译是否有误, 以及用 C++ 编译求解器提高效率 2.4
- `<convex_MPC>`, `<common>`, `<FSM_states>` folders double checked 
- 经过调整左右腿符号和偏置顺序, 给0初始速度, 摆动相正常、雅克比正常, 支撑相异常、反向 2.5
- 以及mini cheetah 模型腿部惯量有问题、不均匀
- work 了???!!! 2.5 [video](images/MPC_trot_first.mkv)
- TODO 用 PyBind11 转译一下solver, 或者测试一下OSQP
- OSQP 求解10ms 总时间12-15ms 大惊喜
- Isaac Gym 升级到 preview 3 2.6
- 仿真步长太长0.01行走有抖动, 不稳定, 太短0.001则很卡, gym渲染时间太长0.04 2.7
- 用CPP重写solver 用pybind11转译 用osqp eigen求解 平均用时1ms 但加上数据转换时间后长达30ms 2.9
- 在谷歌motion imitation中完整剥离了MPC控制器,效果不错,但是是基于pybullet的仿真.重新编译基于c的控制器也很成功 2.16
- 目前谷歌最新的mpc 是 fast and efficient, 编译成功, 但是cc文件做了多线程加速, 需要用自带的 setup.py 编译 2.19
- fast and efficient `cc` 文件编译运行失败的原因应该是 third_party 库和 usr/local/lib 中的版本不一致
- TODO 先不纠结编译问题，用setup编译的so跑移植，测试单独solver移植可行性
- 单独移植yuxiang solver成功，求解总时间0.001 但是mpc仍然有腿软的问题，怀疑是apply force isaac有误 2.19
- mini cheetah trot 完全成功，多机器人mpc成功，抗扰动鲁棒性也不错，调整了body mass 和 inertia 来稳定控制器，目前是100Hz 2.20
- aliengo 似乎朝向错了
- 增加了a1支持, 但是 aliengo和 a1 都在往地上走, 很奇怪 2.20
- TODO 对比两组 a1 的控制器输入输出来找bug 2.26
- debug 完成, aliengo 和 a1 都可以走了, bug 在于宇树 hip、knee 电机正方向和 mit 相反且为 revolute 关节, 对应反向并改为 continuous 关节即可 2.27
- TODO 写个机器人初始化姿态控制即可, 然后把手柄控制器加上, 准备进入 RL 阶段 2.27
- 手柄控制完成, 增加了 pronk, bound, pace 步态和对应的手柄按键逻辑 2.28
- recovery stand 和 passive FSM 写好了, 还需要 debug 一下 2.28
- TODO 调通 FSM 后把transition data 换成 done (已完成)
- mpc stand 有问题, locomotion transition 有问题
- Recovery stand 调试完成 3.1
- mpc stand 需要更换参考轨迹, locomotion transition 到 recovery 的时候自动转移和手动转移冲突了 3.1
- TODO 还差一个地面法向量估计算法设计 

### Roadmap

- [Quadruped](MPC_Controller/common/Quadruped.py),
- [RobotRunner](MPC_Controller/RobotRunner.py) ->
    - [LegController](MPC_Controller/common/LegController.py),
    - [DesiredStateCommand](MPC_Controller/DesiredStateCommand.py),
    - [StateEstimatorContainer](MPC_Controller/StateEstimatorContainer.py),
    - [Parameters](MPC_Controller/Parameters.py),
    - [ControlFSM](MPC_Controller/FSM_states/ControlFSM.py) ->
        - [FSM_State_Locomotion](MPC_Controller/FSM_states/FSM_State_Locomotion.py) ->
            - [ConvexMPCLocomotion](MPC_Controller/convex_MPC/ConvexMPCLocomotion.py) ->
                - [convexMPC_interface](MPC_Controller/convex_MPC/convexMPC_interface.py) ->
                    - [SolverMPC in Python](MPC_Controller/convex_MPC/SolverMPC.py)
                - [mpc_osqp in C](MPC_Controller/convex_MPC/mpc_osqp.cc)
                - [SolverMPC in C++](MPC_Controller/convex_MPC/SolverMPC.cpp)

## User Notes

- [Import URDF model in ISAAC](docs/0-model_import.md)
- [Install MIT Cheetah Software](docs/1-MIT_cheetah_installation.md)
- [Development Log: Cheetah Software in Python](docs/2-development_log.md)
  - [How to imitate a C/C++ STRUCT in Python?](docs/2-development_log.md#how-to-imitate-a-cc-struct-in-python)
  - [How to copy data from list or ndarray into a Python struct?](docs/2-development_log.md#how-to-copy-data-from-list-or-ndarray-into-a-python-struct)
  - [Matrix multiplication and exponential](docs/2-development_log.md#matrix-multiplication-and-exponential)
  - [Quadratic programming with CVXOPT](docs/2-development_log.md#quadratic-programming-with-cvxopt)
- [Upgrade IsaacGym Preview 2 to Preview 3](docs/3-isaac_api_note.md)
- [QP Solver Installation and Testing](docs/6-qp_solver.md)

## Gallery

<img src="images/aliengo_static.png" width=500>
<img src="images/aliengo_train.png" width=500>
<img src="images/aliengo_trot.gif" width=500>
<img src="images/4_cheetah_trot.gif" width=500>
