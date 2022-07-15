# Development Log: Cheetah Software in Python
- [Development Log: Cheetah Software in Python](#development-log-cheetah-software-in-python)
  - [True Dev Logs](#true-dev-logs)
  - [How to imitate a C/C++ STRUCT in Python?](#how-to-imitate-a-cc-struct-in-python)
  - [How to copy data from list or ndarray into a Python struct?](#how-to-copy-data-from-list-or-ndarray-into-a-python-struct)
  - [Matrix multiplication and exponential](#matrix-multiplication-and-exponential)
    - [Product](#product)
    - [Exponential](#exponential)
  - [Quadratic programming with CVXOPT](#quadratic-programming-with-cvxopt)

## True Dev Logs

<details>
  <summary>Dec 3, 2019 -- Jan 7, 2022</summary>
  <ul>
  <li> python模仿结构体必须放在__init__()里面, 否则无法实例化
  <li> class可以声明确定类型的值为None成员变量
  <li> <code>*</code> 对mat做矩阵乘法, 对ndarray做点乘 
  <li> <code>@</code> 矩阵乘法
  <li> <code>ndarray.dot</code> 矩阵乘法 
  <li> <code>np.mutiply()</code> 点乘
  <li> <code>self._quadruped:Quadruped = None</code> 类内类型声明
  <li> Successfully bridge Isaac Gym and MPC Controller on 1.7.2022, it took me 1 month coding blindly.
  <li> 矩阵和列表的等号赋值是不安全的
  </ul>
</details>

<details>
  <summary>Jan 8, 2022 -- Jan 18, 2022</summary>
  <ul>
  <li> solver exp存在指数爆炸 1.8
  <li> <code>np.copyto()</code> numpy ndarray copy
  <li> <code>copy.copy()</code> for shallow copy
  <li> <code>copied_list[:] = original_list</code> shallow copy for list with out a new <code>id</code> 
  <li> <code>scipy.linalg.expm</code> Compute the matrix exponential using Pade approximation.
  <li> 目前CPU跑满了,可能还有不少优化空间 1.9
  <li> 控制器无延时死循环导致cpu跑满 1.11
  <li> 在ISAAC中控制器频率受到仿真器限制(软实时), 500Hz只消耗20%CPU 1.12
  <li> 改写了legController Commands的更新, 全部换成<code>np.copyto</code> 
  <li> <code>ndarray.item()</code> 
  <li> 腿部控制器需要按照<code>SpineBoard.cpp</code> 改写<code>legController.command</code> 到12自由度的torque
  <li> bridged legController, stateEstimator and simulator 1.13
  <li> 关节零点待修正, 控制器基本work 1.14
  <li> 10 horizon MPC average solved time: 0.1 s
  <li> MPC solver needed to be check, like result order etc... 1.17
  <li> joint zero pos and conventions !!! 摆动相和站立相永远差一个pi..... 1.18
  </ul>
</details>

<details>
  <summary>Feb 4, 2022 -- Feb 9, 2022</summary>
  <ul>
  <li> 优化了调试参数传递方式, 目前打算重新检查一遍翻译是否有误, 以及用 C++ 编译求解器提高效率 2.4
  <li> <code>&lt;convex_MPC></code> , <code>&lt;common></code> , <code>&lt;FSM_states></code> folders double checked 
  <li> 经过调整左右腿符号和偏置顺序, 给0初始速度, 摆动相正常、雅克比正常, 支撑相异常、反向 2.5
  <li> 以及mini cheetah 模型腿部惯量有问题、不均匀
  <li> work 了???!!! <a href=images/MPC_trot_first.mkv>video</a> 2.5
  <li> TODO 用 PyBind11 转译一下solver, 或者测试一下OSQP
  <li> OSQP 求解10ms 总时间12-15ms 大惊喜
  <li> Isaac Gym 升级到 preview 3 2.6
  <li> 仿真步长太长0.01行走有抖动, 不稳定, 太短0.001则很卡, gym渲染时间太长0.04 2.7
  <li> 用CPP重写solver 用pybind11转译 用osqp eigen求解 平均用时1ms 但加上数据转换时间后长达30ms 2.9
  </ul>
</details>

<details>
  <summary>Feb 16, 2022 -- Feb 20, 2022</summary>
  <ul>
  <li> 在谷歌 motion imitation 中完整剥离了MPC控制器,效果不错,但是是基于pybullet的仿真.重新编译基于c的控制器也很成功 2.16
  <li> 目前谷歌最新的mpc 是 fast and efficient, 编译成功, 但是cc文件做了多线程加速, 需要用自带的 setup.py 编译 2.19
  <li> fast and efficient <code>cc</code> 文件编译运行失败的原因应该是 third_party 库和 usr/local/lib 中的版本不一致
  <li> TODO 先不纠结编译问题，用 setup 编译的 .so 跑移植，测试单独solver移植可行性
  <li> 单独移植 yuxiang solver 成功，求解总时间 0.001 但是mpc仍然有腿软的问题，怀疑是 apply force isaac 有误 2.19
  <li> mini cheetah trot 完全成功，多机器人mpc成功，抗扰动鲁棒性也不错，调整了body mass 和 inertia 来稳定控制器，目前是100Hz 2.20
  <li> aliengo 似乎朝向错了
  <li> 增加了a1支持, 但是 aliengo和 a1 都在往地上走, 很奇怪 2.20
  </ul>
</details>

<details>
  <summary>Feb 26, 2022 -- Mar 7, 2022</summary>
  <ul>
  <li> TODO 对比两组 a1 的控制器输入输出来找bug 2.26 (已完成)
  <li> debug 完成, aliengo 和 a1 都可以走了, bug 在于宇树 hip、knee 电机正方向和 mit 相反且为 revolute 关节, 对应反向并改为 continuous 关节即可 2.27
  <li> TODO 写个机器人初始化姿态控制即可, 然后把手柄控制器加上 2.27 (已完成)
  <li> 手柄控制完成, 增加了 pronk, bound, pace 步态和对应的手柄按键逻辑 2.28
  <li> recovery stand 和 passive FSM 写好了, 还需要 debug 一下 2.28
  <li> TODO 调通 FSM 后把transition data 换成 done (已完成)
  <li> mpc stand 有问题, locomotion transition 有问题
  <li> Recovery stand 调试完成 3.1
  <li> locomotion transition 到 recovery 的时候自动转移和手动转移冲突了 3.1
  <li> TODO mpc stand 需要更换参考轨迹 (弃用)
  <li> TODO 还差一个地面法向量估计算法 (已完成)
  <li> 全局自动转移通过虚拟按键解决 3.2
  <li> RL train 的时候个体自动转移: 加一个私有域存当前的control mode, 或者在 locomotion unsafe 的时候直接 reset, 固定gait type
  <li> 摔倒以后会乱跑了, com 状态给错了, 给的是 world 状态, 指令全成了朝仿真器坐标朝向 3.2
  <li> TODO 触地检测 用力传感器做, 配合状态变换完成地面法向量估计 (已完成)
  <li> 调节渲染间隔 把力控提到 1k Hz 3.3
  <li> 加了力传感器, 身体系和世界系没有对齐的时候就会乱跑, 趋于一个对齐的参考轨迹, 坐标变换有问题 3.3
  <li> MPC stand 要配合WBC的task才有用 决定放弃 stand 步态 3.4
  <li> 坐标变换修好了, 把所有世界系的指令换成身体系了 3.5
  <li> 地面法向量估计写好了 3.5
  <li> 已完成 uneven terrain 搭建, 目前身体高度估计错误, 同时状态没有变换到和地面法向量对齐的坐标系 3.5
  <li> 修正了坐标变换和身体高度估计, 但是法向量估计有问题, 长时间在斜坡踏步会导致估计反向 3.6
  <li> 修正了位置估计bug, 实现上下斜坡、台阶 3.7
  <li> TODO 设计 RL 算法: step, update, reset, action ... observation, rewards ...
  </ul>
</details>

<details>
  <summary>Mar 11, 2022 -- Mar 21, 2022</summary>
  <ul>
  <li> 添加了isaac gym RL训练环境支持 3.12
  <li> 添加了 Aliengo end to end RL 训练环境 3.14
  <li> 注意 RL 输出的 action 是每个纬度 -1 到 1 的值
  <li> 调整 Aliengo RL 到 60 reward, 并行环境数在512时GPU低于90度, 同时batch size 满载, 比 1024 收敛快 3.14
  <li> TODO 录一下 RL 效果
  <li> 调整到力控, 增加速度和碰撞 penalties, MLP [256, 128, 64] 3.15
  <li> 完全解耦 控制器 和 gymapi, 把和gym交互的逻辑暴露在控制器之外, 交互内容只有 input: dof and rigid body states, output: legtorques 3.15
  <li> MPC 控制器和 RL env 数据初步对接完成, 还差 cmd 和 MPC param 交互 3.15
  <li> 成功剥离网络, 目前可以脱离 RL 环境单独运行 policy 3.16
  <li> 成功对接 MPC cmd, weight交互需要重新设计 3.16
  <li> TODO 修改 MPC weight 更新, 暴露到 conputeContactForce 的位置 (已完成)
  <li> TODO 如何保证 weight 大等于零? 如何缩放 [-1,1] 的 action 到对应 scale？(已完成)
  <li> 修正了地面法向量坐标系至 yaw 对齐, rpy 完全正常, 无需置 yaw 为零 3.17
  <li> 实现 MPC 参数学习 3.18
  <li> 实现 Policy 结合 MPC 运行, 推理速度可以 100 Hz 实时运行 3.18
  <li> TODO 如何权衡 50Hz 的 MPC 和实时的网络更新?
  <li> TODO 训练环境中发 x vel 正向时会无法行走, 坐到地上 3.21 (已完成)
  </ul>
</details>

<details>
  <summary>Apr 8, 2022 -- May 7, 2022</summary>
  <ul>
  <li> 并不是 x vel 正向无法行走, 是因为裁减了输出力矩 4.8
  <li> 实现 random uniform terrain MPC 训练 4.8
  <li> 如何权衡RL reward 和 MPC loss 不一致? RL reward 包括线速度和角速度6维和输出力惩罚, 因为不方便计算,仅不包括位置和角度, 可能是合理的
  <li> 如何权衡 50Hz 的 MPC 和实时的网络更新? 暂时只差两倍, 或许可以接受 4.9
  <li> 实现 MPC Loss 计算 4.9
  <li> TODO 绘制 state 和 torque loss 曲线 (已完成) 4.9
  <li> 注意 Aliengo PD 参数为 (30, 1)
  <li> 将 pybind11, qpoases, eigen3 设置成 submodule, 并通过编译, 通过 windows 编译
  <li> 绘制了速度跟随曲线
  <li> Adapt to flat ground training, modified observations (removed projected gravity, add base pos)
  </ul>
</details>

## How to imitate a C/C++ STRUCT in Python?

For example, we have a `RobotState` struct, and we want to transfer it into a python structure.
```c++
struct RobotState
{
    Vec3<float> pos;
    Vec3<float> vel;
    Vec3<float> omega;
    Mat3<float> R;
    RobotData<float> *data;
};
```

The nature idea is using a `class` in python.
```python
class RobotState:
    pos = np.zeros((3,1), dtype=np.float32)
    vel = np.zeros((3,1), dtype=np.float32)
    omega = np.zeros((3,1), dtype=np.float32)
    R = np.zeros((3,3), dtype=np.float32)
    data = RobotData()
```

In this way, variables in  `RobotState` are all belong to the class, not its instance, which means if we have two instances of `RobotState`, they share the same data. Since we want multiple instances with different data region, we can move atributes of `RobotState` to its `__init__()` function. Problem solved? Not yet.
```python
class RobotState:
    def __init__(self):
        self.pos = np.zeros((3,1), dtype=np.float32)
        self.vel = np.zeros((3,1), dtype=np.float32)
        self.omega = np.zeros((3,1), dtype=np.float32)
        self.R = np.zeros((3,3), dtype=np.float32)
        self.data = RobotData()
```

What about the `data` atribute? It is a pointer in struct but an instance in python class now. We may not want `RobotData()` to be called inside `__init__()` in `RobotState` class, so just declear its type ahead with `:` before `=`.

```python
self.data:RobotData = None
```
Note that this type hints will not initialize `data` but can help IDE auto-complete engine like `data.m -> float`.


## How to copy data from list or ndarray into a Python struct?

Till now we have a `RobotState` class in python looks like a struct, but does it work like a struct? Not yet. As we know, a struct is a user defined data structure and used as a container, but assignment statements in Python do not copy objects, they create bindings between a target and an object. The declearation of member variables like `R = np.zeros((3,3), dtype=np.float32)` will assign them to some initialized data, but how to change their values without replacing their origional data? 

We should use shallow copy or deepcopy operations depending on the situation, and I will take `list` and `np.ndarray` as examples. 

In a non-nested **list**, which means it contains no other objects except numbers can be completely copyed using `copy.copy()`.

```python
original_list = [1, 2, 3, 4]
copied_list = copy.copy(original_list)
```

If a list contains other objects (include other lists) like `[1, 2, [3, 4]]`, 
 `copy.deepcopy()` is needed for deep copying. `copy.deepcopy()` calls `copy.copy()` recursively inside the origional.

```python
original_list = [1, 2, [3, 4]]
copied_list = copy.deepcopy(original_list)
```

In a non-nested **ndarray**, I recommand using `np.copyto()` (numpy >=1.7.1) function for data copying.

```python
np.copyto(dst, src, casting='same_kind')
```

Options for `casting` are `{‘no’, ‘equiv’, ‘safe’, ‘same_kind’, ‘unsafe’}`

- `no` means the data types should not be cast at all.

- `equiv` means only byte-order changes are allowed.

- `safe` means only casts which can preserve values are allowed.

- `same_kind` means only safe casts or casts within a kind, like float64 to float32, are allowed.

- `unsafe` means any data conversions may be done.

## Matrix multiplication and exponential

### Product

|Method|Description|
|-|-|
|`*`|matrix multiplication for `np.mat`; dot product for `np.ndarray`|
|`@`, `np.matmul`| matrix multiplication without scalars|
|`ndarray.dot`, `np.dot`| matrix multiplication|
|`np.multiply`| dot product only|

### Exponential
The matrix exponential of M is defined by
$$
exp(M) = \sum^{\infty}_{k=0}\frac{M^k}{k!}
$$

`scipy.linalg.expm(M)`

Compute the matrix exponential using Pade approximation.

## Quadratic programming with CVXOPT

Convex optimization problems of the form
$$
min \quad \frac{1}{2}x^TPx + q^Tx\\
s.t. \qquad Gx\leq h\\
$$


`cvxopt.solvers.qp(P, Q, G, h, solver="mosek")`


The function `qp` solves the pair of primal and dual convex quadratic programs, which also provides the option of using solver from MOSEK.
