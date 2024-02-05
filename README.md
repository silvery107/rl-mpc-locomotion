# RL MPC Locomotion
This repo aims to provide a fast simulation and RL training framework for a quadruped locomotion task by dynamically predicting the weight parameters of a MPC controller. The control framework is a hierarchical controller composed of a higher-level policy network and a lower-level model predictive controller. 

The MPC controller refers to [Cheetah Software](https://github.com/mit-biomimetics/Cheetah-Software) but written in python, and it completely opens the interface between sensor data and motor commands, so that the controller can be easily ported to any mainstream simulators.

The RL training utilizes the [NVIDIA Isaac Gym](https://developer.nvidia.com/isaac-gym) in parallel using Unitree Robotics Aliengo model, and transferring it from simulation to reality on a [real Aliengo robot](#sim2real_anchor) (sim2real is not included in this codebase).


## Frameworks

<img src="images/controller_blocks.png" width=700>

## Dependencies
- *Python* - 3.8
- [*PyTorch* - 1.10.0 with CUDA 11.3](https://pytorch.org/get-started/previous-versions/)
- [*Isaac Gym* - Preview 4](https://developer.nvidia.com/isaac-gym)
<!-- - *OSQP* - 0.6.2 -->

## Installation
1. Clone this repository
    ```bash
    git clone git@github.com:silvery107/rl-mpc-locomotion.git
    git submodule update --init
    ```
    Or use the `--recurse` option to clone submodules at the same time.

3. Create the conda environment:
    ```bash
    conda env create -f environment.yml
    ```

2. Install the python binding of the MPC solver:
    ```bash
    pip install -e .
    ```

## Quick Start
1. Play the MPC controller on Aliengo:
    ```bash
    python RL_MPC_Locomotion.py --robot=Aliengo
    ```
    All supported robot types are `Go1`, `A1` and `Aliengo`.

    Note that you need to plug in your Xbox-like gamepad to control it, or pass `--disable-gamepad`.
    The controller mode is default to `Fsm` (Finite State Machine), and you can also try `Min` for the minimum MPC controller without FSM.

    - Gamepad keymap
        > Press `LB` to switch gait types between `Trot`, `Walk` and `Bound`.

        > Press `RB` to switch FSM states between `Locomotion` and `Recovery Stand`

2. Train a new policy:
    Set `bridge_MPC_to_RL` to `True` in `<MPC_Controller/Parameters.py>`
    ```bash
    cd RL_Environment
    python train.py task=Aliengo headless=False
    ```
    Press the `v` key to disable viewer updates, and press again to resume. 
    Set `headless=True` to train without rendering.

    Tensorboard support is available, run `tensorboard --logdir runs`.

3. Load a pretrained checkpoint:
    ```bash
    python train.py task=Aliengo checkpoint=runs/Aliengo/nn/Aliengo.pth test=True num_envs=4
    ```
    Set `test=False` to continue training.

4. Run the pretrained weight-policy for MPC controller on Aliengo:
   Set `bridge_MPC_to_RL` to `False` in `<MPC_Controller/Parameters.py>`
    ```bash
    python RL_MPC_Locomotion.py --robot=Aliengo --mode=Policy --checkpoint=path/to/ckpt
    ```
    If no `checkpoint` is given, it will load the default `runs/{robot}/nn/{robot}.pth`


## Roadmaps

<img src="images/MPC_block.png" width=600>

- [x] **MPC Controller**
- [Quadruped](MPC_Controller/common/Quadruped.py),
- [RobotRunner](MPC_Controller/robot_runner/RobotRunnerFSM.py) ->
    - [LegController](MPC_Controller/common/LegController.py),
    - [StateEstimator](MPC_Controller/common/StateEstimator.py),
    - [ControlFSM](MPC_Controller/FSM_states/ControlFSM.py) ->
        - [FSM State RecoveryStand](MPC_Controller/FSM_states/FSM_State_RecoveryStand.py),
        - [FSM State Locomotion](MPC_Controller/FSM_states/FSM_State_Locomotion.py) ->
            - [ConvexMPCLocomotion](MPC_Controller/convex_MPC/ConvexMPCLocomotion.py) ->
                - [FootSwingTrajectory](MPC_Controller/common/FootSwingTrajectory.py),
                - [Gait](MPC_Controller/convex_MPC/Gait.py),
                - [MPC Solver in C](MPC_Controller/convex_MPC/mpc_osqp.cc)

<img src="images/training_data_flow.png" width=400>

- [x] **RL Environment**
- [Gamepad Reader](RL_Environment/gamepad_reader.py),
- [Simulation Utils](RL_Environment/sim_utils.py),
- [Weight Policy](RL_Environment/WeightPolicy.py),
- [Train](RL_Environment/train.py) ->
    - [Vectorized Env](RL_Environment/tasks/base/vec_task.py),
    - [Aliengo Env](RL_Environment/tasks/aliengo.py)

## User Notes

- [Setup a Simulation in Isaac Gym](docs/3-isaac_api_note.md)
- [Install MIT Cheetah Software](docs/1-MIT_cheetah_installation.md)
- [OSQP, qpOASES and CVXOPT Solver Instructions](docs/6-qp_solver.md)
- [Development Logs](docs/2-development_log.md)

## Gallery

<img src="images/4_cheetah_trot.gif" width=500>
<img src="images/RL_Paraller_16.gif" width=500>
<img src="images/MPC_Stair_Demo.gif" width=500>
<img src="images/MPC_Sim2Real.gif" width=500 tag>
<a name="sim2real_anchor"></a>
