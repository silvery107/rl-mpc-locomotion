# Isaac Gym Environments

## Usage

In `<RL_Environment>` folder

To train a policy:
`python train.py task=Ant`

Press the `v` key to disable viewer updates, and press again to resume.

Use the `esc` key or close the viewer window to stop training early.

To load a trained checkpoint:
`python train.py task=Ant checkpoint=runs/Ant/nn/Ant.pth test=True num_envs=4`

Set `test=False` to continue training.