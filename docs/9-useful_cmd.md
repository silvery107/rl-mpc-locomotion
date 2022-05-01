# Train
python train.py task=Aliengo headless=True

# Test
python train.py task=Aliengo checkpoint=runs/Aliengo/nn/Aliengo.pth test=True num_envs=4

# Open tensorboard on localhost 6006
tensorboard --logdir RL_Environment/runs

# Public localhost to internet
ngrok http 6006

# Show GPU usage and tempereture
nvidia-smi

# Show CPU usage and tempereture
sensors

# Send process to background
CTRL + Z & bg