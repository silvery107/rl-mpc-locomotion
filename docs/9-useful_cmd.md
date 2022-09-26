# Useful Commands

### Train
```sh
python train.py task=Aliengo headless=True
```

### Test
```sh
python train.py task=Aliengo checkpoint=runs/Aliengo/nn/Aliengo.pth test=True num_envs=4
```

### Open tensorboard on localhost 6006
```sh
tensorboard --logdir RL_Environment/runs
```

### Public localhost to internet
```sh
ngrok http 6006
```

### Show GPU usage and tempereture
```sh
nvidia-smi
```

### Show CPU usage and tempereture
```sh
sensors
```

### Send process to background
```sh
CTRL + Z & bg
```