# Note for Model Import

## Aliengo

### Modifications
1. Replace `package://aliengo_description/` with `../` in *URDF*.

1. Set `flip_visual_attachments` to `True` when loading asset.

2. Duplicate `python/rlgpu/cfg/anymal.yaml` and modify names in `defaultJointAngles` to Aliengo's cooresponding name.

3. Add `Aliengo` to `retrieve_cfg(...)` in `python/rlgpu/utils/config.py`. Note that both `pytorch_ppo_<name>.yaml` and `<name>.yaml` can be modified for training and model performance respectively.

4. Change `args.task` back to `Anymal` in `python/rlgpu/utils/parse_task.py` due to a builtin restriction.

### Static Import

<img src="../images/aliengo_static.png" width=400>

### Parallelized Training

```shell
python python/rlgpu/train.py --task=Aliengo
```

<img src="../images/aliengo_train.png" width=400>