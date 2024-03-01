import os
from isaacgym import gymutil


def update_cfg_from_args(cfg_train, args):
    if cfg_train is None:
        return None
    
    if args.seed:
        cfg_train.seed = int(args.seed)
    # alg runner parameters
    if args.max_iterations:
        cfg_train.runner.max_iterations = int(args.max_iterations)
    if args.task_name:
        cfg_train.runner.experiment_name = args.task_name
    # if args.load_run is not None:
    #     cfg_train.runner.load_run = int(args.load_run)
    # if args.checkpoint is not None:
    #     cfg_train.runner.checkpoint = int(args.checkpoint)

    return cfg_train

def class_to_dict(obj) -> dict:
    if not  hasattr(obj,"__dict__"):
        return obj
    result = {}
    for key in dir(obj):
        if key.startswith("_"):
            continue
        element = []
        val = getattr(obj, key)
        if isinstance(val, list):
            for item in val:
                element.append(class_to_dict(item))
        else:
            element = class_to_dict(val)
        result[key] = element
    return result

def update_class_from_dict(obj, dict):
    for key, val in dict.items():
        attr = getattr(obj, key, None)
        if isinstance(attr, type):
            update_class_from_dict(attr, val)
        else:
            setattr(obj, key, val)
    return

def get_load_path(root, load_run=-1, checkpoint=-1):
    try:
        runs = os.listdir(root)
        #TODO sort by date to handle change of month
        runs.sort()
        if 'exported' in runs: runs.remove('exported')
        if 'summaries' in runs: runs.remove('summaries')
        if 'nn' in runs: runs.remove('nn')
        if 'config.yaml' in runs: runs.remove('config.yaml')
        last_run = os.path.join(root, runs[-1])
    except:
        raise ValueError("No runs in this directory: " + root)
    if load_run==-1:
        load_run = last_run
    else:
        load_run = os.path.join(root, load_run)

    if checkpoint==-1:
        models = [file for file in os.listdir(load_run) if 'model' in file]
        models.sort(key=lambda m: '{0:0>15}'.format(m))
        model = models[-1]
    else:
        model = "model_{}.pt".format(checkpoint) 

    load_path = os.path.join(load_run, model)
    return load_path
