class DesiredStateCommand:
    # yaw_turn_rate = 0.0
    # x_vel_cmd = 0.0
    # y_vel_cmd = 0.0

    def __init__(self) -> None:
        self.x_vel_cmd = 0.0
        self.y_vel_cmd = 0.0
        self.yaw_turn_rate = 0.0
        self.mpc_weights = None

    def updateCommand(self, commands, _weight=None):
        self.x_vel_cmd = commands[0]
        self.y_vel_cmd = commands[1]
        self.yaw_turn_rate = commands[2]
        
        if _weight is not None:
            weights = list(_weight) # (12,) weights from RobotRunnerPolicy
            weights.append(0.0)
            assert len(weights) == 13
            assert all(w>=0 for w in weights)
            self.mpc_weights = weights

        elif len(commands)>3:
            weights = list(commands[3:]) # (13,) weights from training env
            assert len(weights) == 13
            assert all(w>=0 for w in weights)
            self.mpc_weights = weights
    
    def reset(self):
        self.x_vel_cmd = 0.0
        self.y_vel_cmd = 0.0
        self.yaw_turn_rate = 0.0
        self.mpc_weights = None