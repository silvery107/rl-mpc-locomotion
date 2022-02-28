class Parameters:
    cmpc_x_drag = 3.0
    cmpc_bonus_swing = 0.0
    cmpc_horizons = 10
    
    cmpc_alpha = 4e-5
    cmpc_solver_time = False
    cmpc_total_time = False
    cmpc_py_solver = 1 # 0 cvxopt 1 osqp

    cmpc_gait = 9 # 9 trot 4 stand
    controller_dt = 0.01
    control_mode = 4 # 4 locomotion
    operatingMode = 1 # 0 no transition 1 normal
    c_solver = 2 # 0 py solver 1 cpp solver 2 yuxiang solver
