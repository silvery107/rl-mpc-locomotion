# RL Games API Notes


## Config Parameters

| Field                  | Example Value                             | Default  | Description                                                                                            |
|------------------------|-------------------------------------------|----------|--------------------------------------------------------------------------------------------------------|
| seed                   | 8                                         |  None    | Seed for pytorch, numpy etc.                                                            |
| algo                   |                                           |          | Algorithm block.                                             |
|   name                 | a2c_continuous                            |  None    | Algorithm name. Possible values are: sac, a2c_discrete, a2c_continuous                          |
| model                  |                                           |          | Model block.                                                                                        |
|   name                 | continuous_a2c_logstd                     |  None    | Possible values: continuous_a2c ( expects sigma to be (0, +inf), continuous_a2c_logstd  ( expects sigma to be (-inf, +inf), a2c_discrete, a2c_multi_discrete                      |
| network                |                                           |          | Network description.                                                                            |
|   name                 | actor_critic                              |          | Possible values: actor_critic or soft_actor_critic.                                                                           |
|   separate             | False                                     |          | Whether use or not separate network with same same architecture for critic. In almost all cases if you normalize value it is better to have it False                                                                                           |
|   space                |                                           |          | Network space                                                  |
|     continuous         |                                           |          | continuous or discrete                                |
|       mu_activation    | None                                      |          | Activation for mu. In almost all cases None works the best, but we may try tanh.                             |
|       sigma_activation | None                                      |          | Activation for sigma. Will be threated as log(sigma) or sigma depending on model.                                                                                    |
|       mu_init          |                                           |          | Initializer for mu.                                                   |
|         name           | default                                   |          |                                                                                     |
|       sigma_init       |                                           |          | Initializer for sigma. if you are using logstd model good value is 0.                          |
|         name           | const_initializer                         |          |                                                    |
|         val            | 0                                         |          |                  |
|       fixed_sigma      | True                                      |          | If true then sigma vector doesn't depend on input.                                                   |
|   cnn                  |                                           |          | Convolution block.                    |
|     type               | conv2d                                    |          | Type: right now two types supported: conv2d or conv1d                                               |
|     activation         | elu                                       |          | activation between conv layers.                                  |
|     initializer        |                                           |          | Initialier. I took some names from the tensorflow.                                                             |
|       name             | glorot_normal_initializer                 |          | initializer name                                                                                         |
|       gain             | 1.4142                                    |          | Additional parameter.                                                                  |
|     convs              |                                           |          | Convolution layers. Same parameters as we have in torch.                                                                                        |
|         filters        | 32                                        |          | Number of filters.                                                                                                  |
|         kernel_size    | 8                                         |          | Kernel size.                                                                                                    |
|         strides        | 4                                         |          | Strides                                                                  |
|         padding        | 0                                         |          | Padding                                                                                          |
|         filters        | 64                                        |          | Next convolution layer info.                                                                  |
|         kernel_size    | 4                                         |          |                                                                                                          |
|         strides        | 2                                         |          |                                                                                                |
|         padding        | 0                                         |          |                                                              |
|         filters        | 64                                        |          |                                           |
|         kernel_size    | 3                                         |          |                                                                                                         |
|         strides        | 1                                         |          |                                                |
|         padding        | 0                                         |          |                       
|   mlp                  |                                           |          | MLP Block. Convolution is supported too. See other config examples.                                                                                           |
|     units              |                                           |          | Lorem ipsum dolor sit amet, consecteteur adipiscing elit.                                              |
|     d2rl               | False                                     |          | Use d2rl architecture from https://arxiv.org/abs/2010.09163.                                                                                     |
|     activation         | elu                                       |          | Activations between dense layers.                                |
|     initializer        |                                           |          | Lorem ipsum dolor sit amet, consecteteur adipiscing elit b'duis'.                                      |
|       name             | default                                   |          | Lorem ipsum dolor sit amet, consecteteur adipiscing elit b'urna' b'mi'.                                |
|   rnn                  |                                           |          | RNN block.                                 |
|     name               | lstm                                      |          | RNN Layer name. lstm and gru are supported.                                                                                          |
|     units              | 256                                       |          | Number of units.                                             |
|     layers             | 1                                         |          | Number of layers                                                                                                  |
|     before_mlp         | False                                     | False    | Apply rnn before mlp block or not.                                                                                                  |
| config                 |                                           |          | RL Config block.                               |
|   reward_shaper        |                                           |          | Reward Shaper. Can apply simple transformations.                                              |
|     min_val            | -1                                        |          | You can apply min_val, max_val, scale and shift.                  |
|     scale_value        | 0.1                                       | 1        |  |
|   normalize_advantage  | True                                      | True     | Normalize Advantage.                                                              |
|   gamma                | 0.995                                     |          | Reward Discount                                                              |
|   tau                  | 0.95                                      |          | Lambda for GAE. Called tau by mistake long time ago because lambda is keyword in python :(         |
|   learning_rate        | 3e-4                                      |          | Learning rate.                                                   |
|   name                 | walker                                    |          | Name which will be used in tensorboard.                  |
|   save_best_after      | 10                                        |          | How many epochs to wait before start saving checkpoint with best score.                                                                                    |
|   score_to_win         | 300                                       |          | If score is >=value then this value training will stop.        |
|   grad_norm            | 1.5                                       |          | Grad norm. Applied if truncate_grads is True. Good value is in (1.0, 10.0)                                             |
|   entropy_coef         | 0                                         |          | Entropy coefficient. Good value for continuous space is 0. For discrete is 0.02                                              |
|   truncate_grads       | True                                      |          | Apply truncate grads or not. It stabilizes training.                                                  |
|   env_name             | BipedalWalker-v3                          |          | Envinronment name.            |
|   ppo                  | True                                      | True     | Use ppo loss or actor critic. Should be always true.                                    |
|   e_clip               | 0.2                                       |          | clip parameter for ppo loss.                                                                                 |
|   clip_value           | False                                     |          | Apply clip to the value loss. If you are using normalize_value you don't need it.                                                                                 |
|   num_actors           | 16                                        |          | Number of running actors.                           |
|   horizon_length       | 4096                                      |          | Horizon length per each actor. Total number of steps will be num_actors*horizon_length * num_agents (if env is not MA num_agents==1).                          |
|   minibatch_size       | 8192                                      |          | Minibatch size. total number number of steps must be divisible by minibatch size.                                                           |
|   mini_epochs          | 4                                         |          | Number of miniepochs. Good value is in [1,10]                                                                            |
|   critic_coef          | 2                                         |          | Critic coef. by default critic_loss= critic_coef * 1/2 * MSE.                                                                                    |
|   lr_schedule          | adaptive                                  | None     | Scheduler type. Could be None, linear or adaptive. Adaptive is the best for continuous.                                     |
|   schedule_type        | standard                                  |          | if schedule is adaptive there are a few places where we can change LR based on KL. If you standard it will be changed every miniepoch.                                                                                          |
|   kl_threshold         | 0.008                                     |          | KL threshould for adaptive schedule. if KL < kl_threshold/2 lr = lr * 1.5 and opposite.                                            |
|   normalize_input      | True                                      |          | Apply running mean std for input.                                                                           |
|   bounds_loss_coef     | 0.0                                       |          | Coefficient to the auxiary loss for continuous space.    |
|   max_epochs           | 10000                                     |          | Maximum number of epochs to run.                     |
|   normalize_value      | True                                      |          | Use value running mean std normalization.                                                                                          |
|   use_diagnostics      | True                                      |          | Adds more information into the tensorboard.                                              |
|   value_bootstrap      | True                                      |          | Bootstraping value when episode is finished. Very useful for different locomotion envs.               |
|   bound_loss_type      | 'regularisation'                          | None     | Adds aux loss for continuous case. 'regularisation' is the sum of sqaured actions. 'bound' is the sam of actions higher than 1.1.                                              |
|   bounds_loss_coef     | 0.0005                                    | 0        | Regularisation coefficient               |
|   use_smooth_clamp     | False                                     |          | Use smooth clamp instead of regular for cliping               |
|   player               |                                           |          | Player configuration block.                                                                                |
|     render             | True                                      | False    | Render environment                                                                            |
|     determenistic      | True                                      | True     | Use deterministic policy ( argmax or mu) or stochastic.                                                                                |
|     games_num          | 200                                       |          | Number of games to run in the player mode.                                             |
|   env_config           |                                           |          | Env configuration block. It goes directly to the environment. This example was take for my atari wrapper.                                                                                |
|     skip               | 4                                         |          | Number of frames to skip                                                                           |
|     name               | 'BreakoutNoFrameskip-v4'                  |          | Name of exact atari env. Of course depending on your env this parameters may be different.                                                                                |