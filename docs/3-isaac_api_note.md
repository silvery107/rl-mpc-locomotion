# Isaac API Reading Note


Controlling Actors

## DOF Properties and Drive Modes

- `DOF_MODE_EFFORT` lets you apply efforts to the DOF using `apply_actor_dof_efforts`. If the DOF is linear, the effort is a force in Newtons. If the DOF is angular, the effort is a torque in Nm. The DOF properties need to be set only once, but efforts must be applied every frame. Applying efforts multiple times during each frame is cumulative, but they are **reset to zero at the beginning** of the next frame:

