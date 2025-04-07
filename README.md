# Task Space Controllers

The [**task space**](https://modernrobotics.northwestern.edu/nu-gm-book-resource/2-5-task-space-and-workspace/)
of a robot refers to the space in which the robot's end-effector or tool
operates. The `taskspace_control` ROS metapackage provides a set of packages for
task-space control of robotic manipulators using the
[ros_control](https://github.com/ros-controls/ros_control) framework.

## Package Overview

Each controller created by the `taskspace_control` package subscribes to the
same type of setpoint. This setpoint is defined by a pose $p \in SE(3)$ and a
twist $\xi \in \mathbb{R}^6$. The setpoint is provided to the controller via a
[`PoseTwistSetpoint.msg`](./taskspace_control_msgs/msg/PoseTwistSetpoint.msg).

The `taskspace_control_examples` package provides an example implementation of a
setpoint publisher for the controllers.

## Running the Demos

Launch the demo robot system with the desired robot

```
roslaunch taskspace_control_examples control_bringup.launch robot:=robot6R

# robot (default "robot6R"): One of 'robot6R', 'robot7R
```

In another terminal, launch the control demo with the desired controller

```
roslaunch taskspace_control_examples pose_control_demo.launch controller:=pose_controller

# controller (default "pose_controller"):
#    One of 'pose_controller,
#            task_priority_controller,
#            as_nullspace_controller,
#            as_twist_decomposition_controller'
```

The redundancy resolution objectives, kinematic frames, and controller
parameters can be modified in the configurations files of the examples package.

## Controller Types

A list of available controller plugins can be found in the
`*_controller_plugin.xml` of each package. The controllers have been implemented
under the assumption that the hardware*interface accepts position commands. It
would take a bit of work to template these libraries for velocity or effort
interfaces, but this \_might* be completed in the future.

A basic configuration for the controllers below can be found in the
`taskspace_control_examples` package config.

### Task-space Controllers

- `taskspace_controllers/TaskspaceControllerBase`

  The base task-space controller constructs the kinematic model of the robot.
  It also sets up some boilerplate functionality like a forward kinematics
  solver and the joint limits of the robot. All other controllers inherit from
  this class. _Do not use this controller in practice._

- `taskspace_controllers/PoseController`

  A pose controller tracks a fully defined setpoint using a basic inverse
  Jacobian tracking controller.

### Task-priority Controllers

If the robot is kinematically redundant (i.e. there are more than 6 joints) than
a [task priority](https://roboticsknowledgebase.com/wiki/actuation/task-prioritization-control/)
controller can be used to achieve a secondary set of objectives in addition to
tracking the setpoint. This is done through a nullspace projection using the
robot's Jacobian. The redundancy resolution objective, or secondary priority
task, is defined by a plugin that's loaded at runtime. See
[`objective_plugins.xml`](./task_priority_controllers/objective_plugins.xml) for
a list of provided plugins.

- `task_priority_controllers/PoseController`

  This controller tracks a fully defined setpoint and uses the redundant
  degrees-of-freedom to achieve the objective plugin.

### Axially-symmetric Controllers

The axially-symmetric controllers inherit from the Task-priority controllers.
These controllers treat the setpoint as a 5-DOF task. The manipulator will track
a position and align the z-axis of the end-effector with the z-axis of the
setpoint pose. The rotation about the z-axis of the tool is left as a redundant
axis where the rotation is decided by the redundancy resolution objective.

- `axially_symmetric_controllers/NullspaceController`

  This controller uses the Jacobian nullspace projection for redundancy
  resolution.

- `axially_symmetric_controllers/TwistDecompositionController`

  This controller uses
  [twist-decomposition](https://www.researchgate.net/publication/228961289_The_joint-limits_and_singularity_avoidance_in_robotic_welding)
  for redundancy resolution.
