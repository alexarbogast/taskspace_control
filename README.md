# Task-space Controllers

[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)
[![ros - humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/index.html)
[![ros - jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/jazzy/index.html)

The `taskspace_control` package provides
[**task-space**](https://modernrobotics.northwestern.edu/nu-gm-book-resource/2-5-task-space-and-workspace/)
controllers for robotic manipulators using the
[ros2_control](https://github.com/ros-controls/ros2_control) framework.

## Package Overview

Each controller in the `taskspace_control` package accepts a setpoint defined by
a pose $p \in SE(3)$ and a twist $\xi \in \mathbb{R}^6$, delivered as a
[`PoseTwistSetpoint.msg`](./taskspace_control_msgs/msg/PoseTwistSetpoint.msg).
The `taskspace_control_examples` package provides an example setpoint publisher.

## Running the Demos

Launch the demo robot system with the desired robot and controller

```sh
 ros2 launch taskspace_control_examples robot_bringup.launch.py robot_type:=robot6R controller:=pose_controller
```

```
# robot_type (default "robot6R"): One of 'robot6R', 'robot7R
```

In another terminal, launch the control demo with the same controller

```sh
ros2 launch taskspace_control_examples pose_control_demo.launch.py robot_type:=robot6R controller:=pose_controller
```

```
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
`*_controller_plugins.xml` of each package. The controllers are configured to
work with hardware interfaces that accept a position and (optional) velocity command.
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
