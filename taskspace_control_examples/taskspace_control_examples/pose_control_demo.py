#!/usr/bin/env python3

import numpy as np
import quaternion
import rclpy
import threading

from taskspace_control_examples import ControlDemo
from taskspace_control_examples.trajectory import *


NODE_NAME = "pose_control_demo"

robot_params = {
    "robot6R": {
        "orient": np.quaternion(1.0, 0.0, 0.0, 0.0),
    },
    "robot7R": {
        "orient": np.quaternion(0.5, 0.5, 0.5, -0.5),
    },
}


class PoseControlDemo(ControlDemo):
    def __init__(self, node_name: str, setpoint_hz=1000):
        super().__init__(node_name, setpoint_hz)

        # Declare and get ROS2 parameters
        self.declare_parameter("robot_type", "robot6R")
        robot_type = self.get_parameter("robot_type").value

        self.static_orient = robot_params[robot_type]["orient"]

    def run(self):
        self.circle()
        # self.hypotrochoid()

    def circle(self):
        tf = 5
        tt = np.linspace(0, tf, int(self.hz * tf))
        f, f_dot = circular_traj(1 / 6, tf)

        offset = np.array([0.5, 0.0, 0.1])
        ft, f_dott = f(tt) + offset, f_dot(tt)

        self.path_viz.visualize_path(
            [f(t) + offset for t in np.linspace(0, tf, 500)],
            "base_link",
        )

        self.movel(ft[0], self.static_orient, 2)

        self.execute_path(ft, f_dott, self.static_orient)
        self.path_viz.reset()

    def hypotrochoid(self):
        scale = 1 / 28
        tf = 10
        tt = np.linspace(0, tf, int(self.hz * tf))
        f, f_dot = hypotrochoid_traj(3, 5, 4.5, tf, scaling=Order.THIRD)

        offset = np.array([0.5, 0.0, 0.1])
        ft, f_dott = scale * f(tt) + offset, scale * f_dot(tt)

        self.path_viz.visualize_path(
            [scale * f(t) + offset for t in np.linspace(0, tf, 500)],
            "base_link",
        )

        self.movel(ft[0], self.static_orient, 1)
        self.execute_path(ft, f_dott, self.static_orient)
        self.path_viz.reset()


def main(args=None):
    rclpy.init(args=args)
    node = PoseControlDemo("pose_control_demo")

    try:
        threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
        node.run()
    except Exception as e:
        node.get_logger().error(f"Exception in demo: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
