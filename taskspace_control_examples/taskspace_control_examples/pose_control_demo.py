#!/usr/bin/env python3

# Copyright 2024 Alex Arbogast
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np
import rclpy
import threading

from taskspace_control_examples import ControlDemo
from taskspace_control_examples.quaternion import *
from taskspace_control_examples.trajectory import *


NODE_NAME = "pose_control_demo"

robot_params = {
    "robot6R": {
        "orient": np.array([1.0, 0.0, 0.0, 0.0]),
        "q_diff": np.array([0.9238795, 0, 0, 0.3826834]),
    },
    "robot7R": {
        "orient": np.array([0.5, 0.5, 0.5, -0.5]),
        "q_diff": np.array([0.707107, -0.707107, 0, 0]),
    },
}


class PoseControlDemo(ControlDemo):
    def __init__(self, node_name: str, setpoint_hz=1000):
        super().__init__(node_name, setpoint_hz)

        # Declare and get ROS2 parameters
        self.declare_parameter("robot_type", "robot6R")
        robot_type = self.get_parameter("robot_type").value

        self.static_orient = robot_params[robot_type]["orient"]
        self.q_diff = robot_params[robot_type]["q_diff"]

    def run(self):
        self.line()
        self.circle()
        self.hypotrochoid()

    def line(self):
        tf = 5
        tt = np.linspace(0, tf, int(self.hz * tf))

        p_start = np.array([0.5, -0.5, 0.1])
        p_end = np.array([0.5, 0.5, 0.1])

        f, f_dot = linear_traj(p_start, p_end, tf)
        ft, f_dott = f(tt), f_dot(tt)

        q_start = self.static_orient
        q_end = quaternion_multiply(self.static_orient, self.q_diff)

        q, _ = slerp_traj(q_start, q_end, tf, scaling=Order.FIFTH)
        qt = q(tt)

        self.path_viz.visualize_path(
            [f(t) for t in np.linspace(0, tf, 500)],
            "base_link",
        )

        self.movel(ft[0], self.static_orient, 2)

        self.execute_path(ft, f_dott, qt)
        self.path_viz.reset()

    def circle(self):
        tf = 5
        tt = np.linspace(0, tf, int(self.hz * tf))
        f, f_dot = circular_traj(1 / 6, tf)

        offset = np.array([0.5, 0.0, 0.0])
        ft, f_dott = f(tt) + offset, f_dot(tt)

        q_start = self.static_orient
        q_end = quaternion_multiply(self.static_orient, self.q_diff)

        q, _ = slerp_traj(q_start, q_end, tf, scaling=Order.FIFTH)
        qt = q(tt)

        self.path_viz.visualize_path(
            [f(t) + offset for t in np.linspace(0, tf, 500)],
            "base_link",
        )

        self.movel(ft[0], self.static_orient, 2)
        self.execute_path(ft, f_dott, qt)
        self.path_viz.reset()

    def hypotrochoid(self):
        scale = 1 / 28
        tf = 10
        tt = np.linspace(0, tf, int(self.hz * tf))
        f, f_dot = hypotrochoid_traj(3, 5, 4.5, tf, scaling=Order.THIRD)

        offset = np.array([0.5, 0.0, 0.1])
        ft, f_dott = scale * f(tt) + offset, scale * f_dot(tt)

        q_start = self.static_orient
        q_end = quaternion_multiply(self.static_orient, self.q_diff)

        q, _ = slerp_traj(q_start, q_end, tf, scaling=Order.FIFTH)
        qt = q(tt)

        self.path_viz.visualize_path(
            [scale * f(t) + offset for t in np.linspace(0, tf, 500)],
            "base_link",
        )

        self.movel(ft[0], self.static_orient, 1)
        self.execute_path(ft, f_dott, qt)
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
