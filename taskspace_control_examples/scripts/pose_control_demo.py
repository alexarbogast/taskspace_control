#!/usr/bin/env python3

import numpy as np
import rospy

from geometry_msgs.msg import Quaternion

from taskspace_control_examples import ControlDemo
from taskspace_control_examples.trajectory import *


robot_params = {
    "robot6R": {
        "orient": Quaternion(0.0, 0.0, 0.0, 1.0),
        "home": [0.0, -1.125, 2.275, -1.15, 1.571, 0.0],
    },
    "robot7R": {
        "orient": Quaternion(1.0, 0.0, 0.0, 0.0),
        "home": [0.0, 0.0, 0.0, -np.pi / 2, 0.0, np.pi / 2, 0.0],
    },
}


class PoseControlDemo(ControlDemo):
    def __init__(self, setpoint_hz=1000):
        super(PoseControlDemo, self).__init__(setpoint_hz)

        robot_type = rospy.get_param("robot_type", "robot6R")
        self.static_orient = robot_params[robot_type]["orient"]
        self.home = robot_params[robot_type]["home"]

    def run(self):
        self.start_joint_control()
        self.joint_controller_client.move_joint(self.home, 1.0)

        self.start_taskspace_control()
        self.circle()
        self.hypotrochoid()

        self.start_joint_control()
        self.joint_controller_client.move_joint(self.home, 1.0)

    def circle(self):
        tf = 5
        tt = np.linspace(0, tf, int(self.hz * tf))
        f, f_dot = circular_traj(1 / 5, tf)

        offset = np.array([0.5, 0.0, 0.1])
        ft, f_dott = f(tt) + offset, f_dot(tt)

        self.path_viz.visualize_path(
            [f(t) + offset for t in np.linspace(0, tf, 500)],
            "base_link",
        )

        self.movel(ft[0], 2)
        self.execute_path(ft, f_dott)
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

        self.movel(ft[0], 1)
        self.execute_path(ft, f_dott)
        self.path_viz.reset()


if __name__ == "__main__":
    rospy.init_node("pose_control_demo")

    try:
        demo = PoseControlDemo()
        demo.run()
    except rospy.ROSInterruptException:
        pass
