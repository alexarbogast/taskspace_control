import numpy as np
import quaternion
from rclpy.node import Node

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Vector3, Quaternion
from taskspace_control_msgs.msg import PoseTwistSetpoint

from .controller_client import ControllerClient

from .path_visualization import PathVisualization
from .trajectory import *


def quaternion_msg_to_np(q: Quaternion) -> quaternion.quaternion:
    return np.quaternion(q.w, q.x, q.y, q.z)


def quaternion_np_to_msg(q: quaternion.quaternion) -> Quaternion:
    return Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)


class ControlDemo(Node):
    def __init__(self, node_name: str, setpoint_hz=1000):
        super().__init__(node_name)
        self.declare_parameter("controller", "")
        controller_name = self.get_parameter("controller").value
        if controller_name == "":
            raise RuntimeError("Missing required parameter: controller")

        self.controller_client = ControllerClient(self, controller_name)

        self.path_viz = PathVisualization(
            self, 0.007, ColorRGBA(r=0.96, g=0.38, b=0.21, a=1.0)
        )

        self.static_orient = np.quaternion(1, 0, 0, 0)
        self.hz = setpoint_hz

    def movel(self, p, q, tf):
        pose = self.get_pose()
        if pose is None:
            self.get_logger().error("Failed to retrieve current pose")
            return

        cur_position = np.array([pose.position.x, pose.position.y, pose.position.z])
        cur_orient = quaternion_msg_to_np(pose.orientation)
        self.execute_linear_path(cur_position, p, cur_orient, q, tf)

    def execute_linear_path(self, p_start, p_end, q_start, q_end, tf):
        tt = np.linspace(0.0, tf, int(self.hz * tf))
        f, f_dot = linear_traj(p_start, p_end, tf)
        q, _ = slerp_traj(q_start, q_end, tf)

        self.execute_path(f(tt), f_dot(tt), q(tt))

    def execute_path(self, f, f_dot, orient):
        rate = self.create_rate(self.hz)
        setpoint = PoseTwistSetpoint()

        if isinstance(orient, quaternion.quaternion):
            orient = [orient] * len(f)

        for ft, f_dott, q in zip(f, f_dot, orient):
            setpoint.pose.position = Point(x=ft[0], y=ft[1], z=ft[2])
            setpoint.pose.orientation = quaternion_np_to_msg(q)
            setpoint.twist.linear = Vector3(x=f_dott[0], y=f_dott[1], z=f_dott[2])
            self.controller_client.publish_setpoint(setpoint)
            rate.sleep()

    def get_pose(self):
        return self.controller_client.get_pose()
