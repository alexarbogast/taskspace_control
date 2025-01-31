import numpy as np
import quaternion
import rospy

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Vector3, Quaternion
from taskspace_control_msgs.msg import PoseTwistSetpoint

from .controller_client import ControllerClient, JointControllerClient
from .controller_manager_client import ControllerManagerClient
from .path_visualization import PathVisualization
from .trajectory import *


def quaternion_msg_to_np(q: Quaternion) -> quaternion.quaternion:
    return np.quaternion(q.w, q.x, q.y, q.z)


def quaternion_np_to_msg(q: quaternion.quaternion) -> Quaternion:
    return Quaternion(q.x, q.y, q.z, q.w)


class ControlDemo(object):
    def __init__(self, setpoint_hz=1000):
        controller_name = rospy.get_param("~controller")
        joint_controller_name = rospy.get_param("~joint_controller")

        self.controller_client = ControllerClient(controller_name)
        self.joint_controller_client = JointControllerClient(joint_controller_name)

        self.controller_manager_client = ControllerManagerClient()
        self.path_viz = PathVisualization(0.007, ColorRGBA(0.96, 0.38, 0.21, 1.0))

        self.static_orient = np.quaternion(1, 0, 0, 0)
        self.hz = setpoint_hz

    def movel(self, p, q, tf):
        pose = self.get_pose()
        if pose is None:
            rospy.logerr("Failed to retrieve current pose")
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
        rate = rospy.Rate(self.hz)
        setpoint = PoseTwistSetpoint()

        if isinstance(orient, quaternion.quaternion):
            orient = [orient] * len(f)

        for ft, f_dott, q in zip(f, f_dot, orient):
            setpoint.pose.position = Point(*ft)
            setpoint.pose.orientation = quaternion_np_to_msg(q)
            setpoint.twist.linear = Vector3(*f_dott)
            self.controller_client.publish_setpoint(setpoint)
            rate.sleep()

    def get_pose(self):
        return self.controller_client.get_pose()

    def start_joint_control(self):
        self.controller_manager_client.switch_controller(
            [self.joint_controller_client.name], [self.controller_client.name]
        )

    def start_taskspace_control(self):
        self.controller_manager_client.switch_controller(
            [self.controller_client.name], [self.joint_controller_client.name]
        )
