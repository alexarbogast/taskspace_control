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

from typing import List
from copy import deepcopy
from numpy.typing import NDArray
import numpy as np

from rclpy.node import Node

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3, Quaternion
from std_msgs.msg import ColorRGBA

from taskspace_control_examples.quaternion import quaternion_from_rotation_vec

MARKER_ARRAY_TOPIC = "visualization_marker_array"


class PathVisualization:
    def __init__(self, node: Node, radius: float, color: ColorRGBA, ns: str = ""):
        self.node = node
        self.r = radius
        self.color = color
        self.marker_id = 0

        self.cylinder_marker = Marker()
        self.cylinder_marker.ns = "cylinder"
        self.cylinder_marker.action = Marker.ADD
        self.cylinder_marker.type = Marker.CYLINDER
        self.cylinder_marker.frame_locked = True

        self.reset_marker = Marker()
        self.reset_marker.ns = "cylinder"
        self.reset_marker.header.stamp = self.node.get_clock().now().to_msg()
        self.reset_marker.action = Marker.DELETEALL

        topic = f"/{ns}/{MARKER_ARRAY_TOPIC}" if ns else MARKER_ARRAY_TOPIC
        self.node.get_logger().info(f"Creating publisher on topic: '{topic}'")
        self.vis_pub = self.node.create_publisher(MarkerArray, topic, 1)

    def visualize_path(self, path: List[NDArray], frame: str = "world"):
        marker_array = MarkerArray()

        for i in range(len(path) - 1):
            p1, p2 = path[i], path[i + 1]
            marker = self.make_cylinder(p1, p2)
            marker.id = self.marker_id
            marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.header.frame_id = frame
            marker_array.markers.append(marker)
            self.marker_id += 1

        self.vis_pub.publish(marker_array)

    def make_cylinder(self, p1: NDArray, p2: NDArray):
        v = p2 - p1
        h = np.linalg.norm(v)
        center = np.average([p1, p2], axis=0)

        u = v / h
        z_axis = np.array([0, 0, 1])
        axis = np.cross(z_axis, u)
        norm = np.linalg.norm(axis)
        if norm > 0:
            axis /= norm
        angle = np.arccos(np.dot(z_axis, u))
        q = quaternion_from_rotation_vec(angle * axis)

        marker = deepcopy(self.cylinder_marker)
        marker.pose.orientation = Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])
        marker.pose.position = Point(x=center[0], y=center[1], z=center[2])
        marker.scale = Vector3(x=self.r, y=self.r, z=h)
        marker.color = self.color
        return marker

    def reset(self):
        self.marker_id = 0
        marker_array = MarkerArray(markers=[self.reset_marker])
        self.vis_pub.publish(marker_array)
