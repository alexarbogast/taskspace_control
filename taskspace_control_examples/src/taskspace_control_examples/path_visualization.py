from typing import List
from copy import deepcopy
from numpy.typing import NDArray
import numpy as np
import quaternion

import rospy

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3, Quaternion
from std_msgs.msg import ColorRGBA


MARKER_ARRAY_TOPIC = "visualization_marker_array"


class PathVisualization(object):
    def __init__(self, radius: float, color: ColorRGBA):
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
        self.reset_marker.header.stamp = rospy.Time()
        self.reset_marker.action = Marker.DELETEALL

        self.vis_pub = rospy.Publisher(
            MARKER_ARRAY_TOPIC, MarkerArray, queue_size=1, latch=True
        )

    def visualize_path(self, path: List[NDArray], frame="world"):
        marker_array = MarkerArray()

        for i in range(len(path) - 1):
            p1, p2 = path[i], path[i + 1]
            marker = self.make_cylinder(p1, p2)
            marker.id = self.marker_id
            marker.header.stamp = rospy.Time.now()
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
        q = quaternion.from_rotation_vector(angle * axis)

        marker = deepcopy(self.cylinder_marker)
        marker.pose.orientation = Quaternion(q.x, q.y, q.z, q.w)
        marker.pose.position = Point(center[0], center[1], center[2])
        marker.scale = Vector3(self.r, self.r, h)
        marker.color = self.color
        return marker

    def reset(self):
        self.marker_id = 0
        marker_array = MarkerArray([self.reset_marker])
        self.vis_pub.publish(marker_array)
