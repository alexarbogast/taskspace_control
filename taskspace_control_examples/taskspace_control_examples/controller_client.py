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

from rclpy.node import Node

from taskspace_control_msgs.msg import PoseTwistSetpoint
from taskspace_control_msgs.srv import QueryPose


class ControllerClient:
    def __init__(self, node: Node, name: str, setpoint_type=PoseTwistSetpoint):
        self.name = name
        self.node = node

        self.node.declare_parameter(f"/{self.name}.setpoint_topic", "setpoint")
        setpoint_topic = self.node.get_parameter(f"/{self.name}.setpoint_topic").value

        # Publisher
        self.setpoint_pub = self.node.create_publisher(
            setpoint_type, f"/{self.name}/{setpoint_topic}", 1
        )

        # Service client
        self.pose_client = self.node.create_client(
            QueryPose, f"/{self.name}/query_pose"
        )
        if not self.pose_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error("query_pose service not available")
            raise RuntimeError("Service not available")

        self.node.get_logger().info(f"Connected to controller: {self.name}")

    def publish_setpoint(self, setpoint):
        self.setpoint_pub.publish(setpoint)

    def get_pose(self):
        req = QueryPose.Request()
        # Synchronous call because rclpy.spin() is called in a separate thread
        result = self.pose_client.call(req)
        if result is None:
            self.node.get_logger().error("query_pose call failed")
            return None

        return result.pose
