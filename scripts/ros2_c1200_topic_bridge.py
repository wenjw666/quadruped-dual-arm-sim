#!/usr/bin/env python3
"""Relay selected ROS 2 topics between the local simulation and C1200.

The relay is intentionally one ROS participant.  This lets the simulation
keep its normal multicast configuration while only the relay uses the static
Ethernet/Docker DDS profile.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class C1200TopicBridge(Node):
    def __init__(self) -> None:
        super().__init__("c1200_topic_bridge")
        self.image_pub = self.create_publisher(Image, "/c1200/camera/image_raw", qos_profile_sensor_data)
        self.image_sub = self.create_subscription(
            Image, "/camera/image_raw", self._forward_image, qos_profile_sensor_data
        )
        self.response_pub = self.create_publisher(String, "/c1200/smolvlm/response", 10)
        self.response_sub = self.create_subscription(
            String, "/smolvlm/response", self._forward_response, 10
        )
        self.status_pub = self.create_publisher(String, "/c1200/smolvlm/status", 10)
        self.status_sub = self.create_subscription(
            String, "/smolvlm/status", self._forward_status, 10
        )
        self.get_logger().info("C1200 topic bridge ready: /camera/image_raw -> /c1200/camera/image_raw")

    def _forward_image(self, msg: Image) -> None:
        self.image_pub.publish(msg)

    def _forward_response(self, msg: String) -> None:
        self.response_pub.publish(msg)

    def _forward_status(self, msg: String) -> None:
        self.status_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = C1200TopicBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
