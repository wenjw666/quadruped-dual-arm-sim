#!/usr/bin/env python3
"""Publish one local JPEG through ROS2 and wait for a SmolVLM response."""
from __future__ import annotations

import argparse
import json
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class Smoke(Node):
    def __init__(self, image_path: str, prompt: str) -> None:
        super().__init__("c1200_smolvlm_smoke")
        frame = cv2.imread(image_path, cv2.IMREAD_COLOR)
        if frame is None:
            raise RuntimeError(f"cannot read {image_path}")
        self.frame = frame
        self.prompt = prompt
        self.image_pub = self.create_publisher(Image, "/smolvlm/image", qos_profile_sensor_data)
        self.prompt_pub = self.create_publisher(String, "/smolvlm/prompt", 10)
        self.response_sub = self.create_subscription(String, "/smolvlm/response", self.on_response, 10)
        self.response = None
        self.started = time.monotonic()
        self.timer = self.create_timer(0.2, self.tick)
        self.sent_prompt = False

    def tick(self) -> None:
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height, msg.width = self.frame.shape[:2]
        msg.encoding = "bgr8"
        msg.step = int(self.frame.strides[0])
        msg.data = self.frame.tobytes()
        self.image_pub.publish(msg)
        if not self.sent_prompt and time.monotonic() - self.started > 1.0:
            self.prompt_pub.publish(String(data=self.prompt))
            self.sent_prompt = True
        if self.response is not None or time.monotonic() - self.started > 150.0:
            rclpy.shutdown()

    def on_response(self, msg: String) -> None:
        self.response = msg.data
        print(json.dumps(json.loads(msg.data), ensure_ascii=False, indent=2), flush=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("image")
    parser.add_argument("--prompt", default="Describe the image briefly.")
    args = parser.parse_args()
    rclpy.init()
    node = Smoke(args.image, args.prompt)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if node.response else 2


if __name__ == "__main__":
    raise SystemExit(main())
