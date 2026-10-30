#!/usr/bin/env python3
"""Small ROS2 string publisher/subscriber used to validate the LAN path."""
from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=("pub", "sub"))
    parser.add_argument("--topic", default="/c1200_lan_probe")
    parser.add_argument("--seconds", type=float, default=8.0)
    args = parser.parse_args()
    rclpy.init()
    node = Node(f"c1200_lan_{args.mode}")
    publisher = None
    if args.mode == "sub":
        node.create_subscription(String, args.topic, lambda msg: print(msg.data, flush=True), 10)
    else:
        publisher = node.create_publisher(String, args.topic, 10)
    sequence = 0
    next_publish = 0.0
    try:
        end_time = time.monotonic() + args.seconds
        while time.monotonic() < end_time and rclpy.ok():
            now = time.monotonic()
            if publisher is not None and now >= next_publish:
                sequence += 1
                publisher.publish(String(data=f"lan_probe_seq={sequence}"))
                next_publish = now + 0.5
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
