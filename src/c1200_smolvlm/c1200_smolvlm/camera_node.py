"""Publish a canonical RGB stream for the board-side SmolVLM runner.

``topic`` consumes an already configured ROS camera driver.  ``v4l2`` is for
UVC/RealSense color output.  ``gstreamer`` uses the C1200 ``ispsrc`` element
for the GMSL/CSI/ISP path.  This Python probe copies frames into ROS messages;
the production zero-copy path can replace it without changing the VLM topic.
"""

from __future__ import annotations

import threading
from time import monotonic, sleep
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraNode(Node):
    def __init__(self) -> None:
        super().__init__("c1200_smolvlm_camera")
        self.source = str(self.declare_parameter("source", "topic").value).lower()
        self.input_topic = str(self.declare_parameter("input_topic", "/camera/image_raw").value)
        self.output_topic = str(self.declare_parameter("output_topic", "/smolvlm/image").value)
        self.device = str(self.declare_parameter("device", "/dev/video4").value)
        self.pipeline = str(self.declare_parameter("pipeline", "").value)
        self.width = int(self.declare_parameter("width", 640).value)
        self.height = int(self.declare_parameter("height", 480).value)
        self.fps = float(self.declare_parameter("fps", 10.0).value)
        self.frame_id = str(self.declare_parameter("frame_id", "camera_color_optical_frame").value)
        input_reliability = str(self.declare_parameter("input_reliability", "reliable").value).lower()
        input_qos = qos_profile_sensor_data
        if input_reliability == "reliable":
            input_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(Image, self.output_topic, qos_profile_sensor_data)
        self._capture = None
        self._capture_thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

        if self.source == "topic":
            self.subscription = self.create_subscription(
                Image, self.input_topic, self._on_topic_image, input_qos
            )
            self.get_logger().info(f"source=topic input={self.input_topic} output={self.output_topic}")
        elif self.source in ("v4l2", "gstreamer"):
            self._start_capture()
        else:
            raise ValueError("source must be topic, v4l2, or gstreamer")

    def _on_topic_image(self, msg: Image) -> None:
        if not msg.header.frame_id:
            msg.header.frame_id = self.frame_id
        self.publisher.publish(msg)

    def _start_capture(self) -> None:
        import cv2

        if self.source == "gstreamer":
            pipeline = self.pipeline or (
                f"ispsrc device={self.device} ! video/x-raw,format=NV12,"
                f"width={self.width},height={self.height},framerate={int(self.fps)}/1 "
                "! videoconvert ! appsink drop=true max-buffers=1 sync=false"
            )
            self._capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        else:
            self._capture = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
            self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self._capture.set(cv2.CAP_PROP_FPS, self.fps)
        if not self._capture or not self._capture.isOpened():
            raise RuntimeError(f"cannot open {self.source}: {self.device}")
        self.get_logger().info(f"source={self.source} device={self.device} output={self.output_topic}")
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()

    def _capture_loop(self) -> None:
        import cv2

        period = 1.0 / max(self.fps, 1.0)
        next_frame = monotonic()
        while rclpy.ok() and not self._stop.is_set():
            ok, frame = self._capture.read()
            if not ok:
                self.get_logger().warning("camera read failed", throttle_duration_sec=2.0)
                sleep(0.1)
                continue
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.height, msg.width = int(frame.shape[0]), int(frame.shape[1])
            msg.encoding = "bgr8"
            msg.step = int(frame.strides[0])
            msg.data = frame.tobytes()
            self.publisher.publish(msg)
            next_frame += period
            sleep(max(0.0, next_frame - monotonic()))

    def destroy_node(self):  # type: ignore[override]
        self._stop.set()
        if self._capture is not None:
            self._capture.release()
        if self._capture_thread is not None:
            self._capture_thread.join(timeout=1.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
