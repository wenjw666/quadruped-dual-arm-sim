"""Save overview and detection images so the sim can be checked without a GUI."""

from pathlib import Path

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class FrameDump(Node):
    def __init__(self) -> None:
        super().__init__("qda_frame_dump")
        self.bridge = CvBridge()
        self.out = Path(self.declare_parameter('output_dir', '/ws/output').value)
        self.out.mkdir(parents=True, exist_ok=True)
        self.counts = {"overview": 0, "debug": 0}
        self.create_subscription(Image, "/overview/image_raw", lambda m: self._save("overview", m), qos_profile_sensor_data)
        self.create_subscription(Image, "/perception/debug", lambda m: self._save("debug", m), 10)

    def _save(self, name: str, msg: Image) -> None:
        self.counts[name] += 1
        if self.counts[name] % 8:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"存图失败: {exc}")
            return
        import cv2

        path = self.out / f"{name}_latest.jpg"
        cv2.imwrite(str(path), image)


def main() -> None:
    rclpy.init()
    node = FrameDump()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
