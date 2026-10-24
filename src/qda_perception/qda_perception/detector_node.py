"""YOLO classification fused with color segmentation and depth back-projection."""

from __future__ import annotations

import math
from concurrent.futures import ThreadPoolExecutor
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener


def _median_depth(depth: np.ndarray, u: int, v: int, radius: int = 3) -> float:
    h, w = depth.shape[:2]
    u0, u1 = max(0, u - radius), min(w, u + radius + 1)
    v0, v1 = max(0, v - radius), min(h, v + radius + 1)
    window = depth[v0:v1, u0:u1].astype(np.float32)
    if depth.dtype == np.uint16:
        window = window / 1000.0
    valid = window[np.isfinite(window)]
    valid = valid[(valid > 0.08) & (valid < 4.0)]
    if valid.size < 4:
        return float("nan")
    return float(np.median(valid))


def _blob_candidates(mask: np.ndarray):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        # Reject large wall/table regions with the same hue as the target.
        # This is an image-size prior, not a substitute for measured RGB-D.
        if 40 < area < mask.size * 0.04:
            moments = cv2.moments(contour)
            if moments["m00"] <= 1e-6:
                continue
            u = int(moments["m10"] / moments["m00"])
            v = int(moments["m01"] / moments["m00"])
            candidates.append((u, v, int(area)))
    return sorted(candidates,key=lambda p:p[2],reverse=True)


def _largest_blob(mask: np.ndarray) -> Optional[Tuple[int, int, int]]:
    candidates=_blob_candidates(mask)
    return candidates[0] if candidates else None


def _depth_blob(mask, depth, focal, max_extent):
    """Try all color candidates; a distant red wall must not hide the cube."""
    if not math.isfinite(focal) or focal<=1:return None
    for u,v,area in _blob_candidates(mask):
        z=_median_depth(depth,u,v)
        extent=math.sqrt(area)*z/focal
        if math.isfinite(z) and .008 <= extent <= max_extent:
            return u,v,area
    return None


class DetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("qda_perception")
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.info = None
        self.depth = None
        self.depth_stamp = None
        self.model = None
        self.yolo_error = ""
        # Auxiliary labels must not block RGB-D geometry or its ROS callbacks.
        self.yolo_worker = ThreadPoolExecutor(max_workers=1,thread_name_prefix='yolo_labels')
        self.yolo_load = self.yolo_worker.submit(self._load_yolo)
        self.yolo_pending = None

        self.create_subscription(CameraInfo, "/camera/camera_info", self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/depth/image_raw", self._on_depth, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/image_raw", self._on_color, qos_profile_sensor_data)
        self.pick_pub = self.create_publisher(PoseStamped, "/perception/pick_pose", 10)
        self.place_pub = self.create_publisher(PoseStamped, "/perception/place_pose", 10)
        self.label_pub = self.create_publisher(String, "/perception/labels", 10)
        self.debug_pub = self.create_publisher(Image, "/perception/debug", 5)
        self.marker_pub = self.create_publisher(MarkerArray, "/perception/markers", 10)
        self.frame_index = 0

    def _on_info(self, msg: CameraInfo) -> None:
        self.info = msg

    def _on_depth(self, msg: Image) -> None:
        try:
            self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.depth_stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"深度图转换失败: {exc}", throttle_duration_sec=2.0)

    def _on_color(self, msg: Image) -> None:
        self.frame_index += 1
        if self.info is None or self.depth is None:
            return
        color_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if self.depth_stamp is None or abs(color_stamp-self.depth_stamp)>.15:
            return
        if self.frame_index % 2:
            return
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"彩色图转换失败: {exc}", throttle_duration_sec=2.0)
            return
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        red = cv2.bitwise_or(
            cv2.inRange(hsv, (0, 90, 50), (10, 255, 255)),
            cv2.inRange(hsv, (160, 90, 50), (180, 255, 255)),
        )
        green = cv2.inRange(hsv, (40, 80, 40), (95, 255, 255))
        labels = []
        yolo_boxes = []
        if self.yolo_pending is not None and self.yolo_pending.done():
            boxes, stamp = self.yolo_pending.result()
            now = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            if 0 <= now-stamp <= .3:yolo_boxes = boxes
            self.yolo_pending = None
        if self.frame_index % 4 == 0 and self.model is not None and self.yolo_pending is None:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            self.yolo_pending = self.yolo_worker.submit(lambda image=bgr.copy(), t=stamp:(self._yolo_boxes(image),t))
        debug = bgr.copy()
        for mask, name, publisher, color in (
            (red, "pick", self.pick_pub, (0, 0, 255)),
            (green, "place", self.place_pub, (0, 255, 0)),
        ):
            blob = _depth_blob(mask,self.depth,self.info.k[0],.10 if name=='pick' else .18)
            if blob is None:
                continue
            u, v, area = blob
            pose = self._project(u, v, msg.header.stamp)
            yolo_name = self._overlap_label(u, v, yolo_boxes)
            if pose is None:
                continue
            publisher.publish(pose)
            self._mark(name, pose, color)
            labels.append(f"{name}:{yolo_name or 'color'}:{area}")
            cv2.circle(debug, (u, v), 8, color, 2)
            cv2.putText(
                debug,
                yolo_name or name,
                (u + 8, v - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
                cv2.LINE_AA,
            )
        for x1, y1, x2, y2, cls_name, conf in yolo_boxes:
            cv2.rectangle(debug, (x1, y1), (x2, y2), (255, 200, 0), 1)
            labels.append(f"yolo:{cls_name}:{conf:.2f}")
        text = String()
        text.data = ",".join(labels) if labels else "none"
        self.label_pub.publish(text)
        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding="bgr8"))
        except Exception:
            pass

    def _load_yolo(self):
        try:
            import torch
            torch.set_num_threads(2)
            from ultralytics import YOLO
            self.model = YOLO("yolov8n.pt")
            self.get_logger().info("YOLOv8n 已加载，异步补充颜色分割标签")
        except Exception as exc:
            self.yolo_error=str(exc)
            self.get_logger().error(f"YOLO 加载失败，只保留颜色分割: {exc}")

    def _yolo_boxes(self, bgr: np.ndarray):
        if self.model is None:
            return []
        try:
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            result = self.model.predict(rgb, verbose=False, device="cpu", imgsz=320, conf=0.2)[0]
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"YOLO 推理失败: {exc}", throttle_duration_sec=5.0)
            return []
        boxes = []
        if result.boxes is None:
            return boxes
        names = result.names
        for box in result.boxes:
            xyxy = box.xyxy[0].detach().cpu().numpy()
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            boxes.append(
                (int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3]), names.get(cls_id, str(cls_id)), conf)
            )
        return boxes

    @staticmethod
    def _overlap_label(u: int, v: int, boxes) -> str:
        for x1, y1, x2, y2, name, _conf in boxes:
            if x1 <= u <= x2 and y1 <= v <= y2:
                return name
        return ""

    def _mark(self, name: str, pose: PoseStamped, color) -> None:
        marker = Marker()
        marker.header = pose.header
        marker.ns = "perception"
        marker.id = 1 if name == "pick" else 2
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = marker.scale.y = marker.scale.z = 0.05
        marker.color.b = float(color[0]) / 255.0
        marker.color.g = float(color[1]) / 255.0
        marker.color.r = float(color[2]) / 255.0
        marker.color.a = 0.9
        marker.lifetime.sec = 1
        array = MarkerArray()
        array.markers.append(marker)
        self.marker_pub.publish(array)

    def _project(self, u: int, v: int, stamp) -> Optional[PoseStamped]:
        if self.depth.shape[0] != self.info.height or self.depth.shape[1] != self.info.width:
            return None
        z = _median_depth(self.depth, u, v)
        if not math.isfinite(z):
            return None
        fx, fy = self.info.k[0], self.info.k[4]
        cx, cy = self.info.k[2], self.info.k[5]
        if fx < 1.0 or fy < 1.0:
            return None
        point = PointStamped()
        point.header.stamp = stamp
        point.header.frame_id = "camera_optical_frame"
        point.point.x = (u - cx) * z / fx
        point.point.y = (v - cy) * z / fy
        point.point.z = z
        try:
            transform = self.tf_buffer.lookup_transform("base_link", "camera_optical_frame", rclpy.time.Time())
        except Exception:
            return None
        base_point = do_transform_point(point, transform)
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "base_link"
        pose.pose.position.x = base_point.point.x
        pose.pose.position.y = base_point.point.y
        pose.pose.position.z = base_point.point.z
        pose.pose.orientation.w = 1.0
        return pose


def main() -> None:
    rclpy.init()
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.yolo_worker.shutdown(wait=False,cancel_futures=True)
    node.destroy_node()
    rclpy.shutdown()
