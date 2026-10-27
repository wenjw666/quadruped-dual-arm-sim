"""Occupancy grid for navigation, plus a Velodyne-style cloud for FAST-LIVO2.

Gazebo's ray cloud only has xyz. FAST-LIVO2's Velodyne handler also needs
intensity, time and ring, so this node republishes /fast_livo/velodyne.
"""

from __future__ import annotations

import math
import json

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from .color_map import ColorMap


class MapperNode(Node):
    def __init__(self) -> None:
        super().__init__("qda_mapper")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.origin = (-10.0, -10.0)
        self.resolution = 0.05
        self.width = 400
        self.height = 400
        self.obstacle_z = (0.08, 0.62)
        self.grid = np.full((self.height, self.width), -1, dtype=np.int8)
        self.clouds = []
        self.color_map=ColorMap()
        self.color_output=self.declare_parameter('color_output','/ws/output/mapping').value
        self.livo_locked = False
        self.odom_pose = None
        self.livo_pose = None
        self.odom_from_cam = None
        self.static_tf = StaticTransformBroadcaster(self)
        cloud_topic = self.declare_parameter("cloud_topic", "/fast_livo/cloud").value
        self.publish_velodyne = bool(self.declare_parameter("publish_velodyne", True).value)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, cloud_topic, self._on_cloud, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(Odometry, "/aft_mapped_to_init", self._on_livo, 10)
        color_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PointCloud2, "/cloud_registered", self._anchor_color, color_qos)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", map_qos)
        self.cloud_pub = self.create_publisher(PointCloud2, "/mapping/cloud", 1)
        self.color_pub = self.create_publisher(PointCloud2, "/mapping/colored", 2)
        self.global_color_pub=self.create_publisher(PointCloud2,'/mapping/colored_global',map_qos)
        self.color_status=self.create_publisher(String,'/mapping/color_status',map_qos)
        self.create_subscription(PointCloud2,'/mapping/rgbd_observation',self._rgbd,10)
        self.create_service(Trigger,'/mapping/save_color',self._save_color)
        self.create_timer(2.,self._publish_color)
        self.velo_pub = self.create_publisher(PointCloud2, "/fast_livo/velodyne", 1)
        self.create_timer(0.5, self._publish_map)
        if self.publish_velodyne:
            self.get_logger().info(f"栅格 /map 已启动；{cloud_topic} 转到 /fast_livo/velodyne 给 FAST-LIVO2")
        else:
            self.get_logger().info(
                f"栅格 /map 已启动；Mid-360 点云 {cloud_topic}，高度 {self.obstacle_z[0]:.2f}-{self.obstacle_z[1]:.2f} m 写入障碍"
            )

    def _rgbd(self,msg):
        if (msg.header.frame_id!='odom' or msg.is_bigendian or msg.point_step!=16
            or len(msg.data)!=msg.width*msg.height*16
            or [(f.name,f.offset,f.datatype) for f in msg.fields]!=[(k,4*i,PointField.FLOAT32) for i,k in enumerate(('x','y','z','rgb'))]):return
        a=np.frombuffer(msg.data,dtype='<f4').reshape(-1,4)
        packed=a[:,3].copy().view(np.uint32)
        rgb=np.column_stack(((packed>>16)&255,(packed>>8)&255,packed&255))
        self.color_map.add(a[:,:3],rgb)

    def _publish_color(self):
        if not self.color_map.cells:return
        xyz,rgb=self.color_map.arrays()
        bits=(rgb[:,0].astype(np.uint32)<<16)|(rgb[:,1].astype(np.uint32)<<8)|rgb[:,2].astype(np.uint32)
        points=np.column_stack((xyz,bits.view(np.float32))).astype('<f4')
        msg=PointCloud2(header=Header(stamp=self.get_clock().now().to_msg(),frame_id='odom'),
            height=1,width=len(points),fields=_rgb_fields(),is_bigendian=False,point_step=16,row_step=len(points)*16,is_dense=True,data=points.tobytes())
        self.global_color_pub.publish(msg)
        self.color_status.publish(String(data=json.dumps(dict(points=len(xyz),frames=self.color_map.frames,registration='simulation_truth_RGBD'))))

    def _save_color(self,request,response):
        try:response.message=json.dumps(self.color_map.save(self.color_output));response.success=True
        except Exception as exc:response.message=str(exc);response.success=False
        return response

    def _cell(self, x: float, y: float):
        col = int((x - self.origin[0]) / self.resolution)
        row = int((y - self.origin[1]) / self.resolution)
        if 0 <= col < self.width and 0 <= row < self.height:
            return row, col
        return None

    def _on_scan(self, msg: LaserScan) -> None:
        try:
            transform = self.tf_buffer.lookup_transform("odom", msg.header.frame_id, rclpy.time.Time())
        except Exception:
            return
        yaw = _yaw(transform.transform.rotation)
        ox = transform.transform.translation.x
        oy = transform.transform.translation.y
        angle = msg.angle_min
        for distance in msg.ranges:
            if math.isfinite(distance) and msg.range_min < distance < msg.range_max:
                x = ox + distance * math.cos(yaw + angle)
                y = oy + distance * math.sin(yaw + angle)
                hit = self._cell(x, y)
                if hit is not None:
                    self.grid[hit] = 100
                free_steps = max(1, int(distance / self.resolution) - 1)
                for step in range(free_steps):
                    ratio = step / free_steps
                    cell = self._cell(ox + ratio * (x - ox), oy + ratio * (y - oy))
                    if cell is not None and self.grid[cell] < 50:
                        self.grid[cell] = 0
            angle += msg.angle_increment
        self._clear_body(ox - 0.38 * math.cos(yaw), oy - 0.38 * math.sin(yaw), 0.40)

    def _on_cloud(self, msg: PointCloud2) -> None:
        if self.publish_velodyne:
            self._publish_velodyne(msg)
        try:
            transform = self.tf_buffer.lookup_transform("odom", msg.header.frame_id, rclpy.time.Time())
        except Exception:
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        rot = _rot(rotation)
        offset = np.array([translation.x, translation.y, translation.z])
        raw = np.asarray(list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
        if raw.size == 0:
            return
        if raw.dtype.names:
            raw = np.column_stack([raw[name] for name in ("x", "y", "z")])
        cloud_xyz = (raw @ rot.T + offset).astype(np.float32)
        self._paint_height(cloud_xyz, offset[0], offset[1])
        self._clear_body(float(offset[0]), float(offset[1]), 0.42)
        self.clouds.append(cloud_xyz)
        self.clouds = self.clouds[-6:]
        merged = np.concatenate(self.clouds, axis=0)
        if merged.shape[0] > 40000:
            merged = merged[:: merged.shape[0] // 40000 + 1]
        cloud = point_cloud2.create_cloud(msg.header, _fields(), merged)
        cloud.header.frame_id = "odom"
        cloud.header.stamp = msg.header.stamp
        self.cloud_pub.publish(cloud)

    def _publish_velodyne(self, msg: PointCloud2) -> None:
        """Add the ring and time fields the Velodyne preprocessor requires.

        FAST-LIVO2 reads ``time`` as microseconds from the start of the scan.
        """
        raw = []
        for x, y, z in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = float(x), float(y), float(z)
            span = math.hypot(x, y)
            elevation = math.atan2(z, span if span > 1e-6 else 1e-6)
            ring = int(round((elevation + 0.35) / 0.70 * 15.0))
            ring = max(0, min(15, ring))
            fraction = (math.atan2(y, x) + math.pi) / (2.0 * math.pi)
            raw.append((fraction * 200000.0, x, y, z, ring))
        if not raw:
            return
        raw.sort(key=lambda item: item[0])
        points = [(x, y, z, 80.0, time_us, ring) for time_us, x, y, z, ring in raw]
        cloud = point_cloud2.create_cloud(msg.header, _velodyne_fields(), points)
        cloud.header.frame_id = msg.header.frame_id
        cloud.header.stamp = msg.header.stamp
        self.velo_pub.publish(cloud)
        self.get_logger().info(
            f"三维雷达 {len(points)} 点，16 线，已发布 /fast_livo/velodyne",
            throttle_duration_sec=5.0,
        )

    def _on_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        quat = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        self.odom_pose = (
            np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float),
            _quat_matrix(*quat),
        )

    def _on_livo(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        quat = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        self.livo_pose = (
            np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float),
            _quat_matrix(*quat),
        )
        self._lock_livo_frame(msg)

    def _anchor_color(self, msg: PointCloud2) -> None:
        """Redraw the coloured scan on the kinematic base, not the drifting LIO pose.

        FAST-LIVO publishes in camera_init. That frame is locked once, so a later
        pose error leaves a permanent trail. Each scan is rigid in the body frame;
        placing it with /odom keeps the walls on the factory.
        """
        if msg.header.frame_id != "camera_init":
            return
        if self.odom_pose is None or self.livo_pose is None or self.odom_from_cam is None:
            return
        cam_t, cam_R = self.odom_from_cam
        livo_t, livo_R = self.livo_pose
        est = cam_R @ livo_t + cam_t
        odom_t, odom_R = self.odom_pose
        if float(np.linalg.norm(est[:2] - odom_t[:2])) > 0.30:
            return
        try:
            raw = point_cloud2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)
            arr = np.array(list(raw))
        except Exception:
            return
        if arr.size == 0:
            return
        if arr.dtype.names:
            xyz = np.stack((arr["x"], arr["y"], arr["z"]), axis=1).astype(float)
            rgb = np.asarray(arr["rgb"], dtype=np.float32)
        else:
            xyz = arr[:, :3].astype(float)
            rgb = arr[:, 3].astype(np.float32)
        body = (xyz - livo_t) @ livo_R
        world = body @ odom_R.T + odom_t
        cloud = point_cloud2.create_cloud(
            msg.header,
            _rgb_fields(),
            ((float(p[0]), float(p[1]), float(p[2]), float(c)) for p, c in zip(world, rgb)),
        )
        cloud.header.frame_id = "odom"
        cloud.header.stamp = msg.header.stamp
        self.color_pub.publish(cloud)

    def _lock_livo_frame(self, msg: Odometry) -> None:
        """Keep camera_init, where FAST-LIVO draws the map, fixed in odom."""
        if self.livo_locked:
            return
        try:
            odom_base = self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time())
        except Exception:
            return
        body = msg.pose.pose
        base = odom_base.transform
        inv_t, inv_q = _invert(body.position, body.orientation)
        base_t = (base.translation.x, base.translation.y, base.translation.z)
        base_q = (base.rotation.x, base.rotation.y, base.rotation.z, base.rotation.w)
        odom_cam = _compose(base_t, base_q, inv_t, inv_q)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "camera_init"
        transform.transform.translation.x = odom_cam[0]
        transform.transform.translation.y = odom_cam[1]
        transform.transform.translation.z = odom_cam[2]
        transform.transform.rotation.x = odom_cam[3]
        transform.transform.rotation.y = odom_cam[4]
        transform.transform.rotation.z = odom_cam[5]
        transform.transform.rotation.w = odom_cam[6]
        self.static_tf.sendTransform(transform)
        self.odom_from_cam = (
            np.array(odom_cam[:3], dtype=float),
            _quat_matrix(*odom_cam[3:]),
        )
        self.livo_locked = True
        self.get_logger().info("FAST-LIVO 地图坐标系 camera_init 已对齐到 odom")

    def _paint_height(self, points: np.ndarray, origin_x: float, origin_y: float) -> None:
        """Project body-height lidar returns into the navigation grid."""
        low, high = self.obstacle_z
        band = points[(points[:, 2] > low) & (points[:, 2] < high)]
        if band.shape[0] == 0:
            return
        if band.shape[0] > 500:
            band = band[:: band.shape[0] // 500]
        for x, y, _z in band:
            span = math.hypot(float(x) - origin_x, float(y) - origin_y)
            if span < 0.40 or span > 12.0:
                continue
            self._mark_ray(origin_x, origin_y, float(x), float(y))

    def _mark_ray(self, x0: float, y0: float, x1: float, y1: float) -> None:
        start = self._cell(x0, y0)
        end = self._cell(x1, y1)
        if start is None or end is None:
            return
        row, col = start
        end_row, end_col = end
        steps = max(abs(end_col - col), abs(end_row - row), 1)
        for step in range(steps):
            ratio = step / steps
            cell = self._cell(x0 + ratio * (x1 - x0), y0 + ratio * (y1 - y0))
            if cell is not None and self.grid[cell] < 50:
                self.grid[cell] = 0
        self.grid[end_row, end_col] = 100

    def _clear_body(self, x: float, y: float, radius: float) -> None:
        center = self._cell(x, y)
        if center is None:
            return
        row, col = center
        cells = int(radius / self.resolution)
        for dr in range(-cells, cells + 1):
            for dc in range(-cells, cells + 1):
                if dr * dr + dc * dc > cells * cells:
                    continue
                rr, cc = row + dr, col + dc
                if 0 <= rr < self.height and 0 <= cc < self.width:
                    self.grid[rr, cc] = 0

    def _publish_map(self) -> None:
        msg = OccupancyGrid()
        msg.header.frame_id = "odom"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin[0]
        msg.info.origin.position.y = self.origin[1]
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid.reshape(-1).tolist()
        self.map_pub.publish(msg)


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _rot(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _invert(position, rotation):
    x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
    rot = _rot(rotation).T
    trans = rot @ np.array([-position.x, -position.y, -position.z])
    return trans, (-x, -y, -z, w)


def _compose(left_t, left_q, right_t, right_q):
    """Rigid transform left * right. Translations are xyz, quaternions are xyzw."""
    rot = _quat_matrix(*left_q)
    out = rot @ np.asarray(right_t, dtype=float) + np.asarray(left_t, dtype=float)
    x, y, z, w = _quat_mul(left_q, right_q)
    return (float(out[0]), float(out[1]), float(out[2]), x, y, z, w)


def _quat_matrix(x, y, z, w):
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _rgb_fields():
    return [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]


def _fields():
    return [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]


def _velodyne_fields():
    return [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="time", offset=16, datatype=PointField.FLOAT32, count=1),
        PointField(name="ring", offset=20, datatype=PointField.UINT16, count=1),
    ]


def main() -> None:
    rclpy.init()
    node = MapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
