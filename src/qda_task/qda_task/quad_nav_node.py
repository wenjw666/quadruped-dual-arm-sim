"""Drive the quadruped to a pose on /map. Idle until a goal arrives."""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Bool

from qda_task.quad_nav import plan_path, steer


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class QuadNavNode(Node):
    def __init__(self) -> None:
        super().__init__("qda_quad_nav")
        self.map_msg = None
        self.pos = None
        self.yaw = 0.0
        self.goal = None
        self.goal_yaw = 0.0
        self.path = []
        self.next_plan = 0.0
        self.create_subscription(OccupancyGrid, "/map", self._on_map, 1)
        self.create_subscription(Odometry, "/odom", self._on_odom, 20)
        self.create_subscription(PoseStamped, "/nav/goal", self._on_goal, 10)
        self.create_subscription(Bool, "/nav/cancel", self._on_cancel, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.path_pub = self.create_publisher(Path, "/plan", 1)
        self.create_timer(0.05, self._tick)
        self.get_logger().info("四足导航已启动，向 /nav/goal 发送目标位姿")

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

    def _on_odom(self, msg: Odometry) -> None:
        self.pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        self.yaw = _yaw(msg.pose.pose.orientation)

    def _on_goal(self, msg: PoseStamped) -> None:
        self.goal = np.array([msg.pose.position.x, msg.pose.position.y])
        self.goal_yaw = _yaw(msg.pose.orientation)
        self.next_plan = 0.0
        self.get_logger().info(f"导航目标 [{self.goal[0]:.2f}, {self.goal[1]:.2f}] 朝向 {self.goal_yaw:.2f}")

    def _on_cancel(self, msg: Bool) -> None:
        if msg.data:
            self.goal = None
            self.cmd_pub.publish(Twist())

    def _tick(self) -> None:
        if self.goal is None or self.pos is None:
            return
        if self.pos[2] < 0.42:
            self.cmd_pub.publish(Twist())
            return
        delta = self.goal - self.pos[:2]
        dist = float(np.linalg.norm(delta))
        yaw_err = _wrap(self.goal_yaw - self.yaw)
        if dist < 0.06 and abs(yaw_err) < 0.1:
            self.cmd_pub.publish(Twist())
            self.get_logger().info("已到达导航目标")
            self.goal = None
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now >= self.next_plan:
            self.path = self._plan()
            self._publish_path()
            self.next_plan = now + 0.8
        if dist < 0.06:
            cmd = Twist()
            cmd.angular.z = float(np.clip(1.2 * yaw_err, -0.45, 0.45))
            self.cmd_pub.publish(cmd)
            return
        grid, origin, resolution = self._grid()
        vx, vy, wz = steer(self.pos[:2], self.yaw, self.path or [self.goal], grid, origin, resolution)
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.angular.z = float(wz)
        self.cmd_pub.publish(cmd)

    def _plan(self):
        grid, origin, resolution = self._grid()
        if grid is None:
            return [self.goal]
        path = plan_path(grid, origin, resolution, self.pos[:2], self.goal)
        if path is None:
            self.get_logger().warn("目标被障碍堵住，先朝空闲方向转", throttle_duration_sec=2.0)
            return [self.goal]
        return path

    def _grid(self):
        msg = self.map_msg
        if msg is None or msg.info.width == 0:
            return None, (0.0, 0.0), 0.05
        grid = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        return grid, origin, msg.info.resolution

    def _publish_path(self) -> None:
        msg = Path()
        msg.header.frame_id = "odom"
        msg.header.stamp = self.get_clock().now().to_msg()
        for point in self.path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def main() -> None:
    rclpy.init()
    node = QuadNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
