"""Deploy the trot plus the learned residual as a joint-position policy."""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float64MultiArray

from qda_kinematics.kinematics import (
    CYCLE,
    LEG_JOINTS,
    LegKinematics,
    joint_command,
    stance_command,
)
from qda_kinematics.policy import OBS_DIM, ResidualPolicy

try:
    from tf2_ros import TransformBroadcaster
except ImportError:  # pragma: no cover
    TransformBroadcaster = None


def _quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


class LocomotionNode(Node):
    def __init__(self) -> None:
        super().__init__("qda_locomotion")
        self.declare_parameter("use_neural", False)
        self.legs = LegKinematics()
        self.use_neural = False
        self.policy = None
        self.get_logger().info("B2 使用几何对角步，按底盘速度平滑调整步幅")
        if self.get_parameter("use_neural").value:
            self.get_logger().warn("旧残差策略的关节约定与 B2 不兼容；use_neural=true 暂不生效")
        self.phase = 0.0
        self.cmd = np.zeros(3)
        self.cmd_target = np.zeros(3)
        self.last_cmd_time = None
        self.measured = np.zeros(3)
        self.gravity = np.array([0.0, 0.0, -1.0])
        self.q = np.zeros(12)
        self.dq = np.zeros(12)
        self.have_joints = False
        self.have_odom = False
        self.last_time = None
        self.last_command = joint_command(0.0, (0, 0, 0), (0, 0, 0), self.legs)
        self.odom_pose = None

        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 20)
        self.create_subscription(Odometry, "/odom", self._on_odom, 20)
        self.create_subscription(Imu, "/imu", self._on_imu, qos_profile_sensor_data)
        self.pub = self.create_publisher(Float64MultiArray, "/leg_controller/commands", 10)
        self.ready_pub = self.create_publisher(Bool, "/locomotion/ready", 10)
        self.tf_pub = TransformBroadcaster(self) if TransformBroadcaster else None
        self.timer = self.create_timer(0.02, self._tick)
        self.get_logger().info("运控节点启动，零速度时保持站立")

    def _on_cmd(self, msg: Twist) -> None:
        self.last_cmd_time = self.get_clock().now()
        self.cmd_target = np.array(
            [
                float(np.clip(msg.linear.x, -0.05, 0.22)),
                float(np.clip(msg.linear.y, -0.12, 0.12)),
                float(np.clip(msg.angular.z, -0.7, 0.7)),
            ]
        )
        if not np.all(np.isfinite(self.cmd_target)):
            self.cmd_target[:] = 0.0

    def _on_joints(self, msg: JointState) -> None:
        index = {name: i for i, name in enumerate(msg.name)}
        if not all(name in index for name in LEG_JOINTS):
            return
        self.q = np.array([msg.position[index[name]] for name in LEG_JOINTS])
        if msg.velocity and len(msg.velocity) == len(msg.name):
            self.dq = np.array([msg.velocity[index[name]] for name in LEG_JOINTS])
        self.have_joints = True

    def _on_odom(self, msg: Odometry) -> None:
        self.measured = np.array(
            [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z]
        )
        self.odom_pose = msg
        self.have_odom = True
        if self.tf_pub is None:
            return
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = msg.header.frame_id or "odom"
        transform.child_frame_id = msg.child_frame_id or "base_link"
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.tf_pub.sendTransform(transform)

    def _on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        rot = _quat_to_rot(q.x, q.y, q.z, q.w)
        self.gravity = rot.T @ np.array([0.0, 0.0, -1.0])

    def _observation(self) -> np.ndarray:
        obs = np.zeros(OBS_DIM)
        obs[0:3] = self.gravity
        obs[3:6] = self.cmd
        obs[6:18] = self.q
        obs[18:30] = self.dq
        obs[30] = math.sin(2.0 * math.pi * self.phase)
        obs[31] = math.cos(2.0 * math.pi * self.phase)
        return obs

    def _tick(self) -> None:
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0 or dt > 0.2:
            dt = 0.02
        if self.last_cmd_time is None or (now - self.last_cmd_time).nanoseconds * 1e-9 > 0.5:
            self.cmd_target[:] = 0.0
        alpha = dt / (0.12 + dt)
        self.cmd = (1.0 - alpha) * self.cmd + alpha * self.cmd_target
        moving = float(np.linalg.norm(self.cmd)) > 0.005
        if moving:
            self.phase = (self.phase + dt / CYCLE) % 1.0
        else:
            self.phase = 0.0
            self.cmd *= 0.0
        z = None
        if self.have_odom and self.odom_pose is not None:
            z = self.odom_pose.pose.pose.position.z
        settled = z is not None and 0.48 < z < 0.66
        try:
            if not settled:
                self.phase = 0.0
                self.cmd *= 0.0
                command = stance_command(self.legs)
            else:
                command = joint_command(self.phase, self.cmd, self.measured, self.legs)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"步态逆解失败，保持上一帧: {exc}", throttle_duration_sec=2.0)
            command = self.last_command
        command = self.last_command + np.clip(command - self.last_command, -2.0 * dt, 2.0 * dt)
        self.last_command = command
        msg = Float64MultiArray()
        msg.data = [float(v) for v in command]
        self.pub.publish(msg)
        ready = Bool()
        ready.data = bool(self.have_joints and self.have_odom and settled and self.gravity[2] < -0.9)
        self.ready_pub.publish(ready)
        if self.have_odom and self.odom_pose is not None:
            z = self.odom_pose.pose.pose.position.z
            if z < 0.18:
                self.get_logger().error("机体高度过低，四足可能已经摔倒", throttle_duration_sec=2.0)


def main() -> None:
    rclpy.init()
    node = LocomotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
