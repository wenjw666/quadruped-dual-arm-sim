"""Pick, walk, and place. Arms are planned with MoveIt; the base uses cmd_vel."""

from __future__ import annotations

import math
import json
import sys
import threading
import time
import traceback
from pathlib import Path as FilePath

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from moveit_msgs.msg import (
    AttachedCollisionObject,
    AllowedCollisionEntry,
    BoundingVolume,
    CollisionObject,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    PlanningScene,
    PlanningSceneComponents,
    PositionConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetPlanningScene, GetStateValidity
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from qda_interfaces.srv import Attach, Detach
from qda_kinematics.kinematics import (
    AUBO_CARRY_LEFT,
    AUBO_HOME_LEFT,
    AUBO_HOME_RIGHT,
    AUBO_LIMITS,
    FINGER_OPEN,
    GRASP_FINGER,
    left_grasp_joints,
)
from qda_kinematics.scene import (
    SCENARIO, PLACE_YAW, PICK_YAW, ROUTE_WAYPOINTS, planning_boxes,
    APPROACH_IN_BASE,
    OBJECT_Z,
    OBJECT_SIZE,
    PLACE_MARKER_XYZ,
    PICK_OBJECT_XYZ,
    PLACE_TABLE_XYZ,
    PICK_TABLE_XYZ,
    RACK_SIZE,
    RACK_XYZ,
    TABLE_SIZE,
)

LEFT_ARM_JOINTS = [f"left_joint_link{index}" for index in range(1, 8)]
RIGHT_ARM_JOINTS = [f"right_joint_link{index}" for index in range(1, 8)]
LEFT_GRIPPER = ["left_finger_left_joint", "left_finger_right_joint"]
RIGHT_GRIPPER = ["right_finger_left_joint", "right_finger_right_joint"]


def _quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _yaw(rot: np.ndarray) -> float:
    return math.atan2(float(rot[1, 0]), float(rot[0, 0]))


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _set_allowed_pair(matrix, first: str, second: str, allowed: bool) -> None:
    """Extend the existing ACM without losing SRDF adjacent-link exclusions."""
    for name in (first, second):
        if name not in matrix.entry_names:
            matrix.entry_names.append(name)
            for row in matrix.entry_values:
                row.enabled.append(False)
            row = AllowedCollisionEntry()
            row.enabled = [False] * len(matrix.entry_names)
            matrix.entry_values.append(row)
    i, j = matrix.entry_names.index(first), matrix.entry_names.index(second)
    matrix.entry_values[i].enabled[j] = allowed
    matrix.entry_values[j].enabled[i] = allowed


class PickPlaceTask(Node):
    def __init__(self) -> None:
        super().__init__("qda_pick_place")
        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.joints = {}
        self.base_xyz = np.zeros(3)
        self.base_rot = np.eye(3)
        self.base_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.have_odom = False
        self.locomotion_ready = False
        self.pick_pose = None
        self.place_pose = None
        self.pick_stamp = None
        self.place_stamp = None
        self.labels = ""
        self.log_path = FilePath(self.declare_parameter('output_dir', '/ws/output').value) / 'task.log'
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log_path.write_text("", encoding="utf-8")
        from .run_metrics import RunMetrics
        self.metrics = RunMetrics(self.log_path.parent, self.declare_parameter('backend','mujoco').value)
        self.metrics_pub = self.create_publisher(String, '/task/metrics', 10)
        self.create_timer(1., lambda:self.metrics_pub.publish(String(data=json.dumps(self.metrics.snapshot(),ensure_ascii=False))))

        self.create_subscription(JointState, "/joint_states", self._on_joints, 20, callback_group=self.group)
        self.create_subscription(Odometry, "/odom", self._on_odom, 20, callback_group=self.group)
        self.create_subscription(Bool, "/locomotion/ready", self._on_ready, 10, callback_group=self.group)
        self.create_subscription(PoseStamped, "/perception/pick_pose", self._on_pick, 10, callback_group=self.group)
        self.create_subscription(PoseStamped, "/perception/place_pose", self._on_place, 10, callback_group=self.group)
        self.create_subscription(String, "/perception/labels", self._on_labels, 10, callback_group=self.group)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/task/goal", 10)
        self.nav_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose", callback_group=self.group
        )
        self._nav_handle = None
        self.state_pub = self.create_publisher(String, "/task/state", 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", qos)
        self.plan_client = self.create_client(GetMotionPlan, "/plan_kinematic_path", callback_group=self.group)
        self.scene_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene", callback_group=self.group)
        self.get_scene_client = self.create_client(GetPlanningScene, "/get_planning_scene", callback_group=self.group)
        self.validity_client = self.create_client(GetStateValidity, "/check_state_validity", callback_group=self.group)
        self.attach_client = self.create_client(Attach, "/qda_grasp/attach", callback_group=self.group)
        self.detach_client = self.create_client(Detach, "/qda_grasp/detach", callback_group=self.group)
        self.left_arm = ActionClient(
            self, FollowJointTrajectory, "/left_arm_controller/follow_joint_trajectory", callback_group=self.group
        )
        self.right_arm = ActionClient(
            self, FollowJointTrajectory, "/right_arm_controller/follow_joint_trajectory", callback_group=self.group
        )
        self.left_grip = ActionClient(
            self, FollowJointTrajectory, "/left_gripper_controller/follow_joint_trajectory", callback_group=self.group
        )
        self.finished = False
        self.declare_parameter("clock_stall_timeout", 30.0)

    def _note(self, text: str) -> None:
        self.metrics.event(text,self.get_clock().now().nanoseconds*1e-9)
        self.get_logger().info(text)
        with self.log_path.open("a", encoding="utf-8") as handle:
            handle.write(text + "\n")
        msg = String()
        msg.data = text
        self.state_pub.publish(msg)

    def _on_joints(self, msg: JointState) -> None:
        with self.lock:
            for name, pos in zip(msg.name, msg.position):
                self.joints[name] = pos

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        with self.lock:
            self.base_xyz = np.array([p.x, p.y, p.z])
            self.base_rot = _quat_to_rot(q.x, q.y, q.z, q.w)
            self.base_quat = np.array([q.x, q.y, q.z, q.w])
            self.have_odom = True
            yaw=_yaw(self.base_rot)
        if not getattr(self,'finished',False):
            self.metrics.odom(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,p.x,p.y,yaw)

    def _on_ready(self, msg: Bool) -> None:
        self.locomotion_ready = bool(msg.data)

    def _on_pick(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        with self.lock:
            self.pick_pose = np.array([p.x, p.y, p.z])
            self.pick_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _on_place(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        with self.lock:
            self.place_pose = np.array([p.x, p.y, p.z])
            self.place_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _on_labels(self, msg: String) -> None:
        self.labels = msg.data

    def _model_position(self, name: str):
        # The gz CLI blocks on this master. Entity state is served by gazebo_ros_state.
        from gazebo_msgs.srv import GetEntityState

        if not hasattr(self, "model_client"):
            self.model_client = self.create_client(
                GetEntityState, "/get_entity_state", callback_group=self.group
            )
        if not self.model_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("Gazebo 模型状态服务没有就绪")
        request = GetEntityState.Request()
        request.name = name
        request.reference_frame = "world"
        response = self._call(self.model_client, request, timeout=5.0)
        if response is None or not response.success:
            detail = "" if response is None else response.status_message
            raise RuntimeError(f"读不到 {name} 的位姿: {detail}")
        position = response.state.pose.position
        return float(position.x), float(position.y), float(position.z)

    def _stop(self) -> None:
        handle = self._nav_handle
        self._nav_handle = None
        if handle is not None:
            self._wait_future(handle.cancel_goal_async(), 5.0)
            self._wait_future(handle.get_result_async(), 5.0)
        self.cmd_pub.publish(Twist())

    def _wait_future(self, future, timeout: float, simulation: bool = False):
        clock = (lambda: self.get_clock().now().nanoseconds * 1e-9) if simulation else time.monotonic
        deadline = clock() + timeout
        previous = clock()
        last_progress = time.monotonic()
        while rclpy.ok() and not future.done():
            now = clock()
            if now < previous:
                return None  # A world reset invalidates in-flight actions.
            if now > previous:
                last_progress = time.monotonic()
            previous = now
            if now > deadline or time.monotonic() - last_progress > float(self.get_parameter("clock_stall_timeout").value):
                return None
            time.sleep(0.05)
        return future.result() if future.done() else None

    def _settle(self, seconds: float) -> None:
        """Allow physical settling in simulation seconds, even at a low RTF."""
        start = self.get_clock().now().nanoseconds * 1e-9
        last_sim = start
        last_progress = time.monotonic()
        while rclpy.ok():
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - start >= seconds:
                return
            if now < last_sim:
                raise RuntimeError("仿真时间已重置")
            if now > last_sim:
                last_progress = time.monotonic()
            last_sim = now
            if time.monotonic() - last_progress > float(self.get_parameter("clock_stall_timeout").value):
                raise RuntimeError("仿真时钟停止，无法完成动作")
            time.sleep(0.02)

    def _call(self, client, request, timeout: float = 20.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if client.service_is_ready():
                return self._wait_future(client.call_async(request), timeout)
            time.sleep(0.05)
        raise RuntimeError(f"服务不可用: {client.srv_name}")

    def _current(self, names):
        with self.lock:
            if not all(name in self.joints for name in names):
                return None
            return np.array([self.joints[name] for name in names], dtype=float)

    def _execute(self, client, names, trajectory: JointTrajectory) -> bool:
        if not client.wait_for_server(timeout_sec=20.0):
            self._note(f"轨迹控制器未就绪: {client._action_name}")
            return False
        ordered = list(trajectory.joint_names)
        index = [ordered.index(name) for name in names]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(names)
        for point in trajectory.points:
            cloned = JointTrajectoryPoint()
            cloned.positions = [point.positions[i] for i in index]
            if point.velocities:
                cloned.velocities = [point.velocities[i] for i in index]
            if point.accelerations:
                cloned.accelerations = [point.accelerations[i] for i in index]
            cloned.time_from_start = point.time_from_start
            goal.trajectory.points.append(cloned)
        send = client.send_goal_async(goal)
        handle = self._wait_future(send, 10.0)
        if handle is None or not handle.accepted:
            self._note("轨迹被控制器拒绝")
            return False
        end = trajectory.points[-1].time_from_start
        result = self._wait_future(handle.get_result_async(), end.sec + end.nanosec * 1e-9 + 10.0, simulation=True)
        if result is None:
            self._wait_future(handle.cancel_goal_async(), 5.0)
            self._wait_future(handle.get_result_async(), 5.0)
            self._note("轨迹超时或仿真时钟停止，已请求取消")
            raise RuntimeError("轨迹未在仿真时限内完成，停止任务以避免重叠动作")
        return result.status == GoalStatus.STATUS_SUCCEEDED and int(result.result.error_code) == 0

    def _gripper(self, width: float) -> bool:
        traj = JointTrajectory()
        traj.joint_names = list(LEFT_GRIPPER)
        point = JointTrajectoryPoint()
        point.positions = [width, width]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 400_000_000
        traj.points.append(point)
        return self._execute(self.left_grip, LEFT_GRIPPER, traj)

    def _plan_and_move(self, group: str, names, target: np.ndarray) -> bool:
        current = self._current(names)
        if current is not None and float(np.max(np.abs(current - target))) < 0.02:
            self._note(f"{group} 已在目标关节")
            return True
        for attempt in range(2):
            if self._plan_once(group, names, target):
                return True
            self._note(f"{group} 规划重试 {attempt + 1}")
            time.sleep(0.4)
        return False

    def _plan_once(self, group: str, names, target: np.ndarray) -> bool:
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = group
        motion.num_planning_attempts = 10
        motion.allowed_planning_time = 5.0
        motion.max_velocity_scaling_factor = 0.45
        motion.max_acceleration_scaling_factor = 0.45
        motion.pipeline_id = "ompl"
        motion.planner_id = "RRTConnect"
        motion.start_state.is_diff = True
        motion.workspace_parameters.header.frame_id = "base_link"
        motion.workspace_parameters.min_corner.x = -1.0
        motion.workspace_parameters.min_corner.y = -1.4
        motion.workspace_parameters.min_corner.z = -1.0
        motion.workspace_parameters.max_corner.x = 1.8
        motion.workspace_parameters.max_corner.y = 1.4
        motion.workspace_parameters.max_corner.z = 1.6
        goal = Constraints()
        for name, value, (lo, hi) in zip(names, target, AUBO_LIMITS):
            if not lo - 1e-3 <= float(value) <= hi + 1e-3:
                self._note(f"{name} 目标 {value:.3f} 超出限位")
                return False
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = float(value)
            constraint.tolerance_above = 0.015
            constraint.tolerance_below = 0.015
            constraint.weight = 1.0
            goal.joint_constraints.append(constraint)
        motion.goal_constraints.append(goal)
        started=time.monotonic()
        self.metrics.data['planning_calls']+=1
        response = self._call(self.plan_client, request, timeout=20.0)
        self.metrics.data['planning_wall_s']+=time.monotonic()-started
        if response is None:
            self.metrics.data['planning_failures']+=1
            self._note("MoveIt 规划超时")
            return False
        code = response.motion_plan_response.error_code.val
        if code != 1:
            self.metrics.data['planning_failures']+=1
            self._note(f"MoveIt 规划失败，错误码 {code}")
            return False
        trajectory = response.motion_plan_response.trajectory.joint_trajectory
        client = self.left_arm if group == "left_arm" else self.right_arm
        return self._execute(client, names, trajectory)

    def _grasp_family(self, point_base: np.ndarray, lifts) -> dict | None:
        """One vertical grasp, plus the same arm pose lifted straight up."""
        point = np.array(point_base, dtype=float)
        found = left_grasp_joints(point, count=24)
        if not found:
            self._note("抓取高度没有朝下的可解姿态")
            return None
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ROBOT_STATE | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        current = self._call(self.get_scene_client, request, timeout=5.0)
        if current is None:
            self._note("无法读取碰撞检查所需的机器人状态")
            return None
        state = current.scene.robot_state

        def valid(joints):
            values = dict(zip(state.joint_state.name, state.joint_state.position))
            values.update(zip(LEFT_ARM_JOINTS, (float(q) for q in joints)))
            state.joint_state.name = list(values)
            state.joint_state.position = list(values.values())
            check = GetStateValidity.Request()
            check.robot_state = state
            check.group_name = ""
            result = self._call(self.validity_client, check, timeout=5.0)
            if result is not None and not result.valid:self.metrics.data['ik_collision_rejections']+=1
            return result is not None and result.valid

        for seed in found:
            if not valid(seed):
                continue
            family = {0.0: seed}
            ok = True
            for lift in lifts:
                raised = point.copy()
                raised[2] += lift
                near = left_grasp_joints(raised, count=1, seed=seed)
                if not near or not valid(near[0]):
                    ok = False
                    break
                family[lift] = near[0]
            if ok:
                return family
        self._note("无法在同一条手臂姿态上抬高")
        return None

    def _move_grasp(self, family: dict, lift: float, label: str) -> bool:
        joints = family[lift]
        self._note(f"{label} 关节 {np.round(joints, 2).tolist()}")
        return self._plan_and_move("left_arm", LEFT_ARM_JOINTS, joints)

    def _plan_pose(self, group: str, link: str, point_base: np.ndarray, z_offset: float) -> bool:
        point = np.array(point_base, dtype=float)
        point[2] += z_offset
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = group
        motion.num_planning_attempts = 8
        motion.allowed_planning_time = 3.0
        motion.max_velocity_scaling_factor = 0.4
        motion.max_acceleration_scaling_factor = 0.4
        motion.pipeline_id = "ompl"
        motion.planner_id = "RRTConnect"
        motion.start_state.is_diff = True
        motion.workspace_parameters.header.frame_id = "base_link"
        motion.workspace_parameters.min_corner.x = -1.2
        motion.workspace_parameters.min_corner.y = -1.2
        motion.workspace_parameters.min_corner.z = -0.5
        motion.workspace_parameters.max_corner.x = 1.6
        motion.workspace_parameters.max_corner.y = 1.2
        motion.workspace_parameters.max_corner.z = 1.6
        goal = Constraints()
        position = PositionConstraint()
        position.header.frame_id = "base_link"
        position.link_name = link
        position.weight = 1.0
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.02]
        region = BoundingVolume()
        region.primitives.append(sphere)
        pose = Pose()
        pose.position.x = float(point[0])
        pose.position.y = float(point[1])
        pose.position.z = float(point[2])
        pose.orientation.w = 1.0
        region.primitive_poses.append(pose)
        position.constraint_region = region
        goal.position_constraints.append(position)
        motion.goal_constraints.append(goal)
        response = self._call(self.plan_client, request, timeout=15.0)
        if response is None:
            self._note("MoveIt 位姿规划超时")
            return False
        code = response.motion_plan_response.error_code.val
        if code != 1:
            self._note(f"MoveIt 位姿规划失败，错误码 {code}")
            return False
        trajectory = response.motion_plan_response.trajectory.joint_trajectory
        names = LEFT_ARM_JOINTS if group == "left_arm" else RIGHT_ARM_JOINTS
        client = self.left_arm if group == "left_arm" else self.right_arm
        return self._execute(client, names, trajectory)

    def _publish_scene(self) -> None:
        with self.lock:
            origin = self.base_xyz.copy()
            rot = self.base_rot.copy()
            quat = self.base_quat.copy()
        scene = PlanningScene()
        scene.is_diff = True
        obstacles = planning_boxes()
        for name, center, size, slab in obstacles:
            world = np.array(center, dtype=float)
            size = np.array(size, dtype=float)
            if slab:
                # The arm reaches over the bench. A solid box fills the space
                # under the top and rejects every downward goal.
                world[2] = world[2] + size[2] / 2.0 - 0.02
                size[2] = 0.04
            local = rot.T @ (world - origin)
            obj = CollisionObject()
            obj.header.frame_id = "base_link"
            obj.id = name
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [float(size[0]), float(size[1]), float(size[2])]
            obj.primitives.append(primitive)
            pose = Pose()
            pose.position.x = float(local[0])
            pose.position.y = float(local[1])
            pose.position.z = float(local[2])
            # Axis-aligned world boxes rotate by the inverse base orientation.
            quat /= np.linalg.norm(quat)
            pose.orientation.x, pose.orientation.y, pose.orientation.z = (-quat[:3]).tolist()
            pose.orientation.w = float(quat[3])
            obj.primitive_poses.append(pose)
            obj.operation = CollisionObject.ADD
            scene.world.collision_objects.append(obj)
        request = ApplyPlanningScene.Request()
        request.scene = scene
        if self.scene_client.wait_for_service(timeout_sec=5.0):
            self._call(self.scene_client, request, timeout=8.0)
        else:
            self.scene_pub.publish(scene)

    def _world_from_base(self, point_base: np.ndarray) -> np.ndarray:
        with self.lock:
            return self.base_xyz + self.base_rot @ point_base

    def _payload_scene(self, attached: bool) -> None:
        """Keep MoveIt's carried-object collision geometry in sync with the backend."""
        item = AttachedCollisionObject()
        item.link_name = "left_link7"
        item.touch_links = ["left_link7", "left_gripper", "left_finger_left", "left_finger_right"]
        item.object.id = "carried_cube"
        item.object.header.frame_id = "base_link"
        item.object.operation = CollisionObject.ADD if attached else CollisionObject.REMOVE
        if attached:
            point = self._base_from_world(np.array(self._model_position("pick_object")))
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [OBJECT_SIZE] * 3
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = point.tolist()
            pose.orientation.w = 1.0
            item.object.primitives.append(box)
            item.object.primitive_poses.append(pose)
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(item)
        request = ApplyPlanningScene.Request()
        request.scene = scene
        response = self._call(self.scene_client, request, timeout=8.0)
        if response is None or not response.success:
            raise RuntimeError("MoveIt 抓取物碰撞模型更新失败")

    def _refresh_released_object(self) -> None:
        # Detaching in MoveIt transfers the object into the world at the last
        # attached pose. Refresh after the physical cube has settled on its
        # support; otherwise the stale cube can intersect an opened finger.
        point = self._base_from_world(np.array(self._model_position('pick_object')))
        obj = CollisionObject()
        obj.id = 'carried_cube'
        obj.header.frame_id = 'base_link'
        obj.operation = CollisionObject.ADD
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[OBJECT_SIZE]*3)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = point.tolist()
        pose.orientation.w = 1.
        obj.primitives = [box]
        obj.primitive_poses = [pose]
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = [obj]
        response = self._call(self.scene_client, ApplyPlanningScene.Request(scene=scene), timeout=8.)
        if response is None or not response.success:
            raise RuntimeError('释放物体碰撞位置更新失败')

    def _withdraw_after_release(self, family) -> None:
        touch = ['left_link7','left_gripper','left_finger_left','left_finger_right']
        for link in touch:
            self._allow_payload_contact(link, True)
        if not self._gripper(FINGER_OPEN):
            raise RuntimeError('夹爪打开失败')
        self._settle(0.35)
        self._refresh_released_object()
        # First retrace the same approach branch, then move to home. Only the
        # intentional cube/gripper release contact is allowed during this step.
        if not self._move_grasp(family, 0.08, '释放后抬腕撤离'):
            raise RuntimeError('释放后抬腕规划失败')
        for link in touch:
            self._allow_payload_contact(link, False)
        self._refresh_released_object()
        if not self._plan_and_move('left_arm', LEFT_ARM_JOINTS, np.array(AUBO_HOME_LEFT)):
            raise RuntimeError('撤离规划失败')

    def _allow_payload_contact(self, table: str, allowed: bool) -> None:
        # Touching the support surface is intentional during lift/placement.
        # Keep all other payload/environment collision checks enabled.
        scene = PlanningScene()
        scene.is_diff = True
        get_request = GetPlanningScene.Request()
        get_request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        current = self._call(self.get_scene_client, get_request, timeout=8.0)
        if current is None:
            raise RuntimeError("无法读取现有碰撞规则")
        scene.allowed_collision_matrix = current.scene.allowed_collision_matrix
        _set_allowed_pair(scene.allowed_collision_matrix, "carried_cube", table, allowed)
        request = ApplyPlanningScene.Request()
        request.scene = scene
        response = self._call(self.scene_client, request, timeout=8.0)
        if response is None or not response.success:
            raise RuntimeError("抓取物与支撑台的接触规则更新失败")

    def _base_from_world(self, point_world: np.ndarray) -> np.ndarray:
        with self.lock:
            return self.base_rot.T @ (point_world - self.base_xyz)

    def _walk_to_world(self, target_xy: np.ndarray, timeout: float = 300.0, yaw_goal: float = 0.0, checkpoint: bool = False) -> bool:
        """Drive to the arm approach pose with Nav2. Yaw 0 keeps the left arm toward +x."""
        c, s = math.cos(yaw_goal), math.sin(yaw_goal)
        approach = np.array([[c, -s], [s, c]]) @ np.array(APPROACH_IN_BASE[:2])
        # Navigate to a staging pose with extra bench clearance, then use the
        # existing low-speed alignment for the final tool reach.
        if not checkpoint:approach += .15*np.array([c,s])
        goal_xy = np.array(target_xy, dtype=float) - (0. if checkpoint else approach)
        nav_start=self.get_clock().now().nanoseconds*1e-9
        if not self.nav_client.wait_for_server(timeout_sec=40.0):
            self._note("Nav2 动作服务器没有就绪")
            return False
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(goal_xy[0])
        pose.pose.position.y = float(goal_xy[1])
        pose.pose.orientation.z = math.sin(yaw_goal * 0.5)
        pose.pose.orientation.w = math.cos(yaw_goal * 0.5)
        self.goal_pub.publish(pose)
        self._note(f"Nav2 前往 [{goal_xy[0]:.2f}, {goal_xy[1]:.2f}] 朝向 {yaw_goal:.2f}")
        goal = NavigateToPose.Goal()
        goal.pose = pose
        handle = self._wait_future(self.nav_client.send_goal_async(goal), 20.0)
        if handle is None or not handle.accepted:
            self._note("Nav2 拒绝了目标")
            return False
        self._nav_handle = handle
        wrapped = self._wait_future(handle.get_result_async(), timeout, simulation=True)
        if wrapped is None:
            self._stop()
            self._note("Nav2 超时或时钟停止，已取消导航")
            return False
        self._nav_handle = None
        status = int(wrapped.status)
        self.cmd_pub.publish(Twist())
        self._settle(0.5)
        with self.lock:
            pos = self.base_xyz.copy()
            yaw = _yaw(self.base_rot)
        dist = float(np.linalg.norm(goal_xy - pos[:2]))
        yaw_err = abs(_wrap(yaw_goal - yaw))
        self.metrics.data['nav_legs'].append(dict(goal=[*goal_xy.tolist(),yaw_goal],checkpoint=checkpoint,status=status,
             error_m=dist,yaw_error_rad=yaw_err,sim_s=self.get_clock().now().nanoseconds*1e-9-nav_start))
        self._note(f"Nav2 状态 {status}，偏差 {dist:.3f} m，朝向差 {yaw_err:.2f} rad")
        if status != GoalStatus.STATUS_SUCCEEDED or pos[2] < 0.42 or dist > 0.15 or yaw_err > 0.3:
            return False
        return True if checkpoint else self._trim_approach(np.array(target_xy, dtype=float), yaw_goal)

    def _trim_approach(self, target_xy: np.ndarray, yaw_goal: float, stand_off: float = 0.0) -> bool:
        """Body-frame creep. The vertical grasp only solves near APPROACH_IN_BASE."""
        approach = np.array(APPROACH_IN_BASE[:2], dtype=float)
        approach[0] += stand_off
        deadline = self.get_clock().now().nanoseconds * 1e-9 + 30.0
        while rclpy.ok() and self.get_clock().now().nanoseconds * 1e-9 < deadline:
            with self.lock:
                pos = self.base_xyz.copy()
                rot = self.base_rot.copy()
            if pos[2] < 0.42:
                self.cmd_pub.publish(Twist())
                return False
            yaw = _yaw(rot)
            local = rot.T @ (np.array([target_xy[0], target_xy[1], pos[2]]) - pos)
            miss = local[:2] - approach
            yaw_err = _wrap(yaw_goal - yaw)
            if float(np.linalg.norm(miss)) < 0.025 and abs(yaw_err) < 0.08:
                self.cmd_pub.publish(Twist())
                self._note(f"接近位已对齐，机体系偏差 {np.round(miss, 3).tolist()}")
                self._settle(0.5)
                return True
            cmd = Twist()
            cmd.linear.x = float(np.clip(0.7 * miss[0], -0.04, 0.12))
            cmd.linear.y = float(np.clip(0.7 * miss[1], -0.08, 0.08))
            cmd.angular.z = float(np.clip(0.9 * yaw_err, -0.35, 0.35))
            self.cmd_pub.publish(cmd)
            self._settle(0.1)
        self.cmd_pub.publish(Twist())
        self._note("接近位微调超时")
        return False

    def _settle_grasp(self, world_point: np.ndarray, lifts, hover: float, hover_label: str, down_label: str):
        """Retry a top grasp from nearby points when one IK family cannot descend."""
        nudges = (
            (0.0, 0.0, 0.0),
            (0.015, 0.0, 0.0),
            (-0.015, 0.0, 0.0),
            (0.0, 0.015, 0.0),
            (0.0, -0.015, 0.0),
        )
        for dx, dy, dz in nudges:
            shifted = np.array(world_point, dtype=float) + np.array([dx, dy, dz])
            family = self._grasp_family(self._fresh_base_target(shifted), lifts)
            if family is None:
                continue
            if self._move_grasp(family, hover, hover_label) and self._move_grasp(family, 0.0, down_label):
                return family
            self._note(f"{down_label}未成，换一个接近点")
        return None

    def _wait_until_ready(self) -> None:
        deadline = time.monotonic() + 150.0
        next_note = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            joints = self._current(LEFT_ARM_JOINTS)
            plan_ready = self.plan_client.service_is_ready()
            nav_ready = self.nav_client.server_is_ready()
            if joints is not None and self.have_odom and self.locomotion_ready and plan_ready and nav_ready:
                return
            now = time.monotonic()
            if now >= next_note:
                self._note(
                    f"就绪检查 关节={joints is not None} 里程计={self.have_odom} "
                    f"运控={self.locomotion_ready} 规划={plan_ready} 导航={nav_ready}"
                )
                next_note = now + 5.0
            time.sleep(0.2)
        raise RuntimeError("控制器、里程计或 MoveIt 没有在 150 秒内就绪")

    def _sense_pick(self) -> np.ndarray:
        deadline = time.monotonic() + 12.0
        while rclpy.ok() and time.monotonic() < deadline:
            with self.lock:
                pose = None if self.pick_pose is None else self.pick_pose.copy()
                stamp = self.pick_stamp
                labels = self.labels
            age = float('inf') if stamp is None else self.get_clock().now().nanoseconds * 1e-9 - stamp
            if pose is not None and 0.0 <= age <= 0.5 and np.all(np.isfinite(pose)):
                world = self._world_from_base(pose)
                if abs(world[0] - PICK_OBJECT_XYZ[0]) < 0.35 and abs(world[1] - PICK_OBJECT_XYZ[1]) < 0.35:
                    self._note(f"感知到抓取目标 {pose.round(3).tolist()}，视觉标签 {labels}")
                    return world
            time.sleep(0.2)
        fallback = np.array(PICK_OBJECT_XYZ, dtype=float)
        self._note(f"视觉没有稳定锁定红块，使用工位坐标 {fallback.tolist()}")
        return fallback

    def _sense_place(self) -> np.ndarray:
        deadline = time.monotonic() + 8.0
        while rclpy.ok() and time.monotonic() < deadline:
            with self.lock:
                pose = None if self.place_pose is None else self.place_pose.copy()
                stamp = self.place_stamp
            age = float('inf') if stamp is None else self.get_clock().now().nanoseconds * 1e-9 - stamp
            if pose is not None and 0.0 <= age <= 0.5 and np.all(np.isfinite(pose)):
                world = self._world_from_base(pose)
                world[2] = OBJECT_Z
                if abs(world[0] - PLACE_MARKER_XYZ[0]) < 0.4 and abs(world[1] - PLACE_MARKER_XYZ[1]) < 0.4:
                    self._note(f"感知到放置标记，使用其平面位置 {world.round(3).tolist()}")
                    return world
            time.sleep(0.2)
        fallback = np.array([PLACE_MARKER_XYZ[0], PLACE_MARKER_XYZ[1], OBJECT_Z], dtype=float)
        self._note(f"未看到绿色标记，使用放置工位 {fallback.tolist()}")
        return fallback

    def _fresh_base_target(self, world_point: np.ndarray) -> np.ndarray:
        return self._base_from_world(world_point)

    def run(self) -> None:
        try:
            self._run_impl()
        except Exception as exc:  # noqa: BLE001
            self._stop()
            self._note(f"TASK_FAILED {exc}")
        finally:
            self.finished = True
            self.metrics.finish(self.log_path)

    def _run_impl(self) -> None:
        self._note(f"场景 {SCENARIO}，等待仿真、运控和 MoveIt")
        self._wait_until_ready()
        time.sleep(1.0)
        self._publish_scene()
        self._note("双臂收到 MoveIt 关节目标，先收到预备姿态")
        if not self._plan_and_move("right_arm", RIGHT_ARM_JOINTS, np.array(AUBO_HOME_RIGHT)):
            raise RuntimeError("右臂预备规划失败")
        if not self._plan_and_move("left_arm", LEFT_ARM_JOINTS, np.array(AUBO_HOME_LEFT)):
            raise RuntimeError("左臂预备规划失败")
        if not self._gripper(FINGER_OPEN):
            raise RuntimeError("左夹爪打开失败")

        pick_world = self._sense_pick()
        pick_base = self._base_from_world(pick_world)
        reach_miss = float(np.linalg.norm(pick_base[:2] - np.array(APPROACH_IN_BASE[:2])))
        # The downward grasp is solved for the approach offset. A stance that is
        # only 10 cm short puts the cube outside that solution, so walk anyway.
        if reach_miss > 0.05:
            self._note(f"抓取点偏离接近位 {reach_miss:.2f} m，先走到臂展内")
            if not self._walk_to_world(pick_world[:2],yaw_goal=PICK_YAW):
                raise RuntimeError("没能走到抓取工位")
        else:
            self._note("抓取目标已经在左臂工作空间内")
        self._stop()
        self._publish_scene()
        self._note(f"左臂按指垫逆解，世界坐标 {pick_world.round(3).tolist()}")
        pick_family = self._settle_grasp(pick_world, (0.08, 0.12), 0.08, "预抓取", "下降")
        if pick_family is None:
            raise RuntimeError("下降规划失败")
        if not self._gripper(GRASP_FINGER):
            raise RuntimeError("夹爪闭合失败")
        attach = Attach.Request()
        attach.parent_model = "qda_robot"
        # Gazebo merges the fixed gripper into the last revolute link.
        attach.parent_link = "left_link7"
        attach.child_model = "pick_object"
        attach.child_link = "object_link"
        attached = self._call(self.attach_client, attach, timeout=8.0)
        if attached is None or not attached.success:
            raise RuntimeError(f"吸附失败: {getattr(attached, 'message', '')}")
        self._note(attached.message)
        self._payload_scene(True)
        self._allow_payload_contact("pick_table", True)
        if not self._move_grasp(pick_family, 0.12, "抬起"):
            raise RuntimeError("抬起规划失败")
        self._allow_payload_contact("pick_table", False)

        self._note("搬运前收回左臂，减少前伸范围")
        if not self._plan_and_move("left_arm", LEFT_ARM_JOINTS, np.array(AUBO_CARRY_LEFT)):
            raise RuntimeError("搬运收臂规划失败")

        # The manipulation pose is intentionally closer than a 5 cm costmap
        # cell can resolve. Retrace the fine approach before returning control
        # to MPPI, so its first rollout starts with clear footprint geometry.
        self._note("收臂后沿原接近方向退回预接近位")
        if not self._trim_approach(pick_world[:2],PICK_YAW,stand_off=.15):
            raise RuntimeError("退出抓取工位失败")

        place_world = self._sense_place()
        self._note("抬起完成，走向放置工位")
        for index,(x,y,yaw) in enumerate(ROUTE_WAYPOINTS):
            self._note(f"路线检查点 {index+1}/{len(ROUTE_WAYPOINTS)}")
            if not self._walk_to_world(np.array([x,y]),yaw_goal=yaw,checkpoint=True):
                raise RuntimeError(f"路线检查点 {index+1} 导航失败")
        if not self._walk_to_world(place_world[:2], timeout=300.0,yaw_goal=PLACE_YAW):
            raise RuntimeError("没能走到放置工位")
        self._stop()
        self._settle(0.6)
        place_world = self._sense_place()
        self._publish_scene()
        self._allow_payload_contact("place_table", True)
        self._note(f"左臂按指垫逆解放置，世界坐标 {place_world.round(3).tolist()}")
        place_family = self._settle_grasp(place_world, (0.08,), 0.08, "放置上方", "放置下降")
        if place_family is None:
            raise RuntimeError("放置下降规划失败")
        detach = Detach.Request()
        detach.child_model = "pick_object"
        detached = self._call(self.detach_client, detach, timeout=8.0)
        if detached is None or not detached.success:
            raise RuntimeError("松开失败")
        self._note(detached.message)
        self._payload_scene(False)
        self._allow_payload_contact("place_table", False)
        self._withdraw_after_release(place_family)
        self._stop()
        self._settle(0.8)
        got = self._model_position("pick_object")
        err = math.sqrt((got[0] - place_world[0]) ** 2 + (got[1] - place_world[1]) ** 2 + (got[2] - OBJECT_Z) ** 2)
        self.metrics.data['final_error_m']=err
        self._note(
            f"物体真值 ({got[0]:.3f}, {got[1]:.3f}, {got[2]:.3f})，与放置点距离 {err:.3f} m"
        )
        if err > 0.12 or got[2] < 0.2:
            raise RuntimeError("物体没有留在放置点上")
        self._note("TASK_SUCCESS")


def main() -> None:
    crash_path = FilePath("/ws/output/task_crash.log")

    def _crash(exc_type, exc, tb) -> None:
        crash_path.write_text("".join(traceback.format_exception(exc_type, exc, tb)), encoding="utf-8")
        sys.__excepthook__(exc_type, exc, tb)

    sys.excepthook = _crash
    threading.excepthook = lambda args: _crash(args.exc_type, args.exc_value, args.exc_traceback)
    try:
        rclpy.init()
        node = PickPlaceTask()
        crash_path = node.log_path.with_name('task_crash.log')
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        worker = threading.Thread(target=node.run, daemon=True)
        worker.start()
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        _crash(*sys.exc_info())
        raise
