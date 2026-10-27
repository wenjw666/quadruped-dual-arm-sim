# ROS 2 现场节点与接口完整清单

采集时间（原始 UTC）：`2026-10-27T10:54:00.221770+00:00`。容器 `qda-dev`，ROS_DOMAIN_ID=`8`。
只读采样 20.002 秒，发现 **37 个节点、152 个话题**；无重复节点名；采集探针自身已排除。

本轮启动兼容的 MuJoCo RL、Nav2、MoveIt、双臂 Servo、感知、建图、大脑、任务协调与 RViz 主栈，未下发运动任务。全部功能同时待命不代表互斥动作同时执行。

[架构、功能边界与启动方法](ROS2_NODE_NETWORK.md) · [原始 JSON](../output/ros2_graph/ros2-live-graph.json) · [实际 rqt 全图](../output/ros2_graph/rqt-full-native.png)

## 全部现场节点

P/S 是该节点的发布/订阅话题数；包含 /clock、日志、参数事件与 Action 反馈等框架接口，不能当作算法数量。动态数字后缀会随重启变化。

| 节点 | 功能 | P / S | Action 服务 / 客户端数 |
|---|---|---:|---:|
| `/algorithm_view` | 算法状态、轨迹、地图与识别结果的 RViz 展示汇总 | 10 / 21 | 0 / 0 |
| `/behavior_server` | Nav2 旋转、后退、等待、沿朝向移动等恢复行为 | 15 / 7 | 5 / 0 |
| `/bt_navigator` | 导航行为树入口，接收单目标和多目标导航 Action | 8 / 9 | 2 / 1 |
| `/bt_navigator_navigate_through_poses_rclcpp_node` | 多目标导航行为树内部 Action 客户端 | 3 / 12 | 0 / 5 |
| `/bt_navigator_navigate_to_pose_rclcpp_node` | 单目标导航行为树内部 Action 客户端 | 3 / 12 | 0 / 5 |
| `/controller_server` | MPPI 局部路径跟踪，发布 /cmd_vel_nav | 9 / 5 | 1 / 0 |
| `/global_costmap/global_costmap` | 全局代价地图，服务全局规划 | 7 / 4 | 0 / 0 |
| `/lifecycle_manager_navigation` | Nav2 生命周期配置、激活与监控 | 4 / 3 | 0 / 0 |
| `/local_brain` | 大脑请求入口，汇集反馈，调用外部 VLM，校验并下发受限技能提案 | 4 / 11 | 0 / 1 |
| `/local_costmap/local_costmap` | 局部代价地图，服务路径跟踪与恢复 | 7 / 5 | 0 / 0 |
| `/mission_coordinator` | 显式多步骤任务协调、幂等执行、重试与持久化 | 3 / 3 | 0 / 1 |
| `/move_group` | MoveIt 运动规划、规划场景与轨迹执行入口 | 12 / 4 | 2 / 0 |
| `/move_group_private_101957545829184` | MoveIt 内部规划场景/状态监视辅助节点 | 3 / 9 | 0 / 0 |
| `/moveit_simple_controller_manager` | MoveIt 四组 FollowJointTrajectory 客户端 | 2 / 10 | 0 / 4 |
| `/planner_server` | SmacPlanner2D 全局路径规划 | 10 / 3 | 2 / 0 |
| `/qda_command_mux` | 底盘速度仲裁、控制租约与停稳保护，唯一 /cmd_vel 发布者 | 4 / 5 | 0 / 0 |
| `/qda_frame_dump` | 相机与识别图像文件保存 | 2 / 3 | 0 / 0 |
| `/qda_mapper` | 基于真值里程计的激光栅格和彩色 RGB-D 地图累积 | 9 / 9 | 0 / 0 |
| `/qda_mujoco` | MuJoCo 物理世界、内部 RL/PD 运控、接触抓放、传感器与轨迹执行 | 36 / 4 | 4 / 0 |
| `/qda_perception` | RGB-D 几何/颜色目标检测、异步 YOLO 标签与目标位姿发布 | 7 / 6 | 0 / 0 |
| `/robot_skill_server` | 导航、观察、机械臂、视觉对准与物理抓放技能 Action 服务 | 8 / 10 | 1 / 3 |
| `/robot_state_publisher` | URDF 和关节状态转换为机器人 TF | 5 / 3 | 0 / 0 |
| `/rviz2` | 机器人、地图、点云、图像、路径与状态显示 | 3 / 21 | 0 / 0 |
| `/servo_left` | MoveIt Servo 左臂 Twist → 关节轨迹 | 5 / 5 | 0 / 0 |
| `/servo_left_private_100100294198128` | 左臂 Servo 内部规划场景/状态监视辅助节点 | 3 / 9 | 0 / 0 |
| `/servo_right` | MoveIt Servo 右臂 Twist → 关节轨迹 | 5 / 5 | 0 / 0 |
| `/servo_right_private_97971473235024` | 右臂 Servo 内部规划场景/状态监视辅助节点 | 3 / 9 | 0 / 0 |
| `/static_transform_publisher_yNrgMX9cwEVchBXE` | 固定 world → odom 变换发布 | 3 / 2 | 0 / 0 |
| `/transform_listener_impl_5b0a6aff15a0` | tf2 内部变换监听辅助节点；仅凭发现图不能确定所属进程 | 1 / 4 | 0 / 0 |
| `/transform_listener_impl_5cbad7cc10c0` | tf2 内部变换监听辅助节点；仅凭发现图不能确定所属进程 | 1 / 4 | 0 / 0 |
| `/transform_listener_impl_5cbadbbb0840` | tf2 内部变换监听辅助节点；仅凭发现图不能确定所属进程 | 1 / 4 | 0 / 0 |
| `/transform_listener_impl_5d6bcaeeef70` | tf2 内部变换监听辅助节点；仅凭发现图不能确定所属进程 | 1 / 4 | 0 / 0 |
| `/transform_listener_impl_606df173ad80` | tf2 内部变换监听辅助节点；仅凭发现图不能确定所属进程 | 1 / 4 | 0 / 0 |
| `/transform_listener_impl_70cc24021750` | tf2 内部变换监听辅助节点；仅凭发现图不能确定所属进程 | 1 / 4 | 0 / 0 |
| `/transform_listener_impl_71cef4029ea0` | tf2 内部变换监听辅助节点；仅凭发现图不能确定所属进程 | 1 / 4 | 0 / 0 |
| `/transform_listener_impl_7888100097b0` | tf2 内部变换监听辅助节点；仅凭发现图不能确定所属进程 | 1 / 4 | 0 / 0 |
| `/visual_servo` | 腕部视觉 ArUco/PnP/KLT/IBVS 对准，提供 VisualAlign Action | 8 / 11 | 1 / 0 |

## 生命周期与运行反馈

| 对象 | 实测结果 |
|---|---|
| `/controller_server` | `active` |
| `/planner_server` | `active` |
| `/behavior_server` | `active` |
| `/bt_navigator` | `active` |
| `/local_costmap/local_costmap` | `active` |
| `/global_costmap/global_costmap` | `active` |
| RL | ready=`true`，fault=``，policy_ticks 1366 → 1521，physics_ticks 13652 → 15202 |
| 接触与姿态 | 四足接触 `[1.0, 1.0, 1.0, 1.0]`，高度 0.495 m |
| 彩色地图 | 12184 点、累计 60 帧，配准来源 `simulation_truth_RGBD` |
| 本地 VLM HTTP 服务 | `{'status': 'ok'}`；健康检查成功不等于本轮执行了推理 |
| 大脑健康 | skill_server=True，mission_server=True，front_camera_fresh=True，physical_grasp_ready=True |
| 自主能力标志 | model_autonomous_ready=False；代码保留的能力限制，不能解释为无约束自主规划已完成 |

RL 状态中的 `runtime_ready=false` 是初始化绑定元数据；当前运行状态应看 `ready=true`、空 fault 与持续推进的 tick。

唯一底盘指令发布者为 `/qda_command_mux`，`/cmd_vel` 的订阅者为 `/qda_mujoco` 与 `/algorithm_view`。本次停留在 idle，指令为零。

本次 33 个采样话题中 27 个收到消息。相机与腕部图像、点云、栅格、关节、IMU、运控、抓放状态、识别与大脑健康均有实际消息。

全渲染、RViz 和采图并行时，所收到 /clock 首末值推进 3.100 仿真秒，采样墙钟约 20.002 秒（粗略比值 0.15）。这次图谱审计运行速度低于实时，不能声称已达到实时性能；此数也不是独立标准性能基准。

## 消息采样明细

频率为本探针收到的消息数 / 墙钟采样时间，含发现延迟、QoS、末值保留与渲染负载影响，不是发布器配置频率。

| 话题 | 接收数 | 墙钟 Hz |
|---|---:|---:|
| `/clock` | 311 | 15.548 |
| `/odom` | 156 | 7.799 |
| `/joint_states` | 156 | 7.799 |
| `/imu` | 516 | 25.797 |
| `/scan` | 32 | 1.600 |
| `/livox/lidar_PointCloud2` | 32 | 1.600 |
| `/camera/image_raw` | 15 | 0.750 |
| `/camera/depth/image_raw` | 21 | 1.050 |
| `/cameras/left_wrist/image_raw` | 16 | 0.800 |
| `/cameras/right_wrist/image_raw` | 16 | 0.800 |
| `/mapping/rgbd_observation` | 5 | 0.250 |
| `/mapping/colored_global` | 1 | 0.050 |
| `/mapping/color_status` | 1 | 0.050 |
| `/map` | 7 | 0.350 |
| `/locomotion/rl_status` | 42 | 2.100 |
| `/motion/command_owner` | 96 | 4.799 |
| `/motion/arm_modes` | 42 | 2.100 |
| `/manipulation/status` | 42 | 2.100 |
| `/perception/labels` | 6 | 0.300 |
| `/perception/pick_pose` | 6 | 0.300 |
| `/perception/place_pose` | 0 | 0.000 |
| `/perception/debug` | 4 | 0.200 |
| `/brain/status` | 39 | 1.950 |
| `/brain/health` | 18 | 0.900 |
| `/brain/mission_status` | 41 | 2.050 |
| `/brain/decision_trace` | 0 | 0.000 |
| `/brain/servo_metrics` | 0 | 0.000 |
| `/viz/status_image` | 8 | 0.400 |
| `/cmd_vel` | 981 | 49.045 |
| `/cmd_vel_nav` | 0 | 0.000 |
| `/servo_left/status` | 0 | 0.000 |
| `/servo_right/status` | 0 | 0.000 |
| `/monitored_planning_scene` | 32 | 1.600 |

零消息必须结合触发条件解读：本次未提交导航、决策或视觉对准任务，因此 `/cmd_vel_nav`、`/brain/decision_trace`、`/brain/servo_metrics`、两侧 Servo 状态为空，不能据此认定接口断开，也不能认定闭环任务已通过。本次 `/perception/place_pose` 没有检测输出，不能声称放置目标识别已验证。

`/brain/mission_status` 发布了 SQLite 持久化恢复的历史 SUCCEEDED 任务；不是本次启动执行成功的证明。当前物理抓放状态仍是 idle。

## 实测 Action 服务与调用关系

| 节点 | 提供的 Action | 调用的 Action |
|---|---|---|
| `/behavior_server` | `/assisted_teleop`<br>`/backup`<br>`/drive_on_heading`<br>`/spin`<br>`/wait` | — |
| `/bt_navigator` | `/navigate_through_poses`<br>`/navigate_to_pose` | `/navigate_to_pose` |
| `/bt_navigator_navigate_through_poses_rclcpp_node` | — | `/backup`<br>`/compute_path_through_poses`<br>`/follow_path`<br>`/spin`<br>`/wait` |
| `/bt_navigator_navigate_to_pose_rclcpp_node` | — | `/backup`<br>`/compute_path_to_pose`<br>`/follow_path`<br>`/spin`<br>`/wait` |
| `/controller_server` | `/follow_path` | — |
| `/local_brain` | — | `/robot_skill` |
| `/mission_coordinator` | — | `/robot_skill` |
| `/move_group` | `/execute_trajectory`<br>`/move_action` | — |
| `/moveit_simple_controller_manager` | — | `/left_arm_controller/follow_joint_trajectory`<br>`/left_gripper_controller/follow_joint_trajectory`<br>`/right_arm_controller/follow_joint_trajectory`<br>`/right_gripper_controller/follow_joint_trajectory` |
| `/planner_server` | `/compute_path_through_poses`<br>`/compute_path_to_pose` | — |
| `/qda_mujoco` | `/left_arm_controller/follow_joint_trajectory`<br>`/left_gripper_controller/follow_joint_trajectory`<br>`/right_arm_controller/follow_joint_trajectory`<br>`/right_gripper_controller/follow_joint_trajectory` | — |
| `/robot_skill_server` | `/robot_skill` | `/execute_trajectory`<br>`/navigate_to_pose`<br>`/visual_align` |
| `/visual_servo` | `/visual_align` | — |

## 主要业务 Service

不含通用参数、生命周期和 Action 协议内部服务。完整服务及类型保留在原始 JSON。

| 服务 | 实测类型 |
|---|---|
| `/brain/request` | qda_interfaces/srv/BrainRequest |
| `/manipulation/command` | qda_interfaces/srv/BrainRequest |
| `/mapping/save_color` | std_srvs/srv/Trigger |
| `/mission/request` | qda_interfaces/srv/BrainRequest |
| `/motion/hold` | std_srvs/srv/SetBool |
| `/motion/lease` | qda_interfaces/srv/BrainRequest |
| `/servo_left/change_control_dimensions` | moveit_msgs/srv/ChangeControlDimensions |
| `/servo_left/change_drift_dimensions` | moveit_msgs/srv/ChangeDriftDimensions |
| `/servo_left/pause_servo` | std_srvs/srv/Trigger |
| `/servo_left/reset_servo_status` | std_srvs/srv/Empty |
| `/servo_left/start_servo` | std_srvs/srv/Trigger |
| `/servo_left/stop_servo` | std_srvs/srv/Trigger |
| `/servo_left/unpause_servo` | std_srvs/srv/Trigger |
| `/servo_right/change_control_dimensions` | moveit_msgs/srv/ChangeControlDimensions |
| `/servo_right/change_drift_dimensions` | moveit_msgs/srv/ChangeDriftDimensions |
| `/servo_right/pause_servo` | std_srvs/srv/Trigger |
| `/servo_right/reset_servo_status` | std_srvs/srv/Empty |
| `/servo_right/start_servo` | std_srvs/srv/Trigger |
| `/servo_right/stop_servo` | std_srvs/srv/Trigger |
| `/servo_right/unpause_servo` | std_srvs/srv/Trigger |
| `/skills/query` | qda_interfaces/srv/BrainRequest |
