# 当前仿真工程 ROS 2 节点网络审计

日期：2026-10-27。对象：本机 `qda-dev` 容器，ROS 2 Humble，Domain 8，B2 + 双 Aubo / Robotiq，MuJoCo RL 主路径。代码接口审查与现场发现的网络分别留存；节点存在不等于算法或完整任务验收通过。

## 1. 结论与功能边界

主运行链已采用 ROS 2 Topic、Service、Action 连接：传感器、建图、Nav2、双臂轨迹、物理抓放、视觉伺服、任务协调、大脑入口和可视化均有对应节点或节点内模块。**不能说所有算法都已经拆成独立节点，或已经完成无真值依赖的全自主系统。**

本次完整主栈现场发现 **37 个节点、152 个话题**，没有重复节点名；4 个 Nav2 生命周期主节点与 2 个代价地图均为 `active`。20 秒只读采样中，33 个关注话题有 27 个收到消息，RL `ready=true` 且无 fault，本地模型 HTTP 健康检查成功。全部 37 个节点的职责、Action 和采样结果见 [实测完整清单](ROS2_LIVE_INVENTORY.md)。

本次未下发运动任务。空闲时未产生的决策 trace、导航速度、伺服指标不能计为任务通过；任务状态中的历史 SUCCEEDED 来自持久化恢复。全渲染与 RViz 同时运行时，约 20 秒墙钟仅推进 3.1 秒仿真时间，本轮不具备实时运行性能验收结论。

| 功能 | ROS 接入方式 | 当前边界 |
|---|---|---|
| B2 RL 运控 | `/qda_mujoco` 内部 `SceneController` / ONNX / `DynamicPlant`；订阅 `/cmd_vel`，发布 RL 状态 | 没有单独的“RL 主场景节点”；实验性策略与载荷适配仍需更多验证；PPO 训练是离线工作流 |
| 建图 | `/qda_mapper` 接收激光、RGB-D 点云和里程计，输出栅格与彩色点云 | 当前使用 MuJoCo 真值位姿配准；动态主路径未启动 FAST-LIVO2，不是独立 SLAM 定位 |
| 导航 | Nav2 行为树、SmacPlanner2D、MPPI、代价地图；经命令仲裁驱动 RL | 接入 ROS Action；定位仍使用真值 `/odom` |
| 抓图 / 图像采集 | `/qda_mujoco` 发布头部及辅助相机；`/qda_frame_dump` 保存图像 | 图像来自 MuJoCo 渲染；保存帧是文件输出 |
| 识别与目标位姿 | `/qda_perception` 的 RGB-D 几何检测及异步 YOLO 标签 | 发布识别结果；**物理抓放主链尚未使用该目标位姿代替仿真物体真值** |
| 抓取—转移—放置 | `/robot_skill_server` → Nav2 + `/manipulation/command` → `/qda_mujoco` 接触状态机 | 物理接触主路径已实现；既有验收集中在左臂与南侧转移路线；不是双臂任意物体通用抓取 |
| 双臂运动规划 | `/move_group` 的规划服务与轨迹 Action → MuJoCo 四组轨迹控制接口 | 与物理抓放服务并存；当前抓放主状态机不等于完整 MoveIt Task Constructor 管线 |
| 视觉伺服 | `/visual_servo` → `/servo_left` / `/servo_right` → 后端关节轨迹 | ArUco 标定板、PnP、KLT 过渡、IBVS；不是无标记语义目标抓取伺服 |
| 大脑决策 | `/local_brain` 接收请求和模块反馈；调用 VLM，校验提案，调用技能/计划 | VLM 是独立 HTTP 进程；决策是有界候选与反馈审查；多步骤计划显式提交，不是任意语言自主任务分解 |
| 语音 | 离线 Vosk WAV → 临时 CLI 节点 → `/brain/request` | 没有常驻麦克风监听、唤醒、TTS 或小智在线节点 |
| RViz 展示 | `/algorithm_view` 汇总算法状态，`/rviz2` 展示地图、轨迹和图像 | 本次在虚拟 X 显示器运行，已启动不表示 Windows 桌面上弹出了窗口 |

既有任务证据分别见 [物理抓放](../PHYSICAL_PICK_PLACE.md)、[建图与导航](../NAVIGATION_MAPPING_DEMO.md)、[大脑反馈闭环](../DECISION_LOOP_DEMO.md)。本轮是网络和启动审计，不将旧任务成绩冒充本轮重新跑出的结果。

## 2. 节点关系图

![主要控制与反馈链](diagrams/ros2-network.png)

[可编辑 draw.io 原图](diagrams/ros2-network.drawio)。该图按源码和实际网络整理，保留 Service、Action、外部模型和节点内部模块边界；为了易读，合并 Nav2 与左右 Servo，省略通用参数接口及部分状态订阅。全部端点以原始网络清单为准。

## 3. 主节点功能表

表内 `/` 开头的是运行时节点名，不是 Python 文件名。规划器、代价地图插件与 RL 推理对象不是各自独立 ROS 节点。

| 节点 | 对应包 / 实现 | 功能与主要接口 |
|---|---|---|
| `/qda_mujoco` | `qda_mujoco/node.py`、`scene_control.py`、`dynamics.py`、`manipulation.py` | 物理世界、RL 运控、传感器、接触抓放；输入 `/cmd_vel`；输出 `/clock`、`/odom`、`/joint_states`、`/imu`、`/scan`、`/livox/lidar_PointCloud2`、相机、RL/抓放状态；提供轨迹 Action 和 `/manipulation/command` |
| `/qda_command_mux` | `qda_bringup/command_mux.py` | 仲裁 `/cmd_vel_nav`、`/cmd_vel_task`、`/cmd_vel_manual`；租约、TTL、停稳与故障保护；唯一 `/cmd_vel` 发布者；`/motion/lease`、`/motion/hold` |
| `/robot_state_publisher` | `robot_state_publisher` | URDF + `/joint_states` → `/tf`、`/tf_static`、`/robot_description` |
| `/static_transform_publisher_*` | `tf2_ros` | 发布固定 `world → odom`；名称后缀每次启动变化；动态 `odom → base_link` 由 MuJoCo 发布 |
| `/qda_mapper` | `qda_perception/mapper_node.py` | 激光栅格 `/map`、`/mapping/cloud`；RGB-D 累积 `/mapping/colored_global`、`/mapping/color_status`；`/mapping/save_color` 保存地图 |
| `/qda_perception` | `qda_perception/detector_node.py` | 头部 RGB-D 检测；发布 `/perception/pick_pose`、`place_pose`、`labels`、`debug`、`markers` |
| `/qda_frame_dump` | `qda_perception/frame_dump.py` | 订阅 overview/debug 图像，保存观察帧；不执行机器人控制 |
| `/bt_navigator` | `nav2_bt_navigator` | `/navigate_to_pose`、`/navigate_through_poses` Action；导航行为树与恢复流程 |
| `/planner_server` | `nav2_planner` | SmacPlanner2D 全局规划，输出 `/plan`，提供路径计算 Action |
| `/controller_server` | `nav2_controller` | MPPI 局部跟踪，结合 odom / 局部代价地图，输出 `/cmd_vel_nav` |
| `/behavior_server` | `nav2_behaviors` | 旋转、后退、等待等恢复动作；底盘指令同样走 `/cmd_vel_nav` |
| `/global_costmap/global_costmap` | `nav2_costmap_2d` | 全局静态层与膨胀层；接收 `/map`，支撑全局规划 |
| `/local_costmap/local_costmap` | `nav2_costmap_2d` | 局部障碍与膨胀代价地图，支撑 MPPI 与恢复行为 |
| `/lifecycle_manager_navigation` | `nav2_lifecycle_manager` | 配置、激活与监控 Nav2 生命周期节点 |
| `/move_group` | `moveit_ros_move_group` | 运动学与 OMPL；`/plan_kinematic_path`、`/execute_trajectory`；发布 `/monitored_planning_scene`；四组 FollowJointTrajectory 客户端连接 MuJoCo |
| `/local_brain` | `qda_brain/brain_node.py` | `/brain/request` 入口；图像/里程计/运控/技能/抓放反馈；HTTP 模型调用、提案校验与回执；输出 `/brain/health`、`/brain/decision_trace` |
| `/robot_skill_server` | `qda_brain/skill_server.py` | `/robot_skill` Action 和 `/skills/query`；调用 Nav2、MoveIt、VisualAlign、后端抓放及底盘租约；输出 `/brain/status` |
| `/mission_coordinator` | `qda_brain/mission_node.py` | `/mission/request`；串行技能调用、幂等步骤与任务记录；发布 `/brain/mission_status` |
| `/visual_servo` | `qda_brain/visual_servo.py` | `/visual_align` Action；腕部图像 → ArUco / PnP / IBVS；发布 `/servo_{left,right}/delta_twist_cmds`、`/brain/servo_metrics`、`/brain/servo_overlay` |
| `/servo_left` | `moveit_servo/servo_node_main` | 左臂末端速度、关节状态与规划场景 → `/left_arm_controller/servo_joint_trajectory`；状态 `/servo_left/status` |
| `/servo_right` | `moveit_servo/servo_node_main` | 右臂同上，输出 `/right_arm_controller/servo_joint_trajectory` |
| `/algorithm_view` | `qda_bringup/visualization_node.py` | 汇总地图、路径、关节、识别、RL、技能与健康状态；发布 `/viz/markers`、`status_image`、`status_text`、各类路径 |
| `/rviz2` | `rviz2` | 订阅可视化及原始数据；地图、相机、机器人 TF、点云、轨迹、状态面板 |

框架还会创建 `*_private_*`、`transform_listener_impl_*`、行为树 Action 客户端等辅助节点。完整现场名称及职责附表见 [实测节点清单](ROS2_LIVE_INVENTORY.md)，避免把这些辅助节点误当成新增算法。

## 4. 指令如何形成闭环

1. CLI / 上层应用调用 `/brain/request`。本地脑节点读取头部图像、里程计、RL、控制权、识别、任务和抓放反馈；通过 HTTP 向 SmolVLM 发起受限请求。
2. `/local_brain` 校验返回提案。单技能经 `/robot_skill` 下发；显式计划经 `/mission/request` 交给协调器逐步执行。模型不会直接输出电机指令。
3. 导航由 `/navigate_to_pose` 进入 Nav2，MPPI 输出 `/cmd_vel_nav`，仲裁节点输出 `/cmd_vel`，MuJoCo 节点内 RL + PD 驱动关节。
4. 抓放技能获取底盘与机械臂控制权，经 `/manipulation/command` 触发接触状态机；完成转移需调用 Nav2。抓放状态来自物理世界反馈，但目标定位仍使用仿真真值。
5. 视觉对准技能调用 `/visual_align`，腕部图像误差产生 Twist，经 MoveIt Servo 形成后端关节轨迹。该链与抓放链通过控制权互斥，不可同时抢占同一机械臂。
6. `/odom`、`/locomotion/rl_status`、`/motion/command_owner`、`/motion/arm_modes`、`/manipulation/status`、Action result 和任务状态回到技能/大脑。受限反馈复核与本地门控决定继续、完成或停止；记录 `/brain/decision_trace`。地图和状态同时进入 RViz。

目前缺失的关键连接是“感知识别目标 → 物理抓放目标位置”的生产主路径，以及“独立 SLAM 位姿 → 导航定位”。现有主题名称或空闲 Action 接口不能代替这两条连接的实际实现与验收。

## 5. 实际 rqt_graph 与复现

![rqt_graph 实际节点视图](../output/ros2_graph/rqt-nodes-screenshot.png)

- [节点视图原分辨率](../output/ros2_graph/rqt-nodes-native.png)
- [全部节点/话题截图](../output/ros2_graph/rqt-full-screenshot.png) · [全部节点/话题高清原图](../output/ros2_graph/rqt-full-native.png)
- [原始 DOT](../output/ros2_graph/rqt-full.dot) · [rqt 采集配置与时间](../output/ros2_graph/rqt-capture.json)
- [节点/话题/服务/订阅端点与消息采样 JSON](../output/ros2_graph/ros2-live-graph.json)

截图来自安装的 **rqt_graph 原生 Qt 插件**，现场 DDS 发现后进行窗口抓取和原生画布导出，并非根据架构图伪造。为可读性，节点视图去掉 `/clock`、`/rosout`、`/parameter_events`、`/tf`、`/tf_static` 连线；完整图保留这些话题及所有叶子话题，只排除采图工具自身。这里的“全部功能启动”指兼容的主栈同时运行并待命，不指所有互斥动作同时执行。

**rqt_graph 主要展示 Topic 通信，并不能完整表达 Service 调用和外部 HTTP 模型调用**；Action 通常通过 feedback/status 话题体现。因此必须与上面的人工审核关系图和接口清单结合看，不能用 rqt 图上没有一条线推断服务没有接入。

在容器内复现主栈（先确认无同域旧仿真重复运行）：

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
export ROS_DOMAIN_ID=8
export PYTHONPATH=/opt/qda-rl-v2${PYTHONPATH:+:$PYTHONPATH}
# 独立终端启动本地模型，若该模型服务已经运行则无需重复：
bash /ws/scripts/run_local_vlm.sh
```

另一个同环境终端：

```bash
xvfb-run -a -s '-screen 0 2400x1600x24' \
  ros2 launch qda_bringup pick_place.launch.py \
  backend:=mujoco motion_mode:=rl brain:=true servo_fixture:=true \
  allow_virtual_grasp:=false scenario:=auto gui:=false rviz:=true \
  render_images:=true mapping_rgbd:=true run_task:=false \
  output_dir:=/ws/output/ros2_graph/scene
```

等待模型加载、RL ready 和 Nav2 激活后，在同样已 source 且 Domain 8 的终端：

```bash
python3 /ws/scripts/capture_ros_graph.py --seconds 20
xvfb-run -a -s '-screen 0 2400x1600x24' \
  python3 /ws/scripts/capture_rqt_graph.py
```

若使用可见 X 显示器，可不加 `xvfb-run`，在该显示器直接启动 `rqt_graph`。本轮使用虚拟显示器是为稳定采集实际 Qt 界面。

本轮补齐 Docker 镜像依赖：`ros-humble-rqt-graph`、`graphviz`、`xvfb`、`xauth`；增加只读采集脚本。

## 6. 本次发现并修复的启动问题

新建输出目录会重新运行 CoACD 腕部凸分解。即使固定随机种子，重新生成的组件顺序或几何仍可能不同于已校验参考资产，触发 `Scene robot physics differs from trained plant`。MuJoCo 拒绝启动后，Nav2 等待 `odom → base_link`，其他节点看似在线但无法形成工作闭环。

已将 RL 场景生成改为显式复用 `/ws/output/v2/collision_parts` 中已校验的参考组件；缺失缓存或网格时直接报告缺失，不静默重新生成。**保留原有完整物理签名检查**。参考模型、策略权重与控制参数均未为通过检查而放宽。新输出目录的实际启动与物理契约测试用于验证修复。

新生成场景的物理契约检查 **3 项通过**，记录见 [测试结果](../output/ros2_graph/physics-contract-test.txt)。本次还运行了决策反馈单元检查（5 项通过）；这两类检查不代替导航或抓放的完整任务重测。

## 7. 不应混入本次主栈的组件

| 组件 | 原因 |
|---|---|
| `/qda_locomotion` | V1 规定步态入口；当前 RL 在 MuJoCo 内部，不应同时驱动底盘 |
| `/qda_pick_place` | 旧脚本任务路径；本次使用脑节点 + 技能服务器，`run_task=false` |
| `/v2/b2_rl_bench` | 独立 RL 台架，与主场景不是同一机器人实例 |
| FAST-LIVO2 `/laserMapping` | 动态主路径禁用；mapper 保留的 LIVO 订阅不表示 LIVO 正在运行 |
| `/quad_navigator` | 旧导航器，主路径由 Nav2 替代 |
| `/robot_mcp_bridge` | MCP stdio 服务被外部应用启动时才出现；主 launch 未启动 |
| `/brain_command` | CLI 命令期间短暂出现，非后台常驻节点 |
| Gazebo / controller_manager | V1 备选后端使用另一域；不与本次 MuJoCo 图混用 |
| Vosk / llama-server / PPO | 分别是离线语音工具、外部模型服务、离线训练，不是常驻 ROS 算法节点 |
