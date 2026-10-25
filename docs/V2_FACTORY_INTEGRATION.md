# V2：接入原工厂工程（2026-10-25）

本轮承接用户“接入之前的仿真工程并对照方案 PPT 推进”的要求。保留 B2 + 双 Aubo、Robotiq 夹爪、原扩展工厂和 Gazebo 对照。当前主入口是**实验性 MuJoCo RL 工厂场景**，不再只是独立平地运控脚本。完整取放与板端部署尚未验收，策略状态仍为 `runtime_ready=false`。

## 与方案对应的实际改动

| PPT | 本轮接入 | 当前边界 |
|---|---|---|
| 13、24 运控 | 原 MuJoCo 节点使用自由基座、50 Hz ONNX、500 Hz 限力矩 PD；双臂通过实际执行器运动 | 原版等价 B2 actor + 已测指令校准/接触反馈；不是本轮 PPO 训练的改进成果；非平地和全幅载荷尚未验收 |
| 15、16 导航 | 原场景 Nav2 使用 Smac 2D + Omni MPPI；前进/后退/侧移/偏航上限为 0.15/0.05/0.08 m/s、0.25 rad/s | 仍使用仿真真值 odom 和原传感器占据栅格；AMCL/LIVO 尚未接管定位；南北绕行与侧向交付尚未做 RL 全流程统计 |
| 3、20、21 决策 | `/cmd_vel_nav`、`/cmd_vel_task`、`/cmd_vel_manual` 统一由 `command_mux` 输出 `/cmd_vel`；优先级手动→任务→导航，0.25 秒时效、保持服务、运控故障阻止运动 | 完整 PickPlace 决策核心、技能资源租约和抓放 ActionServer 尚未贯通；没有把仲裁器称为完整任务决策算法 |
| 7、11、17 相机 | 保留头部 640×480 RGB-D/10 Hz；新增机身 C0、左右腕 RGB，各 320×240/5 Hz，发布 CameraInfo 和一致的静态外参 TF | 新增位置是可复现的仿真安装假设，没有新增质量/碰撞体；未当作硬件标定，尚无腕部 Servo/视觉闭环；Gazebo 尚未同步新增三路相机 |
| 15、19 三维场景与操作 | MoveIt 点云 OctoMap 0.05 m 分辨率、最高 2 Hz、自过滤；SRDF 增加 odom→base_link 浮动关节，使地图固定在 odom 且规划模型跟随物理机身 | 仍使用 RRTConnect；尚未实现 MTC/Servo、物理夹持和 2 cm 放置验收 |
| 算法展示 | RViz 显示实际关节/机身轨迹、Smac/MPPI 路径、三维过滤点云、足端接触、期望/实际速度箭头、四路相机入口、故障/控制权/定位来源 | GUI 进程已启动，窗口截图库因 `foreground window did not report a process id` 未完成像素检查；实际相机原图、ROS 数据、图像内参/TF 已核验 |

机器人动力学签名检查质量/惯量、关节、接触形状与参数、执行器、约束、重力及求解参数。允许世界障碍和虚拟相机不同；机器人动力学变化则拒绝绑定旧策略。参考训练模型的原始 SHA-256 未改写。新增世界模型、场景清单和策略绑定记录写入 `output/v2_scene/`。

默认禁用旧版虚拟吸附抓取。`allow_virtual_grasp` 仅用于显式比较，运行状态会标识 `virtual_fixed_attachment`，不能用于证明动力学抓取或承载。

## 验证结果

最终完整场景测试 `output/v2_scene/integration_final.json`：

- 实际 ROS 图只有 `qda_command_mux` 发布 `/cmd_vel`，没有旧版腿部轨迹发布者。
- 左、右臂各完成一次 0.08 rad 小幅运动和回位，四个实际轨迹 Action 均成功。
- Nav2 从初始区域导航到 odom `(-0.2, 1.0, 0)`，成功返回；终点平面误差 **0.11736 m**，最大倾角 **0.04668 rad（2.67°）**，无运控故障。该误差是粗导航容差，不是 2 cm 放置验收。
- 收到实际全局路径、关节、扫描、地图、头部相机和新增三相机数据；三路图像非空、内参与分辨率匹配、相机 TF 可解析。图像保存为 `*_verified.png`。
- OctoMap 查询返回 **391,705 字节**、0.05 m 分辨率、`odom` 坐标系；到点后 MoveIt 浮动基座与实际 odom 相差 **0.00122 m**。

独立 MoveIt 联调 `output/v2_scene/moveit_integration.json`：先等待连续五帧速度/倾角稳定，再分别调用左右臂 `GetMotionPlan` 和 `ExecuteTrajectory`，执行小幅运动与回位；**4 次 RRTConnect 规划与实际执行均成功**。这是完整规划到控制器链路的检查，范围仍仅为当前姿态附近的小幅运动。

另通过实际 ROS 指令仲裁测试（优先级、过期、保持、解除保持不重放旧指令、故障停令），2 项场景动力学契约测试和既有 10 项决策守卫测试。Python 编译和 Git 空白检查通过。摘要与源文件指纹见 `docs/validation/v2_factory_20261025.json`，大体积原始记录留在本机 `output/`。

前两轮诊断数据保留：`integration_test.json` 为无新增传感器的首次接入；`integration_with_sensors.json` 是发现地图误挂 `base_link` 前的诊断，附有范围说明。最终验收采用 `integration_final.json`，不掩盖坐标系修复过程。

## 运行

当前本机资产已准备好，Windows 项目目录运行：

```powershell
.\run_sim.ps1 -Background
# 带 MuJoCo、RViz，默认不自动取放；RViz 的 2D Goal Pose 可发送导航目标
.\stop_sim.ps1
.\run_sim.ps1 -MotionMode kinematic -Scenario north -Background  # V1 对照
.\run_sim.ps1 -Backend gazebo -Scenario north -Background      # Gazebo 对照
```

各模式不能同时占用同一个 ROS domain：MuJoCo 为 8，Gazebo 为 7。可使用 `-Headless`、`-NoRviz`、`-Manual`。`-VirtualGraspComparison` 明确启用旧虚拟抓取并允许旧任务启动，但该组合尚未做完整取放验证。`run_scenarios.ps1` 的既有成绩仍是 V1 运动学对照。

保持服务 `/motion/hold` 为 `std_srvs/srv/SetBool`：true 清空并阻止速度命令，false 清空旧命令后等待新命令；不会打开夹爪。测试结束会保持底盘，交互导航前需解除保持。

容器内验证（先启动场景）：

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
export ROS_DOMAIN_ID=8
python3 /ws/scripts/test_v2_scene_ros.py --output /ws/output/v2_scene/integration_repeat.json
python3 /ws/scripts/test_v2_moveit_scene.py
```

全新克隆仍需按基础文档拉取 LFS、安装/构建 ROS 工作区，并在 `/opt/qda-rl-v2` 安装 `docker/requirements-rl-v2.txt`。Dockerfile 已补充 Nav2/Smac 和 MoveIt perception 依赖；本轮在现有容器实测，未声称全新镜像重建已通过。

RL 场景还依赖本机生成的参考模型、原版等价 ONNX/元数据、开源配置和响应校准表：`output/v2/dynamics.xml`、`output/v2/runs/b2_transfer_v1/initial_policy.onnx[.json]`、`output/v2/upstream_b2/`、`output/v2_audit/b2_command_response.json`。这些不在源码提交中。生成流程见 [RL 适配记录](V2_RL_ADAPTATION.md)：共享场景→`build_v2_dynamics.py --training`→动力学审计→固定版本参考下载→transfer PPO 保存初始等价 actor→ONNX 导出→命令响应标定与独立评估。新的模型/权重必须重新匹配校准及契约，不能复制旧成功标签。

`check_v2_scene_assets.py` 在 Windows 启动入口检查资产、依赖和指纹，缺失则明确报错，不静默切回运动学。直接使用 ROS launch 时须显式传 `motion_mode:=rl run_task:=false` 并加载隔离 Python 依赖。

## 后续顺序

1. 继续速度跟踪/滑移/延迟与负载验证，再扩展为北、南、侧向通道的 RL 对照；保持失败留档。
2. 实现完整 IMU 加速度、逐点激光采样时间，核验 C0/LIVO 和 AMCL 的独占 TF；当前 RL 模式禁用不完整的 LIVO 输入链路。
3. 标记 PnP/6D 位姿质量与新鲜度、双臂技能/MTC/Servo 互斥，接通已存在的决策守卫与严格验收接口。
4. 单独验证物理夹持、承载、释放/空爪/撤臂/稳定与 2 cm 视觉判据，之后做至少 30 轮完整取放。RDK X5 性能另在板端测试。

配置依据：[MoveIt Humble 点云感知说明](https://moveit.picknik.ai/humble/doc/examples/perception_pipeline/perception_pipeline_tutorial.html)和容器安装的插件 XML/参数接口。未把“有点云订阅”或“OctoMap 非空”等同于完整定位与操作验收。
