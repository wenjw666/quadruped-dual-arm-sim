# V2 模块协同改进与验收（2026-10-26）

本轮从 `faf4287` 继续，保留现有 B2 + 双 Aubo、实验性 RL actor、训练模型和工厂布局。新增多步骤技能编排，修复导航接口、控制权与跨时钟协同问题。总结构见 [CODE_ARCHITECTURE.md](CODE_ARCHITECTURE.md)。

## 改进内容

1. **Mission 序列**：完整计划先校验，1–12 步白名单技能。固定派生步骤 ID、SQLite 持久化、重复 ID 读取旧结果、参数冲突拒绝。失败不继续后续步骤；当前没有自动重做运动步骤。
2. **技能会话**：Mission 整段持有排他会话，外部单技能不能插入步骤之间。结束记录与内存 active 状态在同一个锁内更新，避免终态交接时丢失新任务或遗留 RESERVED 记录。
3. **底盘租约**：每个运动技能 acquire/renew/mode/release；导航仅放行 nav 来源，机械臂操作保持底盘。令牌过期不能复活；租约过期锁定 hold，需要明确 reset。普通 `/motion/hold False` 不能绕过活动租约。
4. **双时钟**：速度命令同时检查墙钟与仿真 TTL；租约心跳使用 steady clock。暂停 `/clock` 不会冻结看门狗。停稳要求五份状态和至少 0.5 s 仿真稳定窗口，避免提高状态发布频率后过早判稳。
5. **导航 frame**：修复 skill_server 提交 map 目标、实际 Nav2/costmap 使用 odom 的不一致；现在使用 `navigation_frame=odom`，与当前真值定位架构一致。
6. **状态发布与慢渲染**：RL/手臂控制状态从真实物理更新处按 0.2 s 墙钟节流发布。渲染使用复制的 MjData 快照，在控制锁外进行，保持原始采样时间戳；慢渲染不再阻塞控制/续约/取消服务。没有独立线程重发缓存状态；物理停滞仍触发看门狗。图像继续检查 0.5 s 仿真年龄，离线慢渲染的墙钟上限为 15 s。物理状态和时钟的墙钟停机阈值仍为 5 s。
7. **Servo 所有权和命令有效期分离**：视觉控制者每秒以令牌续约 5 s 腕部租约；心跳仅访问独立所有权锁，不等候物理/射线/渲染锁。命令订阅使用单独回调组和深度 1 队列，避免等待数据锁的订阅挤占所有服务线程。关节输出暂停不会被误当作控制者退出。运动命令仍在 0.35 s 仿真时间或 1 s 墙钟后失效归零。续约只保留控制权，不延长旧运动命令。
8. **取消确认**：技能取消等待下游 Action 终态，之后才释放资源和结束。迟到的 goal acceptance 会被补发取消；停机/释放未确认时进入 quarantine，保持底盘并阻止后续技能，需要整套 launch 重启检查。
9. **视觉跟踪**：ArUco ID 四角→PnP→IBVS；最多两帧且不超过 0.45 s 仿真时间的 KLT 过渡，带前后向一致性/残差/位移/四边形/PnP 复投影检查。四帧成功必须来自重新检测的 ArUco，不能靠漂移的光流累计通过。
10. **接口与显示**：MCP 从五个扩展为八个工具；CLI 增加 plan/mission-status/mission-cancel；RViz 显示计划步骤、传感状态新鲜度、IBVS 来源与底盘保持原因。刷新率 10 FPS，另提供 `-NoPhysicsGui` 减少本机双窗口渲染开销。

视觉技能墙钟总预算可通过 `visual_skill_timeout` 设置，离线默认 360 s；图像闭环预算 180 s，Mission 子技能上限 420 s。上述上限不替代实时性验收：物理/时钟停滞、无效视觉、碰撞/奇异性和控制租约失效仍各自停止任务。

## 验证入口与范围

原始记录保存在 `output/v2_brain/`，可提交的机器可读摘要为 [v2_coordination_20261026.json](validation/v2_coordination_20261026.json)。摘要保留失败开发回合；不要只取最终成功回合推算统计成功率。

最终 handoff 和 dual_visual 均完成 4/4 步。双腕误差分别为 **21.87 → 1.42 px**、**23.42 → 0.72 px**，整段未出现 RL fault；这不替代载荷/长路线统计验证。八项 Servo 门控均通过，暂停时钟与延迟取消检查通过。完整重启后确认仅 `qda_command_mux` 发布 `/cmd_vel`，本地模型健康、模块状态新鲜，RViz 已恢复。实测曲线在 `output/v2_brain/coordination_ibvs.png`。

| 检查 | 方法与边界 |
|---|---|
| 四步 handoff | 真实 Nav2 到起点→左回位→右回位→inspect；验证交接、会话排他、资源释放与重复提交 |
| 四步 dual_visual | 真实 Nav2 到起点→左 IBVS→右 IBVS→inspect；光学校准板场景，无 GUI 离线物理运行，使用实际图像与电机力矩 |
| Mission 顺序/失败/取消/幂等 | 独立 ROS 域中的真实 Action 协议，受控技能驱动；不把它当物理任务通过 |
| 底盘仲裁 | 独立域消息：优先级、双 TTL、hold、状态过期、令牌排他、过期/reset |
| 暂停时钟 | 固定 `/clock=100 s`，持续新命令但不续约；墙钟租约按期保持底盘 |
| 动作迟到和慢取消 | 受控 MoveIt Action 驱动；等待取消终态，迟到接受补发取消并隔离新技能 |
| KLT | 实际 OpenCV 算法，合成 ArUco 图像的位移、失检、重获、空白和错误 ID；没有遮挡统计结论 |
| Servo 门控 | 真实 MuJoCo：轨迹/Servo 互斥、过期/越界拒绝、零时间戳、旧令牌拒绝、心跳与电机命令独立到期 |
| MCP 与共享场景 | 真实 stdio 会话与 ROS 服务、工具名单和提案；左右 Servo 接收 move_group 的 odom/0.05 m OctoMap |

本轮导航到起点用于验证 Nav2→停稳→操作的接口交接；没有把近零距离到点当成长距离导航性能。完整工厂路线、RL 再训练和真实物理夹持仍需后续专项验收。前轮 SmolVLM 10/12、离线 ASR 3/4 是历史小样本结果，本轮没有声称提高了模型泛化能力。

## 使用

Windows 项目目录：

```powershell
# MuJoCo 物理在后台，RViz 显示算法与机器人，保留较多 CPU 余量
.\scripts\windows\start_brain.ps1 -WithSimulation -ServoFixture -NoPhysicsGui
# 如需 MuJoCo 原生窗口，同时启动：
# .\scripts\windows\start_brain.ps1 -WithSimulation -ServoFixture
# 完全离线自动验收可加 -Headless
```

先停止已运行的同后端 launch 再重新启动。容器内：

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
export ROS_DOMAIN_ID=8
python3 /ws/scripts/make_mission_plan.py --preset dual_visual --out /ws/output/v2_brain/dual_visual.json
python3 /ws/scripts/brain_cli.py --plan /ws/output/v2_brain/dual_visual.json
python3 /ws/scripts/brain_cli.py --plan /ws/output/v2_brain/dual_visual.json --execute --task-id demo-dual-001
python3 /ws/scripts/brain_cli.py --mission-status demo-dual-001
python3 /ws/scripts/brain_cli.py --mission-cancel demo-dual-001
```

计划重复 ID 不重新执行；需要再次实验时使用新 ID。`inspection` 预设提供检查点往返路线，但不在本轮短距离交接验收范围内。`dual_visual` 只在启用校准板时使用。

## 复核脚本

运行单元/受控 ROS 测试时使用独立域，避免与真实场景重名节点冲突：

```bash
python3 /ws/scripts/test_v2_brain_unit.py
python3 /ws/scripts/test_v2_coordination_unit.py
ROS_DOMAIN_ID=38 python3 /ws/scripts/test_v2_mission_protocol.py
ROS_DOMAIN_ID=39 python3 /ws/scripts/test_v2_command_mux.py
ROS_DOMAIN_ID=40 python3 /ws/scripts/test_v2_paused_clock.py
ROS_DOMAIN_ID=41 python3 /ws/scripts/test_v2_skill_cancellation.py
```

真实场景空闲时按顺序运行：

```bash
ROS_DOMAIN_ID=8 python3 /ws/scripts/test_v2_coordination_ros.py --preset handoff
ROS_DOMAIN_ID=8 python3 /ws/scripts/test_v2_coordination_ros.py --preset dual_visual
ROS_DOMAIN_ID=8 python3 /ws/scripts/test_v2_servo_gate.py
ROS_DOMAIN_ID=8 python3 /ws/scripts/test_v2_brain_control.py
ROS_DOMAIN_ID=8 PYTHONPATH=/opt/qda-brain-python:$PYTHONPATH python3 /ws/scripts/test_v2_mcp.py
ROS_DOMAIN_ID=8 python3 /ws/scripts/test_v2_shared_scene.py
ROS_DOMAIN_ID=8 python3 /ws/scripts/check_v2_runtime.py
python3 /ws/scripts/summarize_v2_coordination.py
```

KLT 调用依据：[OpenCV 光流接口](https://docs.opencv.org/4.6.0/dc/d6b/group__video__track.html)。实现中的阈值与保护属于本工程配置，效果以本地测试记录为准。
