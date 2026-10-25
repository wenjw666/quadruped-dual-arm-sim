# B2 双臂运控适配：本轮结果与复现

2026-10-25；开发分支 `v2/decision-rl`。当前实际机器人运动由自由基座 MuJoCo 接触动力学、原版 B2 RL 策略与限力矩 PD 产生；没有在控制循环中覆盖根位姿。运控测试在空平面进行，尚未接管工厂地图导航或抓放任务。

## 本轮选择与结果

继续核查了裸 B2 移植问题。积分器、物理步长、关节限位、摩擦、转子惯量及自由关节阻尼的单因素探针没有消除失稳。使用 robot_lab 当前训练 URDF 的转换对照也失败，不能声称已经复现上游基线。其若干固定传感器 link 质量约 1e-15 kg、惯量约 1e-27，MuJoCo 原生导入拒绝；诊断转换将这些固定 link 的惯量明确记为零，并保留碰撞，最终质量为 74.5803 kg。这个诊断模型没有替换项目本体。导入修正、引擎和版本差异仍需继续核对。

已完成从开源 actor 初始化的 PPO 适配链路。前 45 维观测与开源策略保持一致，追加双臂位置/速度、接触和负载信息到 78 维；新增输入的初始权重为零。128 组随机输入的 actor 输出差为 **0**。PPO 在当前完整模型上训练 65,536 步，使用双臂质量、载荷、质心和摩擦随机化，双臂目标保持 home。

**这轮 PPO 未带来有效的低速改善。** 同样的 18 个独立评估回合，前进误差从 0.1466 变为 0.1473 m/s，侧移从 0.1008 变为 0.1011 m/s。保存了前后 checkpoint 和失败证据；没有把训练完成标为运控通过。训练结果位于 `output/v2/runs/b2_transfer_v1/`。

随后对未修改的原版 actor 进行了 22 点响应扫描。当前模型上，正向低速与正向侧移存在明显的指令响应不足，例如侧移输入 0.10 m/s 时几乎静止，输入 0.45 时才接近 0.11 m/s。以此构建有模型/策略指纹的分方向插值表，并比较以下三种运行方式：原始指令、指令前馈校准、前馈校准加接触速度估计反馈。

这个改善来自**原版 RL + 外层速度适配器**，不是新的 PPO 策略训练成果。反馈估计使用关节编码器、IMU 角速度/姿态和足端接触，通过足端运动学估计机身速度；不读取自由基座平移速度。仿真真值只用于离线评分。当前假设平地、支撑足近似不滑动；接触少于两个或估计不一致时报告无效并暂停积分。速度指令、校正量、积分量和变化速率均受限。

## 对照数据

3 个独立于响应标定的种子 × 7 种指令 × 3 种运行方式，共 63 个 10 秒回合；质量、载荷、质心、摩擦有随机化。全部未跌倒，未记录到非足端异常接触。以下均为全回合逐时刻误差的均值，包含起步瞬态，不是只选稳定片段。

| 指令 | 原始平面速度误差 m/s | 校准+反馈误差 m/s | 原始偏航速度误差 rad/s | 校准+反馈误差 rad/s |
|---|---:|---:|---:|---:|
| 站立 | 0.0044 | 0.0044 | 0.0115 | 0.0115 |
| 前进 0.15 m/s | 0.1466 | 0.0547 | 0.0144 | 0.0912 |
| 后退 0.15 m/s | 0.1164 | 0.0493 | 0.0657 | 0.0812 |
| 左移 0.10 m/s | 0.1007 | 0.0659 | 0.0157 | 0.0663 |
| 右移 0.10 m/s | 0.1002 | 0.0437 | 0.0098 | 0.0732 |
| 左转 0.25 rad/s | 0.0321 | 0.0510 | 0.1331 | 0.0681 |
| 右转 0.25 rad/s | 0.0051 | 0.0497 | 0.2508 | 0.0529 |

前进、侧移和偏航速度跟踪得到改善，但开始运动后也引入了横向/偏航耦合；原始策略的一些较小漂移来自其几乎没有执行运动。不能只报告改善的一列：转向时平移漂移增加，左侧移和偏航仍不满足精细操作要求。

原始对照报告：`output/v2_audit/b2_velocity_adapter.json`。其输入文件按报告记录的 SHA-256 保留为 `b2_command_response_v1_unfingerprinted.json`；相同测量数据补充模型/策略指纹后保存为 `b2_command_response.json`。新增元数据没有改动测量行。

另以新种子 42007–42011 检查双臂运动：两臂 joint1 振幅 0.08 rad、joint4 振幅 0.04 rad、频率 0.25 Hz，相位相差 π/2；通过实际 PD 目标运动，不直接改位姿。35 回合结果单独保存在 `output/v2_audit/b2_moving_arms_validation.json`。这些是运控回合，不是 PPT 的 30 轮抓取放置验收。

35 回合全部完成 10 秒，没有跌倒或非足端异常接触；最大倾角 0.0782 rad（约 4.48°）。前进、后退、左移、右移的平均平面速度误差分别为 0.0537、0.0516、0.0666、0.0490 m/s；左右转向的平均角速度误差分别为 0.0587、0.0586 rad/s。小幅双臂运动下未出现明显退化，但这不覆盖大幅伸展、抓取冲击或外力推搡。

## 运行端与可视化

- 原版 actor 的 78 维等价版本，以及首轮 PPO actor 均已导出 ONNX。分别使用 1,000 个真实 rollout 观测核验，最大绝对差均为 **1.91e-6**。
- 原版等价 ONNX 桌面 CPU 推理 P99 约 **0.043 ms**；首轮 PPO 版本约 **0.053 ms**。这是本机 Docker CPU 的观测值，不代表 RDK X5 实时性能。
- `qda_locomotion.rl_bench_node` 是实际 ROS 2 + ONNX + MuJoCo 运控入口。发布 `/v2/joint_states`、`/v2/odom_truth`、`/v2/imu_ideal`、`/v2/clock`、`/v2/rl_status`，订阅 `/v2/cmd_vel`；0.25 秒未收到指令即请求零速。跌倒/策略异常锁存故障并切换阻尼保持。
- 原始与校准两种 ROS 运行模式均通过实际消息测试：各收到 250 帧关节状态，指令到达、过期归零、未验收策略默认拒绝、显式实验模式运行均通过。
- 加速度尚未实现，IMU 消息按 ROS 规范将加速度协方差首元素置 -1，不能当作完整 LIVO IMU 输入。
- 校准表只能用于原版等价 actor；不能拿旧响应表套到微调后权重。模型/策略指纹和 ONNX 一致性均检查。
- `output/v2_audit/b2_contact_preview.mp4` 是实际 16 秒连续动力学预览：站立→前进→侧移→转向→停止。完整运行 800 个控制步，未跌倒；对应 JSON 记录每步指令和实际状态。

## 决策修复

修复了决策核心把 `transfer` 搬运阶段也要求“全程静止”的问题。搬运时允许底盘移动，持续要求持物确认；技能返回完成后，必须停稳达到规定时间才进入放置。增加移动期间持物丢失及完成后等待停稳的回归测试，决策测试共 10 项通过。尚未连接真实抓放技能 ActionServer，不能以逻辑测试代替物理抓取。

## 复现入口

Windows 项目目录中运行 `./run_v2_rl_bench.ps1 -Seconds 30`，使用本轮原版等价 ONNX 与校准表；默认零速，30 秒后退出。另一个容器终端可持续发布速度指令：

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub -r 10 /v2/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.15}, angular: {z: 0.0}}'
```

实验依赖和本轮生成文件存在时，可在容器复现：

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
export PYTHONPATH=/opt/qda-rl-v2:$PYTHONPATH
python3 -m qda_locomotion.transfer_ppo --steps 65536 --envs 4 --output /ws/output/v2/runs/NEW_UNIQUE_RUN
python3 /ws/scripts/probe_b2_command_response.py --output /ws/output/v2_audit/NEW_RESPONSE.json
python3 /ws/scripts/evaluate_b2_velocity_adapter.py --response /ws/output/v2_audit/NEW_RESPONSE.json --output /ws/output/v2_audit/NEW_COMPARISON.json
python3 /ws/scripts/evaluate_b2_velocity_adapter.py --final-validation --output /ws/output/v2_audit/NEW_VALIDATION.json
```

生成 ONNX 使用 `scripts/export_b2_onnx.py --checkpoint <run>/initial_policy.zip --output <run>/initial_policy.onnx`；脚本拒绝覆盖已有 ONNX。首次从干净仓库运行，还需按基础文档生成共享 URDF/场景、安装隔离依赖并通过 V2 动力学审计。本轮权重与大量原始输出保留在本机 `output/`，不混入第一版基础仓库的正式验收标签。

## 尚需完成

低速精度、左右不对称和运动耦合仍需处理；接触速度估计需要噪声、延迟、滑移和非平地验证；PPO 需改进探索与训练课程，并用独立测试集证明收益。后续已完成原工厂工程的简单 Nav2 到点和小幅双臂规划执行接入，见 [工厂接入记录](V2_FACTORY_INTEGRATION.md)；复杂路线、动态场景避障、全幅双臂操作、视觉闭环与物理抓放仍未验收。所有导出策略与运行状态保持 `runtime_ready=false`。
