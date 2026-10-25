# B2 开源 RL 核查与首次移植测试

日期：2026-10-25。分支：`v2/decision-rl`。用户要求优先复用官方及开源 B2 经验，保留 B2 + 双 Aubo 本体。

## 选择结论

采用 **robot_lab 的 B2 PPO 配置 + rl_sar 的 B2 策略契约与部署流程** 作为主要参考；宇树官方 MuJoCo/SDK2 用来核对动力学与底层接口。已有 SB3 训练器保留为小规模动力学回归工具。主任务仿真仍以 MuJoCo 为目标，训练框架与任务仿真后端可以不同。

这是有代码和公开权重的 B2 路线，不必从 Go2 策略猜测。现有原版权重已经在本项目自由基座模型上执行，但速度跟踪未通过，`runtime_ready=false`。尚未接管 ROS 导航或真实机器人，也未完成双臂载荷再训练。

## 已核实的来源

| 来源 | 固定版本 | 可用内容及边界 |
|---|---|---|
| [宇树 unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco/tree/1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d/unitree_robots/b2) | `1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d` | B2 MJCF、碰撞和执行器；BSD-3-Clause。不是 RL 训练算法或训练权重。 |
| [宇树 SDK2 B2 示例](https://github.com/unitreerobotics/unitree_sdk2/tree/d1abf001220d3e5f59e00a0c4aaa595e00089a4a/example/b2) | `d1abf001220d3e5f59e00a0c4aaa595e00089a4a` | 底层站立和高层运动接口；站立示例 Kp=1000、Kd=10，不能当作行走策略增益。BSD-3-Clause。 |
| [rl_sar B2 策略与配置](https://github.com/fan-ziqi/rl_sar/tree/376d42c9b128f963ab08579762d5a216a976ce39/policy/b2) | `376d42c9b128f963ab08579762d5a216a976ce39` | `policy.pt`、45 维观测、增益/动作缩放/关节映射、MuJoCo/Gazebo 部署流程；Apache-2.0。README 的 B2 真机栏未标记完成，不能宣传为已验证真机部署。 |
| [robot_lab B2 配置](https://github.com/fan-ziqi/robot_lab/tree/26acb225aad129e222da7a34ba11f39e29b73e39/source/robot_lab/robot_lab/tasks/manager_based/locomotion/velocity/config/quadruped/unitree_b2) | `26acb225aad129e222da7a34ba11f39e29b73e39` | 平地/粗糙地形、RSL-RL PPO、奖励、随机化及 B2 资产；Apache-2.0。PPO actor/critic 512/256/128，ELU，24 steps/env，5 epochs，4 minibatches。当前训练代码版本不等于公开 checkpoint 的训练版本，不能声称已精确复现原训练。 |
| [IIT basic-locomotion-isaaclab](https://github.com/iit-DLSLab/basic-locomotion-isaaclab/tree/92687729f3b85ca4c88f87d075f64c1ff99a0777) | `92687729f3b85ca4c88f87d075f64c1ff99a0777` | 备选：B2 资产、DirectEnv 配置、sim-to-sim 框架；BSD-3-Clause。本轮未执行其训练或确认 B2 专用预训练权重。 |

PPT 提到的 `unitree_rl_mjlab` 仍可作为后续 GPU MuJoCo 训练候选，但其 README 未列 B2；这不代表开源社区没有 B2 支持。B2-W 是轮足机型，本轮没有混用 B2-W 的动作空间或策略。

## 已落实的适配

- `scripts/fetch_b2_reference.py`：固定提交和 SHA-256，缓存策略、两份配置及许可证；不跟随 main，不将未核对的模型自动投入运行。
- `qda_locomotion/upstream_b2.py`：加载原始 TorchScript；45 维观测按 `omega*0.25, gravity, command, q-q0, dq*0.05, previous_action` 排列；不把地面真值线速度输入 actor。
- 显式按关节名映射 FR/FL/RR/RL，避免本项目 FL/FR/RL/RR 顺序造成错误。动作 hip 缩放 0.125，thigh/calf 0.25，q0=[0,0.8,-1.5]；行走 Kp=160、Kd=5。50 Hz 策略与 500 Hz 物理分开。
- 原版 action clip 为 ±100，本地额外保留 URDF 关节限位和力矩上限，记录触发次数；没有偷偷使用站立增益替换行走增益。
- `scripts/evaluate_b2_reference.py`：站立、前进、侧移、转向，记录速度误差、倾角、跌倒、限位/限矩、非预期接触；只重置初始位置，运动过程中通过 PD 力矩推进物理。
- `scripts/test_b2_reference.py`：核验物理关节到观测槽位的映射、动作到关节的逆映射、双臂隔离、原版 checkpoint 输出及非法命令拒绝。

## 首次实测结果

本项目完整模型 85.665 kg，双臂固定在 home，通过 PD 保持；空平面、理想 IMU/编码器、物理步长 2 ms。每个指令 3 个种子，每轮策略阶段最长 10 秒，之前使用项目 PD 初始化 2 秒。种子仅扰动初始腿关节 ±0.005 rad，不代表完成广泛随机化。

| 指令 | 21 轮中的对应轮数 | 平均平面速度误差 m/s | 平均偏航速度误差 rad/s |
|---|---:|---:|---:|
| 站立 | 3，均未跌倒 | 0.0044 | 0.0115 |
| 前进 0.15 m/s | 3，均未跌倒 | 0.1465 | 0.0138 |
| 侧移 0.10 m/s | 3，均未跌倒 | 0.1008 | 0.0162 |
| 转向 0.25 rad/s | 3，均未跌倒 | 0.0326 | 0.1201 |
| 前进 0.40 m/s | 3，均未跌倒 | 0.1981 | 0.1281 |
| 侧移 0.25 m/s | 3，均未跌倒 | 0.2493 | 0.0426 |
| 转向 0.50 rad/s | 3，均未跌倒 | 0.0658 | 0.2076 |

结论：原版策略能保持平衡并在部分指令下行走，但低速和侧移几乎未跟踪；较高前进速度仍存在偏航及明显速度误差。不能把“未跌倒”作为导航控制通过。21 轮是短时运控探针，不是 PPT 要求的 30 轮完整抓放验收。

额外测试：使用官方 B2 MJCF，仅删除零密度、无碰撞的视觉 mesh，保留全部惯性和碰撞，添加相同平地及 2 ms 步长。相同移植器在 21 轮中均触发高度/倾角终止，约 1–1.7 秒；这是**本移植探针失败**，不能据此断言官方模拟器或 rl_sar 本身有问题。需要对齐上游场景、执行器和 checkpoint 版本再复现。

发现必须保留的模型差异：官方 MJCF 与 rl_sar_zoo 当前 MJCF 总质量为 83.498 kg；robot_lab 当前 B2 URDF 为 74.580 kg；本项目还采用另一版宇树 URDF并加装上肢。因此上述两个整机模型不是“只增加双臂”的严格消融。官方 MJCF calf 限矩为 300 Nm，部署 YAML/本项目 URDF 为 320 Nm；本地执行时取较小值。官方关节 armature=0.1，本项目转换器为 0.01。尚无证据将失败唯一归因于载荷或低速死区，不修改惯性来追求表面成功。

原始数据在 `output/v2_audit/b2_reference_evaluation.json`、`b2_loaded_moderate.json`、`b2_official_low.json`、`b2_official_moderate.json`。策略 SHA-256：`d4aeb84894acda40f64feadaaad25d2be2ed3b8593b695b148e28673dff21e44`。

## 复现

在 qda-dev 容器中，已有 `/ws/output/v2/dynamics.xml` 且完成动力学审计后：

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
export PYTHONPATH=/opt/qda-rl-v2:$PYTHONPATH
python3 /ws/scripts/fetch_b2_reference.py
python3 /ws/scripts/test_b2_reference.py
python3 /ws/scripts/evaluate_b2_reference.py
python3 /ws/scripts/evaluate_b2_reference.py --speed-profile moderate --output /ws/output/v2_audit/b2_loaded_moderate.json
python3 /ws/scripts/evaluate_b2_reference.py --official-source /ws/output/v2/upstream_b2/official_b2.xml --output /ws/output/v2_audit/b2_official_low.json
```

下一步先复现锁定版本的 robot_lab/rl_sar 裸 B2 基线，核对惯性、限矩和电机速度包络；再将同一腿部模型扩展到双臂姿态/动作、负载、质心与延迟扰动的 PPO 训练。保留原始 45 维策略作为对照；新增上肢观测时需要新的策略契约和训练，不能把 78 维向量送入现有权重。通过低速、侧移、转向及持物站立验证后，再连接 Nav2 与任务状态机。
