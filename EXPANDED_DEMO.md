# MuJoCo 主后端与扩展工厂验收

新版 B2 + Aubo A3 双臂 + Robotiq 2F-85 已作为共享机器人模型接入。`run_sim.ps1` 和 ROS launch 默认使用 MuJoCo；Gazebo 保留为可选对照后端。正式研究方案等用户验收本轮后再对接。

## 启动与切换

```powershell
# 默认：MuJoCo、自动选路、GUI + RViz、执行一次完整任务
.\scripts\windows\run_sim.ps1
# 若同一后端已经运行，先停止再切换配置
.\scripts\windows\stop_sim.ps1
.\scripts\windows\run_sim.ps1 -Scenario north
.\scripts\windows\run_sim.ps1 -Scenario south
.\scripts\windows\run_sim.ps1 -Scenario side
# 静止检查场景，不启动任务
.\scripts\windows\run_sim.ps1 -Scenario auto -Manual
# 保留旧小场景作为回归基线
.\scripts\windows\run_sim.ps1 -Scenario legacy
# 同配置的 Gazebo 对照
.\scripts\windows\run_sim.ps1 -Backend gazebo -Scenario side
```

`-Background` 后台启动，`-Headless` 关闭仿真 GUI 与 RViz，`-NoRviz` 只关闭 RViz。`show_rviz.ps1`、`stop_sim.ps1` 的默认后端也改为 MuJoCo。运行中的同后端不会被启动命令静默覆盖。

| 配置 | 路线与操作 | 检查重点 |
|---|---|---|
| auto | 不指定中途检查点，由 NavFn / MPPI 选择绕行路线，到东侧工位 | 自动全局选路、局部避障、最终对齐继续推进呢 |
| north | 经北走廊两个检查点，到东侧工位 | 较长路线、横移、转弯和携物移动 |
| south | 经南走廊两个检查点，到同一个东侧工位 | 不同拓扑路径、相同交付目标 |
| side | 经北走廊，先在开放区转至 90°，再到北侧工位 | 带物转向、不同基座朝向、坐标变换和放置 |
| legacy | 原小工厂场景 | 历史回归，不与扩图路线混算成绩 |

路线检查点不是预生成轨迹。两点之间实际执行 Nav2 的路径规划和 MPPI 控制，不能把蓝/黄色参考线当作算法输出。当前仍使用固定配置的左臂执行搬运，另一臂保持预备姿态；不是已实现双臂协同操作。

## 场景与接口

场地边界 x∈[-2,7] m、y∈[-3.8,4.5] m，约 9 × 8.3 m。中央货架形成南北绕行通道；两处交付台、边缘货架和立柱增加可视特征与操作朝向。地图 `/map` 仍由实际仿真传感器生成，没有把场景真值占用栅格直接喂给导航。

单一配置入口为 `src/qda_kinematics/qda_kinematics/scene.py`。场景生成、MoveIt 环境碰撞体、任务目标、RViz 参考标记和两个后端共用它。启动参数 `scenario:=...` 会同步传给所有 ROS 进程，每次启动写出 `scene_manifest.json`。

RViz 增加南北路线选项、选中检查点、目标基座朝向箭头、场景名、实走距离、规划调用/失败数和视觉回退计数，保留地图、MPPI、LIVO、实际路径、机械臂轨迹、图像与感知标记。MuJoCo 和 RViz 的默认视角扩大到全场。

## 记录、复现与验收

```powershell
# 串行执行真实仿真，不并行争抢本机算力；运行前先停止 MuJoCo
.\scripts\windows\stop_sim.ps1
.\scripts\windows\run_scenarios.ps1
# 正式重复测试的入口；本轮演示成功不等于统计成功率
.\scripts\windows\run_scenarios.ps1 -Scenarios north,south -Repeat 3
```

每次任务保存在 `output/mujoco/runs/<日本时间戳>_<场景>_<唯一编号>/`，失败记录不会被下一次运行覆盖：

- `summary.json`：成功/失败状态、里程、累计转角、机械臂规划调用/失败/墙钟耗时、碰撞候选剔除次数、每段导航状态/位置/朝向误差、最终放置误差、视觉回退记录。
- `trajectory.csv`：由 `/odom` 采样的时间、位置与偏航角；不是规划轨迹。
- `events.jsonl`、`task.log`：阶段事件、仿真时间和墙钟时间。
- `scene.json`、`configuration/`、`source_sha256.json`：场景、URDF、导航与算法配置快照。最早一轮 south 在快照功能加入前运行，没有这组源码快照。
- `clearance_audit.json`：离线计算 Nav2 加余量轮廓与参考静态障碍的间隙；不是全机械臂碰撞、接触力或足端动力学验证。

批量测试会另外保存 `output/expanded_demo/suite_*/summary.json` 与每次启动日志。`task_metrics.json` 只是最近一次结果，不应替代历史目录。场景加载失败与超时也作为失败保留。

本轮部分最终功能复验使用 `scripts/run_isolated_checks.py`，分别运行于 ROS 域 21/22/23，输出在 `output/expanded_demo/isolated_*/<场景>/`；每次运行另存 `validation_environment.json`，明确标记并行功能测试和线程设置。这些运行的墙钟耗时不用于算法性能对比。常规 `run_scenarios.ps1` 仍按顺序执行。ROS launch 支持 `output_dir:=绝对路径`，可用于独立实验目录，避免覆盖另一实例的模型和日志。

容器内验证/报告命令（先 source ROS 和 `/ws/install/setup.bash`）：

```bash
python3 /ws/scripts/test_expanded_scene.py
python3 /ws/scripts/render_factory_snapshot.py
python3 /ws/scripts/test_demo_evidence.py
python3 /ws/scripts/audit_run_clearance.py
python3 /ws/scripts/render_expanded_map.py
```

## 本轮发现并修复

1. 扩图后红砖墙的局部区域可能成为最大红色候选。颜色分割改为检查多个候选，通过真实深度和估计尺寸筛选，仍由 RGB-D 反投影定位；不使用物体真值生成检测结果。
2. YOLO 模型加载和 CPU 推理从 RGB-D 主回调移到独立单线程队列，限制同时在途任务数，丢弃过时标签；标签是辅助分类信息。
3. RGB 与深度时间差超过 0.15 秒时不生成目标位姿。未识别时的工位坐标回退继续显式记录。
4. Mid360 对开放天空无回波的射线不再编码成零点送给 LIVO；无有效回波的包不送入 LIVO 预处理，保留实际有效返回点。
5. FAST-LIVO 高频调试输出写入 ROS 日志，减少 Windows 挂载目录的终端日志流量；运行状态仍由 RViz 与任务日志显示。
6. 初次自动路线虽然到达目标，但离线审计发现轮廓擦过低工作台与货架，保留为未通过验收的历史记录。局部代价地图增加 `/map` 层，将三维扫描生成的低障碍占用同步给 MPPI；全局/局部膨胀半径改为 0.8 m。Nav2 先到距操作位后方 0.15 m 的预接近位，粗定位容差为 0.12 m，再低速对齐；任务层仍要求操作接近偏差小于 0.025 m。批量验收要求任务成功且轮廓审计无重叠，不能只看 `TASK_SUCCESS`。
7. MuJoCo 地板保留棋盘纹理，避免覆盖地板的场景装饰层把视觉特征变成纯白；材质仍与 Gazebo 有差异。手动模式不会再读上一轮 `task.log` 并误显示旧任务成功。
8. 全向控制关闭强迫机身对准路径切线的 PathAngleCritic，终点朝向仍由 GoalAngleCritic 约束；路径跟随前视从 4 个点增到 12 个点、权重从 5 增到 10，改善膨胀区边缘的停滞。CostCritic 的近目标范围与目标评分切换距离统一为 1.2 m，只抑制近目标软排斥，保留轮廓碰撞检查；其语义核对了 [Nav2 Humble 官方实现](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/src/critics/cost_critic.cpp)。运行中调参的诊断样本保存 `runtime_overrides.json`，不能当作固定配置验收。
9. 抓取收臂后，沿原接近方向低速退回后方 0.15 m 的预接近位，再交给 Nav2。这样避免操作位贴近台面时，5 cm 占用栅格的离散边界把导航起点轮廓判为碰撞。离台超时会终止任务并记录失败，不跳过检查。

## 当前边界

这是可验收的导航与操作演示平台，机器人仍由规定运动驱动，抓取仍为虚拟固定连接，释放后使用接触动力学。导航 odom 来自仿真真值；LIVO 目前是独立诊断/可视化流，不是导航定位闭环。不能把当前结果解释为真实四足稳定性、力矩控制、抓取力闭环或纯视觉闭环的证明。不同路线、GUI 状态或不同调试版本的墙钟耗时不能直接作为算法优劣结论。

正式方案尚未接入；本轮验收后再按用户提供的算法、对照方法和指标建立实验矩阵。

本轮分阶段验收汇总见 `output/expanded_demo/ACCEPTANCE.md`。北侧、90° 侧向路线通过后，新增的离台步骤又在自动、南侧路线完成专项复测；四条记录使用相同最终导航参数，但任务节点源码并非同一版本。记录中四次抓取均使用 RGB-D，最终放置均使用已记录的配置坐标回退。需要同版本重复统计时，使用上述串行测试入口重新运行。
