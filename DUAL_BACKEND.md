# Gazebo / MuJoCo 双后端与 RViz

当前默认主后端已切换为 **MuJoCo**，采用精修机器人与扩展工厂。路线/工位选择、验收记录和批量执行入口见 [EXPANDED_DEMO.md](EXPANDED_DEMO.md)。以下双后端机制仍适用；文末旧场景数据保留为历史记录。

## 一键运行（Windows PowerShell）

工作目录：`D:\20261022ubunturos_win`。容器为 `qda-dev`，VcXsrv 沿用本机现有配置。

```powershell
.\scripts\windows\run_sim.ps1 -Backend gazebo
.\scripts\windows\run_sim.ps1 -Backend mujoco
# 同时启动两个后端，各自带一个 RViz；已运行的后端自动跳过
.\scripts\windows\run_compare.ps1
# 只重开 RViz，不重置正在运行的仿真
.\scripts\windows\show_rviz.ps1 -Backend gazebo
.\scripts\windows\show_rviz.ps1 -Backend mujoco
# 结束指定后端；不关闭 Docker
.\scripts\windows\stop_sim.ps1 -Backend gazebo
.\scripts\windows\stop_sim.ps1 -Backend mujoco
.\scripts\windows\stop_sim.ps1 -Backend all
```

`run_sim.ps1` 默认打开后端 GUI 与 RViz，并执行一次抓取—导航—放置任务。`-Headless` 关闭两个 GUI；`-NoRviz` 仅关闭 RViz；`-Manual` 不自动执行任务；`-Background` 后台运行并把启动日志保存到 `output/*_session.log`。旧的 `-Rviz` 参数仍可使用。

Gazebo 使用 ROS_DOMAIN_ID=7，MuJoCo 使用 8。直接在容器运行 ros2 命令时务必选择对应的域。双开用于观察算法行为；比较速度时应分别运行同样的 GUI 配置，避免 CPU/软件渲染资源竞争。

## RViz 显示内容

| 分组 | 显示内容与来源 |
|---|---|
| 状态面板 | 后端名称、实时因子 RTF、速度指令、任务阶段、感知标签、LIVO 数据接收状态 |
| Navigation | NavFn 全局规划（青）、实际底盘路径（绿）、MPPI 局部参考路径与采样轨迹、机器人碰撞轮廓 |
| 障碍栅格 | `/map` 的已占用单元与局部 costmap 的非零代价单元；以几何方块显示，兼容 VcXsrv |
| Manipulation | MoveIt 规划关节轨迹换算的左臂指垫路径（紫）、实测关节换算的指垫路径（金）、RGB-D 目标标记 |
| Mapping | 三维实测几何点云、LIVO 彩色扫描；点云保留时间和路径长度受限，避免越跑越慢 |
| Cameras | 目标检测调试图与工厂俯视图 |

界面标签用英文，防止 RViz 字体缺失；任务日志仍为中文。参考工位的半透明方框是场景配置的参考轮廓，不是算法检测结果。纹理版地图默认关闭，仍可在显示列表手动启用；当前 VcXsrv 曾报告 indexed8bit 纹理着色器错误，因此默认使用方块栅格。

仓库中的 FAST-LIVO2 已将 `/cloud_registered` 转回 `imu_link`，显示时由仿真真值 odom 的 TF 定位。它是“真值定位的 LIVO 彩色扫描”，不能作为纯 LIVO 全局建图精度证据。`/path` 是估计轨迹诊断项；导航继续使用原有 `/odom`，不是 LIVO 闭环导航。原 `/mapping/colored` 要求 camera_init 输入，与这份 C++ 发布端不一致，因此本版 RViz 直接订阅实际有效的 `/cloud_registered`。

## 两个后端如何同步维护

场景只维护一份：`src/qda_kinematics/qda_kinematics/scene.py` 定义工位/障碍关键尺寸，`src/qda_bringup/qda_bringup/world_sdf.py` 生成完整场景。机器人来自同一份 Xacro 与 Aubo 注入代码。

MuJoCo 的 `model.py` 读取同一次生成的 URDF 和 SDF，导入关节树、惯量、碰撞体、视觉网格、工作台、货架、墙、桶、红块、绿色放置标记和相机位置，生成 MJCF。Aubo COLLADA 的毫米单位转换为米；网格转换缓存于容器 `/tmp/qda_mujoco_meshes`。重建容器后会重新生成，不需要复制隐藏资产。

| 内容 | Gazebo | MuJoCo |
|---|---|---|
| 导航/感知/规划/任务 | 同一套 Nav2 Omni MPPI + NavFn、RGB-D/YOLO辅助标签、MoveIt RRTConnect、自定义抓取 IK、任务状态机 | 完全复用 |
| 机器人运动 | 运动学底盘、位置式关节和几何步态 | 同样的规定运动；底盘速度约束/看门狗一致 |
| 抓取 | 插件虚拟固定连接 | 保存父子相对变换的虚拟固定连接 |
| 释放物体 | Gazebo 接触动力学 | MuJoCo 接触动力学 |
| 相机 | Gazebo 渲染 RGB-D | MuJoCo 实时渲染 RGB-D；不是读取目标真值生成检测 |
| 雷达 | Mid360 扫描模式 + 水平扫描 | 同一 CSV 扫描模式和 2400 点规模，基于场景射线求交；自机器人几何被排除 |
| 输出目录 | `output/` | `output/mujoco/` |

MuJoCo 兼容现有 `/cmd_vel`、`/joint_states`、`/odom`、相机/雷达/IMU话题、4个 FollowJointTrajectory 动作和 attach/detach/entity_state 服务。轨迹按给定导数使用线性/三次/五次插值，支持取消。保留 `gazebo_msgs/GetEntityState` 类型是为了任务层兼容，不代表后台依赖 Gazebo 服务器。

**当前是运动学对照后端，不是已经完成动力学等价验证的四足控制器。** 两边的纹理、接触求解器、网格凸化、传感器噪声和渲染颜色存在差异；MuJoCo 的机器人根位姿/关节被规定，不能拿本版衡量真实足端接触、力矩控制或跌倒恢复。当前 MuJoCo 仅支持 `lidar:=mid360`，相机可以沿用共享的 d435/d455/generic 标定。

## 本次联调发现并修复

- Aubo DAE 资产单位的 1000 倍尺寸问题；增加网格尺寸和 FK 一致性检查。
- MuJoCo 射线跳过透明碰撞几何：使用同尺寸的场景视觉几何进行射线求交，单独开启对应渲染分组。
- FAST-LIVO2 对 RGB/IMU 的 Reliable QoS 要求；桥接发布端与之兼容。
- 红色墙面误入目标色域：颜色分割增加面积上限，仍然使用真实 RGB-D 反投影与任务层工作区检查。
- 释放后 MoveIt 保留旧物体位置导致撤离碰撞：更新实际落点，先沿原接近分支抬腕，再恢复全部碰撞限制并收臂。
- 双后端 ROS 域、图片和任务日志隔离，按后端启动/停止，避免误停另一后端。

## 安装与验证

本机容器已经安装 MuJoCo 依赖。版本固定在 `docker/requirements-mujoco.txt`，Dockerfile 已加入安装步骤。现有容器可用：

```bash
pip3 install -r /ws/docker/requirements-mujoco.txt
source /opt/ros/humble/setup.bash
cd /ws
colcon build --symlink-install --base-paths src --packages-select qda_task qda_perception qda_bringup qda_mujoco --parallel-workers 2
source /ws/install/setup.bash
python3 /ws/scripts/test_expanded_scene.py
python3 /ws/scripts/test_mujoco_model.py
```

`test_mujoco_model.py` 检查模型编译、RGB-D 目标位置、墙体/障碍射线、网格单位、URDF与抓取FK的一致性、释放物体在桌面上的稳定性。它使用共享启动文件生成的 `output/mujoco/qda_robot.urdf` 与 `output/mujoco/pick_place.world`，将测试模型写入独立目录，不覆盖运行中的模型，不需要运行 Gazebo。`output/optimization_v1/` 内的脚本为旧装配历史记录，不再作为新模型的验收入口。

`compare_monitor.py` 只读记录各结果话题、仿真时间、实际路径和任务终态，例如：

```bash
ROS_DOMAIN_ID=8 python3 /ws/scripts/compare_monitor.py --output /ws/output/dual_backend/mujoco_metrics.jsonl --duration 600
```

实际联调日志与统计位于 `output/dual_backend/`；修改前备份位于 `output/dual_backend/baseline/`。早期 run1/run2/run3 是调试失败记录，不应当作成功对照数据。最终验收结果见本文件末尾。

## 下一版改进顺序

1. 固定随机种子，批量重复同场景的成功率、落点误差、路径长度和耗时对照；先把 GUI 性能与算法耗时分开。
2. RGB-D 颜色分割加入更可靠的候选几何检查、时间同步与遮挡恢复；减少工位坐标回退。
3. 保留当前运动学模式作为回归基线，另加关节驱动/足端接触/WBC 的动力学模式，再比较 Gazebo 和 MuJoCo 的真实接触表现。
4. 校验 LIVO 的时间戳、外参和独立估计误差，再考虑接入导航闭环；不能用真值对齐的彩色点云证明定位精度。

## 精修前版本的补充验收记录

2026-10-24，在本轮机器人外形精修之前，两后端最终回归均得到 TASK_SUCCESS：Gazebo 最终目标误差 0.021 m，MuJoCo 0.072 m（各一次运行，不是统计结论）。原始任务日志保存在 `output/dual_backend/gazebo_final_task.log` 与 `mujoco_final_task.log`。

随后用户要求参考照片精修机器人，安装位置、工具和碰撞轮廓已变化；这一版的模型、验证与使用入口请以 `MODEL_REFINEMENT.md` 为准，不能将上面的旧装配验收结果直接当作新模型的验收证据。
