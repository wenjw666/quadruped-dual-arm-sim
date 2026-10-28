# 精修机器人模型 V1（参考照片驱动）

本版保留现有宇树 B2 四足和现有 Aubo A3 双臂 URDF，重做了上身装配。单张参考照片不能恢复精确尺寸，当前立柱、安装法兰和相机外壳按照片比例与现有 A3 网格尺度建模；不是实机测绘或可直接加工的工程图。

## 查看当前版本

```powershell
.\scripts\windows\show_robot_model.ps1
# 调整代码/尺寸后重新生成 URDF、SDF、MJCF 与预览
.\scripts\windows\show_robot_model.ps1 -Refresh
```

独立查看窗口中用鼠标拖动旋转、滚轮缩放；O 打开夹爪，C 闭合夹爪，R 恢复默认视角。该窗口只用于模型外形和机构联动评审，没有运行搬运任务。完整仿真仍使用 `run_sim.ps1 -Backend gazebo` 或 `-Backend mujoco`，需要静止查看时加 `-Manual`。

## 本版改动

- 下移并收窄背板，增加圆角外壳、金属顶板、四个短支撑、侧面通风槽、安装螺栓和服务电缆。
- 将两个独立竖向臂座改为中央渐缩立柱、横向肩梁和两侧圆形安装法兰；左右 A3 保留原有关节树与网格，不缩放机械臂。
- 顶部相机采用圆角横向外壳、深色前窗、双镜头、中心小孔、指示灯和带接缝的支架；保留原相机成像参数，外参按新 URDF 更新。
- 保留 Mid360 仿真传感器，移至中央柱后侧并增加支架。
- 用 Robotiq 2F-85 原始 CAD 网格替换原先的方块二指夹爪。保留外连杆、内连杆与指尖关节，两个主驱动沿用现有 ROS 控制器名称，辅助关节按 URDF mimic 联动。
- 重算双臂自然下垂姿态与搬运姿态，更新夹爪角度、TCP、IK、MoveIt 刚性装配/夹爪机构内部碰撞规则和传感器外参。
- 按新肩宽增大导航碰撞轮廓，并调整抓取接近点的横向偏置；这属于新装配的必要参数同步。

## 主要比例与坐标

坐标为 base_link 下的米制值，+X 向前、+Y 向左、+Z 向上。

| 项目 | 当前值 |
|---|---|
| 背板 | 长 0.54 m、宽 0.40 m、厚 0.036 m |
| 背板安装位置 | (0.02, 0, 0.105) |
| 中央柱基座 | (0.16, 0, 0.151) |
| 左右机械臂安装位置 | (0.16, ±0.105, 0.43) |
| 左右安装朝向 | 绕 X 分别旋转 −90° / +90°，肩部朝两侧 |
| 相机位置 | (0.185, 0, 0.592)，俯角 0.35 rad |
| 相机外壳 | 宽约 0.198 m、高约 0.052 m、厚约 0.044 m |
| 夹爪最大开口 | CAD 几何测得约 85 mm |
| 当前 40 mm 方块的夹爪命令 | 0.42 rad，对应约 43 mm 开口；沿用虚拟吸附抓取 |

## 维护入口

- `src/qda_kinematics/qda_kinematics/assembly.py`：安装坐标和工具尺寸。
- `src/qda_bringup/qda_bringup/refined_assembly.py`：中央柱、背板、相机、法兰、夹爪装配及 SRDF 同步。
- `scripts/build_refined_meshes.py`：可重复生成自建圆角壳体与线缆网格。
- `src/qda_description/meshes/refined/`：自建网格。
- `src/qda_description/meshes/robotiq_2f85/`：带来源、固定版本和许可证的 Robotiq 网格。
- `output/model_refinement_v1/qda_refined.urdf`：已展开的完整机器人 URDF；Gazebo SDF 与 MuJoCo studio.xml 在同目录。
- `output/model_refinement_v1/`：真实模型渲染、姿态求解记录、碰撞检查与控制器验证日志。
- `output/model_refinement_v1/baseline/`：本轮修改前的装配与运动学文件。

Robotiq 资产来自官方 [robotiq/robotiq-ros](https://github.com/robotiq/robotiq-ros/tree/be8aa38abb2f84e85adb9d6862092b9dfc0124c5/grippers/robotiq_description)，固定提交 `be8aa38abb2f84e85adb9d6862092b9dfc0124c5`。BSD-3-Clause 许可证随资产保留。模型关节已针对现有双驱动控制接口适配；未接入真实 Robotiq 硬件驱动或力控。

## 已验证与边界

- 完整 URDF 树无重复链接，SDF 转换与 MuJoCo 编译成功。
- URDF 与现有解析 FK 的指垫位置一致到数值精度；新工作区的抓取/抬升 IK 有解。
- 双臂 home、搬运姿态和可用抓取分支通过 MoveIt 全机器人碰撞检查。部分逆解分支仍存在真实腕部自碰撞；任务节点会在规划前通过 `/check_state_validity` 筛选完整抓取/抬升姿态组，并保留负例记录，没有关闭腕部碰撞检查。
- MuJoCo 和 Gazebo 的左夹爪驱动及四个从动关节开合验证均通过。Gazebo Humble 将从动关节状态名字加 `_mimic` 后缀，TF 使用 URDF mimic 关系，测试同时核对真实发布值。
- 相机壳体外观按照片制作，传感器内参仍为既有 d435 配置；不会因为外壳看起来不同而宣称它是某个精确实机型号。
- 中央装配惯量仍是仿真用近似参数，当前平台仍采用规定运动与虚拟抓取。外形精修不等于已完成真实动力学辨识。

### 本版完整任务回归（2026-10-24）

新装配在 MuJoCo 后端完成抓取、抬升、收臂、Nav2 搬运、放置、释放和撤腕，记录为 `TASK_SUCCESS`。最终物体位置 `(2.177, -0.984, 0.578)` m，目标 `(2.150, -1.000, 0.570)` m，三维误差约 **0.032 m**。日志见 `output/model_refinement_v1/final_task.log`。本次放置标记未被视觉稳定识别，使用已配置工位坐标；此结果不是纯视觉闭环成功率评估。

Gazebo 已成功加载相同新版 URDF，RViz 正在显示该模型；新版夹爪驱动/从动关节测试通过。此次完整搬运回归在 MuJoCo 执行，不能把旧模型的 Gazebo 完整任务结果当作新版回归。

任务适配修复：扩大现有数值 IK 的分支保留数量，并使用 MoveIt 完整网格和附着物状态筛选抓取/抬升姿态组，解决仅靠关节采样点无法识别腕部壳体自碰撞的问题。失败过程保留在 `task_before_collision_filter.log`，成功结果独立存档。

本版是一轮可审阅的实际模型。下一轮可直接针对肩宽、柱高、相机尺寸、背板位置、双臂姿态与表面细节调整。

