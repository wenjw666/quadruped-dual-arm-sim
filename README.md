# 四足双臂机器人仿真 · V2 开发分支

ROS 2 Humble 仿真工作区：宇树四足底盘、Aubo A3 双臂、Robotiq 2F-85 夹爪与顶部 RGB-D 相机。MuJoCo 为默认主后端，Gazebo 保留为对照后端，两者复用机器人、场景、感知、导航、操作与任务代码。

当前分支的 `run_sim.ps1` 默认启动 **V2 实验性 RL 工厂场景**：自由基座 + ONNX/PD、Smac 2D/MPPI、物理双臂、四相机和 MoveIt OctoMap。默认不自动启动旧版虚拟抓取任务。定位仍使用仿真真值，完整方案尚未验收。详情见 [V2 工程接入记录](docs/V2_FACTORY_INTEGRATION.md)。

## 本版内容

- 精修的机器人装配模型及共用 URDF → MuJoCo 转换。
- 9 × 8.3 m 扩展工厂，自动、南侧、北侧及 90° 侧向交付场景。
- NavFn / Omni MPPI 导航、MoveIt / 抓取 IK、RGB-D 感知、FAST-LIVO2 诊断流。
- RViz 算法可视化、任务指标、独立运行记录与轮廓间隙审计脚本。
- Docker 配置、Windows PowerShell 启停入口与模型/场景验证脚本。

## 当前环境运行

当前验证环境是 Windows + Docker Desktop + VcXsrv，容器名为 `qda-dev`，工作区挂载至 `/ws`。模型大文件由 Git LFS 保存，克隆后先执行 `git lfs pull`。

在已构建并配置完成的工作区中：

```powershell
.\run_sim.ps1 -Scenario auto -Background
.\stop_sim.ps1
.\run_sim.ps1 -Scenario side -Background
# V1 运动学取放基线
.\run_sim.ps1 -MotionMode kinematic -Scenario auto -Background
# Gazebo 对照
.\run_sim.ps1 -Backend gazebo -Scenario auto -Background
```

本提交保存当前源码基础，不包含本机 `build/`、`install/`、Docker 镜像、日志、实验输出、下载缓存或 YOLO 权重。全新环境需要安装依赖并执行 colcon 构建；`docker/` 是现有构建配置，`docs/baseline/` 保存当前容器的软件清单供核对。本次上传未进行从空机器重建验证，不应把它视为已经验证的一键部署镜像。YOLO 权重可由 Ultralytics 下载，RGB-D 颜色分割主流程不依赖其标签输出。

## 说明与验证边界

- [扩展场景、使用方法与本轮验收说明](EXPANDED_DEMO.md)
- [双后端机制与 RViz](DUAL_BACKEND.md)
- [机器人模型精修](MODEL_REFINEMENT.md)
- [前期优化记录](OPTIMIZATION_V1.md)
- [第三方源码快照说明](docs/THIRD_PARTY_SNAPSHOT.md)

V1 的四类路线均取得完整任务通过记录，但属于分阶段功能验证，不能作为同版本统计成功率。详细原始实验记录仍保存在本机 `output/`，未上传本仓库；可通过 `run_scenarios.ps1` 重新生成。

V1 基线采用规定机器人运动、虚拟抓取与真值里程计；感知失败时的配置坐标回退会显式记录。V2 已进入自由基座动力学和 RL 场景接入阶段，但物理抓取、纯视觉/激光定位、完整取放与板端性能仍待验收；不能沿用 V1 成功标签。

## 版本

`v0.1.0-foundation` 标记本次奠基快照。第三方组件保留其原有许可证与声明；本仓库没有为所有第三方代码重新授予统一许可证。
