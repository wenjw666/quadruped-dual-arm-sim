# 四足双臂机器人仿真 · V2 工程稳定基线

**当前版本：v0.2.0** · [版本发布与安装](docs/RELEASE_V0.2.0.md) · [代码结构](docs/CODE_ARCHITECTURE.md) · [本轮验证摘要](docs/validation/release_v0.2.0.json)

本版本固定已经联调的工程接口、运行资产和验证记录，适合继续开发和复现实验。RL 仍为实验性，完整物理取放、真实定位与板端部署尚未验收。

ROS 2 Humble 仿真工作区：宇树四足底盘、Aubo A3 双臂、Robotiq 2F-85 夹爪与顶部 RGB-D 相机。MuJoCo 为默认主后端，Gazebo 保留为对照后端，两者复用机器人、场景、感知、导航、操作与任务代码。

当前分支的 `run_sim.ps1` 默认启动 **V2 实验性 RL 工厂场景**：自由基座 + ONNX/PD、Smac 2D/MPPI、物理双臂、四相机和 MoveIt OctoMap。默认不自动启动旧版虚拟抓取任务。定位仍使用仿真真值，完整方案尚未验收。详情见 [V2 工程接入记录](docs/V2_FACTORY_INTEGRATION.md)。

## 本版内容

- 精修的机器人装配模型及共用 URDF → MuJoCo 转换。
- 9 × 8.3 m 扩展工厂，自动、南侧、北侧及 90° 侧向交付场景。
- V2 Smac 2D / MPPI 导航、MoveIt / IBVS Servo、RGB-D 感知；V1 保留 NavFn 与 FAST-LIVO2 诊断路径。
- 本地 SmolVLM、离线 WAV 语音、MCP、带持久化记录的多步骤技能编排和控制租约。
- RViz 算法可视化、任务指标、独立运行记录与轮廓间隙审计脚本。
- Docker 配置、Windows PowerShell 启停入口与模型/场景验证脚本。

## 当前环境运行

当前验证环境是 Windows + Docker Desktop + VcXsrv，容器名为 `qda-dev`，工作区挂载至 `/ws`。模型大文件由 Git LFS 保存，克隆后先执行 `git lfs pull`。

在已构建并配置完成的工作区中：

```powershell
.\start_brain.ps1 -WithSimulation -ServoFixture -NoPhysicsGui
# 普通场景与 V1 对照入口：
.\run_sim.ps1 -Scenario auto -Background
.\stop_sim.ps1
.\run_sim.ps1 -Scenario side -Background
# V1 运动学取放基线
.\run_sim.ps1 -MotionMode kinematic -Scenario auto -Background
# Gazebo 对照
.\run_sim.ps1 -Backend gazebo -Scenario auto -Background
```

源码不包含本机 `build/`、`install/`、Docker 镜像、任务数据库或日志。V2 Release 附件提供带 SHA-256 校验的精确运行资产包，补齐策略、校准表及模型网格依赖；大脑模型由固定版本下载脚本获取。全新环境仍需安装依赖并执行 colcon 构建，见[安装说明](docs/RELEASE_V0.2.0.md)。已经验证资产在独立空目录恢复和加载，尚未完成空机器 Docker 全量重建验收。YOLO 标签不是 RGB-D 颜色几何主流程的必要条件。

## 说明与验证边界

- [扩展场景、使用方法与本轮验收说明](EXPANDED_DEMO.md)
- [双后端机制与 RViz](DUAL_BACKEND.md)
- [机器人模型精修](MODEL_REFINEMENT.md)
- [前期优化记录](OPTIMIZATION_V1.md)
- [第三方源码快照说明](docs/THIRD_PARTY_SNAPSHOT.md)
- [整体代码结构、模块关系和执行链](docs/CODE_ARCHITECTURE.md)
- [本地大脑与语音启动](docs/V2_BRAIN_DEPLOYMENT.md)
- [多模块协同改进与验收](docs/V2_COORDINATION.md)

V1 的四类路线均取得完整任务通过记录，但属于分阶段功能验证，不能作为同版本统计成功率。详细原始实验记录仍保存在本机 `output/`，未上传本仓库；可通过 `run_scenarios.ps1` 重新生成。

V1 基线采用规定机器人运动、虚拟抓取与真值里程计；感知失败时的配置坐标回退会显式记录。V2 已进入自由基座动力学和 RL 场景接入阶段，但物理抓取、纯视觉/激光定位、完整取放与板端性能仍待验收；不能沿用 V1 成功标签。

## 版本

`v0.1.0-foundation` 保留第一版奠基快照；`v0.2.0` 固定第二版工程基线。第三方组件保留其原有许可证与声明；本仓库没有为所有第三方代码重新授予统一许可证。
