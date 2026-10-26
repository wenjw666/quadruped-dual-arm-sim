# v0.2.0 · 第二版仿真工程稳定基线

此版本的“稳定”指当前 Windows + Docker / ROS 2 Humble 环境下，接口、资产契约、任务协同与失败保护形成可保存、可复查的工程基线。**不代表完整 PPT 方案、实机或全场景算法已经验收。** 默认后端为 MuJoCo，保留 Gazebo V1 对照。

## 发布前完善

- 增加运行资产 pack/verify/install，包含匹配的 ONNX、元数据、校准表、参考配置、精确参考 MJCF 和全部引用网格。SHA-256 校验、路径约束、完整预检及不同内容文件拒绝覆盖。
- 启动检查增加网格缓存及上游配置校验，避免只通过模型文件哈希却在加载时发现 `/tmp` 网格丢失。
- Docker 依赖补齐 MoveIt Servo / MPPI；补齐 bringup 的后端与大脑依赖，去掉 MuJoCo 对 bringup 的无实际使用反向依赖，检查 colcon 拓扑顺序。
- 构建限定 `--base-paths src`；视频项目使用 COLCON_IGNORE，避免扫描 Node 依赖。
- RViz 长篇世界坐标文字改为独立状态图像面板 `/viz/status_image`，完整文字仍在 `/viz/status_text`。默认隐藏无图的 IBVS 面板及冗余相机面板，需要时在 Displays 打开。
- 只读视频证据录制使用有界后台写盘，明确记录丢帧数、原始消息时间和来源。验收脚本支持独立报告路径，保留不同回合。
- 自研 qda 包统一版本号 0.2.0。

## 本次复验

- `coord-dual_visual-1793010424`：真实 Nav2 起点交接 → 左腕对准 → 右腕对准 → inspect，4/4 完成。左腕 **18.43 → 1.42 px**；右腕 **24.16 → 1.01 px**。
- 任务结束资源释放、外部技能排他、同 ID 重放一致、RL 无 fault；最大物理状态接收间隔 3.51 s。离线渲染低于实时，不将此数字作为实时控制保证。
- 8 项 Servo 门控、实际取消/重放、物理抓放拒绝、8 个 MCP 工具检查通过。
- 隔离域 Mission 顺序/失败/取消、仲裁/租约、冻结时钟、迟到动作取消检查通过。
- 4 项大脑单元检查、4 项协同检查、2 项资产包安装检查通过。
- 资产包恢复到独立空目录：172 个 MJCF 资产引用加载成功，未借用原网格路径；物理签名不变，ONNX 输出 (1,12) 有限值。原始 MJCF 文件字节与策略契约保持不变；仅加载测试时重定位资产路径。
- 完整启动后检查图、模型健康、唯一速度出口与传感器新鲜度；RViz 画面已检查。

第一轮同时采集、打包和运行受控测试时，多个话题出现约 9 s 墙钟间隔，触发 `MISSION_SESSION_EXPIRED` / `SKILL_SESSION_LOST`，任务按保护逻辑停止。记录保留为 take01；降低观察者磁盘压力并错开重任务后 take02 通过。没有放宽控制 TTL、租约期限或看门狗。不能把两轮开发试验称为统计成功率。

## 获取与恢复

仓库保持私有；以下命令需已有权限的 gh 账号。Release 运行包不包含私密配置、任务 SQLite、语音账号或模型云端凭据。

```powershell
git clone https://github.com/wenjw666/quadruped-dual-arm-sim.git
cd quadruped-dual-arm-sim
git checkout v0.2.0
git lfs pull
gh release download v0.2.0 --pattern 'qda-v0.2.0-runtime.tar.gz*' --dir output/release_v0.2.0
```

工作区须挂载到容器 `/ws`。容器以仓库 Dockerfile 为基础，具备 ROS Humble、编译工具、Gazebo/MoveIt/Nav2 和基础 Python 依赖。保留 DDS 与 X11 配置；当前 Windows 入口约定容器名 `qda-dev`。

首次安装（容器内，校验值取 Release 同名 `.sha256` 文件）：

```bash
cd /ws
bash scripts/setup_v2_release.sh \
  /ws/output/release_v0.2.0/qda-v0.2.0-runtime.tar.gz \
  64c1551ea2f412bdec2025bfac2a4e733c6c57d0996da472284268da85930e43
```

脚本会安装隔离 RL / 大脑 Python 依赖、恢复资产、构建 ROS 包并获取固定版本本地模型。尚未在全新 Docker 镜像完成这一整条安装验证；已验证运行资产恢复和现有容器构建。它不是可直接拉取的一键部署镜像。

已配置容器仅恢复丢失缓存时：

```bash
python3 scripts/release_runtime.py install \
  output/release_v0.2.0/qda-v0.2.0-runtime.tar.gz \
  --sha256 64c1551ea2f412bdec2025bfac2a4e733c6c57d0996da472284268da85930e43
```

恢复不会改写已有不同版本资产；冲突时应使用另一个干净工作区，不要覆盖新的实验结果。`/tmp` 清理后可再次恢复。

Windows 启动：

```powershell
.\start_brain.ps1 -WithSimulation -ServoFixture -NoPhysicsGui
```

避免在运行运动任务时同时进行视频最终渲染、大规模构建或高负载训练。本版本可视化不保证实时帧率。暂停/过期会停止，任务不会自动重做运动步骤。

## 仍未开放或未完成

完整物理抓取放置、无标记语义抓取、复杂长路线统计、载荷鲁棒 RL、真实定位主源、完整 BT.CPP/MTC、小智云端/真人语音、RDK X5 板端性能。Gazebo 保留 V1 功能，不宣称完成 V2 同等验证。

视频是单独交付的本地审片成果，确认后才将入口加入 README。当前 Release 不含未经用户确认的演示视频。
