# Docker 镜像的打包、更新与迁移

## 当前 tar 是如何产生的

先对已经安装好依赖的 `qda-dev` 做 `docker commit qda-dev qda-ros:humble-v0.2.0-container`，再执行 `docker save --output qda-ros-humble-v0.2.0-container.tar qda-ros:humble-v0.2.0-container`。前者保存容器可写层，后者保存镜像配置及其全部镜像层。这不是 `docker export`，可通过 `docker load` 恢复镜像。

镜像 ID：`sha256:673a0aa2f82918926c6dc5dcaad23dae2d9d4b9d47c6b0507b47d7881ce917a0`。
tar 字节数：`2280816128`；SHA-256：`10a6860cf369d621779c9277859912c36c722d510a1633b7b48cf8dfb26fe7e9`。

**镜像保存安装环境，不保存挂载进来的 `/ws`。** 源码、编译结果、策略、模型权重和 `output/` 内资产需要另行提供。`commit` 也不保存内存中的 ROS 节点状态或正在执行的任务；启动后应从初始场景重新运行。

## Windows 上恢复

Docker Desktop 必须启用 Linux 容器（WSL2），使用 x86_64/amd64。以下命令从仓库根目录执行，PowerShell 脚本支持 Windows PowerShell 5.1 / PowerShell 7。运行一次性脚本时如遇本机执行策略拦截，可用 `powershell -ExecutionPolicy Bypass -File scripts/windows/qda.ps1 help`，无需修改系统策略。

```powershell
.\scripts\windows\qda.ps1 image -Mode load
.\scripts\windows\qda.ps1 image -Mode start
```

`start` 通过 Compose 使用快照镜像，挂载当前仓库到 `/ws`，保留 GPU、2 GB 共享内存和 DDS 配置。默认容器仍为 `qda-dev`；若已有同名开发容器，Compose 可能重新创建它，先保存其未打包改动。镜像加载本身不会修改旧容器。镜像依赖版本位于环境内部，项目目录不能只复制 README，必须包含源码和运行资产。

全新源码目录还需要：`git lfs pull`、从 Release 获取并校验运行资产包，执行 [v0.2.0 安装恢复流程](RELEASE_V0.2.0.md)；编译到 `/ws/install` 并恢复本地 VLM/ASR 资产。`build/`、`install/` 中绝对路径都按 `/ws` 构建。验证原始 tar 及现有完整工作区，不等于验证只克隆源码即可离线运行。

```powershell
# 无窗口运行；渲染和 RGB-D 建图仍开启
.\scripts\windows\qda.ps1 sim -Brain -Mapping -Manual -Headless -Background
.\scripts\windows\qda.ps1 pick -FastValidation
.\scripts\windows\qda.ps1 stop

# 检查并启动本地模型、仿真与 Windows 窗口
.\scripts\windows\qda.ps1 brain -WithSimulation
```

桌面 MuJoCo / RViz 另需 Windows X11 服务，允许 Docker 连接 `host.docker.internal:0.0`。GUI 显示服务属于宿主机，不包含在镜像中。软件 OpenGL 开销较大；`-Headless` 关闭窗口，`-NoImages` 才关闭图像渲染。物理抓放当前使用仿真真值定位，不等同于视觉抓取。

## 后续开发后如何更新

导入/导出镜像、批量模型检查、压缩视频会竞争 CPU 和磁盘；请与实际机器人任务串行执行。首次启动会生成场景、加载网格和策略，后台入口返回只表示启动请求已发出。日志中出现 MuJoCo 就绪、Nav2 managed nodes active 后再提交任务；启动阶段暂时缺少 TF 不应当成最终运行结果。

| 改动 | 更新方式 |
| --- | --- |
| Python 源码、配置、launch | 提交源码；重启相应节点。symlink-install 的 Python 改动通常不需要重做镜像 |
| C++、消息接口、新 ROS 包 | 在 `/ws` 内重新 colcon build，然后重启；编译结果仍在挂载目录 |
| ONNX、动力学模型、校准 | 同步对应资产、哈希及运行包，完成资产契约检查；不能只换单个模型文件 |
| apt/pip/系统运行库 | 同步修改 Dockerfile/依赖清单，验证后生成新版本镜像与 tar |

先停止运动任务，使用新版本标签；保留上一版归档。比如：

```powershell
.\scripts\windows\qda.ps1 stop -Backend all
.\scripts\windows\qda.ps1 image -Mode snapshot -Image qda-ros:humble-v0.2.1-container
.\scripts\windows\qda.ps1 image -Mode save -Image qda-ros:humble-v0.2.1-container -Archive qda-ros-humble-v0.2.1-container.tar
```

脚本拒绝覆盖已有归档，并生成同名 `.sha256` 校验文件。默认从 `qda-dev` 打快照，可设置 `$env:QDA_CONTAINER='另一个容器名'`。快照默认短暂暂停容器保证文件系统一致；不会恢复运行任务的内存状态。

长期发布同时维护两条路径：快照留存当前可用环境；Dockerfile + 固定依赖 + 对应代码提交 + 运行资产包用于可重建部署。每次更新后在新容器中跑资产检查、ROS 网络采样和实际取放，记录验证代码提交及镜像 ID。不要直接把数 GB 的 tar 提交 Git；镜像归档和运行包使用 Release 或镜像仓库分发。

## Ubuntu 迁移

同一份 Linux amd64 镜像可在 Ubuntu x86_64 Docker Engine 使用：

```bash
docker load --input qda-ros-humble-v0.2.0-container.tar
QDA_IMAGE=qda-ros:humble-v0.2.0-container docker compose up -d --no-build
docker exec -e PYTHONPATH=/opt/qda-rl-v2 qda-dev python3 /ws/scripts/check_v2_scene_assets.py
docker exec -it -e PYTHONPATH=/opt/qda-rl-v2 -e ROS_DOMAIN_ID=8 qda-dev bash -lc 'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; ros2 launch qda_bringup pick_place.launch.py backend:=mujoco motion_mode:=rl brain:=true gui:=false rviz:=false run_task:=false'
```

当前 Compose 请求 NVIDIA GPU；Ubuntu 需配好 NVIDIA Container Toolkit，或者按实际硬件调整 Compose，当前默认 MuJoCo 软件渲染和 CPU ONNX 不要求 CUDA。Ubuntu 桌面 X11 的 DISPLAY 与 socket 挂载需要按宿主环境配置，不能沿用 Windows 的 `host.docker.internal:0.0`。Ubuntu GUI 与 ARM64 平台本轮未验证。

## 2026-10-28 Windows 实测范围

使用原始 tar 执行 `docker load`，以镜像 `673a0aa2f829` 创建独立容器 `qda-tar-validation`，挂载当前完整工作区；原 `qda-dev` 未替换。已通过归档 SHA-256 校验、3 项场景契约测试、资产检查、完整启动后的 ROS 网络采样和本地模型健康检查。采样包含 35 个节点、150 个话题及 27 个有数据的话题；这不表示每个话题都已做功能验收。RViz 已在 Windows X11 窗口实际显示模型、场景点云及路线标记。

任务实测发现雷达地图把四足俯仰时的地面回波标成障碍，已在挂载源码中修正按采样时间取 TF 和完整三维投影，新增 3 项回归测试通过。该修复属于工作区代码，不需要改写环境 tar。修复后路线可规划，但一轮实测发生携物滑落，保护逻辑终止任务；完整取放的最终结果见下方 take06 验收记录，不将启动成功等同于任务成功。测试明细保存在本机 `output/docker_tar_validation/`。

最终取放复验 `physical-pick-place-1793176988996585822`（take06）通过：生产 Brain API 返回 SUCCEEDED，后端终态 `placed` / completed=true / error 为空；真实接触夹持、RL 自由基座携物导航、释放、退臂和台面稳定校验均完成，无虚拟附件。最大滑移 7.62 mm、最大夹持接触穿透 0.86 mm，物体由 `place_marker` 支撑，终点约 (4.9863, 0.2000, 0.5780) m。本轮使用无图像渲染的物理任务配置；先前已另行验证有图像和建图的完整启动，不能把两项合称为全负载同步取放通过。验收脚本增加启动就绪等待及 30 秒物理预热；此前失败说明鲁棒性仍需多轮评估，不宣称统计成功率。原始归档未改写，部署需配套本轮修正的源码。

补充桌面复核：MuJoCo 原生 `qda_factory` 窗口已在 Windows VcXsrv 中实际显示 B2 双臂、取放台、红色物体、目标标记和障碍场景；同时读取到 RL ready=true、fault 为空、physics_time=34.204 s，确认不是仅打开静态模型。该复核仍使用原始 tar 创建的 `qda-tar-validation` 容器。RViz 的显示复核已在前一轮通过；相机面板在 `-NoImages` 配置下显示 No Image 属于预期，需要图像时应去掉该参数。
