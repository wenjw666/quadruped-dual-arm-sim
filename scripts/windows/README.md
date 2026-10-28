# Windows 操作入口

从仓库根目录使用 `scripts/windows/qda.ps1`。该入口统一分发参数，功能脚本仍可直接执行；公共路径和容器选择在 `common.ps1` 中维护。从任何目录调用脚本，都会定位到仓库根目录。

| 命令 | 功能 | 示例参数 |
| --- | --- | --- |
| `sim` / `stop` | 启动/停止仿真 | `sim -Brain -Mapping -Manual -Background` |
| `brain` | 本地模型检查及启动 | `-WithSimulation -Headless` |
| `pick` | 物理抓取、转移、放置 | `-FastValidation` |
| `navigation` | 绕场/多地点导航和视频输出 | `-Mode perimeter -Take demo` |
| `rviz` / `model` | RViz / 独立模型查看 | `model -Refresh` |
| `compare` / `scenarios` | 双后端及场景对照 | `scenarios -Repeat 3` |
| `train` / `bench` | PPO 训练 / RL 台架 | `bench -Seconds 30` |
| `voice` | Windows 离线语音样本 | `-TestSet` |
| `image` | 导入、启动、快照、导出镜像 | `-Mode load` |
| `product-render` / `product-narrate` | 历史演示片渲染/配音辅助 | 需要原有视频工程依赖 |

```powershell
.\scripts\windows\qda.ps1 help
.\scripts\windows\qda.ps1 sim -Brain -Manual -Headless -Background
.\scripts\windows\qda.ps1 pick -FastValidation
.\scripts\windows\qda.ps1 stop
```

可选环境变量 `QDA_CONTAINER` 指定目标容器，默认 `qda-dev`。详细的镜像生成原理、版本更新和 Ubuntu 迁移见 [部署说明](../../docs/DOCKER_DEPLOYMENT.md)。Windows SAPI 语音和历史视频工具属于 Windows 专用工具；仿真核心在 Linux 容器中运行。
