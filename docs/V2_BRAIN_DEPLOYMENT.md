# V2 本地大脑、语音与视觉伺服

本轮对应方案 PPT 第 18–23 页，保留 B2 + Aubo 本体、MuJoCo 主后端、现有实验性 RL 策略。所有新动作经 ROS 技能服务执行。模型输出只允许语义字段，不能生成关节命令、任意坐标或 shell 命令。

## 模块与边界

| PPT 模块 | 当前实现 | 验收边界 |
|---|---|---|
| SmolVLM-500M Q8 + 投影器 | 官方 GGUF 和匹配 mmproj，llama.cpp CPU 本地服务，约束 JSON，事件触发 | 本机容器测试；不是 RDK X5 板端性能结论 |
| 小智语音 | 固定版本官方源码和 MCP 配置，白名单工具桥 | 未配置账号与音频设备，不启动云端 ASR/LLM/TTS |
| 本地离线语音 | Vosk 中文小模型 + Windows 中文 SAPI 合成文件 | 测试文件不等于真人麦克风评测；短机械臂口令有同音误识别 |
| 任务决策/黑板 | 本地语义入口、目标登记/版本、SQLite、幂等 ID、1–12 步计划、技能会话、取消确认 | 受限序列状态机；不是完整 BehaviorTree.CPP / MTC 实现 |
| 导航与回位 | 调用现有 Nav2 / MoveIt 规划与执行 Action | 导航仍使用仿真真值定位 |
| 腕部 IBVS | ArUco 四角 + PnP 深度 + 阻尼伪逆，限速，准备姿态，MoveIt Servo | 独立光学校准板测试；未证明物体抓取或任意纹理伺服 |
| 控制互斥 | 底盘 hold/nav 租约、技能会话、同臂轨迹/Servo 互斥、双时钟新鲜度与看门狗 | 取消保持关节；下游停止未确认会隔离执行并要求整套重启 |
| 物理抓放 | `pick_place` 语义接口已登记，但明确返回未验证 | 不能以虚拟吸附报告成功；力控、2 cm 放置验证、完整 MTC 尚待实施 |

## 启动

容器名称 `qda-dev`，MuJoCo ROS 域号 `8`，Gazebo 域号 `7`。MCP 必须显式使用与 MuJoCo 相同的 CycloneDDS 配置，不能依赖 MCP 子进程继承环境。

首次部署（容器内）：

```bash
bash /ws/scripts/setup_v2_brain.sh
```

脚本安装 MoveIt Servo，将语音/MCP Python 依赖放在 `/opt/qda-brain-python`，下载并校验模型，解压运行时并编译 ROS 包。网络有代理时给 Docker exec 传入已有代理环境变量即可，不要关闭 TLS 校验。模型、下载缓存、日志和 SQLite 位于 git 忽略的 `output/v2_brain/`。

日常启动（Windows 项目目录）：

```powershell
.\scripts\windows\start_brain.ps1 -WithSimulation
# 带光学校准板的视觉验收场景：
.\scripts\windows\start_brain.ps1 -WithSimulation -ServoFixture
# 无图形界面验证：
.\scripts\windows\start_brain.ps1 -WithSimulation -Headless
```

已启动仿真时只运行 `.\scripts\windows\start_brain.ps1` 启动/检查模型；模型仅监听容器环回地址 `127.0.0.1:8088`。独立仿真入口为 `.\scripts\windows\run_sim.ps1 -Brain -Manual -Background`。不会启动麦克风或连接小智云。

容器内使用语义入口：

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
export ROS_DOMAIN_ID=8
export PYTHONPATH=/opt/qda-brain-python:$PYTHONPATH
python3 /ws/scripts/brain_cli.py --text '查询状态'
python3 /ws/scripts/brain_cli.py --text 'Navigate to the north entry' --vlm
python3 /ws/scripts/brain_cli.py --wav /ws/output/v2_brain/voice/status.wav
python3 /ws/scripts/brain_cli.py --text '左臂回位' --execute --task-id demo-home-001
python3 /ws/scripts/brain_cli.py --pause
```

默认仅解释并返回提案。显式 `--execute --task-id` 才提交动作；相同 ID/参数重复调用返回已有结果，不重复运动。`--status` 查询执行结果；`--pause` 取消当前技能并保持底盘。语音识别置信度不是语义正确率，不能把高置信度同音误识别强行映射成机械臂动作。

SmolVLM 当前仍有语义错误，未通过自主执行验收。带 `--vlm` 的提案只有与受限本地语法一致时才允许 CLI 提交；其余返回 `model_requires_review`，不会自动改成正确结果再声称模型成功。

SQLite 会把进程重启时尚未完成的任务标记为 `INTERRUPTED`。目前请通过整个仿真 launch 停止/重启，尚未实现下游 Action 的精确热恢复。此实现不能替代 PPT 中完整的故障恢复状态机。

本地语音测试文件生成：

```powershell
.\scripts\windows\synthesize_offline_voice.ps1 -TestSet
```

脚本只写 WAV，不录制麦克风、不播放音频、不上传声音。SAPI 是本轮离线 TTS 替代，不是已验证的小智云端 TTS。

## 视觉验证与显示

标准场景不含光学校准板。专用验收启动参数 `brain:=true servo_fixture:=true` 会增加两块不参与碰撞的板，并保持机器人质量、接触、驱动参数不变。标定板世界位姿只在场景构建时计算一次；运行控制使用图像角点和 PnP 深度，不读取板真值位置。

`visual_align` 先用 MoveIt 规划到腕部余量充足的准备姿态，底盘稳定后切换 Servo 控制权。相机为仿真时间 5 Hz；图像仿真年龄超过 0.5 s 或离线墙钟年龄超过 15 s 时停止。ArUco 失检时最多采用两帧、0.45 s 的 KLT 过渡，必须通过前后向、残差、位移、凸四边形与 PnP 重投影检查；失败立即零速度，短暂等待重获，未恢复则退出。四角 RMS 连续四帧低于阈值且四帧都重新检测到标记才报告成功。

准备姿态通过 FCL 检查腕部间隙，避免照搬通用机械臂参数导致长期减速。Servo 使用规划组的基座和夹爪末端；相机速度通过 TF 的旋转与参考点偏移转换。适配层处理 Humble 的零时间戳约定，并按 MuJoCo 物理时间积分有界关节速度，避免墙钟与仿真时钟混用。选定手臂启用关节偏置力矩补偿，仍经过原有电机力矩上限；四足策略、机器人质量、接触参数不变。取消后保持位置，不开启夹爪。

RViz 新增 `/brain/servo_overlay` 角点画面，现有状态文字增加技能结果和 IBVS 误差/深度。诊断话题：

- `/brain/status`：活动技能、最近结果、场景版本。
- `/brain/servo_metrics`：像素误差、深度、矩阵条件数、速度、MoveIt 状态。
- `/motion/arm_modes`：轨迹/Servo 所有权、实际接受的 Servo 消息数。
- `/cameras/{left,right}_wrist/image_raw`：真实 MuJoCo 渲染图像。

## 小智与 MCP

`config/xiaozhi_mcp.local.json` 使用官方 `mcpServers` stdio 格式，启动 `scripts/run_robot_mcp.sh`。八个工具为 `status`、`interpret_text`、`submit_skill`、`pause`、`cancel`、`submit_mission`、`mission_status`、`cancel_mission`，动作参数再次经过本地白名单校验。官方源码位于 `/opt/qda-brain-runtime/xiaozhi-in-rdk-<commit>/`，保持未激活状态。未来接入账号和音频设备时单独验证 MQTT/UDP、Opus、断网与端到端延迟。

## 多步骤任务与模块协同

见 [代码结构与关系](CODE_ARCHITECTURE.md) 和 [协同改进与验收](V2_COORDINATION.md)。容器内可生成并审阅有界计划：

```bash
python3 /ws/scripts/make_mission_plan.py --preset handoff --out /ws/output/v2_brain/handoff.json
python3 /ws/scripts/brain_cli.py --plan /ws/output/v2_brain/handoff.json
python3 /ws/scripts/brain_cli.py --plan /ws/output/v2_brain/handoff.json --execute --task-id demo-handoff-001
python3 /ws/scripts/brain_cli.py --mission-status demo-handoff-001
python3 /ws/scripts/brain_cli.py --mission-cancel demo-handoff-001
```

`handoff` 是到起点→左臂回位→右臂回位→查询；`dual_visual` 是到起点→左/右视觉对准→查询，需要 `-ServoFixture`；`inspection` 是到检查点→查询→返回，长路线尚未作本轮正式验收。计划先检查全部步骤，未开放的物理抓放会在任何动作前拒绝。单句中的复合指令会要求改用显式计划。

计划持有技能会话，外部技能不能插入；每个运动技能持有底盘租约。租约过期、状态/时钟超过 5 s 墙钟不更新均保持底盘。MuJoCo RL 状态按墙钟频率从真实物理更新处发布，避免慢速渲染时按仿真间隔造成误报。版本号包含场景和目标登记，新增目标后必须重新生成计划。

## 可复核入口

本轮记录见 [验收汇总 JSON](validation/v2_brain_20261026.json)。双腕标定板分别为 **21.90 → 1.42 px** 和 **23.34 → 0.93 px**；两侧 Servo 已验证接收相同的 `odom`、0.05 m 分辨率 OctoMap，接通共享场景后另一次深度调整从 **5.47 → 1.49 px** 通过。控制互斥、错误输入拒绝、任务取消/重放、WAV→状态技能和真实 MCP 协议检查通过。

SmolVLM 小样本 **10/12**，短句离线 ASR **3/4**，均保留失败样本；不能解释为模型泛化准确率。一次图像请求约 5 s，文本请求主要在 1 s 左右，冷缓存请求更慢。所有时间来自当前 x86 容器，不代表 X5。视觉误差曲线位于 `output/v2_brain/ibvs_validation.png`。

```bash
python3 /ws/scripts/test_v2_brain_unit.py
python3 /ws/scripts/test_v2_brain_ros.py
python3 /ws/scripts/test_v2_brain_ros.py --arm right
python3 /ws/scripts/test_v2_servo_gate.py
python3 /ws/scripts/test_v2_brain_control.py
python3 /ws/scripts/test_v2_shared_scene.py
PYTHONPATH=/opt/qda-brain-python:$PYTHONPATH python3 /ws/scripts/test_v2_voice.py
PYTHONPATH=/opt/qda-brain-python:$PYTHONPATH python3 /ws/scripts/test_v2_mcp.py
python3 /ws/scripts/test_v2_model.py
python3 /ws/scripts/summarize_v2_brain.py
```

ROS 测试需要运行带标定板的场景；模型测试需要本地服务就绪。每项报告保存在 `output/v2_brain/*acceptance.json`。只有真实通过的检查才能进入汇总，不把 JSON 格式正确等同于指令语义正确。

## 上游依据与固定版本

- [SmolVLM 官方 GGUF](https://huggingface.co/ggml-org/SmolVLM-500M-Instruct-GGUF)，revision `72e986006ef53e37cdd3f6d4241c90b0f01df376`，语言模型和投影器分别校验上游 SHA-256。
- [llama.cpp b11878](https://github.com/ggml-org/llama.cpp/releases/tag/b11878)，CPU Ubuntu x64 官方二进制及 release SHA-256。
- [地瓜机器人官方小智项目](https://github.com/D-Robotics/xiaozhi-in-rdk/blob/main/README_cn.md)，commit `ade6cd1cc25ee105dc7d0fee600c56b8dd8b4305`。
- [Vosk 模型目录](https://alphacephei.com/vosk/models)，`vosk-model-small-cn-0.22`；记录下载实际校验值，不声称存在上游公布的 SHA。
- [MoveIt Humble Servo](https://moveit.picknik.ai/humble/doc/examples/realtime_servo/realtime_servo_tutorial.html)，使用已安装 `ros-humble-moveit-servo`；保留碰撞/奇异性/限位停止。
