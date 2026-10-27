# 功能实录

此轮按用户要求替换文字解说片：主体为同一轮运行的原始 MuJoCo 图像，辅以真实腕部图像、OccupancyGrid、Nav2 路径及脑/任务接口状态。无机器人姿态重放、无配音、无架构动画。只调整画面布局和明确标注的播放时钟。

`scripts/run_live_demonstration.py` 调用已有接口，记录每条输入、实际技能提案、语义门控、整份计划校验和执行结果。**输入顺序由操作者指定**；SmolVLM 负责单技能提案，不宣传为自由多步骤规划。`--vlm` 下提案不通过会停止，不会静默退回规则。

容器中先 source ROS 和工作区（ROS_DOMAIN_ID=8），运行：

```bash
export PYTHONPATH=/opt/qda-brain-python:${PYTHONPATH:-}
python3 /ws/scripts/run_live_demonstration.py --mode dual_visual \
  --record-to /ws/output/live_demo/dual_new --out /ws/output/live_demo/dual_new_result.json
python3 /ws/scripts/run_live_demonstration.py --mode brain_home --vlm \
  --status-wav /ws/output/v2_brain/voice/status.wav \
  --record-to /ws/output/live_demo/brain_new --out /ws/output/live_demo/brain_new_result.json
python3 /ws/scripts/run_live_demonstration.py --mode inspection \
  --record-to /ws/output/live_demo/nav_new --out /ws/output/live_demo/nav_new_result.json
```

先双臂对准，再回位，保证录像能看到实际位姿变化。WAV 是原有合成语音测试文件，不是现场麦克风。录制目录不可重复使用，失败回合也保留。运行器先等待各模块就绪和真实近景帧，再提交计划，退出时结束自己的只读录制进程。

Windows 合成：

```powershell
python scripts/render_live_demonstration.py --take output/live_demo/brain_new --out output/live_demo/brain.mp4 --title '语义提案与双臂回位'
```

默认按仿真时钟 1× 播放，源帧不插值生成；可选 `--clock wall --speed 4` 展示明确标注的墙钟倍速。JSON 保留原始时钟范围、掉帧和来源。地图来自本轮订阅的栅格，不是手绘路线。大脑栏是对实际收到的数据的同步显示，不展示或编造模型隐含推理。

`--clock hybrid --speed 0.5` 保留规划阶段的墙钟 1×，提交后按仿真时间 0.5× 展示动作；片头栏随阶段准确标注。每段末尾有 2 秒“结果定格”。`scripts/build_live_demo_video.py` 只接受三个任务实际成功且写盘队列无丢失的明确输入目录，再按导航、对准、大脑回位顺序拼接；仅附加原始测试 WAV 输入声，无解说和配乐。图像发布/接收的间隔仍保留，不宣称所有源帧都被采集。

录制是观察者，不改变碰撞、动力学、控制 TTL、策略或任务成功条件。近景只隐藏机器人重复碰撞显示层，保留场景外观层。新片通过用户验收前不加入 README 首页。

## 2026-10-27 实录审片结果

本地成片：`output/live_demo/review/qda-functional-demo-review.mp4`，165.82 秒，1920×1080，15 fps，内置三个章节。`evidence.json` 保存所选任务和所有试拍结果；`qc.json` 保存媒体检查及源帧间隔。画面直接采用运行中的渲染与传感器帧，没有用离线姿态重放代替运动。

| 章节 | 本轮来源 | 实际结果 |
| --- | --- | --- |
| 到点检查与返回 | nav02 | 3/3 完成；记录真实地图、路径、机体运动 |
| 固定起点双臂对准 | dual05 | 4/4 完成；同时记录近景、腕部图像和真实 Servo 检测叠加 |
| 大脑提案与回位 | brain04 | 3/3 完成；实际 Vosk WAV 识别、三次 SmolVLM 提案、校验与动作；附颜色/深度目标检测 |

必须保留的失败与限制：

- `nav01` 返回阶段触发原有 240 秒期限；暂时关闭后台 RViz 显示后 `nav02` 通过，未修改期限或控制保护。录制结束已恢复 RViz，并复验完整图与唯一速度出口。
- `dual04` 在导航停靠后，左臂对准受奇异位形/碰撞限速影响未完成。成片使用固定起点独立演示，不宣称导航到对准的鲁棒衔接已经验收。
- `brain02` 中短句“右臂回位”导致模型给出非法参数组合，整份计划在执行前被拒绝；后续明确句式通过。没有修改模型权重或偷偷回退规则，不能据此宣称开放式自主任务规划。
- `dual02` 的启动未就绪被拒绝；运行器已补充就绪与实际相机帧检查。
- 写盘队列无丢失不代表 ROS 图像发布/传输无间隔；源帧间隔保存在质检记录，播放没有插值生成机器人运动。

发现 `/perception/debug` 未带源图像 header，已补上源 header；重启后验证与真实 RGB 消息的 frame/stamp 匹配。本片采集时的旧节点图像如无 header，明确按接收时间同步，不伪造图像时间戳。
