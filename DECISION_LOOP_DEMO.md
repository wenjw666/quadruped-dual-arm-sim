# 大脑决策闭环实录与接口说明

视频：[整理后的闭环实录](output/brain_loop/decision-loop-clean.mp4)。以“前往南侧入口”为闭环实例，使用同一份实际录像与接口记录，按阶段展示模型输入、原始返回、校验下发、机器人执行和结果回读，无配音、无合成机器人运动。

## 展示方式

主画面保留实际仿真，右侧每次只显示当前步骤，底部用“汇总输入 → 调用 VLM → 程序校验 → 机器人执行 → 结果返回”的单一流程串联。移除常驻日志表格、长凭据、重复视角及无关状态字段；完整数据仍保存在下方原始记录中。

视频明确区分三次模型调用：反馈审查、语义转换、完成审查。前两次调用只记录了合并后的返回事件，未分别记录精确起止时间，因此用带有“结果展开 · 暂停”标记的静止画面依次展开已收到的结果，不伪造独立调用计时。执行阶段播放实际运动和目标/实际速度，显示 VLM 待命；任务成功返回后才点亮反馈路径，进入完成审查。

## 实际执行流程

1. 操作者提供“前往南侧入口”；有界命令解析得到 `navigate / south_entry` 候选技能。
2. `/brain/request` 的 `review_step` 获取相机、里程计、RL 就绪与故障、速度、足接触、夹持状态及上次任务结果，连同候选技能交给本地 SmolVLM。
3. 模型返回受约束的 `dispatch / wait / stop / complete` JSON。程序在模型调用前后分别检查传感器、就绪状态、活动任务、故障和任务终态。
4. 本次模型原始输出错误地给出 `complete`。独立语义转换调用得到 `navigate / south_entry`，与操作者候选严格一致，同时程序前后检查均为 `dispatch`，才发放一次性 `decision_id`。视频用黄色标出原始误判与转换，不能将其称为原始 VLM 正确决策。
5. `submit` 校验场景版本、凭据有效期、候选一致性并再次回读反馈，通过 `/robot_skill` 下发。执行模块控制 Nav2、控制权切换和 RL 运控；大脑接收技能反馈并查询任务状态，未持续调用 VLM。
6. 到达后，技能报告 `SUCCEEDED` 和实测误差。大脑把到达结果、新状态和新图像交给模型；模型返回 `complete`，程序也确认完成，本轮结束。

任务 ID：`decision-loop-1793077537120811189`。目标位置 `(0.1, -1.85)` 米，实际位置 `(0.0391, -1.7882)` 米，平面误差 **8.68 厘米**，偏航误差 **0.0263 弧度**。见 [完整运行报告](output/brain_loop/loop-final-pack/report.json)。

## 各模块反馈

| 来源 | 实际反馈 | 大脑用途 |
|---|---|---|
| `/camera/image_raw` | 头部图像、时间戳 | 视觉输入与图像新鲜度检查；本次缩放为 320×240 |
| `/odom` | 机体位置 | 观测快照与到达核对；当前为仿真真值 |
| `/locomotion/rl_status` | ready、fault、速度、足接触、倾斜 | 执行前检查及反馈快照 |
| `/motion/command_owner`、`/motion/arm_modes` | 底盘控制权、双臂控制模式 | 模块健康与状态检查 |
| `/manipulation/status` | stage、carrying、error | 夹持执行状态和故障；本次导航中为 idle |
| `/perception/labels` | 目标语义标签 | 本次为陈旧数据，保留提示；导航放行不依赖它 |
| `/skills/query`、`/robot_skill` | 活动任务、任务状态、技能反馈、到达结果 | 防止任务冲突、获取终态、决定等待/停止/完成 |

大脑先形成有界快照。输入阶段展示该次调用的实际冻结图像和少量关键状态；执行阶段只显示目标/实际速度与控制权；结果返回后切换为新的输入图像和成功状态。下一次审查使用新的快照。

## 证据与实现

- [录像清单](output/brain_loop/frames-final-pack/manifest.json)：丢弃记录 0，写入错误 0。
- [原始录像事件](output/brain_loop/frames-final-pack/events.jsonl)：实际图像、模型输出、下发、技能结果与模块反馈。
- [模型审计](output/brain_loop/audit/events.jsonl)：模型上下文、原始 JSON、程序期望、图像路径与 SHA256。第一次调用先于录像开始，其输入取自相同 `decision_id` 的实际审计记录；视频从第一张实际跟随相机帧开始。
- [视频时间信息](output/brain_loop/decision-loop-clean.json)：原始模型处理段按实时时钟 1×，运动段按物理仿真时钟 1×；另有明确标记的结果暂停展开段，各阶段的源录像时间与显示时长均可核对。
- `src/qda_brain/qda_brain/brain_node.py`：反馈汇总、模型调用、审计及一次性凭据。
- `src/qda_brain/qda_brain/feedback_review.py`：状态检查和放行检查。
- `scripts/run_decision_loop_demo.py`：实际 review → submit → 查询执行 → completion review。
- `scripts/record_product_demo.py`：只采集实际话题，不发布控制指令。
- `scripts/render_decision_loop.py`：用真实图像和收到的状态组装中文视频，不生成运动或模型推理过程。

## 当前边界与本轮防错修复

候选任务由操作者提供，本地 500M 模型承担受约束审查和语义转换，`runtime_ready=false`。视频验证导航实例闭环，尚不能作为“VLM 自主分解抓取—搬运—放置、多任务重规划”的证据，也未演示故障自主恢复。图像确实送入模型，但本例不足以证明模型依靠图像完成目标定位。

录制代码原有程序兜底分支，可能在语义目标不匹配时仍放行。本轮已去掉该分支，要求转换候选严格一致、前后反馈检查一致；完成判定同时要求模型 `complete` 与程序门控通过。本次实录 `supervisor_override=false`，没有使用被移除的分支。新增 5 项单元测试覆盖误判、语义不匹配、反馈变化、完成确认和输出格式，均通过；相关 Python 文件语法检查通过。收紧后的代码尚未在容器重新构建联调，旧实录不能当作修改后的再次运行验证。

README 尚未更新，视频等待用户验收。

## 复现

先启动模型与仿真，再开两个终端，先录像、后执行任务。新代码需先按项目流程构建并加载。

```powershell
.\scripts\windows\run_sim.ps1 -Brain -Manual -Background -NoPhysicsGui -NoRviz

# 终端一：先录像
docker exec -e ROS_DOMAIN_ID=8 qda-dev bash -lc 'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; python3 /ws/scripts/record_product_demo.py --out /ws/output/brain_loop/frames-01 --seconds 900'

# 终端二：执行实际闭环
docker exec -e ROS_DOMAIN_ID=8 qda-dev bash -lc 'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; python3 /ws/scripts/run_decision_loop_demo.py --out /ws/output/brain_loop/take-01'

# 录像结束后在宿主机导出；需 Pillow、NumPy、FFmpeg、中文字体
python scripts/render_decision_loop.py --take output/brain_loop/frames-01 --out output/brain_loop/decision-loop-01.mp4
```
