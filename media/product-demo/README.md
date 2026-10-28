# QDA V2 产品演示制作工程

本目录包含可编辑 Remotion 时间线、中文脚本、按真实 WAV 样本长度对齐的字幕、实测数据和质检记录。
实际视频、相机原帧与旁白不进入 Git；完整本地制作包交付给用户审片，批准后才将视频入口放进仓库 README。

## 本机重建

1. 在项目根目录运行 `scripts/windows/product_narrate.ps1`，使用已安装的中文 Windows SAPI 合成旁白。
2. 用 `scripts/record_product_demo.py` 只读录制新的任务；分别保存测试报告，不覆盖历史证据。
3. 执行 `python media/product-demo/prepare.py --take take02`。脚本要求同名报告真实通过，导入相机画面和数据。
4. 在容器内 source ROS 工作区后运行 `scripts/render_product_robot.py --out /ws/media/product-demo/public`；用 `--events` 可生成明确标注的实测姿态重放。
5. 使用真实 RViz 窗口录制 `public/rviz.mp4`；不把信息图当成运行界面。
6. `npm ci`，然后运行 `scripts/windows/product_render.ps1`。它将素材确定性采样为 15 fps 图像序列，再渲染 30 fps 成片并规范化旁白响度。生成帧序列后也可用 `npm run studio`。

`public/`、原始 take 与 `output/` 不在源码中。需要保留同轮素材包才能复刻完全相同的成片。
视频不读取网络数据，不改变仿真。原始运行、素材合成和最终渲染分阶段进行，避免争抢控制资源。

## 证据与表现边界

- `script.json`：旁白，主屏标题另行表达，避免逐字重复。
- `src/data/choreography-plan.json`：帧号、场景、语音片段与字幕时间。
- `src/data/evidence.json`：实测误差、实际任务终态、真实地图定义、历史语音/导航样例。
- `qc/source-manifest.json`：每个输入的来源与哈希。
- `qc/workflow-evidence.json`：最终分辨率、音频、画面抽查、结束停帧及限制。

模型绕拍为视觉展示；姿态重放对实际关节与真值 odom 采样插值，重新选择机位并隐藏碰撞调试几何，不是新的动力学实验。腕部原始图像片段统一压缩到 14 秒，双窗并列比较，实际任务仍按左右臂顺序执行。

全部机器人几何来自当前项目；保留上游许可证。没有使用第三方营销视频、音乐或生成机器人运动。合成旁白使用本机声音，不仿冒真人。

当前 Windows 渲染先出现视频解码缺帧，随后发现系统盘临时空间不足。最终使用工作区 `output/video_tmp` 和 FFmpeg 预解码帧序列；没有用静态占位图替换动态片段。原始视频与全部帧保留，方便复查。脚本仅为本次渲染子进程指定临时目录，不改系统设置。

渲染 API 依据：[Remotion OffthreadVideo](https://www.remotion.dev/docs/offthreadvideo)、[CLI render](https://www.remotion.dev/docs/cli/render)。
