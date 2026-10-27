# 建图与导航验收演示

## 建图

MuJoCo 后端现在可以把真实渲染的 RGB-D 像素投影到 `odom` 并做有界体素融合。点云来源是相机观测，颜色来自 RGB 帧，深度来自同一时间戳的深度帧；动态机器人和抓取物体通过 MuJoCo 分割结果过滤。地图不从场景网格补点，也不会把未观测表面当成已建成。

本次完整绕行采集得到 **120,815 个彩色体素点，3.5 cm 体素**。四面墙和全部静态台架均有观测点，地面观测覆盖约 64.3 m²；遮挡面仍保留为空，这是有意保留的测量事实。

- 交互查看器：[colored_map.html](output/mapping/colored_map.html)
- 彩色点云：[colored_map.ply](output/mapping/colored_map.ply) / [colored_map.pcd](output/mapping/colored_map.pcd)
- 预览图：[colored_map.png](output/mapping/colored_map.png)
- 俯视图：[colored_map_top.png](output/mapping/colored_map_top.png)
- 点云旋转实录：[colored_map_orbit.mp4](output/mapping/colored_map_orbit.mp4)
- 覆盖审计：[coverage_report.json](output/mapping/coverage_report.json)

RViz 的 `Accumulated RGB-D map / simulation-truth registration` 显示 `/mapping/colored_global`；保存服务为 `/mapping/save_color`。这一版的位姿注册使用仿真真值，尚不是无真值 SLAM 评估。

## 导航

导航演示通过生产 `/brain/request`、Nav2 `NavigateToPose`、RL 自由基座和到达复核执行。每站点必须在停稳后重新检查位置与朝向；停稳时 RL 行走到站立的回弹会触发有界终点补偿，仍由 Nav2 进行规划和碰撞检查。

本次干净回合成功完成 9 个目标：南侧中段、南侧、东侧中段、东南、东北、北侧、西北、西侧，再回到南侧中段。实际行程约 **27.18 m**，所有目标均记录了实测位置和朝向误差，最大位置误差约 11.5 cm。

- 导航实录：[navigation-perimeter.mp4](output/navigation/perimeter-accepted/navigation-perimeter.mp4)
- 验收报告：[report.json](output/navigation/perimeter-accepted/report.json)
- 事件流：[events.jsonl](output/navigation/perimeter-accepted/events.jsonl)
- 最近画面：[latest.jpg](output/navigation/perimeter-accepted/latest.jpg)

画面同时显示 MuJoCo 实际视角、Nav2 规划线、实际轨迹、目标列表、到达状态、RGB-D 点云数量和定位来源。目标点刻意避开墙体边界，避免把机器人贴墙旋转误当作典型导航能力。

## 复现

```powershell
.\stop_sim.ps1 -Backend mujoco
.\run_sim.ps1 -Brain -Manual -Background -Mapping -NoPhysicsGui -NoRviz
.\run_navigation_demo.ps1 -Mode perimeter -Take clean
```

运行前应从新仿真回合开始；脚本会启用观察相机、RGB-D 累积和内侧跟拍视角。`-Mode destinations` 会使用检查点、南北入口和东侧不同朝向的目标集合。

导航和建图证据均来自实际 ROS 消息和 MuJoCo 物理运行；视频没有姿态回放或人工绘制轨迹。
