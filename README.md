# P85 电机 R 核控制基础工程

本分支 `codex/p85-rt0-motor-control` 用于 AURA/C1200 开发板上的嵌入式电机控制：Linux A 核通过 MsgBox 下发任务，R0 核以 10 ms 周期生成 P85MAX-S 的 CAN FD 控制帧，为后续四足底盘的多关节控制提供基础。

**2026-11-20：R0 演示固件已编译、写入并通过整板冷启动验收；R0 镜像与 SRAM 抽查匹配，AON 为 normal，A/R 真实 MsgBox 的 25 轮状态/诊断通过且 CAN 发送为零。当前状态为 OFF、fault=0。真实 P85 尚无反馈、使能或运动实测。**

此前 R0/R2 配套固件执行、A/R 通信、CAN0 内部回环及 CANFD0 ↔ CANFD1 双向物理通信均已通过验证。

[本轮电机演示与写入记录](firmware/p85/docs/MOTOR_DEMO.md) · [完整开发与调试记录](firmware/p85/README.md) · [首次电机接入流程](firmware/p85/docs/BRINGUP.md) · [物理 CAN FD 验证记录](firmware/p85/docs/PHYSICAL_CAN_VALIDATION.md)

## 当前工程

```text
Linux A 核：p85ctl.py → libp85_msgbox.so → 官方 libmsgbx
                                             ↓ MsgBox
R0 核：业务队列 → 10 ms 控制状态机 → CAN FD → P85 电机
                      ↑                  ↓
                 反馈与保护 ← CAN 接收事件
```

| 路径 | 内容 |
| --- | --- |
| `firmware/p85/include/`、`firmware/p85/src/` | P85 协议、控制状态机和 A/R 业务协议 |
| `firmware/p85/rt0/` | R0 运行时、官方 MsgBox/MCAL 适配和板级任务 |
| `firmware/p85/rt2/` | R2 配套固件及 CAN1 资源释放 |
| `firmware/p85/host/` | Linux 控制工具、真实 IPC 适配和软件演练程序 |
| `firmware/p85/tools/` | SDK 构建、镜像检查、运行报告解析和诊断采集 |
| `firmware/p85/tests/` | 协议、运行时、主机流程和诊断解析测试 |
| `firmware/p85/docs/` | 资料核对、构建、部署、CAN 验证及首次实机调试说明 |

## 已验证的阶段性结果

| 项目 | 结果 |
| --- | --- |
| R0/R2 固件 | ELF/SRAM 内容匹配，R2 心跳推进，CAN1 资源交接生效 |
| CAN0 内部回环 | 发送、发送确认、接收各 512 帧，错误与超时为 0 |
| CAN0 → CAN1 | 正常模式下发送、确认、对端接收各 512 帧 |
| CAN1 → CAN0 | 正常模式下发送、确认、对端接收各 512 帧 |
| CAN FD 参数 | 仲裁 1 Mbps、数据 5 Mbps，FD+BRS，覆盖 16 种合法长度 |
| A/R 真实业务 | HELLO 及 25 轮 STATUS/DIAG 通过，全程 OFF、fault=0 |
| 部署恢复 | 采集后恢复原始 Flash，工作站逐字节核对一致 |

上表记录截至 11 月 19 日的 CAN 诊断固件实测。11 月 20 日另行写入默认 OFF 的电机演示 R0 镜像，仅修改 `0xB00000` 起始 1 MiB；冷启动后已核对 SRAM 镜像并完成 A/R OFF 状态验收。该验收不包含真实电机反馈和运动。详细过程和证据见[完整调试记录](firmware/p85/README.md)及[本轮演示记录](firmware/p85/docs/MOTOR_DEMO.md)。

## Linux 软件构建与测试

在 Linux/WSL 中，从仓库根目录执行：

```sh
cmake -S firmware/p85 -B firmware/p85/build -DCMAKE_BUILD_TYPE=Debug
cmake --build firmware/p85/build -j4
ctest --test-dir firmware/p85/build --output-on-failure
```

依赖 GCC、CMake 和 Python 3。板端存在 `/usr/bst` 下的官方 MsgBox 开发包时，会额外构建真实 A 侧适配器及兼容性测试。

该 CMake 工程用于 Linux 验证，**不会生成可烧写的 R0 固件**。R0/R2 使用固定版本的 Hanhai SDK 单独构建：

- [R0 完整构建记录](firmware/p85/docs/R0_BUILD.md)
- [R0 板级接入说明](firmware/p85/rt0/PORTING.md)
- [R0 部署、启动与恢复](firmware/p85/docs/R0_DEPLOYMENT.md)
- [R2 CAN1 配套固件](firmware/p85/docs/R2_CAN1_HANDOFF.md)

## 接入真实电机

本轮固件冷启动验收已通过。接入电机后，按[首次实机调试](firmware/p85/docs/BRINGUP.md)核对电源、ID、通信速率、接线和终端电阻，从反馈探测及失能确认开始，再验证小范围、低速度的位置控制。

新 `run-demo` 由 R0 顺序执行 `−A → 0 → +A → 0` 多点位置、位置正弦及速度正弦，按 `--repeat` 循环；主机打印阶段和反馈、保存 CSV，并在结束后确认失能。命令与小幅示例见[电机演示说明](firmware/p85/docs/MOTOR_DEMO.md)。

单电机接入前拆除 CANFD0 ↔ CANFD1 的诊断跨接，使用 CANFD0 连接电机，并核对总线端接。多电机扩展还需集中管理完整 64 字节命令帧和各关节反馈，具体边界见[工程说明](firmware/p85/README.md)。

## 分支内容约定

本分支仅跟踪本首页、Git 配置文件和 `firmware/` 下的嵌入式工程。根目录 `.gitignore` 使用允许列表，防止本地保留的 ROS 工作空间、仿真资源、Docker 镜像和外部 SDK/硬件资料被误提交。

ROS 2、MuJoCo/Gazebo、双臂仿真及 C1200 感知/推理开发保存在 [`codex/c1200-yolo-smolvlm-adaptation`](https://github.com/wenjw666/quadruped-dual-arm-sim/tree/codex/c1200-yolo-smolvlm-adaptation)；原仿真版本继续保存在 [`v2/decision-rl`](https://github.com/wenjw666/quadruped-dual-arm-sim/tree/v2/decision-rl) 和 `main`。这些分支中的已有代码和提交记录均保留。
