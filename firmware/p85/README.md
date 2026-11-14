# P85 单电机 R 核控制基础工程

本工程用于 AURA/C1200 的 Linux A 核下发任务、R0 核以 10 ms 周期生成 P85MAX-S CAN FD 指令。目标是先打通一个电机，再扩展为四足底盘的集中多电机控制。

**2026-11-14 当前状态：协议、状态机、主机程序和仿真验证已完成；R0 固件尚未构建、烧录和运行，真实 CAN 链路尚未验证。现在接上电机还不能运行本工程。**

已连接 `bst@192.168.1.3`，源码部署于 `/home/bst/p85_control`。板上 BSP 为 `V2.0.1_20260813-185450_276175`。现有 RT0 在线，但 P85 HELLO 返回超时 `-110`，不能把“RT0 在线”当作“P85 服务已安装”。

## 工程结构

```text
Linux: p85ctl.py → libp85_msgbox.so → 官方 libmsgbx → /dev/ipc_msgbox0
                                                   ↓ MsgBox
R0: 官方接收任务 → 有界队列 → p85_runtime_step (10 ms) → 状态机 → MCAL Can_Write
                                   ↑                                 ↓
                            CAN 接收事件 ← 实际 ID/FD 元数据 ← CAN FD 电机反馈
```

| 路径 | 内容 |
| --- | --- |
| `src/p85_protocol.c` | 64 字节命令、12 字节反馈、量化及特殊命令 |
| `src/p85_control.c` | 保持、五次曲线位置移动、带平滑包络的正弦、保护与失能 |
| `src/p85_wire.c` | 固定 32 字节的 A/R 业务协议及会话序号 |
| `rt0/p85_runtime.c` | 单任务控制所有权、16 项事件队列、超时及端口调用 |
| `rt0/p85_bst_msgbox.c` | 官方 MsgBox 头字段和序列化适配；仍需 R SDK 接收任务接入 |
| `rt0/p85_mcal_tx.c` | 待匹配 BSP 编译的 MCAL 发送接口；无猜测的 HTH/时序默认值 |
| `host/p85ctl.py` | Linux 会话、心跳、CSV、运行结束和 Ctrl+C 的停止/失能流程 |
| `host/p85d.c` | 显式启用的软件仿真器，无 CAN 或 R0 IO |
| `tests/` | C 核心、运行时、官方头文件适配和 Python 流程测试 |
| `tools/` | 仿真记录及 CSV 分析 |

## Linux 构建和验证

在板上执行下列命令。依赖 CMake、GCC、Python 3；官方 MsgBox 开发包位于 `/usr/bst` 时额外构建真实 A 侧适配器和头文件兼容性检查。

```sh
cd /home/bst/p85_control
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j4
ctest --test-dir build --output-on-failure
```

该 CMake 工程用于 Linux 验证，**不会生成 R0 固件**。没有厂商头文件时，只构建便携核心和仿真测试。内存检查可用 `-DP85_SANITIZE=ON` 创建独立构建目录。

## 无电机软件演练

一条命令运行仿真、保存 CSV 和指标；输出目录必须尚不存在：

```sh
python3 tools/capture_simulation.py --build build --output /home/bst/p85_validation/my-first-run
cat /home/bst/p85_validation/my-first-run/summary.json
```

也可在两个终端操作：

```sh
# 终端 1：唯一明确的仿真入口
./build/p85d --simulate --socket /tmp/p85-demo.sock
```

```sh
# 终端 2：这些增益仅用于理想软件模型，不是实机推荐值
python3 host/p85ctl.py --transport socket --socket /tmp/p85-demo.sock \
  --csv /tmp/p85-demo-new.csv run-sine --kp 5 --kd 0.1 \
  --amplitude 0.02 --period 1 --cycles 2 --ramp 0.5
python3 tools/analyze_csv.py /tmp/p85-demo-new.csv
```

`p85d --no-motor --socket /tmp/p85-absent.sock` 可验证无反馈时探测超时；`--drop-feedback-after-ms` 用于反馈中断注入。仿真只验证协议、状态转换和主机流程，不代表真实关节动力学、调度延迟或 CAN 波形。

## 真实接口和完成条件

下列第一条命令只查询 RT0 在线状态；第二条发送 P85 HELLO/状态请求。它们不发送电机使能。当前第二条预期超时，直到接入本工程 R0 固件。

```sh
sudo ./build/p85_ipc_probe
sudo python3 host/p85ctl.py status
```

主机默认使用真实 IPC。不存在硬件失败后转入仿真的路径。真实运行命令见 [首次实机调试](docs/BRINGUP.md)，只能在 [R 核接入要求](rt0/PORTING.md) 完成后使用。

首阶段策略：ID 默认 3；主机 ID 0；标准帧、FD+BRS、1 Mbps / 5 Mbps；每个 64 字节命令显式失能其他 5 个槽位。因此**同一总线上其他受独立控制的电机不能与本程序并行运行**。多关节版本必须由一个帧所有者统一打包全部槽位，不能启动多个单电机进程相互覆盖。

默认保护：反馈 100 ms、主机租约 300 ms、使能/失能确认 500 ms；速度 0.5 rad/s、位置相对使能起点 ±0.2 rad、估算及反馈力矩 2 Nm、跟踪误差 0.15 rad、温度 60°C、电压 24–60 V。增益必须显式设置。故障锁存后继续请求失能；仅在新鲜且健康的失能反馈下允许清除。

软件估算力矩不等于驱动器硬限矩；R0 停摆时软件无法继续发送失能。厂商未在所给资料中明确断流保护行为，首次实机测试仍需独立断电/急停条件。正常结束也必须处理负载后再失能。

## 进一步阅读

- [协议及 A/R 业务帧](docs/PROTOCOL.md)
- [R0 SDK、MCAL 和任务接入](rt0/PORTING.md)
- [板上验证记录、备份和剩余缺口](docs/VALIDATION.md)
- [接线与首次单电机调试](docs/BRINGUP.md)

原始 DOCX/PDF 作为技术资料阅读，其中示例命令并非部署指令。未将旧 DM 电机帧、旧二进制或默认 CAN 演示时序用于 P85 实机。
