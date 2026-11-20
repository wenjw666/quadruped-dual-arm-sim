# P85 单电机 R 核控制基础工程

本工程在 AURA/C1200 上由 Linux A 核下发任务，R0 核以 10 ms 周期生成 P85MAX-S CAN FD 指令，作为后续四足底盘集中多电机控制的基础。开发分支为 `codex/p85-rt0-motor-control`。本 README 记录从资料核查、SDK 获取、协议接入、固件构建和写入，到冷启动、内部回环、双向物理链路验证成功的完整过程。

记录范围为 2026-11-14 至 2026-11-20，日期采用工作站 Asia/Tokyo 时间。目标板 RTC/墙钟不准确，各次运行通过 boot ID、uptime 和产物哈希区分。

[11 月 20 日电机演示与写入](docs/MOTOR_DEMO.md) · [历史物理链路结果](#本轮成功的范围与结束状态) · [调试全过程](#开发与调试全过程) · [问题与修复](#关键问题与修复对照) · [证据复核](#复核本轮结果) · [Linux 构建](#linux-构建和验证) · [后续单电机调试](docs/BRINGUP.md)

## 2026-11-20：电机演示固件已写入，等待整板上电

已对照 P85 手册复核 64 字节命令、12 字节反馈、ID3 槽位、小端量化、使能/失能和故障位。新增 `run-demo`：R0 依次执行负向位置、回中、正向位置、回中、位置正弦、速度正弦，按循环数重放后返回 HOLD；主机打印阶段和反馈、保存 CSV，再确认失能。每段必须等反馈连续稳定 200 ms；规划结束 3 s 仍未到位则锁存 `settle_timeout`。

WSL 软件 CTest 9/9、板端 Linux 软件 CTest 11/11 均通过。最终 Cortex-R5 固件完成编译、链接和签名，SRAM 剩余 397056 字节；已写入 R0 `0xB00000` 起始 1 MiB。下载完整 64 MiB 独立回读核对，候选内容逐字节一致，R1/R2 和其他字节不变。

**当前 Flash 已是本轮电机演示 R0 镜像，但当前 SRAM 仍为之前上电加载的旧程序。需要整板主电源断电再上电；新镜像默认 OFF，不会自动转动。自动服务只做镜像和 HELLO/STATUS/DIAG 验收，失败保留 15 分钟回退。真实电机运动尚未实测。**

板端新源码位于 `/home/bst/p85_validation/demo-20261120/source`，Linux 构建与 IPC 库在同级 `build/`，部署及完整备份位于 `/var/lib/p85-rt0-deploy/20261120-motor-demo-01`。首次接入先拆除 CANFD0↔CANFD1 诊断跨接，使用 CANFD0 接 P85，经过探测、保持和小幅单点验证后再启动演示。

动作参数、反馈稳定判据、运行命令、产物 SHA256 和部署证据见 [电机演示与写入记录](docs/MOTOR_DEMO.md)。以下物理链路结果和恢复状态均为 11 月 19 日的历史记录。
## 本轮成功的范围与结束状态

**2026-11-19，CANFD0 ↔ CANFD1 真实双向物理收发通过，R0/R2 配套固件执行及 A/R 应用通信通过。真实 P85 尚未连接，电机反馈、使能和位置运动尚未进行实物验收。**

| 检查 | 实测结果 |
| --- | --- |
| 本轮 boot ID | `3d8fbaa2-a575-4987-b452-19bb40c0ad6e` |
| R0 固件身份 | physical-v6 ELF 的代码/只读数据 57,411 字节与 SRAM 匹配 |
| R2 固件执行 | CAN1 release-v2 内容匹配，stage=2，心跳 2,153→2,181，CAN1 inactive，CAN2/3 START 成功 |
| CAN0 内部回环 | 提交、发送确认、接收各 512 帧，错误和超时均为 0 |
| CAN0→CAN1 物理收发 | 提交、发送确认、接收各 512 帧 |
| CAN1→CAN0 物理收发 | 提交、发送确认、接收各 512 帧 |
| 长度与载荷 | 每方向覆盖 16 种合法 FD 长度，每种 32 帧，帧信息与完整载荷一致 |
| 总线配置 | 仲裁 1 Mbps / 数据 5 Mbps，FD+BRS，物理测试启用 TDC、关闭内部回环和自接收 |
| 物理错误 | 两控制器 faults=0、timeouts=0，bad_metadata/bad_payload/unexpected 均为 0，ECR=0 |
| A/R 应用通信 | 真实 MsgBox HELLO + 25 轮 STATUS/DIAG 通过，R0 时钟推进 6,010 ms |
| 电机应用状态 | 全程 OFF、fault=0、motor_status=0，电机 tx_ok/tx_failed/rx_ok/rx_rejected 均为 0 |
| Flash 恢复 | 两个槽位恢复成功，64 MiB 读回与原基线逐字节一致，`accepted=false` |

验收结束时，SRAM 仍运行本轮诊断候选，Flash 已按预设恢复到此前通过验收的内部回环 v3 基线。**再次完整上电会加载恢复后的基线，不会继续加载本轮双向物理诊断版本。** collector 与 rollback timer 已禁用、armed marker 已移除，未额外发起重启。这是诊断事务的预定结束方式，不能把 `accepted=false` 解读为物理测试失败。

这里的成功证明板上控制器、MCAL、中断接收、收发器和现有线缆在本轮参数下完成双向 CAN FD 通信；1024 帧物理测试载荷不是电机控制命令。完整实测见 [物理链路记录](docs/PHYSICAL_CAN_VALIDATION.md)。

## 开发环境与连接关系

| 项目 | 本轮使用情况 |
| --- | --- |
| 工作站 | Windows，项目目录 `D:/20261022ubunturos_win` |
| Linux 构建环境 | WSL `MediaOps-Ubuntu`，Linux x86-64 |
| 目标板 | AURA/C1200，DT model 为 BST C1200 CDCU，Linux aarch64 |
| 板端系统 | Ubuntu 22.04.5 LTS，BSP `V2.0.1_20260813-185450_276175` |
| 网络 | 网线连接，工作站以太网 `192.168.1.2/24`，目标板 `192.168.1.3`，SSH 用户 `bst` |
| Linux 工具及 IPC | GCC 11.4、CMake、Python 3.10，官方 hanhai-ipc 4.4.7.0-r0 |
| IPC 设备/开发包 | `/dev/ipc_msgbox0`、`/usr/bst/include/msgbx`、`/usr/bst/lib/libmsgbx.so` |
| R 核源码基线 | `C1200SW_V1.6_RC0930`，R0 固定 19 项 commit，R2 构建固定 8 项 revision |
| R 核工具链 | Arm GNU 12.2.MPACBTI-Rel1，`arm-none-eabi-gcc 12.2.1` |
| 板端应用源码 | `/home/bst/p85_control` |
| 最近软件回归 | `/home/bst/p85_validation/demo-20261120/{source,build}`，CTest 11/11 |

以太网用于 SSH、传输文件和板上 Linux 操作；P85 控制数据经 R0 所属 CAN FD 外设送出。Micro-B 串口用于观察启动日志。三者承担不同工作，SSH 连通本身不能证明 CAN 或 R 核业务可用。

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
| `src/p85_control.c` | 保持、五次曲线位置移动、位置/速度正弦、顺序循环演示、反馈稳定判定、保护与失能 |
| `src/p85_wire.c` | 固定 32 字节的 A/R 业务协议及会话序号 |
| `rt0/p85_runtime.c` | 单任务控制所有权、16 项事件队列、超时及端口调用 |
| `rt0/p85_bst_msgbox.c` | 官方 MsgBox 服务、路由和回复序列化，已编入 R0 |
| `rt0/p85_mcal_tx.c` | 已编入 R0 的 CAN0 HTH12 发送接口 |
| `rt0/hanhai/` | FreeRTOS 任务、SDK 接收队列、CAN0 时序回读和状态命令 |
| `rt2/hanhai/` | 固定 R2 源码版本、CAN1 释放变体的缓存同步运行报告 |
| `host/p85ctl.py` | Linux 会话、心跳、CSV、运行结束和 Ctrl+C 的停止/失能流程 |
| `host/p85d.c` | 显式启用的软件仿真器，无 CAN 或 R0 IO |
| `tests/` | C 核心、运行时、官方头文件适配和 Python 流程测试 |
| `tools/` | 仿真、构建、SRAM 报告检查、诊断采集与受约束的 Flash 恢复 |

## 开发与调试全过程

### 1. 核对资料和实验 sample，明确需要自己构建 R 核固件

最初阅读了跨域通信实验、P85MAX-S 协议手册、外形及性能参数、正弦位置跟踪实验手册，并检查附件的 `sample/` 工程。跨域实验提供了 A/R MsgBox 的使用方式和 R 核槽位写入线索；P85 文档用于核对命令打包、数值范围、特殊命令和反馈读取。附件中的命令作为技术资料审阅，实际部署以板端布局及已授权的开发操作为准。

sample 中有已编译的 R0 BIN、`flash_R_bin.sh`、A 核测试程序和 R 核增量补丁，但缺少补丁引用的 `acore_ipc_server.c`、完整 RTOS/MCAL 底座、链接配置和工具链。因此它能说明实验流程，不能独立重新编译 P85 控制固件。原有 IPC 示例使用 fid0/sid1；P85 业务采用 fid0/sid2/cmd0x85，刷入旧示例也不能代替本工程。

此时确定两项原则：在板上先确认实际 BSP、IPC 和 Flash 布局；在工作站获取可构建的完整 R 工程，再接入业务，不能把 Linux 测试可执行文件或静态库当作 R 核固件。

资料审阅细节见 [R0 示例及写入流程](docs/R0_FLASHING.md)。

### 2. 建立单电机控制、反馈和保护的软件基础

将协议、控制状态机和板级 IO 分开实现：64 字节电机命令、12 字节反馈解析，以及固定 32 字节的 A/R 业务帧；A 核负责会话、配置和任务下发，R0 任务集中处理控制事件、反馈和 CAN 发送。

控制流程覆盖探测、使能、保持、五次曲线位置移动、带平滑包络的正弦、停止和失能。加入反馈超时、主机租约、位置/速度/力矩/温度/电压保护及故障锁存。初始状态为 OFF，未显式设置增益、探测和使能时不主动发送电机控制帧。

Linux 侧先用显式仿真器验证编码、状态转换、超时、异常退出和 CSV 记录；真实 IPC 与仿真入口分开，真实接口失败会报告错误。R0 接入采用单任务控制所有权、有界事件队列，并在厂商 IPC 节点回收之前复制业务内容，保留真实 CAN ID 和 FD 元数据。

这些软件验证使接口和状态机具备可检查的基础，但尚不能证明 R 核运行或物理总线可用。协议细节见 [协议与业务帧](docs/PROTOCOL.md)，任务与驱动接入见 [R0 接入说明](rt0/PORTING.md)。

### 3. 扩大 SDK 检索，修正“没有完整 R 工程”的早期判断

初次检查官网软件下载页、SDK Manager 和 sesameX GitHub 时，没有找到足以构建目标业务的完整 R 工程。继续检查附件目录和厂商公开资料后，定位到 Gitee 的 Hanhai BSP `rt-manifest`。**早期“未找到完整 R 工程”只是当时的检索结果，后来已被公开 manifest 的获取和实际构建结果修正。**

`C1200SW_V1.6_RC0930` 的 `realtime_freertos_core0.xml` 列出 19 个应用、HAL、MCAL 配置、OS、BSW 和中间件项目。取得清单对应源码后，确认其中有 CAN MCAL、FreeRTOS、MsgBox、IDL 生成器、签名工具和随附签名材料，具备接入与打包条件。另找到的裸机示例可帮助核对工具链和入口，但不能替代完整 P85 RTOS 工程。

把 manifest 的实际 commit 和待补丁文件哈希固定下来，避免混用 V1.7/V1.8 或已有工作树中的临时修改。V1.6 R 源码与板上 V2.0.1 BSP 的名称不同，不能仅凭版本号断言兼容或不兼容；后续通过镜像身份、冷启动、真实 MsgBox 和 CAN 硬件结果逐层验证本次组合。

检索路径与更正记录见 [SDK 获取及核查](docs/R0_SDK_FINDINGS.md)。厂商源码、签名材料及工具链保存在本地忽略目录，没有随本工程源码纳入 Git。

### 4. 从便携对象编译推进到完整 R0 链接和签名打包

先在 WSL 中完成协议、状态机和运行时的 Cortex-R5 对象编译，验证目标架构下的基础 C 代码。早期探测对象采用的 soft-float 配置只用于编译检查；完整厂商 SDK 使用 Cortex-R5 / ARM / hard-float / VFPv3-D16 / small-enum ABI，最终以完整工程的编译和 ELF 检查为准。

随后通过 `prepare_hanhai_sdk.py`、`build_hanhai_sdk.py` 接入 P85 源码、FreeRTOS 控制/接收任务、官方 MsgBox 路由、CAN0 HTH12 发送、实际接收入口和错误处理。关闭冲突的 CAN/VCan/时间同步演示，精简无关网络应用，检查 ELF 入口、LOAD 范围、SRAM 余量及 P85 关键符号。

首个完整基线的入口为 `0x04000000`，原始 BIN 为 294244 字节，签名发布包为 1 MiB，LOAD 范围使用 646592 字节，未超出 1 MiB R0 SRAM。构建脚本记录实际退出码和产物哈希，不能仅依据厂商脚本打印“完成”认定构建通过。后续诊断变体都有独立 ELF、签名包和 build report。

Linux CMake 构建用于主机程序及软件测试；完整 R 固件通过 WSL/Linux 中的厂商工程和 R 交叉工具链生成。详细命令与产物见 [完整 R0 构建](docs/R0_BUILD.md)。

### 5. 确认 Flash 布局，建立可恢复的部署流程

先保存板上完整 64 MiB Flash 备份，再从备份中的 SBL DTB、附件脚本及厂商资料交叉核对槽位。不是根据 BIN 文件名猜测偏移。

| 核 | Flash 偏移 | 槽位大小 | SRAM 加载地址 |
| --- | --- | --- | --- |
| R0 | `0xB00000` | `0x100000` | `0x04000000` |
| R1 | `0xC00000` | `0x100000` | `0x04100000` |
| R2 | `0xD00000` | `0x100000` | `0x04200000` |

部署前固定候选包、ELF、工具和基线哈希，核对 origin boot、槽位大小与全片基线。恢复标记和下次启动的采集/回退单元先持久化，再更新指定槽位。写入后读取完整 Flash，验证候选区域，并逐字节确认其余区域未变；保留完整恢复路径。

早期只更新 R0；最终双控制器诊断还需要更新 R2，按 R2→R0 顺序写入各 1 MiB。R1、SBL 及其余区域保持原样。诊断事务使用 `can_handoff_flash_guard.py`，其包元数据经固定 SHA256 约束，不是一个可对任意地址写入的通用刷机入口。

部署和历史恢复记录见 [实板部署记录](docs/R0_DEPLOYMENT.md)。

### 6. 区分 Linux 重启、Flash 写入和 R 核实际加载

第一次写入、回读及 Linux 重启后，R0 SRAM 中仍不是新 P85 固件。后续完整上电中也曾遇到 Flash 已被前一轮回退服务恢复的情况。这两类现象都不能简单记为“候选固件启动失败”。

调查时同时检查：新旧 boot ID、完整 Flash 哈希、回退日志、AON 状态、R0 SRAM 与候选 ELF 的代码/只读数据匹配情况。`release realtime0 done` 仅表示启动链释放 R0；RT0 在线也只说明核心在线。需要真实 P85 HELLO 回复和持续推进的诊断时钟，才能确认业务执行。

用户接入 Micro-B 后，以 115200 8N1、DTR/RTS 关闭监听四路串口：当时 COM10 为 SBL、COM7 为 Safety、COM8 为 A-console、COM9 为 Linux；COM 编号属于这台工作站当时的枚举结果。串口日志与网络端口恢复一起帮助区分“板尚未启动完”和“启动的不是预期镜像”。

最终通过整板主电源断电上电、启动后自动验收及固定身份检查，建立了可重复的 R0 冷启动验证流程。2026-11-17 的 OFF 验收 boot `3b54e8d9-bfa4-40da-baa8-1d3e870abced` 确认 P85 基线已经执行。

### 7. 验证真实 A/R MsgBox 连续会话

接入真实 MsgBox 后，不能以一次 HELLO 或一次在线查询作为验收。A 侧 `host/p85_msgbox.c` 将在线状态检查放到句柄打开时执行一次并缓存，随后清理旧动态库并重建，确保目标板实际使用修改后的 `.so`。

验收连续执行 HELLO 和 25 轮 STATUS/DIAG，检查回复身份、版本和序号、OFF 状态、fault=0、R0 时钟推进，以及电机发送计数为 0。这样同时验证请求接收、R0 业务处理、回复路由和重复请求能力。真实接口没有自动回退到仿真的路径。

2026-11-17 完成首轮冷启动 OFF 验收后，才继续开展 CAN 硬件自检；软件侧最新目标板 CTest 于同日达到 11/11 通过，双槽位 guard 的 9 个子测试也通过。软件回归和硬件验收分别保存证据。

### 8. 内部回环首帧失败，修正 TDC 和模式切换

没有连接电机时，先让 CAN0 在控制器内部发送并接收测试帧，验证 MCAL 发送确认、真实接收中断、FD 长度及载荷处理。覆盖 `0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64` 字节，每种 32 帧，共 512 帧。

v2 在 boot `db7c9e4d-03c8-4dfd-86cc-06806ccb6b8e` 的首帧失败：submitted=1、confirmed=0、received=0、faults=1。检查发现内部回环仍启用了面向外部收发器的发送延时补偿 TDC。

v3 在控制器 START 后进入 freeze 配置内部回环，避免生成配置覆盖回环设置；只在回环期间清除 `FDCTRL[TDCEN]`，保留 FD+BRS、1M/5M 和其余配置。结束后通过 STOP/START 恢复正常模式、接收过滤器及完整 TDC 设置。报告比较排除 TDCVAL/TDCFAIL 等动态测量字段，并明确检查回环时 TDCEN=0、正常模式恢复后 TDC 配置正确。

v3 在 boot `3b0710a2-ed60-4704-9308-db792345aef1` 完成 512 帧硬件回环，错误、超时、元数据和载荷差异均为 0，随后 OFF 应用通信通过。这一阶段证明控制器内部路径可用，尚未覆盖外部收发器和线缆。

首次失败及修复细节见 [内部回环验收](docs/CAN_FD_VALIDATION.md)。

### 9. 修正诊断报告的缓存一致性，避免读取假象

后续 boot 中出现同一缓存行内统计字段读数陈旧的现象。R 核写入 SRAM 后，Linux 直接读取物理 SRAM 并不自动保证获得最新的缓存内容；仅将结构声明为 `volatile` 不能解决这种跨核可见性问题。

对 R0 的回环/物理/跟踪报告加入缓存同步与发布顺序控制；R2 的运行报告按 32 字节对齐并显式执行缓存维护。采集器检查 R0 已发布报告在快照期间稳定，并用两份间隔快照验证 R2 心跳推进。

此前从 Linux A 核映射 CAN 控制器 MMIO 曾引发 RCU stall 和 SSH 失联。后续由拥有外设的 R0 读取寄存器并发布到 SRAM，Linux 检查器只读取已审核的 R0/R2 SRAM 区域。这样保留诊断能力，也避免重复采用已出现问题的跨域 MMIO 读取方式。

### 10. 连接 CANFD0 与 CANFD1，定位配置被覆盖

用户将两路 CAN FD 接口接成外部双节点链路：

| 信号 | J0801 连接 |
| --- | --- |
| CAN H | CAN0H pin9 ↔ CAN1H pin13 |
| CAN L | CAN0L pin7 ↔ CAN1L pin11 |
| 地 | 使用公共 GND pin5/6/19 |

原理图显示每端分裂终端约 120.8 Ω，尚未测量实装电阻。接线与原理图依据见 [CAN FD 接线和验收范围](docs/CAN_FD_VALIDATION.md)。

physical-v6 在 boot `54a61001-fce1-4de4-b9f1-aec42a0ba508` 先通过 CAN0 内部回环，但物理测试在 CAN1 START 前失败，没有发送外部测试帧。跟踪显示 R0 初始化 CAN1 后寄存器正确，随后却变成 R2 默认配置。

| 寄存器 | R0 初始化 CAN1 后 | CAN1 START 前 |
| --- | --- | --- |
| CTRL1 | `00DB0006` | `01DB4006` |
| CBT | `80139A73` | `80339A73` |
| FDCBT | `00031C63` | `00332C63` |
| FDCTRL | `86DB0000` | `86DB9400` |

同时纠正状态枚举的解释：SDK 中 UNINIT=0、STARTED=1、STOPPED=2。`start_enter aux=1` 表示软件状态已经 STARTED，上层重复 START 被拒绝；不能据此声称底层退出 freeze 超时。

源码进一步显示 R2 原工程在启动时也执行 `Can_Init`，CAN1 为 active，并无条件启动 CAN1/2/3、注册 CAN1 中断。这与寄存器变化相互支持，排查重点转为跨核配置冲突。当时失败发生在物理发送之前，没有依据将其归因于接线或终端电阻。

### 11. 构建 R2 CAN1 释放固件，配套部署两个槽位

新增 `rt2/hanhai/sdk-lock.json`、R2 运行报告和 `build_r2_can1_handoff.py`。从固定 revision 通过 `git archive` 导出源码，核对哈希及补丁锚点，生成配套变体：

- 将 R2 CAN1 activation 改为 FALSE，移除 CAN1 START 和中断注册。
- 禁用原始 `can` 测试命令，拒绝 `cansend` 对 bus1 的操作，避免其他入口再次配置 CAN1。
- 保留 CAN2/3 active 和正常启动流程。
- 在原有周期任务内发布实际配置、阶段、CAN2/3 启动返回值和递增心跳。

R2 ELF 入口为 `0x04200000`，SRAM 结束为 `0x04262230`，剩余 646608 字节，报告地址为 `0x0425b8a0`。检查器同时要求 ELF 内容匹配、阶段正确和心跳推进；报告中的 CAN1 owner=R0 只是配置声明，还需结合 R0 的寄存器及外部收发结果判断。

2026-11-17，将 R0 physical-v6 与 R2 CAN1 release-v2 写入事务 `/var/lib/p85-rt0-deploy/20261117-can-handoff-01`，只更新 R0/R2 两个槽位。写后完整 64 MiB 读回下载到工作站，再与候选和基线独立逐字节比较。origin boot 为 `a6ebb93d-d849-4ab2-8ffb-d9e1a14c37b8`，collector 已实测拒绝在 origin boot 上采集新镜像结果。

collector 的约定是：新 boot 后保存 R0/R2 报告及 SRAM，再无条件恢复原 Flash；15 分钟 timer 提供后备恢复。即使链路测试成功，也不自动把诊断候选设为常驻固件。完整构建与恢复说明见 [R2 CAN1 配套固件](docs/R2_CAN1_HANDOFF.md)。

### 12. 2026-11-19 完整上电，双向物理通信和 OFF 验收通过

用户重新上电后，首次 SSH 连接超时。继续核对本机以太网地址、路由和邻居项，再做有限次数的 ping/TCP 检查后，目标板恢复响应并出现 SSH banner。因此没有把一次连接超时记为固件失败，也没有要求重复上电。

本次 boot 为 `3d8fbaa2-a575-4987-b452-19bb40c0ad6e`。自动采集确认：R0 physical-v6 内容匹配，R2 CAN1 release-v2 内容匹配且心跳递增，AON R0/R1/R2 均为 cold、normal；CAN0 内部回环 512 帧通过。

外部测试在两控制器正常模式下进行，关闭内部回环和自接收，CAN0→CAN1 使用标准 ID `0x5A5`，反向使用 `0x5A6`。两方向各完成 512 次提交、512 次发送确认和 512 次对端接收；每种长度各 32 帧，完整载荷一致，无超时和控制器错误。物理自检总耗时 2048 ms，这包含程序节拍，不能当作总线吞吐率或单帧延迟。

释放 R2 CAN1 后，在保持现有物理接线的条件下双向测试通过，支持此前关于 CAN1 配置冲突的诊断。测试结束后 CAN0 恢复正常运行，CAN1 停止，应用保持 OFF。

collector 于 uptime 39.47 s 前完成采集和 Flash 恢复。之后使用真实 MsgBox 执行 HELLO 和 25 轮 STATUS/DIAG，R0 时钟从 278892 ms 推进到 284902 ms，增量 6010 ms；全程 OFF、fault=0，电机帧计数为 0。物理测试帧计数与应用电机帧计数是独立统计。

最后下载三份 SRAM 快照核对采集哈希，并下载完整 `flash-restored.bin`，在工作站与原始基线逐字节比较一致。至此取得本轮“固件身份→R2 执行→内部回环→外部双向收发→A/R OFF 会话→Flash 恢复”的完整证据。

## 关键问题与修复对照

| 现象 | 排查得到的结论 | 修改或处理 | 验证方式 |
| --- | --- | --- | --- |
| sample 不能重新构建 | 只有旧 BIN、A 侧示例和增量补丁，缺完整底座 | 获取并固定 Hanhai manifest 工程 | 完整 R0 链接、签名包和构建报告 |
| Linux 重启后没有新 P85 | SRAM 身份未匹配；部分启动前 Flash 已被恢复 | 核对回退日志与 Flash，完整上电后自动验收 | boot ID、ELF/SRAM 匹配、AON、真实 HELLO |
| 连续 IPC 会话需要核验 | 在线状态查询及旧动态库必须与会话行为一起检查 | 在线检查改为句柄打开时一次，清理重编译动态库 | HELLO + 25 轮 STATUS/DIAG |
| 内部回环首帧错误 | 回环期间仍开启外部发送延时补偿 | 回环清 TDCEN，结束后恢复；修正模式切换顺序 | 512 帧、16 种长度、错误为 0 |
| SRAM 报告字段陈旧 | R 核缓存内容没有及时对 A 核可见 | 缓存维护、对齐、稳定性及双快照检查 | 完整统计一致、R2 心跳推进 |
| Linux 读取 CAN MMIO 后失联 | 该读取路径曾引发 RCU stall | 改由 R0 采样并发布 SRAM | 从 SRAM 获得寄存器和跟踪报告 |
| CAN1 START 前失败 | 配置变化与 R2 默认初始化吻合，状态已 STARTED | R2 释放 CAN1 并配套部署 | R2 心跳/配置通过，双向物理 1024 帧通过 |
| 新上电首次 SSH 超时 | 连接尚未恢复，一次超时不足以判断固件 | 检查以太网/路由，有限重试 | SSH 连通及新 boot 采集完成 |

## 复核本轮结果

### 已保存的证据

原始数据保存在 `output/p85_r_core_reading/`，该目录被 Git 忽略。README 和源码可以随分支查看；完整证据、厂商源码和工具链需要保留工作站上的相应目录，不能仅克隆仓库后假定这些文件存在。

| 相对项目根目录的路径 | 内容 |
| --- | --- |
| `output/p85_r_core_reading/p85-r0-integration-20261115-final/` | 首个完整 R0 构建产物 |
| `output/p85_r_core_reading/deployment-20261117/monitor-20261117-1026/` | 四路启动日志与 OFF 冷启动验收 |
| `output/p85_r_core_reading/can-loopback-20261117/` | 内部回环失败、修复与 v3 验收 |
| `output/p85_r_core_reading/physical-can-20261117/build-v6/` | 本轮 R0 physical-v6 ELF、包及构建报告 |
| `output/p85_r_core_reading/physical-can-20261117/build-r2-handoff-v2/` | R2 CAN1 release-v2 ELF、包及构建报告 |
| `output/p85_r_core_reading/physical-can-20261117/diagnostic-followup/` | CAN1 配置覆盖的寄存器及跟踪证据 |
| `output/p85_r_core_reading/physical-can-20261117/handoff-deployment/` | 配套部署包、全片读回及独立比较 |
| `output/p85_r_core_reading/physical-can-20261119/` | 最终硬件报告、三份 SRAM、OFF 查询和恢复校验 |

最后一个目录中的 `validation-summary.json` 为汇总；`diagnostic-3d8fbaa2-a575-4987-b452-19bb40c0ad6e.json` 为板上原始诊断，包含 image、loopback、link、trace、r2_handoff、aon、restore 检查；`off-runtime-passed.json` 保存 25 轮真实会话；`restored-flash-independent-check.json` 保存工作站逐字节比较结论。

本轮关键身份如下，不能用其他版本 ELF 解析同一快照：

| 项目 | SHA256 |
| --- | --- |
| R0 physical-v6 签名包 | `d40443d311edc290535e65de892f253d16bf4bc7d27f974792f52ac6ba8601eb` |
| R0 physical-v6 ELF | `3c5488ae23bedc3ec6504b4f1827e995227ce44feecc3819fa1ad6763edb8f8b` |
| R2 CAN1 release-v2 签名包 | `27eb3e398a68f81fb80a7692d7066e87953ab0b3e0814c806046788b1f4d2926` |
| R2 CAN1 release-v2 ELF | `50fd98518664f2e651f6c8d45c977578bd36080403f39274c9fe7f59dbc415bc` |
| 双槽位写入后整片 | `d92aa47071b39e587eb7de875a2b21faa86f6c34e3fda963fd7d091552b00743` |
| 恢复后的整片基线 | `235ebdfd03cb5be6dd19dd5eb986dcec521948ba105f1d97a911cbb8d1eb302e` |

### 离线重新解析硬件快照

以下命令在工作站的 WSL/Linux 中从项目根目录执行，读取已保存的数据，不连接板子或修改 Flash。需要前述 ELF 和 SRAM 文件仍在本地。

```sh
p85_boot=3d8fbaa2-a575-4987-b452-19bb40c0ad6e
p85_evidence=output/p85_r_core_reading/physical-can-20261119
p85_builds=output/p85_r_core_reading/physical-can-20261117

python3 firmware/p85/tools/inspect_can_link.py \
  --elf "$p85_builds/build-v6/realtime_freertos_core0.elf" \
  --snapshot "$p85_evidence/diagnostic-r0-$p85_boot.bin"

python3 firmware/p85/tools/inspect_r2_handoff.py \
  --elf "$p85_builds/build-r2-handoff-v2/realtime_freertos_core2.elf" \
  --snapshot "$p85_evidence/diagnostic-r2-first-$p85_boot.bin" \
  --second-snapshot "$p85_evidence/diagnostic-r2-second-$p85_boot.bin"
```

两项应为 `status=passed`。`collection_status=complete` 只表示采集完成，必须检查各报告退出码和内容；本次 `all_report_exit_codes_zero=true`、`r2_handoff_runtime_verified=true`、`baseline_restored=true`。

### 在板上查看启动与采集记录

```sh
ssh bst@192.168.1.3
cat /proc/sys/kernel/random/boot_id
cat /proc/uptime
sudo -n journalctl -b -u p85-can-handoff-collect.service --no-pager
sudo -n cat /var/lib/p85-rt0-deploy/20261117-can-handoff-01/rollback-result.json
```

`journalctl -b` 只显示当前 boot；如果之后又上电，应结合事务目录中保存的原始 JSON 和 boot ID 查看历史结果。当前 Flash 已恢复为基线，再次启动后不能继续用 physical-v6 或 R2 release-v2 的 ELF 验收运行镜像；这不会改变本次已归档测试的结论。

## 后续接入真实电机的工作

本轮结束时尚无真实电机反馈，也没有执行使能、保持、位置移动或正弦运动。下一阶段按 [首次实机调试](docs/BRINGUP.md) 核对电机电源、ID、速率和接线，从反馈探测及失能确认开始，再进行小范围、低速度的位置控制。接入前还需明确使用的常驻固件及是否启用诊断自检；双控制器物理自检需要 CAN0↔CAN1 的外部连接，不能把该接线条件直接当作单电机运行条件。

当前实现以一个 64 字节命令帧的单一所有者管理总线，其他槽位显式失能。扩展到四足多关节时，需要集中打包和调度全部槽位、管理每个关节反馈及保护；不能启动多个单电机进程互相覆盖命令帧。现有保护参数及真实命令使用条件见下文。

## Linux 构建和验证

在板上执行下列命令。依赖 CMake、GCC、Python 3；官方 MsgBox 开发包位于 `/usr/bst` 时额外构建真实 A 侧适配器和头文件兼容性检查。

```sh
cd /home/bst/p85_control
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j4
ctest --test-dir build --output-on-failure
```

该 CMake 工程用于 Linux 验证，**不会生成 R0 固件**。没有厂商头文件时，只构建便携核心和仿真测试。内存检查可用 `-DP85_SANITIZE=ON` 创建独立构建目录。

## 完整 R0 构建

首个完整 P85 基线已按固定的公开 manifest 将协议、状态机、运行时、MCAL/MsgBox 及板级任务全部编入 R0。该基线原始 BIN 为 294244 字节，签名包为 1 MiB，ELF 在 R0 的 1 MiB SRAM 中使用 646592 字节。这些大小属于首个基线；后续回环和物理诊断变体以各自 build report 为准。构建脚本、SDK 固定版本、命令、哈希、告警和当时的 5 组软件测试见 [完整构建记录](docs/R0_BUILD.md)。

入口为 `tools/build_hanhai_sdk.py`，在 Linux/WSL 的新隔离目录构建，不访问开发板。原始 SDK 和签名材料保留在忽略目录，不提交到本工程。实际板级流程见 [R0 接入说明](rt0/PORTING.md)；此前固件已通过冷启动和 OFF 状态 A/R 握手；11 月 20 日新演示镜像尚待冷启动验收。CAN FD 控制器回环、物理收发及带电机验证见 [CAN FD 验证范围](docs/CAN_FD_VALIDATION.md)。

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

下列第一条命令只查询 RT0 在线状态；第二条发送 P85 HELLO/状态请求。它们不发送电机使能。第二条须在确认候选镜像已载入 R0 后执行；本轮已完成镜像、内部回环、双向外部物理收发和 OFF 通信验收；11 月 20 日新演示固件的 Flash 状态和待冷启动条件见文首。

```sh
sudo ./build/p85_ipc_probe
sudo python3 host/p85ctl.py status
```

主机默认使用真实 IPC。不存在硬件失败后转入仿真的路径。真实运行命令见 [首次实机调试](docs/BRINGUP.md)，只能在 [R 核接入要求](rt0/PORTING.md) 完成后使用。

首阶段策略：ID 默认 3；主机 ID 0；标准帧、FD+BRS、1 Mbps / 5 Mbps；每个 64 字节命令显式失能其他 5 个槽位。因此**同一总线上其他受独立控制的电机不能与本程序并行运行**。多关节版本必须由一个帧所有者统一打包全部槽位，不能启动多个单电机进程相互覆盖。

默认保护：反馈 100 ms、主机租约 300 ms、使能/失能确认 500 ms；速度 0.5 rad/s、位置相对使能起点 ±0.2 rad、估算及反馈力矩 2 Nm、跟踪误差 0.15 rad、温度 60°C、电压 24–60 V。增益必须显式设置。故障锁存后继续请求失能；仅在新鲜且健康的失能反馈下允许清除。

软件估算力矩不等于驱动器硬限矩；R0 停摆时软件无法继续发送失能。厂商未在所给资料中明确断流保护行为，首次实机测试仍需独立断电/急停条件。正常结束也必须处理负载后再失能。

## 进一步阅读

- [11 月 20 日协议复核、顺序演示与 Flash 写入](docs/MOTOR_DEMO.md)

- [协议及 A/R 业务帧](docs/PROTOCOL.md)
- [完整 P85 R0 构建及产物](docs/R0_BUILD.md)
- [R0 SDK、MCAL 和任务接入](rt0/PORTING.md)
- [实验 sample 审阅与 R0 固件写入流程](docs/R0_FLASHING.md)
- [官网、附件及 R5 编译核查](docs/R0_SDK_FINDINGS.md)
- [板上验证记录、备份和剩余缺口](docs/VALIDATION.md)
- [CAN FD 内部回环、接线与终端依据](docs/CAN_FD_VALIDATION.md)
- [双向物理链路、最终结果与恢复状态](docs/PHYSICAL_CAN_VALIDATION.md)
- [R2 CAN1 释放固件的构建、运行与恢复](docs/R2_CAN1_HANDOFF.md)
- [完整部署、冷启动和回退历史](docs/R0_DEPLOYMENT.md)
- [接线与首次单电机调试](docs/BRINGUP.md)

原始 DOCX/PDF 作为技术资料阅读，其中示例命令并非部署指令。未将旧 DM 电机帧、旧二进制或默认 CAN 演示时序用于 P85 实机。
