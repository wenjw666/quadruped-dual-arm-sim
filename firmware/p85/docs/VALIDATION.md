# 板上验证记录

记录起始日期：2026-11-14；更新：2026-11-19（工作站 Asia/Tokyo）。目标板 RTC/墙钟不准确，本记录不采用板上日期作为归档日期。

## 目标环境与范围

| 项目 | 实际结果 |
| --- | --- |
| SSH | `bst@192.168.1.3`，以太网连接 |
| 主机名 / 架构 | c1200-ubuntu / aarch64 |
| 系统 | Ubuntu 22.04.5 LTS |
| 内核 | `6.1.64-rt17 #1 SMP PREEMPT_RT Thu Aug 13 18:19:56 CST 2026` |
| BSP | `V2.0.1_20260813-185450_276175` |
| DT model | BST C1200 CDCU |
| Linux 工具 | GCC 11.4、CMake、make、Python 3.10 |
| 官方 IPC | hanhai-ipc-ubuntu / -dev / -demo / -dbg，4.4.7.0-r0 |
| 设备及 SDK | `/dev/ipc_msgbox0`、`/usr/bst/include/msgbx`、`/usr/bst/lib/libmsgbx.so` |
| 源码部署 | `/home/bst/p85_control` |
| 本地分支 | `codex/p85-rt0-motor-control` |

检查时未发现可直接使用的 SocketCAN 接口、remoteproc 实例、完整 R SDK 或 R 交叉编译器。R/CAN 路线仍通过 RT0 MCAL 接入；没有把 Linux SocketCAN 当作已存在的后备通路。

2026-11-19：整板重新上电后，R0 physical-v6 与 R2 CAN1 release-v2 已在实板执行，CANFD0 ↔ CANFD1 双向物理通信通过，每方向 512 帧、共 1024 帧，仲裁 1 Mbps / 数据 5 Mbps、FD+BRS，16 种长度各 32 帧，错误与超时均为 0。CAN0 内部回环另通过 512 帧。R2 镜像匹配、CAN1 inactive、心跳推进，真实 MsgBox HELLO 加 25 轮 STATUS/DIAG 通过，保持 OFF/fault=0、电机发送计数为 0。诊断采集器已按预设恢复原 R0/R2 Flash 并通过整片校验；当前 SRAM 仍运行候选，下次完整上电加载已恢复的基线。真实电机尚未接入验收。 本次 boot 为 `3d8fbaa2-a575-4987-b452-19bb40c0ad6e`，R0 运行时钟在应用查询中推进 6,010 ms。11/11 板上 CTest 与 guard 9 个子测试于 2026-11-17 已通过；本次没有代码修改或重复执行这些软件测试。详情见 [物理链路记录](PHYSICAL_CAN_VALIDATION.md)。

以下冷启动记录为 2026-11-17 的历史验收：

2026-11-17 历史状态：工作站 10:48:17 捕获本轮整板冷启动，10:50:34 归档自动验收成功结果。boot ID 为 `3b54e8d9-bfa4-40da-baa8-1d3e870abced`。候选 ELF 的代码/只读数据 51,307 字节与 R0 SRAM 全部匹配，AON realtime0 为 `4/normal`，真实 MsgBox 的 HELLO 和 25 轮 STATUS/DIAG 全部通过。

R0 设备时间推进 5,280 ms，STATUS/DIAG 一对请求最大往返时间为 19.9221 ms；保持 OFF、fault=0，CAN tx_ok/tx_failed/rx_ok 均为 0。该耗时反映 A/R 查询，不是 CAN FD 总线时延。A 侧 MsgBox 单次在线查询缓存修复已通过这些连续请求验证。

guard 已重新核对完整 Flash、接受候选并移除 armed 标记，rollback timer 已 disabled；自动验收服务 Result=success、ExecMainStatus=0。该历史阶段无需再次重启。**当时未连接真实电机，未发送电机使能或 CAN 控制命令，尚未进行后续内部回环与物理测试。当前进展以文首为准。** 详细状态和证据见 [实板部署记录](R0_DEPLOYMENT.md)，回环支持依据和测试范围见 [CAN FD 验证范围](CAN_FD_VALIDATION.md)。

以下 2026-11-14 至 16 的检查为各次历史结果；涉及“已回退”的描述不代表最新冷启动验收后的当前 Flash。

## 软件验证

普通构建使用 C99、`-Wall -Wextra -Werror -Wpedantic -Wshadow -Wstrict-prototypes`。2026-11-15 在 `/home/bst/p85_validation/integration-20261115` 隔离构建并运行 CTest，完整 5 组全部通过（8.31 s）。

| 组 | 结果 | 核心覆盖 |
| --- | --- | --- |
| p85_core | 9,905 检查，0 失败 | 帧/槽位/端序、零值、边界及非有限值、错误反馈、状态与保护、轨迹幅值/周期、MSGID/时间回绕、重放 |
| p85_runtime | 406 检查，0 失败 | 有界队列、过期请求、bus fault、IPC 回复失败、TX 失败、握手序号回绕 |
| p85_host | 6 个流程通过 | 完整正弦和失能确认、位置移动、无电机探测失败、反馈中断、主机租约过期、Ctrl+C 清理 |
| p85_bst_adapter | 最终 46 检查，0 失败 | 使用板上真实官方头文件、假传输的服务生命周期、路由、长度单位、回复头字段、端点访问及错误路径 |
| p85_hanhai_receive | 35 检查，0 失败 | 官方头文件/假传输的多 packet、分片拒绝、query 回收前复制、token/序号和错误返回 |

C 检查合计 10,392 项；这包括大量数值采样断言，不应解读成相同数量的独立实机测试。

AddressSanitizer + UndefinedBehaviorSanitizer + leak checking：核心、运行时和主机流程通过；最终 MsgBox 适配组也单独通过（6.07 s）。这些检查运行在 Linux A 核进程上，不代表 R0 栈水位、中断安全或 DMA 缓冲生命周期已验证。

## 有记录的正弦软件演练

板上执行 `tools/capture_simulation.py`；理想位置伺服模型经过实际 P85 帧编解码，但完全不访问 CAN 或 R0。

- 参数：A=0.02 rad、周期 1 s、稳定段 2 周期、两侧各 0.5 s 包络；Kp=5、Kd=0.1 **仅为仿真参数**。
- 全部采样 400 行，活动采样 395 行。
- 位置 RMSE：`9.806470351736833e-05 rad`。
- 最大绝对位置误差：`0.000377 rad`。
- 最大绝对速度：`0.125961 rad/s`。
- 最大绝对力矩：`0.009155 Nm`，由理想模型与量化形成，非电机实测。
- 设备采样间隔中位数 10 ms，最大 11 ms；这里的设备为 Linux 仿真进程，**不是 R0 定时测试**。
- 无锁存故障；控制程序日志含新鲜反馈的 `disabled_confirmed=true`。

记录：

```text
板上 /home/bst/p85_validation/software-validation-20261114/
本地 output/p85_r_core_reading/software-validation-20261114/
  sine.csv
  summary.json
  controller.log
  simulator.log
```

## 真实 IPC 验证

`sudo ./build/p85_ipc_probe` 使用官方公共库，只查询在线状态，无 payload：

```text
CPU_0 -> REALTIME_0 fid=0 sid=2: query=2
R core online; P85 protocol unverified
```

`sudo python3 host/p85ctl.py status` 使用真实 MsgBox 发送 P85 HELLO，结果：

```text
P85 MsgBox exchange failed (-110)
```

该结果表明**本工程的 A/R 应用通路尚未通过**。在线查询证明端点存在，不证明其运行 P85 服务。A 侧适配器已在实际官方头文件/库上构建并执行；R 适配器和 MCAL 已在后续公开 V1.6 R0 SDK 中完整编译链接；后续试烧后 HELLO 仍超时，但 SRAM 对比表明 R0 仍载有原固件，不能用这次超时判断新 P85 代码执行结果。

## 原始 Flash 只读备份

通过目标板只读字符设备 `/dev/mtdblock0ro`（major90/minor1）执行 MEMGETINFO 并按偏移只读，取得 64 MiB 基线；没有调用写或擦除。读取信息：size=67108864、erase=4096、write=1、type=1、flags=7168。

| 文件 | SHA256 |
| --- | --- |
| flash-baseline-64m.bin | `81f90bfb533be3b374e8a55787d525cf64066f5b8982b8138bc29a58cafd3417` |
| rt0-offset-b00000-unverified-1m.bin | `2378169de79bf9d5c4f62ed9ce3dd6740d6d21717ba7c00b6ba401ae49c613ac` |

板上位置 `/home/bst/p85_control/artifacts/2026-11-14/`；本地位置 `output/p85_r_core_reading/board_backup/`，均不进 Git。第二个文件最初按旧例偏移截取，文件名保留 `unverified` 以保持记录一致。2026-11-15 又解析了同一 Flash 备份中的 SBL DTB：RT0 配置为 `0xB00000/1 MiB`，SRAM 为 `0x04000000`，且 DTB 与本地 V2.0.1 对应文件 SHA256 相同，因此现在有布局配置依据。区间中可见 `CanAppTask`、`welcome to bst-rtos`、`v1.3.0-2025build Aug 13 2026 17:38:28`，但这些不证明已有 P85 服务，也不验证待刷镜像。

## SDK 与资料检查

- 用户的 V2.0.1 镜像 SDK ZIP 已检查：包含镜像与烧录相关文件，未找到完整 R0 工程。
- A/R 实验的 `rcore_msgbox_server.patch` 引用了未附带的 `acore_ipc_server.c`，不能仅凭补丁恢复完整服务。
- 两份同名 RT0 固件哈希不同，分别可见 IPC/DM 示例线索，没有可确认的 P85 组合固件；未烧录。
- 官方 SDK Manager 25（V2.1.0）已完整下载（4,517,041,049 字节），流式遍历 Docker 各层的 332,916 个文件条目，扫描正常结束。未发现 `realtime_code_core0`、CAN MCAL 配置或可用的 R 核完整工具链。`r5baremetal.zip` 有 6 个 ZIP 条目（1 个目录和 5 个文件，含启动、链接脚本、主程序、Makefile 及 readme）；已检查的 Linux IPC 源包、应用包及启动脚本也不能替代 RTOS/CAN MCAL 工程。完整归档和扫描清单保存在本地忽略目录 `output/p85_r_core_reading/`，未运行其安装脚本。
- 官方在线 CAN 示例讲解 `realtime_code_core0`，默认 500 kbps/1 Mbps，不能直接用于 P85 的 1/5 Mbps；网页时序表须与实际结构/寄存器编码核对。
- 官方硬件图纸中找到 V1.2 J0801 的 RT_CANFD0 引脚，但尚未确认用户实物载板修订版。接线表见 `BRINGUP.md`。

## 2026-11-15 R5 交叉编译核查

已在本机 WSL 使用 ARM 官方 12.2.MPACBTI-Rel1 工具链构建公开 SDK 中的 `r5baremetal`：生成 66,888 字节 BIN，ARM ELF32，入口/加载地址 `0x04000000`。原示例入口名称不匹配，在隔离副本中将链接脚本 `_reset` 改成汇编实际导出的 `_boot` 后消除相应告警；未将其认定为适合当前板部署的发布镜像。

`tools/check_r5_compile.py` 还用 Cortex-R5/soft-float 探测配置将 P85 四个便携源文件编译为 `libp85_r5.a`，严格告警检查通过。这是此前的对象探测，不是当时的固件链接或实机测试；后续完整集成结果见下面的新记录。官方指南确认可独立构建及完整包名 `opensource_code/realtime_code.zip`。详细检索范围、构建证据、布局与签名依赖见 [源码与编译核查](R0_SDK_FINDINGS.md)。

## 2026-11-15 P85 完整 R0 接入构建

公开 V1.6 manifest 的全部 19 个项目已固定 commit。`prepare_hanhai_sdk.py` 在新目录接入 P85，`build_hanhai_sdk.py` 完成 IDL、FreeRTOS/驱动、8 个 P85 源文件编译、ELF 链接、objcopy 和签名包生成。最终原始 BIN 294244 字节，签名包 1048576 字节；SRAM LOAD 跨度 646592 字节，余量 401984 字节。完整哈希与复现命令见 [构建记录](R0_BUILD.md)。

目标属性已检查为 ARM ELF32、Realtime CPU profile、VFPv3-D16、VFP 寄存器参数，入口/加载起点 `0x04000000`。P85 的任务/接收/运行时/MCAL/状态命令符号已进入 ELF。启动默认 OFF，软件测试中没有 CAN 发送。新增接收测试本轮尚未执行 sanitizer；既有 sanitizer 记录不扩展到新组。

日志和报告位于本地 `output/p85_r_core_reading/p85-r0-integration-20261115-final/`，其中 `linux-tests/` 保存本轮板上 Linux 编译和测试日志。SDK 自带签名材料只用于本地工具运行，未加入本工程版本管理。

## 完成边界和剩余验收

| 层级 | 2026-11-15 当轮状态（历史） |
| --- | --- |
| P85 帧编码与反馈解析 | 完成软件实现与边界验证，待实机交叉核对 |
| 轨迹、保护、会话和 CSV | 完成软件实现与 Linux 仿真验证 |
| A 核真实 MsgBox 库 | 已构建并执行；试烧前后 HELLO 均超时，R0 SRAM 仍为原固件 |
| R0 任务、IPC 接收与 MCAL | 已在公开 V1.6 SDK 接入并完整编译链接 |
| 完整 P85 R0 固件构建 | 已完成 ELF/BIN/HEX/MAP 及 1 MiB 签名包 |
| 烧录、启动与 BSP 互通 | 槽位写入/回读及 Linux 重启已执行；R0 未载入 P85，已回退；兼容性仍未验收 |
| CAN 1/5 Mbps、FD/BRS/TDC、过滤器 | 两层配置和寄存器门控已实现；TDC/波形/ACK 未实测，硬件仍宽过滤、软件筛 ID3 |
| 电机保持、位置移动、正弦 | 未接实物，未验证 |

下一步为重新部署后进行整板冷启动和启动日志核对，确认 R0 SRAM 中确为本次构建，再做 OFF 状态 A/R 握手、CAN FD 台架验收及电机测试。协议实现、固件构建和软件测试不能替代实物 ID、增益、方向、供电/接线、调度及断流停止行为的确认。

## 2026-11-16 复位与启动诊断

第二轮 RT0 写入/回读通过，Safety 核复位请求回复成功；但新 ELF 的 44 个代码/只读区间与 R0 SRAM 全部不匹配，因此未执行 P85 OFF 状态采样。已恢复原槽位并以整片 64 MiB SHA256 确认恢复，timer disabled、armed 标记清除。`sys_rt0` 名称复测也未观察到启动计数增加。详情和证据见 [部署与回退记录](R0_DEPLOYMENT.md)。

新增 `tools/inspect_aon_state.py` 及 10 个 unittest 方法，验证固定 AON 布局与损坏数据拒绝。实板连续 25 轮 Safety IPC 与 AON 直接读取交叉检查，50 次字段比对一致；R0 为状态 0/default、启动原因 1/cold、计数 1。AON 读数和成功的复位回复都不构成 R0 程序运行证明。

板上软件 CTest 为 6/6 通过：核心、运行时、AON 解析、主机仿真、BST 适配器和 Hanhai 接收。本轮不增加电机硬件验收范围；无真实电机、CAN ACK/波形或位置反馈测试。下一项关键验收是重新部署后的整板断电再上电，并结合串口启动日志确认镜像加载，再进入 HELLO/STATUS/DIAG 的 OFF 状态检查。该历史轮次结束时已回退；2026-11-17 已重新写入候选，当前状态见文首及部署记录。
