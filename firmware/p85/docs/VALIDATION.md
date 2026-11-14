# 板上验证记录

记录日期：2026-11-14（工作站 Asia/Tokyo）。目标板 RTC/墙钟不准确，本记录不采用板上日期作为归档日期。

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

已执行的写入为用户目录内源码、Linux 构建产物及软件测试记录。**未擦写 Flash、未部署新 R0 固件、未重启、未向真实电机发使能或 CAN 控制帧。**

## 软件验证

普通构建使用 C99、`-Wall -Wextra -Werror -Wpedantic -Wshadow -Wstrict-prototypes`。完整 CTest 四组通过，随后因新增 MsgBox 接收任务路由访问接口，针对该组重新构建并验证通过。其余控制与运行时源未因该接口变更而修改。

| 组 | 结果 | 核心覆盖 |
| --- | --- | --- |
| p85_core | 9,905 检查，0 失败 | 帧/槽位/端序、零值、边界及非有限值、错误反馈、状态与保护、轨迹幅值/周期、MSGID/时间回绕、重放 |
| p85_runtime | 406 检查，0 失败 | 有界队列、过期请求、bus fault、IPC 回复失败、TX 失败、握手序号回绕 |
| p85_host | 6 个流程通过 | 完整正弦和失能确认、位置移动、无电机探测失败、反馈中断、主机租约过期、Ctrl+C 清理 |
| p85_bst_adapter | 最终 46 检查，0 失败 | 使用板上真实官方头文件、假传输的服务生命周期、路由、长度单位、回复头字段、端点访问及错误路径 |

C 检查合计 10,357 项；这包括大量数值采样断言，不应解读成相同数量的独立实机测试。

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

该结果表明**本工程的 A/R 应用通路尚未通过**。在线查询证明端点存在，不证明其运行 P85 服务。A 侧适配器已在实际官方头文件/库上构建并执行；R 适配器仅通过 Linux AArch64 的头文件/API 检查和假传输测试，尚未用实际 R 编译器链接或执行。MCAL 发送适配器尚未用 R MCAL 头文件编译。

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

`tools/check_r5_compile.py` 还用 Cortex-R5/soft-float 探测配置将 P85 四个便携源文件编译为 `libp85_r5.a`，严格告警检查通过。这不是固件链接或实机测试；MsgBox/MCAL 适配器仍待完整 R SDK。官方指南确认可独立构建及完整包名 `opensource_code/realtime_code.zip`。详细检索范围、构建证据、布局与签名依赖见 [源码与编译核查](R0_SDK_FINDINGS.md)。

## 完成边界和剩余验收

| 层级 | 当前状态 |
| --- | --- |
| P85 帧编码与反馈解析 | 完成软件实现与边界验证，待实机交叉核对 |
| 轨迹、保护、会话和 CSV | 完成软件实现与 Linux 仿真验证 |
| A 核真实 MsgBox 库 | 已构建并执行，P85 服务握手仍失败 |
| R0 任务、IPC 接收与 MCAL | 可移植核心及适配基础已写，缺匹配 SDK 进行最终接入 |
| R5 目标编译 | 裸机示例构建、P85 便携对象编译已通过 |
| 完整 P85 R0 固件构建/烧录 | 未完成，缺完整 SDK 及签名依赖 |
| CAN 1/5 Mbps、FD/BRS/TDC、过滤器 | 未完成实际配置及波形验证 |
| 电机保持、位置移动、正弦 | 未接实物，未验证 |

下一项外部依赖是匹配该板 BSP 的 `opensource_code/realtime_code.zip` 及 IDL/签名依赖；R5 工具链已有。随后按 `../rt0/PORTING.md` 逐项完成接入。没有这部分不能承诺“只接电机即可运行”。即使无电机时固件和分析仪验证通过，实际 ID、动力学增益、方向和断流停止行为仍需要首次台架验收。
