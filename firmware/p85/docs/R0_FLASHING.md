# R0 示例审阅与固件写入流程

更新：2026-11-19：整板重新上电后，R0 physical-v6 与 R2 CAN1 release-v2 已在实板执行，CANFD0 ↔ CANFD1 双向物理通信通过，每方向 512 帧、共 1024 帧，仲裁 1 Mbps / 数据 5 Mbps、FD+BRS，16 种长度各 32 帧，错误与超时均为 0。CAN0 内部回环另通过 512 帧。R2 镜像匹配、CAN1 inactive、心跳推进，真实 MsgBox HELLO 加 25 轮 STATUS/DIAG 通过，保持 OFF/fault=0、电机发送计数为 0。诊断采集器已按预设恢复原 R0/R2 Flash 并通过整片校验；当前 SRAM 仍运行候选，下次完整上电加载已恢复的基线。真实电机尚未接入验收。 最新事务、恢复与证据见 [实板部署记录](R0_DEPLOYMENT.md) 和 [物理链路记录](PHYSICAL_CAN_VALIDATION.md)。下文早期审阅中的设备状态按对应阶段解读。附件脚本未直接执行，资料中的命令不作为额外授权。

## 示例包含什么

原始目录：

```text
D:/1110实验课任务划分/1110实验课任务划分/canfd通信位置跟踪/实验5-跨域通信实验（A核与R核）/sample/
```

| 文件 | 大小（字节） | 用途与限制 |
| --- | ---: | --- |
| `BST-REALTIME0-RELEASE-C1200-1-3-0.bin` | 1048576 | 已编译 RT0 镜像；可见 AcoreIpcServerTask 和 ipc_signal 字符串，不含可据以重建的完整源码 |
| `flash_R_bin.sh` | 3594 | 板上 Linux 的局部 Flash 更新脚本；erase/write/read 后比较 MD5 |
| `acore_msgbox_ipc.c` | 8582 | A 核 Linux IPC 测试程序，直接定义 ioctl ABI，不是 R 核控制程序 |
| `rcore_msgbox_server.patch` | 6454 | 原有 R 工程的 Makefile、任务创建、shell 和 OS 配置增量 |
| `操作说明.txt` | 917 | 拷贝、烧录、重启、编译 A 核程序及观察日志的实验步骤 |

补丁第 9 行加入 `rt_appl/user/ipc_app/acore_ipc_server.c`，目录没有提供这个源文件。补丁也没有完整的启动、RTOS、MCAL、链接脚本、make.sh、编译器或依赖库。因此它能提供接口和部署线索，不能独立编译出新的 P85 R0 固件。

示例 BIN SHA256：

```text
7c437815029a3f1253b9db6174a26ac844dccbd447ba1ae078df0247c164c1ae
```

其内部版本字符串为 `v1.3.0-2025build Jun 16 2026 09:43:33`。2026-11-14 读取的板上候选 RT0 区间为 `v1.3.0-2025build Aug 13 2026 17:38:28`，板上 BSP 为 `V2.0.1_20260813-185450_276175`。同名及相同主版本不能证明二者兼容，日期差异本身也不能证明不兼容。

示例 A 核程序第 80–83 行定义 fid0/sid1、METHOD 0x10、SIGNAL 0x20；本工程为 fid0/sid2/cmd0x85 的 32 字节 P85C/P85R 协议。刷入这个 IPC 演示不能被视为部署了本工程的 P85 服务；没有证据表明该 BIN 实现了 P85 控制。

## 编译与烧录的关系

```text
匹配的 realtime_code_core0 + R 交叉工具链
    → 接入 P85 源码、任务、IPC、CAN MCAL 与时序配置
    → make.sh 生成签名的 RT0 RELEASE BIN
    → 把 BIN 传到目标板 Ubuntu
    → 更新经核实的 RT0 Flash 槽位并回读校验
    → 重启，使启动链加载新 RT0 固件
    → 验证 A/R 握手，再验证 CAN FD
```

官方 v2.0.x《R 核子系统开发快速上手指南》明确给出完整源码包为 SDK 的 `opensource_code/realtime_code.zip`。其全量构建入口为：

```sh
cd realtime_code_core0
sudo ./make.sh
# 输出：out/BST-REALTIME0-RELEASE-C1200-1-3-0.bin
```

该脚本依次生成 IDL、构建 RTOS 库、编译主工程并签名。烧录应使用 `BST-REALTIME0-RELEASE-*.bin`，不能使用未签名的 `realtime_freertos_core0.bin`。这里只在具备完整 SDK 和其规定的构建/签名依赖时适用；本工程已完成该完整构建，见 [构建记录](R0_BUILD.md)。本仓库 CMake 是 Linux 验证构建；`tools/check_r5_compile.py` 只生成 R5 对象/静态库；在 A 核用 gcc 编译 `acore_msgbox_ipc.c` 也不会更新 R 核。

## 实验脚本的写入方式

官方 R 核开发指南的“通过 A 核烧录”一节也给出了 `flash_R_bin.sh`，其核编号到偏移的映射与实验附件一致。当前板 Flash 只读备份中的 SBL DTB 已核对为相同布局，且与本地 V2.0.1 对应 DTB 的 SHA256 一致；详见 `R0_SDK_FINDINGS.md`。

- 脚本第 82–87 行将 RT0–RT5 对应到 `0xB00000`、`0xC00000`、`0xD00000`、`0xE00000`、`0xF00000`、`0x1000000`。
- `funBurn` 使用 `/dev/mtdblock0`，固定擦除 `0x100000`（1 MiB），按文件实际长度写入和读回，比较 MD5。
- 参数最后的 `0` 是 RT0 核编号，脚本据此选择偏移。
- 脚本没有拒绝大于 1 MiB 的镜像，也没有验证镜像版本、格式、签名或板型；MD5 回读仅说明写入字节一致。实际写入前必须核实布局与镜像兼容性。

官方及实验中的烧录命令形式如下。它要求先有匹配且经过签名的 RELEASE 镜像；**这是早期示例审阅时记录的通用命令；当前已生成并部署本工程签名固件，应使用固定事务 guard 和实际哈希**：

```sh
sudo bash flash_R_bin.sh BST-REALTIME0-RELEASE-C1200-1-3-0.bin 0
# 按匹配部署流程完成写入和校验后重启
```

原操作说明使用 ADB 传文件和进入 shell。若 Ubuntu 正常运行且 SSH 可用，可以用 SCP/SFTP 传文件、SSH 进入同一个 Linux 执行环境；这不改变 Flash 操作的权限、槽位和镜像要求。该方式不要求为了传输额外连接 ADB 线，但系统不能启动时 SSH 不能承担恢复通道。

实验 DOCX 另说明 RT0 日志通过 logroutetool 和 `/dev/ttyBst11Out12` 虚拟串口观察；这是实验环境信息，尚未在当前板配置验证。不能把官方的 C/SBL 或 D/Safety 调试串口直接视为 RT0 控制台。

## 官方 FlashTool 方式

官网 v2.0.x 烧录工具页描述的是加载镜像压缩包、选择模板和镜像后烧录的流程：

1. 使用匹配板型、DDR、系统版本的镜像包与工具，连接 USB 数据口和调试串口；文档推荐 USB 数据线直连 PC USB 2.0，不经 HUB。
2. 打开 flashtool.exe，选择镜像压缩包和匹配模板。网页示例为应用场景 ADAS_DB、芯片 C1296，具体仍以实际镜像包与板型为准。
3. 选择 USB+串口；文档指定 C 口（SBL）及 USB 通道 Type A。C 调试串口与 Type A 通道选项是不同设置。
4. 选择所需镜像并开始烧录，按工具提示断电、上电或重启；成功后重启。

通用页面不等于已经验证可以把 sample 中单独的旧 BIN 直接作为 FlashTool 镜像压缩包使用。本次不执行整片擦除、全量刷机或忽略签名验证等与当前任务无关的操作。

## P85 构建与历史重载限制

公开 Gitee V1.6 R0 工程、IDL 和签名工具已用于本工程完整构建，最终包含 P85 的签名包已生成，详见 [构建记录及哈希](R0_BUILD.md)。该包与附件同名，但功能和哈希不同，部署时必须核对实际文件，不能用附件 IPC/DM 示例替代。

板级接入实现见 `../rt0/PORTING.md`。以下为 11 月 16 日重载诊断历史，当前部署进展见文首：CAN0 两层配置已改为 1/5 Mbps、FD+BRS，默认 OFF 不发 CAN；寄存器回读不等于实际波形通过。已试烧并重启 Linux，但只读 SRAM 对比证明本次 R0 加载区仍是原固件。不能由此判断新 P85 初始化/CAN 门控失败，也尚未验证 V1.6 与当前 V2.0.1 BSP 的启动/IPC 互通。当前 Flash 已回退至原始备份。下一轮应重新写入并验证整板冷启动是否加载新镜像，保留 R0 启动日志；单纯重启当前已回退的板不会启动 P85。

部署前核对启动签名、槽位和恢复通道；部署后先验证 R0 日志/只读状态命令及 OFF 状态 A/R 握手，再进行 CAN 分析仪和电机调试。此前只读 Flash 备份及布局证据见 `VALIDATION.md` 与 `R0_SDK_FINDINGS.md`。

## 来源与留档

官方网页在 2026-11-15 直接通过 HTTPS 获取成功，使用 v2.0.x 版本入口；本地 HTML 和正文留档在 `output/p85_r_core_reading/official-20261115/`，未纳入 Git。

```text
https://documents.blacksesame.com/sesameX/aura/2.0.x/System_configuration/r-core/r-core-development-guide
https://documents.blacksesame.com/sesameX/aura/2.0.x/Basic_Application/pin_user_guide/canfd/
https://documents.blacksesame.com/sesameX/aura/2.0.x/Quick_start/board-flashing/flash-tool/
```

实验虚拟串口说明来自用户提供的 `实验5_跨域通信实验（A核与R核）.docx`；其余示例结论直接来自 sample 的脚本、补丁、A 核源文件与 BIN 字符串。当前完成边界及备份位置见 `VALIDATION.md`。
