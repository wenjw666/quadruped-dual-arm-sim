# CANFD0 ↔ CANFD1 物理链路测试

更新：2026-11-19（工作站 Asia/Tokyo），分支 `codex/p85-rt0-motor-control`。本轮真实硬件验证通过；板上墙钟不准确，运行身份以下述 boot ID 和 uptime 为准。

## 2026-11-19 实板结果

boot ID 为 `3d8fbaa2-a575-4987-b452-19bb40c0ad6e`，与 origin boot 不同。R0 physical-v6 和 R2 CAN1 release-v2 的 SRAM 与对应 ELF 内容匹配。采集器各报告退出码均为 0，`all_report_exit_codes_zero=true`、`r2_handoff_runtime_verified=true`。

| 检查 | 实测结果 |
| --- | --- |
| R0 镜像 | 代码/只读数据 57,411 字节匹配 |
| R2 执行 | 每份快照匹配 13,915 字节；stage=2，心跳 2,153→2,181，delta=28 |
| R2 CAN 配置 | CAN1 inactive；CAN2/3 active 且 START 返回 0 |
| AON | R0、R1、R2 均为 cold、normal |
| CAN0 内部回环 | submitted/confirmed/received 均为 512，耗时 1,024 ms，无错误 |
| CAN0→CAN1 物理收发 | submitted/confirmed/received 均为 512 |
| CAN1→CAN0 物理收发 | submitted/confirmed/received 均为 512 |
| 长度与载荷 | 每方向 16 种合法 FD 长度各 32 帧；ID、标志和完整载荷一致 |
| 物理模式 | 两控制器正常模式，无内部回环、无自接收；仲裁 1 Mbps / 数据 5 Mbps，FD+BRS/TDC |
| 物理错误与耗时 | CAN0/CAN1 faults=0，timeouts=0，bad_metadata/bad_payload/unexpected=0，ECR=0；流程总耗时 2,048 ms |
| 恢复控制器配置 | CAN0 保持正常运行、CAN1 已停止，restored=1 |
| 真实 A/R 应用通信 | MsgBox HELLO + 25 轮 STATUS/DIAG 通过，R0 时钟推进 6,010 ms |
| 应用状态 | OFF、fault=0、motor_status=0；电机 tx_ok/tx_failed/rx_ok/rx_rejected 均为 0 |

诊断的 1,024 帧是 CAN FD 测试载荷，不是电机命令；应用中的电机帧计数独立统计，保持为 0。2,048 ms 是含程序节拍的自检总耗时，不能当作总线吞吐率或单帧延迟。当前结果证实 R0 使用两路物理 CAN FD 收发成功，也支持此前 CAN1 配置冲突已通过 R2 释放得到解决；owner 字段本身仍只是配置声明。

用户保持 J0801 的 CAN0H pin9↔CAN1H pin13、CAN0L pin7↔CAN1L pin11 和公共地接线即可对应本次结果。测试通过证明这组线缆和板载收发器在本轮配置下可用；没有测量终端电阻实装值或总线波形。真实 P85 尚未接入，因此本次结果不代表电机协议反馈、使能和位置运动已通过实物验收。

## 运行镜像与 Flash 状态

collector 在 uptime 39.47 s 前完成证据采集和恢复。按诊断事务预设，即使报告全部通过，仍恢复 R0/R2 两个 Flash 槽位；`accepted=false` 是预期行为。恢复后整片 64 MiB SHA256 为 `235ebdfd03cb5be6dd19dd5eb986dcec521948ba105f1d97a911cbb8d1eb302e`。读回文件已下载到工作站，与本地原始基线逐字节比较一致。R1 和槽位外数据保持原样。

`p85-can-handoff-collect.service` Result=success、ExecMainStatus=0，恢复后 collector 与 rollback timer 均为 disabled/inactive，`armed.json` 已移除。未发出额外重启。当前 SRAM 仍运行本轮候选，OFF 查询是在 Flash 恢复后完成；再次整板上电会加载恢复后的内部回环 v3 基线，而非这份双向物理测试固件。

## 本轮证据

目录：`output/p85_r_core_reading/physical-can-20261119/`（Git 忽略）。

- `validation-summary.json`：本轮汇总，保留硬件报告和应用检查。
- `diagnostic-3d8fbaa2-a575-4987-b452-19bb40c0ad6e.json` 及三个 SRAM `.bin`：板端原始采集；下载后哈希与报告一致。
- `off-runtime-passed.json`、`verify-off-runtime.py`：真实 MsgBox 应用查询与 25 轮完整记录；只发 hello/status/diag。
- `boot-and-collector.txt`：启动身份、unit 状态、日志和恢复结果。
- `rollback-result.json`、`flash-restored.bin`、`restored-flash-independent-check.json`：Flash 恢复及工作站独立逐字节验证。

以下保留部署前诊断和可复现参数，按对应日期解读。

更新：2026-11-17（Asia/Tokyo），分支 `codex/p85-rt0-motor-control`。板上墙钟不准确，现场身份以 boot ID 和 uptime 为准。

## 2026-11-17 部署前状态（历史）

2026-11-17 部署时，R0 physical-v6 与 R2 CAN1 release-v2 已配套编译、签名、写入并完成整片读回验证，当时等待整板主电源断电上电，尚无物理双向收发结果。2026-11-19 的实际通过结果见文首。电机未连接，用户已将 CANFD0 与 CANFD1 连接。

当前 boot 为 `a6ebb93d-d849-4ab2-8ffb-d9e1a14c37b8`，SRAM 仍运行已验收的内部回环 v3，不能把 Flash 写入成功当作新固件执行成功。本 boot 已再次确认 53,771 字节代码/只读数据匹配；内部回环 submitted/confirmed/received 均为 512，16 种长度各 32 帧，错误与超时为 0，耗时 1,024 ms，restored=1。

本轮新事务为 `/var/lib/p85-rt0-deploy/20261117-can-handoff-01`。下次上电后自动读取 R0/R2 已审核的 SRAM 区域，分别验证镜像身份、R2 心跳推进和 CAN1 释放声明、内部回环、物理链路、启动跟踪和 AON，随后恢复两个 Flash 槽位到原有验收基线，不自动重启，也不自动接受候选为新基线。SRAM 中的诊断现场仍可继续读取。15 分钟定时器负责采集异常时的后备恢复。

## 2026-11-17 CAN1 失败定位（历史）

先前 diagnostic-v6 在 boot `54a61001-fce1-4de4-b9f1-aec42a0ba508` 已实际执行，内部回环 PASS，物理测试在首次发送前失败，两个方向 submitted/confirmed/received 均为 0。启动跟踪有 35 条完整记录，无丢失。

| 时点 | CAN1 寄存器快照 |
| --- | --- |
| R0 初始化完成 | MCR `0x59821853`，CTRL1 `0x00DB0006`，CBT `0x80139A73`，FDCBT `0x00031C63`，FDCTRL `0x86DB0000` |
| CAN0 回环后、CAN1 START 前 | MCR `0x00831802`，CTRL1 `0x01DB4006`，CBT `0x80339A73`，FDCBT `0x00332C63`，FDCTRL `0x86DB9400` |

R0 先正确配置 CAN1，随后其时序、消息缓冲数等被改为原 R2 示例使用的值。**SDK 枚举为 UNINIT=0、STARTED=1、STOPPED=2。** `start_enter aux=1` 表示 R0 的 CAN1 软件状态已变成 STARTED，上层 START 因重复状态请求直接拒绝；此次 START 没有进入底层退出 freeze 的路径，不能解释成已经证实的 exit-freeze 超时。`Can_MainFunction_Mode()` 会遍历控制器，并可根据硬件状态更新 R0 的软件状态。

匹配 SDK 的 R2 源码中，`main.c` 调用 `Can_Init(&Can_Config)`，CAN1 为 active，`CanApp.c` 无条件启动 CAN1/2/3，并注册 CAN1 中断。原 SBL 也记录了 `release realtime2 done`。这些证据支持跨核 CAN1 争用的判断；尚未在硬件上证明具体覆盖指令来自 R2。仅凭 AON default、镜像存在或静态快照不变，不能确认 R2 是否执行。

v6 原诊断事务在 uptime 29.43 秒完成采集并恢复完整 Flash 基线，`accepted=false`。之后用户再次上电得到当前 boot，旧诊断 collector/timer 已停止并禁用。原始证据位于 `output/p85_r_core_reading/physical-can-20261117/diagnostic-followup/`。

## R2 配套修改

新变体基于固定 `C1200SW_V1.6_RC0930` 源码构建：关闭 R2 CAN1 activation、删除其 START 和 CAN1 ISR 注册，禁用原始 CAN shell 测试并拒绝 `cansend` 操作 CAN1。CAN2/3 的初始化与启动保留。

R2 在 `Can_Init` 后发布实际 activation 配置，并在任务中发布阶段、CAN2/3 START 返回值和周期心跳；报告按 32 字节对齐并显式清理缓存。验收必须同时匹配 ELF 不变内容和两个间隔快照中的心跳推进，镜像存在不等于代码执行。报告中的 CAN1 owner 字段是配置声明，需结合固定源码和 R0 的实际寄存器/物理收发结果判断。

R2 完整编译、链接和签名打包通过，入口 `0x04200000`，SRAM 结束 `0x04262230`，余量 646,608 字节，报告地址 `0x0425b8a0`。可复现构建说明见 [R2 CAN1 释放构建与部署](R2_CAN1_HANDOFF.md)。

## 双槽位诊断部署记录（2026-11-17）

| 项目 | 值 |
| --- | --- |
| 板端事务 | `/var/lib/p85-rt0-deploy/20261117-can-handoff-01` |
| origin boot | `a6ebb93d-d849-4ab2-8ffb-d9e1a14c37b8` |
| R0 槽位 | offset `0xB00000`，长度 `0x100000` |
| R2 槽位 | offset `0xD00000`，长度 `0x100000` |
| R0 签名包 SHA256 | `d40443d311edc290535e65de892f253d16bf4bc7d27f974792f52ac6ba8601eb` |
| R2 签名包 SHA256 | `27eb3e398a68f81fb80a7692d7066e87953ab0b3e0814c806046788b1f4d2926` |
| R0 ELF SHA256 | `3c5488ae23bedc3ec6504b4f1827e995227ce44feecc3819fa1ad6763edb8f8b` |
| R2 ELF SHA256 | `50fd98518664f2e651f6c8d45c977578bd36080403f39274c9fe7f59dbc415bc` |
| 基线整片 SHA256 | `235ebdfd03cb5be6dd19dd5eb986dcec521948ba105f1d97a911cbb8d1eb302e` |
| 写后整片 SHA256 | `d92aa47071b39e587eb7de875a2b21faa86f6c34e3fda963fd7d091552b00743` |
| 槽位外比较 | 整片验证一致；R1 `0xC00000..0xCFFFFF` 保持原样 |
| 部署时验收 | 当时 `accepted=false`，等待新 boot；2026-11-19 实测已通过并恢复基线 |

写入顺序为 R2、R0。备份、包哈希和 origin boot 先校验，恢复标记与下次启动服务先持久化，再擦写。两个槽位逐一写入后读取完整 64 MiB 验证；读回文件也已下载工作站，与本地候选和基线独立逐字节比较。没有改动 SBL。

`p85-can-handoff-collect.service` 和 `p85-can-handoff-rollback.timer` 在 origin boot 均为 enabled、inactive，未启动；2026-11-19 已执行采集、恢复并禁用。collector 已实际拒绝 origin boot，避免把旧 SRAM 当成新固件结果。旧的 rt0、loopback、physical 自动接受服务已禁用，旧事务没有 armed marker。

## 测试流程与通过条件

R0 先做 CAN0 内部回环，再切换两路正常模式执行外部双向测试。仲裁 1 Mbps、数据 5 Mbps，FD+BRS、TDC 开启，关闭内部回环、监听和自接收。CAN0 → CAN1 使用标准 ID `0x5A5`，反向为 `0x5A6`；每方向 512 帧，共 1024 帧，覆盖 `0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64` 字节，每种 32 帧，逐帧核对 ID、标志、长度、发送确认与完整载荷。测试数据不是 P85 电机命令。

测试完成后关闭 CAN1、恢复 CAN0 正常模式，应用保持 OFF。失败关闭 CAN 控制器及 P85 MsgBox 服务，板级 startup=-8。Linux 不读取 CAN MMIO；先前这种读取曾引发 RCU stall，后续由 R0 采样并发布寄存器报告。

继续保留用户已经连接的 J0801：CAN0H pin9 ↔ CAN1H pin13、CAN0L pin7 ↔ CAN1L pin11，公共 GND pin5/6/19。公开原理图显示分裂终端约 120.8 Ω/端，尚未测量实装值。详细依据见 [CAN FD 接线与终端](CAN_FD_VALIDATION.md)。2026-11-17 的失败发生在外部发送前，不能归因于接线或终端；2026-11-19 保持现有接线的双向物理测试通过。

本次诊断预设的实测条件要求新 boot、R0/R2 镜像匹配、R2 心跳与 CAN1 inactive、内部回环通过、外部双向严格通过、AON normal。`collection_status=complete` 仅表示采集完成；需单独检查 `link` 结果。诊断结果即使通过，也会先恢复原 Flash；真实 HELLO、25 轮 STATUS/DIAG、OFF/fault=0、运行时钟推进及电机发送计数为 0 的应用验收现已通过；本事务仍按预设保持 accepted=false，不提供接受候选操作。

## 软件验证与历史结果

本轮目标板 `/home/bst/p85_validation/handoff-20261117/build` 的 11/11 CTest 通过，覆盖协议、运行时、主机流程、官方 BSP/MsgBox、真实接收队列、各报告解析与双槽位 Flash guard。guard 增加候选路径/哈希约束后，9 个子测试在工作站通过，并在目标板单独回归通过。部署前 R2 实际 ELF 的离线快照是合成测试，不代表硬件运行；2026-11-19 已另行取得实际 SRAM 与心跳证据。

此前内部回环 v3 在 boot `3b0710a2-ed60-4704-9308-db792345aef1` 已完整通过硬件及 OFF 验收。随后 boot `ce894ce6-3640-4f5b-863c-16dfc7586028` 出现同一缓存行中的统计字段陈旧，因此为报告加入缓存同步。physical-v3 在 boot `ede639a4-f27a-4f83-813b-606d54a815c7` 的完整回环报告通过，外部测试在 CAN1 START 前失败，诊断结果进一步由本轮 v6 跟踪明确。

原始材料保留于 `output/p85_r_core_reading/physical-can-20261117/`：`coldboot-check/`、`diagnostic-followup/`、`build-v6/`、`build-r2-handoff-v2/`、`handoff-board-software-tests.txt` 和 `handoff-deployment/`。最后一目录包含固定部署包、Flash 读回、origin boot 拒绝、check-only、unit 状态和独立本地校验结果。
