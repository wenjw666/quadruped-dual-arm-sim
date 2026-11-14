# 接入匹配的 R0 工程

此文件是接入说明，**不是已完成的板级实现**。`p85_runtime.c` 可移植，两个厂商适配器仍需与实际 R SDK 一起编译、链接和上板验证。Linux 上对官方头文件的编译成功不证明 R0 ABI、库或驱动可用。

## 当前缺少的输入

1. 与板上 `V2.0.1_20260813-185450_276175` 相容的 `realtime_code_core0` 源码和所需库，包含启动、链接脚本、RTOS、MsgBox、CAN MCAL、CanIf 配置和时钟配置。
2. 与完整 SDK 匹配的浮点/枚举 ABI、构建脚本及 IDL/签名依赖。本机 ARM 12.2 工具链已经可用，裸机示例和 P85 四个便携源文件已交叉编译；还没有链接完整 CAN/IPC 固件。完整包名为 `opensource_code/realtime_code.zip`，详见 `../docs/R0_SDK_FINDINGS.md`。
3. 拟部署镜像的启动校验/签名、版本兼容性及恢复流程。当前板只读备份中的 SBL DTB 与本地 V2.0.1 对应文件哈希相同，已确认配置为 RT0 Flash `0xB00000`、容量 1 MiB、SRAM `0x04000000`；这尚不证明任意镜像可用。
4. CAN0 实际 HTH、RxPduId 到仲裁 ID 的传递、时钟和 TDC 配置、载板版本及收发器控制。

使用 SDK 2.1.0 材料时仍需确认其与当前 2.0.1 BSP 的兼容性。当前所有板上改动限于 `/home/bst/p85_control` 和仿真记录；Flash 只读备份已完成。

## 构建单位

将以下源文件加入 R0 工程，加入 `include/`、`rt0/` 头文件路径：

```text
src/p85_protocol.c
src/p85_control.c
src/p85_wire.c
rt0/p85_runtime.c
rt0/p85_bst_msgbox.c
rt0/p85_mcal_tx.c
```

核心要求 C99、32 位 `uint32_t`、IEEE 单精度浮点和数学库（`sinf/cosf/fabsf/floor/round/isfinite`）。核查 SDK 的浮点 ABI、FPU 上下文保存、数学库链接和栈水位。控制核心不分配堆内存；`p85_runtime`、MsgBox serializer 应静态分配。库对同一实例不提供跨核心并发控制。

## 初始化和任务所有权

使用一个静态 `p85_runtime`，由唯一控制任务调用 `p85_runtime_step`。初始化先建立所有 port 回调，再创建 MsgBox 服务并启动生产者，最后运行 10 ms 周期任务。默认配置 ID3，增益尚未设置，启动状态 OFF 不发送 CAN。

```c
/* 结构示意；board_* 必须由匹配 BSP 实现，此段不能独立编译。 */
static p85_runtime motor;
p85_config cfg;
p85_port port = {
    .context = &board_context,
    .lock = board_lock_and_save_interrupt_state,
    .unlock = board_restore_interrupt_state,
    .can_send = p85_mcal_send,
    .reply = p85_bst_msgbox_reply
};
p85_default_config(&cfg, 3);
if (!p85_runtime_init(&motor, &cfg, &port, board_monotonic_ms())) {
    board_report_init_failure();
}
if (p85_bst_msgbox_open(&motor) != 0) {
    board_report_init_failure();
}
/* 周期任务：p85_runtime_step(&motor, board_monotonic_ms()); */
```

所有初始化失败都应阻止 P85 任务启动。关闭顺序：确认电机失能，停止 IPC/CAN 生产者和控制任务，然后 `p85_bst_msgbox_close`。尚有回调在运行时不能销毁句柄。

- `board_monotonic_ms` 来自 R0 单调计时器，单位 ms、uint32 回绕；不能使用 Linux/RTC 墙钟。
- 周期源使用绝对时刻调度，每 10 ms 一次。任务持续时间和抖动要测量；活动时超过 20 ms 的 tick 间隔会锁存保护。
- 同一任务处理 RX、IPC、轨迹和 CAN TX；回调只复制事件，不能直接调用状态机命令。
- 锁必须适用于该实例全部生产者上下文，保存并恢复先前中断状态；不能简单在退出时无条件开中断，不能只用 `volatile`。若存在其他核心访问还需正确的跨核同步，推荐限定为一个 R 核所有。
- 16 项有界队列满时保护锁存；一次 step 最多取 16 项，防止生产者持续到达导致饿死周期。过期 IPC（超过 20 ms）不执行、不续租。

## MsgBox 接收任务

`p85_bst_msgbox_open` 使用官方 stub API 创建 RT0/fid0/sid2/cmd0x85 服务并 mmap 官方会话。用 `p85_bst_msgbox_endpoint(&handle,&queue)` 在任务启动前获取此服务的句柄及厂商队列引用，接入**实际 R SDK**的接收/唤醒/节点释放流程。该访问器不转移队列生命周期或所有权。

接收任务的每条完整单包 `rw_msg_t` 调用：

```c
p85_bst_msgbox_ingest(&vendor_message, board_monotonic_ms());
```

它校验路由、METHOD、长度和分包头，然后只复制 32 字节与回复 cookie 到队列。控制任务通过 `p85_bst_msgbox_reply` 回复。回复 serializer 为静态对象，只能由控制任务使用。

必须核查实际 RTOS 侧接收 API、队列指针语义、节点确认/释放以及缓存一致性。Linux `ipc_trans_layer_get_rwmsg` 的 ioctl 实现不能直接用于 R0；A 核 mmap 的 KERNEL_2_USER 指针变换也不能直接套到 R 核。不要在 R 侧使用 `query_info(...,is_from_user=1)` 来推断会话请求。厂商路由若已使用 sid2，应同步更改两端并重测，而非占用既有服务。

最先上板的验收应停留在 OFF：A 侧 HELLO/STATUS/DIAG 返回正确版本/序号，`tx_ok=0`。只有这一关通过后才开始 CAN 失能探测。

## CAN 发送、接收和错误

`p85_mcal_tx.c` 需要两个显式宏，缺失会编译失败：

- `P85_CAN_TX_HTH`：由实际 Can/CanIf HOH 表确认的 CAN0 TX 硬件对象。旧文档中 can0_tx0/12 只能作为线索。
- `P85_CAN_TIMING_VERIFIED=1`：完成下面的实际配置核查后才可设置。这是人工接入断言，不是时序自动验证器。

确认 `Can_Write` 在返回 E_OK 前已复制 64 字节 payload。若端口异步保留调用者指针，应在板级适配层使用持久缓冲和 TX 完成管理，不能使用当前栈缓冲。BUSY/拒绝要返回 false；连续 3 次失败锁存。Bus-off/错误回调调用 `p85_runtime_bus_fault`，不能只写日志。

接收回调需提供**真实仲裁 ID**、FD、扩展帧标志、实际长度和接收时刻：

```c
p85_runtime_rx(&motor, actual_arbitration_id, is_fd, is_extended,
               bytes, actual_length, capture_ms);
```

特别注意：`CanFd_Demo_RxIndication(PduIdType CanId, ...)` 中名称叫 CanId 的值可能是 **RxPduId 逻辑索引**。先检查实际 `CanIf_RxIndication` 和生成配置，在元数据仍存在的入口复制到事件；不能把逻辑索引作为 ID3，不能凭猜测给每条帧硬编码 ID3。缺少 FD 标志的 CanIf 回调也需从驱动入口补传元数据。

过滤标准 ID3、12 字节为本阶段目标，但底层入口仍应保留真实标志。关闭旧 CAN 演示的周期 TX、shell 发帧以及其他对相同邮箱/总线的所有者。避免回调大量串口打印阻塞控制任务。

## CAN 时钟、波特率、FD 与 TDC

P85 要求 1 Mbps 仲裁 / 5 Mbps 数据 / FD+BRS。官方网页演示默认 500 kbps / 1 Mbps，不适用。检查实际工程中的：

```text
rt_hal/config/mcal_cfg/output/src/Can_PBcfg.c
rt_hal/config/mcal_cfg/output/src/FlexCAN_Ip_PBcfg.c
bsw/mcal/mcu/src/Mcu.c
Can/CanIf 的 HOH、控制器、过滤器、64 字节邮箱设置
载板的 pinmux、收发器供电、enable/standby 管脚
```

在两个配置层保持一致，确认实际 FLEXCAN 时钟。对照 **MCAL 结构定义及寄存器写入代码**确认 Prescaler/PropSeg/PhaseSeg/SJW 是否为 N−1 编码、CBT/ENCBT 是否切换，再计算采样点与速率。网页示例表与简化公式存在歧义，因此本仓库不写猜测的寄存器五元组。

配置 BRS、ISO FD 模式、64 字节邮箱、数据段 TDC 与收发器延迟。即使没有电机，也应使用 CAN FD 分析仪或第二个支持 1/5 Mbps 的节点验证帧格式、64 字节内容及 ACK/错误计数。单节点无 ACK 的结果不能证明通信成功。没有对端时不要持续重发造成 bus-off。

## 部署及验收顺序

1. 保留原固件，确认匹配 SDK 可重复构建原版、镜像大小和 Flash 槽位；验证恢复方式。
2. 构建新 R0 固件，保留 map/ELF/二进制哈希和 SDK/编译器版本。先验证 OFF 状态的 A/R 通路。
3. 验证 CAN 分析仪下 1/5 Mbps、标准 FD+BRS、64 字节内容和 RX 元数据；确保旧 demo 停止。
4. 接电机后只做失能探测，校验温度/电压、回传 MSGID、状态和单位，再开展低增益保持与小幅移动。
5. 记录周期抖动、队列峰值、TX 完成/错误、反馈新鲜度、主机断开保护和实际停止行为。

这里只给出部署顺序；官方烧录流程和当前板布局证据见 `../docs/R0_FLASHING.md`、`../docs/R0_SDK_FINDINGS.md`。只读备份及当前验证边界见 `../docs/VALIDATION.md`。
