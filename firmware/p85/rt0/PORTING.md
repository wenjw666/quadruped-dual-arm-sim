# P85 在 Hanhai C1200 R0 的接入

2026-11-16：已完成公开 Hanhai V1.6 R0 工程的板级接入、完整链接和签名包生成。实现位于 `hanhai/`，由 `../tools/prepare_hanhai_sdk.py` 应用到隔离 SDK 副本。当前板的 BSP 为 `V2.0.1_20260813-185450_276175`；已试烧并重启 Linux，后续 Safety 核复位请求也未观察到新镜像加载；R0 SRAM 仍是原镜像，P85 服务未通过握手；当前已恢复原 Flash。下一步是重新部署后的整板冷启动和日志核对，版本互通、启动校验、实际调度和 CAN 电气时序仍待验收。详见 [实板部署记录](../docs/R0_DEPLOYMENT.md)。

完整构建命令和产物见 [R0 构建记录](../docs/R0_BUILD.md)。Linux CMake 仅用于软件验证，不生成 R0 固件。

## 固定底座和构建单位

使用 Gitee `rt-manifest` 的 `C1200SW_V1.6_RC0930` / `realtime_freertos_core0.xml`。`hanhai/sdk-lock.json` 记录全部 19 个项目的实际 commit，以及 8 个被修改 SDK 文件的 LF 归一化 SHA256。接入脚本检查各项目 HEAD 和这些源文件哈希，遇到其他版本即停止，避免将补丁静默套到不同 SDK。HEAD 检查不等同于验证所有工作区文件未被改动，应保留原始检出作为输入。

SDK 工程实际使用 Cortex-R5、ARM 指令集、hard-float、VFPv3-D16 和 small enum ABI，P85 沿用相同编译配置并链接 libm。不要将早期 soft-float 探测归档混入固件。

```text
src/p85_protocol.c
src/p85_control.c
src/p85_wire.c
rt0/p85_runtime.c
rt0/p85_bst_msgbox.c
rt0/p85_mcal_tx.c
rt0/hanhai/p85_board.c
rt0/hanhai/p85_hanhai_msgbox.c
```

接入启用 SDK `CLEAN_MODE=1`，关闭网络/时间同步演示，移除 CAN/VCan 应用任务和 CAN 测试 shell 入口。精简模式下排除仍引用完整网络应用的 GMAC/VLAN 测试源文件。CAN MCAL、底层虚拟 CAN 支持和必要 IPC 服务仍保留；这不是删除所有原厂系统任务的最小固件。

## 初始化、周期任务和所有权

SDK `main` 初始化 CAN/IPC 后，`BswInit.c` 的 `StartupTask` 调用 `p85_board_create_tasks()`。静态控制任务 `P85_CTRL` 优先级 5、栈 1536 words，调用 `portTASK_USES_FLOATING_POINT()` 保存浮点上下文；接收任务 `P85_IPC` 优先级 4、栈 1024 words。这些栈大小尚须通过实际栈水位验收。

控制任务先初始化单电机 ID3、OFF 状态、未设置增益的 runtime，检查 CAN 寄存器，建立 MsgBox 服务、启动 CAN0 并创建接收任务。失败时保持关闭；CAN 启动或接收任务创建失败会销毁服务，后者还请求停止 CAN0。成功后使用 `vTaskDelayUntil` 每 10 ms 运行一次。OFF 不主动发送 CAN。

`p85stat` 是只读 R0 shell 命令，输出初始化结果、状态/故障、TX/RX/拒绝计数、IPC 错误、过期命令、回复失败和两任务栈水位（单位 words）：

| init | 含义 |
| --- | --- |
| -1 | 任务尚未初始化 |
| -2 | runtime 初始化失败 |
| -3 | CAN 寄存器配置与接入值不符 |
| -4 | MsgBox 服务建立失败 |
| -5 | 接收任务创建失败 |
| -6 | CAN0 启动请求失败 |
| 0 | 软件初始化完成；不代表 CAN 对端已连通 |

唯一控制任务拥有状态机、轨迹、TX 和回复 serializer；CAN IRQ/IPC 任务只复制事件。临界区保存 CPSR 的 IRQ 屏蔽位，屏蔽 IRQ 并执行 DMB，退出恢复原 IRQ 状态，适用于任务与已屏蔽中断的 ISR；仅适用于此 R0 实例。CAN0 中断配置优先级 150，FreeRTOS API 阈值为 `17 << 3 = 136`，满足该端口 FromISR 调用的静态要求。

运行时队列容量 16，一次 step 最多处理 16 项；溢出锁存保护。IPC 排队超过 20 ms 不执行、不续租；活动时 tick 间隔超过 20 ms 会保护。周期抖动和满载行为仍需实测。

## MsgBox 接收及节点回收

服务为 RT0 `0x90` / CPU0 `0x10` / fid0 / sid2 / cmd `0x85`。打开流程使用 stub create/register/mmap；发送端和接收端都校验固定 32 字节业务帧。目标系统若占用这一路由，必须同步更改 A/R 两端后重新测试。

`p85_hanhai_receive_once` 使用实际 RTOS `ipc_trans_layer_query_info(0x90, handle, 0, 0)` 等待，遍历 `completed_pack_list`。仅接受单节点完整报文，并调用 `p85_bst_msgbox_ingest` 复制内容和回复 token/version/security 位。多节点分片和空节点跳过。正返回值表示端点状态变化，不将旧列表当作新命令；负值交给接收任务记录并通知保护。

关键生命周期：SDK 下一次 query 会先回收上一批 completed 节点，然后整理新消息。接收方不手动释放、不修改队列，不与 `get_rwmsg` 快速路径混用。SDK 自己提供的 `KERNEL_2_USER` 宏在 RTOS 是恒等变换，POSIX 才做地址偏移；沿用宏而不自行套用 Linux mmap 指针算法。

新增测试使用官方 Linux 头文件与假传输，在下一次 query 毒化原报文存储后再执行 runtime，验证命令副本、回复序号/token 和 OFF 零发送；还覆盖多个 packet、分片、空报文及错误返回。这不替代新固件实际 A/R 握手。

## CAN0 发送、反馈及故障

HTH 为 `CanHardwareObject_can0_tx0 = 12`，邮箱 64 字节、TX 中断模式。`P85_CAN_CONFIG_REVIEWED=1` 只声明配置已审阅，不能命名或解释成电气时序验证完成。P85 使用 `swPduHandle=0xffff`，在 SDK 的单条 CanIf TX demo 表外，避免发送完成被分派给时间同步测试。

已检查 `Can_Ipw_Write → FlexCAN_Ip_Send → FlexCAN_SetTxMsgBuff` 在返回前同步复制 payload，因此允许传入当前 step 的栈帧。`Can_Write == E_OK` 仅表示控制器接受请求，不证明发送完成、对端 ACK 或电机执行。DMA/其他 SDK 变体必须重新核查这一生命周期。

接收在 `CanIf_RxIndication` 入口拦截 controller0，保留原始 `CanId` 的 IDE(bit31)、FD(bit30)、实际仲裁 ID 和长度，避免旧回调丢失元数据或将逻辑 RxPduId 当 ID。软件仅接纳标准 FD、ID3、12 字节反馈；硬件过滤器仍为原厂宽范围，繁忙总线负载仍待评估。RX 时刻取 ISR tick，单位 ms。

每周期调用 CAN bus-off/mode 主函数；bus-off、错误被动、控制器错误读取失败、增强 FIFO 溢出均通知 runtime 故障。原厂 CAN0 bus-off 配置为轮询，TX/RX 为中断，无需为此 HTH 添加 TX 轮询。

## 1/5 Mbps 配置依据

MCU 初始化将 CAN0 时钟分频写为 400 MHz / 5 = 80 MHz。`Can_PBcfg.c` 与 `FlexCAN_Ip_PBcfg.c` 两层配置同步修改，使用 CBT/FDCBT，禁用增强 ENCBT 模式。

| 段 | 原始字段 propSeg / phase1 / phase2 / prescaler / rjw | 时序计算 | 目标 |
| --- | --- | --- | --- |
| 仲裁 CBT | 38 / 19 / 19 / 0 / 19 | 1+39+20+20=80 tq | 1 Mbps、75% 采样 |
| 数据 FDCBT | 7 / 3 / 3 / 0 / 3 | 1+7+4+4=16 tq | 5 Mbps、75% 采样 |

FDCBT 的 FPROPSEG 是直接计数，其他相位字段按 N−1；不能统一加一。FD+BRS 开启，TDC offset 暂为 12 个时钟，是初值，尚未测量实际收发器传播延迟。启动检查分频字段、CBT/FDCBT、模式和 FD/BRS/TDC 回读；它不测量实际时钟、引脚波形或物理延迟。

## 上板顺序

1. 核对版本/启动签名和恢复入口；已有 Flash 只读备份与 SBL 布局证据见 `../docs/R0_FLASHING.md`。
2. 部署后先检查 RT0 启动日志/`p85stat`，再运行 A 侧 HELLO/STATUS/DIAG；保持 OFF 且 `tx_ok=0`。
3. 用 CAN FD 分析仪或第二节点核对 1/5 Mbps、FD+BRS、64 字节命令、12 字节反馈及 ACK；单节点无 ACK 不能证明链路通过。
4. 接电机只做失能探测，再低增益保持、小幅移动和正弦。测栈水位、调度、反馈/主机失联保护与实际停止行为。

完整编译和签名包生成已完成，局部 Flash 写入/回读及 Linux 重启已实测；新 R0 镜像加载、握手与其余实机步骤仍未通过。
