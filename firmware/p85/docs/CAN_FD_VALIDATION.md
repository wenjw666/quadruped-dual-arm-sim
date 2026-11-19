# CAN FD 内部回环与物理链路验收

更新：2026-11-19：整板重新上电后，R0 physical-v6 与 R2 CAN1 release-v2 已在实板执行，CANFD0 ↔ CANFD1 双向物理通信通过，每方向 512 帧、共 1024 帧，仲裁 1 Mbps / 数据 5 Mbps、FD+BRS，16 种长度各 32 帧，错误与超时均为 0。CAN0 内部回环另通过 512 帧。R2 镜像匹配、CAN1 inactive、心跳推进，真实 MsgBox HELLO 加 25 轮 STATUS/DIAG 通过，保持 OFF/fault=0、电机发送计数为 0。诊断采集器已按预设恢复原 R0/R2 Flash 并通过整片校验；当前 SRAM 仍运行候选，下次完整上电加载已恢复的基线。真实电机尚未接入验收。 分支 `codex/p85-rt0-motor-control`。本次 boot 为 `3d8fbaa2-a575-4987-b452-19bb40c0ad6e`。见 [物理链路记录](PHYSICAL_CAN_VALIDATION.md) 与 [R2 配套构建](R2_CAN1_HANDOFF.md)。

## v3 冷启动硬件验收通过

本次 boot ID 为 `3b0710a2-ed60-4704-9308-db792345aef1`，与写入时不同；AON 记录 R0 为 `4/normal`、cold 启动、启动计数 1。板上墙钟不准确，记录日期采用工作站日期，启动身份采用 boot ID。

| 检查 | 实测结果 |
| --- | --- |
| SRAM 镜像身份 | v3 ELF 代码/只读数据 53,771 字节全部匹配，`image_matches=true` |
| 回环状态 | `state=2`、`failure=0`、`status=passed` |
| 帧计数 | requested/submitted/confirmed/received 均为 512 |
| 长度覆盖 | `0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64` 字节，各 32 帧 |
| 帧校验 | bad_metadata=0、bad_payload=0、unexpected=0 |
| 控制器错误 | faults=0、timeouts=0，报告中的 ESR1/ECR 均为 0 |
| 回环耗时 | 1,024 ms，属于该自检流程总耗时，不是总线吞吐率测量 |
| 模式和速率 | 内部回环启用；1M/5M、FD+BRS 配置检查通过；回环期间 TDCEN=0 |
| 配置恢复 | restored=1，正常模式恢复；TDC 和 FD 设置与测试前一致 |
| A/R 后续运行 | HELLO 加 25 轮 STATUS/DIAG 通过，R0 时间推进 5,280 ms，保持 OFF、CAN tx=0 |
| 部署接受 | 自动验收退出码 0，accepted=true，回退 timer 已停止并禁用 |

上述结果证明本轮 CAN0 控制器、MCAL 发送确认、真实接收中断、FD 长度与载荷处理，以及恢复正常模式后的 A/R 通信均通过。该历史验收时尚未连接外部 CAN 节点，未证明收发器、外部 ACK、线缆、电阻实装、波形或电机反馈正常。2026-11-19 两接口连接后的配套固件实测已通过，详见文首和物理链路记录。

原始证据已归档为 `hardware-loopback-v3-after-powercycle.json`（现场查询）、`hardware-loopback-v3-passed.json`（回环报告）和 `v3-coldboot-acceptance.json`（板端自动验收及接受记录），位于下文的本地证据目录。

## 首次硬件执行结果及修正（历史）

boot ID `db7c9e4d-03c8-4dfd-86cc-06806ccb6b8e` 中，SRAM 精确匹配 v2 ELF。实际报告为 state=3、failure=5、submitted=1、confirmed=0、received=0、faults=1、elapsed_ms=2、restored=1。即首帧触发控制器错误，随后恢复正常模式；未取得任何成功收发计数，不能标记 PASS。

检查发现内部回环仍启用了外部收发器发送延时补偿 TDC。v3 仅在 freeze 中清除 FDCTRL[TDCEN]，保留 FD+BRS、1M/5M、TDCOFF 和其余 FD 配置；STOP/START 后恢复正常模式完整配置，包括 TDCEN。验收读取器现在明确要求回环时 TDCEN=0，并拒绝仍启用 TDC 的报告。首次错误时将 ESR1/ECR 锁存到 loopback 数组对应字段，便于后续诊断。后续 v3 冷启动实测全部通过，说明该修正在本轮测试条件下有效；单次通过不能代表外部总线已经验证。

失败 v2 已恢复为此前验收的 P85 基线并整片校验，再从该基线部署 v3，未将失败候选作为新基线。原始报告为 `hardware-loopback-v2-failed.json`，恢复记录为 `v2-baseline-restored.json`，v3 软件回归为 `board-software-tests-v3.json`，部署为 `v3-staging-and-install.json`。

## 已完成的测试实现

使用固定 `C1200SW_V1.6_RC0930` SDK。`p85_can_loopback.c` 在唯一 CAN0 控制任务中、处理 A 核请求之前执行启动自检，构建参数为 `build_hanhai_sdk.py --boot-loopback`，默认构建不运行回环。

- 保持标准 ID、FD+BRS、1 Mbps 仲裁/5 Mbps 数据配置。独立测试 ID 为 `0x5A5`，不构造电机命令。
- 覆盖长度 `0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64`，每种 32 帧，共 512 帧。
- 通过 MCAL 发送和真实接收中断入口，检查硬件 CS 中的 FD/BRS/IDE/RTR、实际 ID 和长度、载荷；发送完成和接收完成均需发生。记录逐长度计数、超时、重复/意外回调和总线错误。
- CAN STARTED 后才进入 freeze 并启用 LOOPBACK/self reception；提前设置会被 STARTED 的生成配置覆盖。标准范围接收过滤器覆盖 `0x000..0x7FF`，包含测试 ID。
- 结束后通过 STOP/START 重新初始化正常模式、接收过滤和缓冲，验证 LPB/LOM 清除、SRXDIS 恢复及速率配置一致。FDCTRL 比较排除测量/状态字段 TDCVAL/TDCFAIL；回环中仅允许 TDCEN 清零，正常模式要求完整恢复。
- 测试失败时启动状态 `-7`，停止 CAN，不进入电机控制。成功后回到 OFF，重新进行 HELLO 和 25 轮 STATUS/DIAG 验收。

`inspect_can_loopback.py` 仅通过只读 `/dev/mem` 快照取报告；先核对当前 SRAM 的代码/只读数据与指定 ELF，再从 ELF 符号定位报告，拒绝旧镜像、未完成或计数错误的结果。对设备映射采用从对齐基址完整复制后解析，避免在符号偏移直接切片产生非对齐设备访问。

## 本次构建及部署证据

| 项目 | 已核实结果 |
| --- | --- |
| 完整 R0 构建 | 成功，SRAM 余量 399,120 字节 |
| 签名测试包 SHA256 | `e8f57478c91b2ab89c084898d572a16c74dfd07d6d5d61a16f7c29d9c96b22a4` |
| ELF SHA256 | `10a787b1e95f6827291aad90b9f5a31b5a85016c6e2d5ab052acba1f5b23aed3` |
| 报告对象 | ELF 中 `p85_can_loopback_report`，地址 `0x04097EB8`，220 字节 |
| 软件回归 | 目标板 CTest 7/7 通过，包括报告错误/未完成拒绝测试；属于软件验证 |
| 写入范围 | `/dev/mtdblock0`，offset `0xB00000`，长度 `0x100000` |
| 写后校验 | 完整 64 MiB 读回，RT0 槽位匹配，槽位外字节不变 |
| 写后整片 SHA256 | `235ebdfd03cb5be6dd19dd5eb986dcec521948ba105f1d97a911cbb8d1eb302e` |
| 本次写入 boot ID | `db7c9e4d-03c8-4dfd-86cc-06806ccb6b8e` |
| 硬件回环结果 | v3 冷启动实测 PASS，512 Tx 确认 / 512 Rx，零错误，配置已恢复 |

本地原始证据在 `output/p85_r_core_reading/can-loopback-20261117/`（Git 忽略）：`build-v3/build-report.json`、`board-software-tests-v3.json`、`v3-staging-and-install.json`、`hardware-loopback-v2-failed.json`、`deployment/flash-before.bin`。此前已接受的固件及原始厂商备份均保留。

板上事务目录为 `/var/lib/p85-rt0-deploy/20261117-loopback-02/`。本次 `p85-can-loopback-accept.service` 已完成新 boot、精确镜像、回环 PASS、AON normal、OFF 通信及 Flash 校验，保存 `accepted.json` 并移除 `armed.json`。回退 timer 已停止且禁用，v3 已接受保留；oneshot 服务成功结束后的 inactive/dead 为正常状态。

可只读复查本次启动报告：

```sh
sudo python3 /var/lib/p85-rt0-deploy/20261117-loopback-02/inspect_can_loopback.py \
  --elf /var/lib/p85-rt0-deploy/20261117-loopback-02/realtime_freertos_core0.elf
sudo systemctl status p85-can-loopback-accept.service
sudo journalctl -b -u p85-can-loopback-accept.service --no-pager
```

## 物理连接：RT_CANFD0 与 RT_CANFD1

依据厂家 `bst-sesamex/sesamex-hardware-docs` 仓库：

- `aura-nx-5000/schematics-and-pcb/QSC801XP-AP_EVB_SCH_V1.2_20260430.pdf`：PDF 第 9 页（连接器 J0801）及第 16 页（CAN FD 收发器、终端）。
- `aura-nx-5000/datasheets/Aura DevKit Carrier Board Specification_DX-5000-D_V1.2.5.pdf`：Expansion Header J0901/J0902/J0801 的 pin assignment 和 Pin1 方向图。

本地归档为上述证据目录中的 `schematic.pdf`、`carrier.pdf`；清晰截图为 `connector-can-crop.png`、`header-pin1.png`、`termination-rt0.png`、`termination-rt1.png`。本结论对应公开 V1.2 底板，接线前须核对实物底板版本/丝印和 Pin1，不用 BSP 软件版本推断硬件版本。

| 通道 | CANH | CANL |
| --- | --- | --- |
| RT_CANFD0 | J0801-9 | J0801-7 |
| RT_CANFD1 | J0801-13 | J0801-11 |

同板双通道测试：**9 ↔ 13，7 ↔ 11**。J0801 的 5/6/19 为 GND，可用于需要参考地的外部节点。这里连接的是收发器后的差分 CANH/CANL，不能连接单端 TXD/RXD。内部回环阶段无需这些跳线。

此前运行固件仅启用 CAN0。现已完成 CAN1 配置、独立双向收发入口和相同的 1M/5M 速率，新候选已写入 Flash，但仍需冷启动确认 R0 加载和物理收发。

## 板载 120Ω 终端电阻

公开原理图明确画有分裂终端：

| 通道 | 收发器 | 终端电阻 | 差分等效 |
| --- | --- | --- | --- |
| RT_CANFD0 | U1604，SIT1462QT/3 | R1624、R1625，各 60.4Ω，中点 C1625=4.7nF 接地 | 120.8Ω |
| RT_CANFD1 | U1608，SIT1462QT/3 | R1644、R1645，各 60.4Ω，中点 C1644=4.7nF 接地 | 120.8Ω |

图中这两组终端未标为 NC/DNP，未见终端开关跳帽。**原理图有终端不等同于已测量确认当前实物装配。**断电、与其他总线设备断开后测量各通道 H-L：单路预期约 120Ω；两路 H-H/L-L 相连后预期约 60Ω。若实测符合该装配，就无需额外并接 120Ω。勿在带电状态用电阻档测量。

## 验收边界

内部回环可验证控制器、缓冲/中断、元数据和载荷；不能证明收发器、板外线缆、外部 ACK 或波形质量。后续双通道正常模式收发才能验证物理链路，再连接真实 P85 核对协议反馈和运动。若硬件不能在 FD+BRS 下完成内部回环，应保留失败报告并转做物理测试，不能降为经典 CAN 后宣称通过目标配置。
