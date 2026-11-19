# P85 RT0 实板部署、复位诊断与回退记录

## 2026-11-19：双向物理链路及 OFF 应用通信通过

本次 boot `3d8fbaa2-a575-4987-b452-19bb40c0ad6e` 不同于 origin boot。R0/R2 ELF 内容匹配，R2 心跳由 2,153 推进至 2,181，CAN1 inactive，CAN2/3 START 成功；AON R0/R2 均为 cold、normal。CAN0 内部回环 512 帧通过；正常模式 CAN0→CAN1、CAN1→CAN0 各 512 帧全部提交、确认和接收，错误、超时、载荷差异均为 0，测试总耗时 2,048 ms。每个方向覆盖 16 种长度，每种 32 帧。

随后真实 MsgBox HELLO 和 25 轮 STATUS/DIAG 全部通过，R0 时钟推进 6,010 ms，全程 OFF、fault=0，电机 tx_ok/tx_failed/rx_ok/rx_rejected 均为 0。链路诊断帧计数与应用电机帧计数是不同统计。

collector 在 uptime 39.47 s 完成采集和恢复；完整 Flash 读回 SHA256 恢复为 `235ebdfd03cb5be6dd19dd5eb986dcec521948ba105f1d97a911cbb8d1eb302e`，64 MiB 读回文件下载后与本地基线再次逐字节一致。collector 与 rollback timer 均已 disabled/inactive，armed marker 已移除，`accepted=false` 是诊断事务的预期结果。当前运行的 SRAM 未被恢复操作替换，下次完整上电将加载原基线。

证据保存在 `output/p85_r_core_reading/physical-can-20261119/`，包括 `validation-summary.json`、原始诊断 JSON/SRAM、`off-runtime-passed.json`、`rollback-result.json` 和 `restored-flash-independent-check.json`。这次验证了外部 CAN FD 收发链路，尚未接入真实 P85，未验证电机反馈、使能或运动。详见 [物理链路记录](PHYSICAL_CAN_VALIDATION.md)。

## 2026-11-17 历史阶段：R0/R2 配套 CAN1 诊断已写入，等待整板上电

当时 boot `a6ebb93d-d849-4ab2-8ffb-d9e1a14c37b8` 已确认运行验收基线内部回环 v3，512 帧严格通过。之前 diagnostic-v6 在 boot `54a61001-fce1-4de4-b9f1-aec42a0ba508` 实际执行，发现 CAN1 在 R0 正确初始化后被改为 R2 默认配置。`start_enter aux=1` 为 STARTED，不是 STOPPED；上层重复 START 被拒绝，没有进入底层退出 freeze 路径。R2 默认确实初始化并启动 CAN1，争用判断有源码和寄存器证据，具体硬件覆盖来源仍待配套试验确认。

已构建关闭 R2 CAN1 初始化/启动/中断的配套镜像，与 R0 physical-v6 一同写入 `/var/lib/p85-rt0-deploy/20261117-can-handoff-01`。仅 R0 `0xB00000` 和 R2 `0xD00000` 各 1 MiB 变化，R1 及其余区域逐字节一致。完整 64 MiB 读回 SHA256 为 `d92aa47071b39e587eb7de875a2b21faa86f6c34e3fda963fd7d091552b00743`，本地独立比较通过。

下一次完整断电上电后，新 collector 将采集 R2 两次快照/心跳和 R0 全套报告，随后恢复两槽位至整片基线 `235ebdfd03cb5be6dd19dd5eb986dcec521948ba105f1d97a911cbb8d1eb302e`，不重启或自动接受候选。15 分钟回退 timer 已为下次启动启用，当前 inactive；collector 实测拒绝 origin boot，旧自动接受服务已禁用。Flash 更新未切换当前 SRAM。11/11 板上软件测试通过，物理收发仍未通过验收。详见 [物理链路记录](PHYSICAL_CAN_VALIDATION.md) 和 [R2 配套构建与恢复](R2_CAN1_HANDOFF.md)。

以下保留此前各阶段原始结果，请按对应 boot ID 解读。

工作站日期 2026-11-15 至 2026-11-17（Asia/Tokyo），分支 `codex/p85-rt0-motor-control`。板上墙钟不准确，以下用 boot ID 和 uptime 区分前后。操作依据是用户明确要求烧写并检查运行；附件中的命令仅作为接口资料。

## 历史验收：v3 冷启动、CAN0 内部回环与 OFF 通过

用户整板重新上电后，本次 boot ID 为 `3b0710a2-ed60-4704-9308-db792345aef1`。SRAM 中 v3 ELF 的代码/只读数据 53,771 字节全部匹配；R0 AON 为 `4/normal`，cold 启动、启动计数 1。

CAN0 在 1 Mbps 仲裁 / 5 Mbps 数据、FD+BRS 配置下完成控制器内部回环：requested/submitted/confirmed/received 均为 512，16 种合法 FD 长度各 32 帧，faults、timeouts、bad_metadata、bad_payload、unexpected 均为 0，耗时 1,024 ms。恢复正常模式与 TDC/FD 设置检查全部通过。随后真实 MsgBox HELLO 和 25 轮 STATUS/DIAG 通过，R0 时钟推进 5,280 ms，保持 OFF，CAN tx=0。

`p85-can-loopback-accept.service` 已成功退出，`accepted=true`、`inputs_pinned=true`，镜像、回环、AON、OFF 和 Flash 接受检查退出码全部为 0。guard 保存本 boot 的 `accepted.json`，移除 `armed.json`，回退 timer 已停止并禁用。当前固件已接受保留，不需要再次烧写或重启来确认本轮结果。

事务为 `/var/lib/p85-rt0-deploy/20261117-loopback-02/`。签名包 SHA256 为 `e8f57478c91b2ab89c084898d572a16c74dfd07d6d5d61a16f7c29d9c96b22a4`，ELF 为 `10a787b1e95f6827291aad90b9f5a31b5a85016c6e2d5ab052acba1f5b23aed3`，整片为 `235ebdfd03cb5be6dd19dd5eb986dcec521948ba105f1d97a911cbb8d1eb302e`。原始自动验收及现场只读报告在 `output/p85_r_core_reading/can-loopback-20261117/v3-coldboot-acceptance.json`、`hardware-loopback-v3-passed.json`、`hardware-loopback-v3-after-powercycle.json`。

前一 boot `db7c9e4d-03c8-4dfd-86cc-06806ccb6b8e` 的 v2 首帧错误已被自动验收拒绝；恢复此前通过的完整基线后才部署 v3。v3 在内部回环期间关闭 TDCEN，正常模式恢复原 TDC 配置；本轮实测已通过。电机和外部节点未连接，物理收发器、外部 ACK、接线与电机反馈仍未验收。接线、终端电阻和完整检查表见 [CAN FD 验收](CAN_FD_VALIDATION.md)。

## 2026-11-17 前一阶段：冷启动及 R0 OFF 通信验收通过

用户再次整板断电上电后，工作站 10:48:17（Asia/Tokyo）捕获 SBL 冷启动输出；10:50:34 通过 SSH 归档板上自动验收结果。boot ID 为 `3b54e8d9-bfa4-40da-baa8-1d3e870abced`。板上墙钟有误，以下使用工作站日期、boot ID 和 uptime。

| 检查 | 本次结果 |
| --- | --- |
| R0 实际镜像 | SRAM 中候选 ELF 的代码/只读数据 51,307 字节全部匹配，`image_matches=true` |
| AON | realtime0 为 `4/normal`，启动原因为 cold，启动计数 1 |
| A/R 应用通信 | 真实 MsgBox HELLO 加 25 轮 STATUS/DIAG 全部通过 |
| R0 时间推进 | 25 轮观测中推进 5,280 ms |
| 请求耗时 | STATUS/DIAG 一对请求的最大往返时间 19.9221 ms；不是 CAN 时延或控制周期测量 |
| 应用状态 | OFF、fault=0；CAN tx_ok=0、tx_failed=0、rx_ok=0 |
| 自动验收服务 | `p85-rt0-accept.service` Result=success、ExecMainStatus=0；oneshot 执行结束后 inactive/dead 是正常结果 |
| Flash 接受 | guard 再读完整 64 MiB，验证候选 RT0 槽位及槽位外字节不变后保存 `accepted.json` |
| 回退保护 | `armed.json` 已移除，`p85-rt0-rollback.timer` disabled |

自动验收在该 boot 的 uptime 13.94 s 开始，21.39 s 结束，`accepted=true`、`inputs_pinned=true`。接受的 RT0 签名包 SHA256 为 `57c2d22f38a82cf2e2526ef28825f197ee6d6880e34fa74ea411a7341c624c07`，ELF 为 `6c43b128ece5b4d2b66e6ea784b3bfc8194a29274ff2164fc86659c9639d13ae`。连续请求同时验证了 A 侧在线状态单次查询缓存修复。

候选固件现已保留，无需为了确认本次运行再次重启或烧写。四路只读串口监听继续运行。以上证明 R0 固件加载和 A/R 应用通信已经打通；当前未接电机，未验证 CAN 波形、外部 ACK、电机反馈或运动。CAN FD 回环与物理链路的下一阶段范围见 [CAN FD 验证范围](CAN_FD_VALIDATION.md)。

完整证据：`output/p85_r_core_reading/deployment-20261117/monitor-20261117-1026/second-coldboot-check.json`（Git 忽略），其中包含板上 `boot-acceptance-3b54e8d9-bfa4-40da-baa8-1d3e870abced.json`、`accepted.json`、service/timer 状态和 journal；同目录保留 `combined.log`、`events.log` 和监听心跳 `status.json`。

## 2026-11-17 10:29 至 10:43 重新部署过程（历史记录）

本次用户断电上电已被四路串口监听捕获。工作站 10:29:31 网络断开，10:29:44 SBL 输出 `release realtime0 done` 与 `run soc succ`，10:30:20 SSH 端口恢复；Linux 控制台完成启动。boot ID 为 `d76181bf-ef55-4cb8-8e6c-6102c54b7302`。板上日期不准，上述时间均为工作站 Asia/Tokyo。

本次启动后的实际核对结果：

- R0 AON 状态为 `0/default`，cold 启动，计数 1；候选 ELF 的 51,307 字节代码/只读数据与 SRAM 不匹配。
- 直接读回整片 Flash，SHA256 为 `81f90bfb533be3b374e8a55787d525cf64066f5b8982b8138bc29a58cafd3417`，与原始备份完全一致。
- 先前启动的 rollback service 日志明确记录 `original-restored`，当前启动中该服务无执行记录，检查时 timer disabled、armed 标记不存在。因此，本次冷启动加载的是已恢复的原 Flash，不能把它当作候选固件启动失败。

10:39:55 已通过原有固定偏移、大小和哈希校验的 guard 重新写入候选固件，未发起软件重启：

| 检查 | 结果 |
| --- | --- |
| 写入范围 | `/dev/mtdblock0`，`0xB00000` 起始的 1 MiB RT0 槽位 |
| RT0 回读 SHA256 | `57c2d22f38a82cf2e2526ef28825f197ee6d6880e34fa74ea411a7341c624c07` |
| 全片回读 SHA256 | `cf7b524e9749445bac4e46c0f63194b33e6fb90a6ddb27d6e9b8b06ab604ed68` |
| 槽位之外 | 与原始 64 MiB 备份逐字节相同 |
| 当前正在执行的程序 | 本轮 boot 未变；写 Flash 不会替换正在运行的 SRAM 程序 |
| 自动验收 | `p85-rt0-accept.service` enabled / inactive，下一次 Linux 启动执行 |
| 回退保护 | timer enabled / inactive，下一次 Linux 启动 15 分钟后触发，验收成功则取消 |

### 当时配置的下一次启动自动验收（现已通过）

新增板上脚本 `/var/lib/p85-rt0-deploy/20261115-01/accept_after_coldboot.py`，由 root 持有；systemd 单元为 `/etc/systemd/system/p85-rt0-accept.service`。脚本固定核对 ELF、检查脚本、主机 Python 和动态库的 SHA256，并依次检查：

1. boot ID 已改变，armed 标记中的候选哈希正确。
2. `inspect_r0_image.py` 确认当前 boot 的候选代码和只读数据匹配，ELF SHA256 为 `6c43b128ece5b4d2b66e6ea784b3bfc8194a29274ff2164fc86659c9639d13ae`。
3. AON 的 R0 状态为 `4/normal`。
4. 真实 MsgBox 执行 HELLO 和 25 轮 STATUS/DIAG，保持 OFF、fault=0、设备时间推进、CAN TX/RX 成功及 TX 失败计数均为 0。
5. 全部通过后才调用原 guard 的 `accept`；guard 再读全片、核对槽位及外部字节，取消回退并保存 accepted 记录。任一前置检查失败均保留保护。

已验证两个拒绝路径：`--check-only` 在原 SRAM 程序上因镜像不匹配退出 1；重新写 Flash 后在同一 boot 调用验收也退出 1，提示 `Refusing acceptance before a new boot`，没有误接受。systemd 单元语法检查通过。当时成功路径尚待冷启动；已在上述 10:48 冷启动中通过。

A 侧 `host/p85_msgbox.c` 已改为在打开句柄时查询一次在线状态，随后使用缓存；板上 `.so` 已删除旧产物后重新构建，SHA256 为 `7e8bfb289199d0cd85f524c2204e49da1c52ec832149f4d8451ef21770baf8cc`。运行时连续请求已由最新冷启动中的 25 轮 OFF 验收确认。

四路串口监听继续运行：COM10/SBL、COM7/Safety、COM8/A、COM9/Linux，115200 8N1，只读且 DTR/RTS 关闭。**这是再次冷启动前的历史状态；上述 10:48 冷启动已完成验收。** 未连接电机，未执行任何电机控制命令，也未验证 CAN 波形、ACK 或真实反馈。

本轮证据位于 `output/p85_r_core_reading/deployment-20261117/monitor-20261117-1026/`（Git 忽略）：`combined.log`、`post-coldboot-check.json`、`guard-inspection.json`、`flash-coldboot-hash.json`、`host-runtime.json`、`boot-validator-negative-check.json`、`candidate-reinstall.json`、`ready-for-next-coldboot.json`，以及自动验收脚本和 systemd 单元副本。板上每次验收输出为 `boot-acceptance-<boot_id>.json`，另保留 guard 的 `accepted.json`。

## 2026-11-15 首次试烧结果（历史记录）

**RT0 槽位写入、回读和 Linux 重启成功，但新 P85 固件的加载及运行未通过。已恢复原 Flash。**

| 检查 | 实测结果 |
| --- | --- |
| SSH / sudo | `bst@192.168.1.3` 可用，`sudo -n` 可用 |
| 更新范围 | `/dev/mtdblock0`，`0xB00000` 起始，仅 1 MiB RT0 槽位 |
| 新镜像写入及回读 | 成功，SHA256 与最终构建一致 |
| 槽位以外内容 | 全片 64 MiB 对比，全部保持原值 |
| Linux 软件重启 | boot ID 改变，约 46 秒后 SSH 恢复 |
| 重启后 RT0 Flash | 仍与新签名镜像一致 |
| 通用 IPC 查询 | `query=2`，仅说明驱动报告 R0 在线 |
| P85 HELLO | 超时 `-110`，未进入后续 STATUS/DIAG 采样 |
| R0 SRAM 代码 | 与原固件一致，未匹配新 P85 ELF |
| 最终 Flash | 已恢复；整片 64 MiB SHA256 与烧写前备份完全相同 |
| 回退机制 | `armed.json` 已清除，定时器 inactive / disabled |
| 电机/CAN | 未接电机，未调用 probe/enable/hold/move/sine 等电机命令 |

未把此结果标记为“P85 已正常运行”，也未将新镜像在 SRAM 不存在时的 HELLO 超时归因于 P85 初始化、CAN 时序门控或接收任务缺陷。

## 2026-11-16 Safety 复位路径测试

阶段结果：**固件写入和恢复已验证，软件测试通过；新 P85 程序仍未进入 R0 加载区，运行验收未通过。** 本轮没有执行 Linux 重启；Linux boot ID 一直为 `aa7d3a74-85bb-49ae-acba-0080bd76bb2f`。

1. 从官方公开 `sf-app` 的 `C1200SW_V1.6_RC0930` 源码核对 Safety 公共服务：Safety0 PID `0x80`、FID 0、SID 3；方法 120 查询 AON 状态/启动信息，方法 42 请求核复位。序列化使用板上公开 MsgBox 头文件和库。
2. 在原固件下测试 `realtime0` 核复位请求。返回错误值为 0，但启动记录没有变化。此后通过有固定槽位/哈希约束的 guard 再写入同一 P85 签名包，回读验证通过；发送相同复位请求后等待 3 秒，再逐项比较 P85 ELF 与 SRAM。
3. 新 ELF 的 44 个代码/只读区间全部不匹配，镜像检查退出码为 1。脚本在此停止运行验收，因此本轮 **25 轮 P85 OFF/STATUS/DIAG 验收没有执行**。
4. 自动恢复原 RT0 槽位，再读取全片 64 MiB。整片 SHA256 恢复为 `81f90bfb533be3b374e8a55787d525cf64066f5b8982b8138bc29a58cafd3417`。`armed.json` 不存在，回退 timer 为 disabled；SSH 和 sudo 正常。试验服务退出码 0 代表清理流程完成，其 `result.json` 明确记录 `passed: false, original_restored: true`，不能当作 P85 验收成功。
5. 对已恢复的原固件，额外测试启动 DTB 中的系统节点名 `sys_rt0`。仍收到成功回复，等待后 `realtime0` 启动计数没有增加；没有据此再进行相同的 P85 试烧。

公开源码 `sys_onoff_request_handle()` 即使没有匹配到目标分区名，也会无条件回复 `ST_PUBLIC_NO_ERROR`。其 R0 状态机在复位分支应经过核复位、QSPI 拷贝和释放核的回调。因此，**方法回复成功不等于核复位、QSPI 重载或新程序运行成功**。板上 BSP 为 V2.0.1，不能把 V1.6 源码的路径直接当作当前 Safety 固件的执行证据，也不能确定失败原因是目标名、状态机、加载拒收还是版本行为差异。

已复核完整启动 DTB：`/sys_boot/sys_rt0` 的 `sys_img` 为 phandle 5，指向 `/flash_boot_part/realtime0`；该分区偏移 `0xB00000`、大小 1 MiB，加载 SRAM 为 `0x04000000`。这提供槽位及名称的配置依据，但不能证明运行中的 Safety 已按此路径复位。

### AON 只读诊断与交叉检查

新增 `tools/inspect_aon_state.py`，只读映射固定 `0x06000000` 起始的 32 KiB AON SRAM。依据公开结构检查 magic `0xCAFEFEED`、version 0、记录标记、名称和重复域；不接受任意物理地址。可通过 `--snapshot` 离线读取同样大小的快照。退出码 0 表示读取/解析成功，2 表示读取失败或布局/数据不符合预期；**不以退出码 0 表示 R0 正常运行**。

```sh
sudo -n python3 /home/bst/p85_deploy/20261115-01/inspect_aon_state.py
```

对当前板连续执行 25 轮 Safety IPC 查询，并与直接 SRAM 解析交叉比对，共 50 次状态/启动信息比对全部一致：

| 字段 | 实测值 | 含义 |
| --- | --- | --- |
| R0 AON 状态 | 0 / `default` | 默认记录值，不能标记为 `normal`（4） |
| R0 启动原因 | 1 / `cold` | Safety 记录中的启动原因 |
| R0 启动计数 | 1，采样期间不变 | 没有观察到本轮请求导致计数增加；字段为 8 位计数 |
| Safety 本身的 AON 状态 | 4 / `normal` | 说明读取的状态值可区分正常和默认 |

AON 是启动记账信息，可能陈旧；单独状态 0 或计数不变不能证明核停止执行。这里与 SRAM 镜像不匹配共同用于判定“尚未验证 P85 加载/运行”。这 25 轮是 **Safety/AON 一致性检查**，不是 P85 OFF 运行验收，也不是 CAN 或电机测试。

新增解析测试覆盖正常/默认/错误/未知值、计数边界、长度错误、布局错误、记录标记损坏、非法名称、重复域、缺少 RT0 和稀疏记录。目标板软件 CTest 现为 6/6，通过结果包括原有五组及新增 AON 测试。在独立的新构建目录中重新配置并完成全部 C 目标的编译和链接，CTest 用时 8.37 秒。板上墙钟与源码时间戳不一致，GNU make 报告 clock skew 警告；构建日志保留该警告，没有修改板上时钟。所有软件模拟组仍不能替代 R0 调度或 CAN 硬件验收。

## 写入与恢复的校验值

| 对象 | 字节 | SHA256 |
| --- | ---: | --- |
| 新鲜整片备份 / 最终恢复结果 | 67108864 | `81f90bfb533be3b374e8a55787d525cf64066f5b8982b8138bc29a58cafd3417` |
| 原 RT0 槽位 | 1048576 | `2378169de79bf9d5c4f62ed9ce3dd6740d6d21717ba7c00b6ba401ae49c613ac` |
| 试烧的 P85 签名包 | 1048576 | `57c2d22f38a82cf2e2526ef28825f197ee6d6880e34fa74ea411a7341c624c07` |
| 新镜像写入后的整片 Flash | 67108864 | `cf7b524e9749445bac4e46c0f63194b33e6fb90a6ddb27d6e9b8b06ab604ed68` |

操作使用官方指南中的 `mtd_debug erase/write/read` 原语，以固定路径、槽位和已知哈希的部署脚本约束输入，并在写入前后比较全片。没有运行附件的原始刷写脚本，没有修改其他核、启动程序、设备树或板上时钟。

Linux boot ID：

- 试烧前：`ceec793e-98ba-41ba-9fc9-5a11e9fb2f08`。
- 试烧后：`aa7d3a74-85bb-49ae-acba-0080bd76bb2f`。

## 为什么确认 R0 仍载有原代码

从 SBL 布局和 ELF 已核实的 RT0 SRAM `0x04000000` 只读映射 1 MiB。直接 `pread` 曾返回 Bad address，但 `mmap(PROT_READ)` 可以读取；没有向该内存写入任何内容。

在试烧后的 SRAM 快照中：

- 偏移 `[0x10000, 0x40000)` 的连续 196608 字节与原槽位镜像加载代码逐字节一致，与新镜像不一致。
- 原镜像可见的 `Aug 13 2026 17:38:28` 版本/启动字符串仍在；没有新镜像的 `P85 R0 startup=` 字符串。
- 原镜像 441752 字节加载内容中有 440760 字节一致；不一致部分包含启动后可变数据，不能用整段 SRAM 哈希判断执行代码。
- 新增只读工具逐个比较新 ELF 的 P85 函数及只读数据，共 44 个区间、51307 字节，全部不匹配。离线快照与板上实时读取的结果一致。

这些证据说明本次 Linux 软件重启后，R0 加载区仍是原固件。**尚未区分复位范围、启动链拒收/加载路径或其他原因；不能断言整板冷启动一定能解决，也不能据此认定新 SDK 不兼容。**

## 只读镜像识别工具

`tools/inspect_r0_image.py` 只使用标准 Python，解析 ELF32 ARM 符号/只读段，检查所有读取范围位于固定 RT0 SRAM 中，再以只读方式访问 `/dev/mem`。它不改 Flash、寄存器、复位状态或电机状态。

已在目标板部署工具和对应 ELF，以下命令可再次检查：

```sh
sudo -n python3 /home/bst/p85_deploy/20261115-01/inspect_r0_image.py \
  --elf /home/bst/p85_deploy/20261115-01/realtime_freertos_core0.elf
```

退出码：0 表示代码/只读数据均匹配，1 表示镜像不匹配，2 表示输入或读取失败。**匹配也仅证明镜像存在，不能替代 HELLO、时钟推进和 OFF 状态验收。** 当前板预期返回 1，因为已经恢复原镜像。

离线使用 `--snapshot` 指定完整 1 MiB SRAM 快照。工具已检查完整参考内容匹配、单条代码损坏被拒绝、截断/错误 ELF 被拒绝，以及实际旧固件不匹配等情况；板上实时读取已执行。

## 下一次加载与运行验收

1. 当前已回退，先重新核对并写入上述最终签名包；单纯现在重启或断电不会启动 P85。保留原备份和恢复脚本。
2. 现场进行整板断电再上电，验证启动链是否重新加载 RT0；该物理动作无法仅通过目前的 SSH 连接完成。已测试的 Safety 定向复位请求没有使新镜像进入 SRAM，当前仍未验证可通过 SSH 执行的整板冷复位入口。
3. 建议同步接入官方 Type-C debug 口，通过 PC 上的 `logroutetool` 观察 R0 启动；也可按官方 UART 指南使用 RT0 硬件串口。当前 PC 未发现已接入的此类调试串口。R0 控制台不等于 A 核 Linux 的 `/dev/ttyS0`。
4. 冷启动后先运行只读镜像识别工具，再检查 `P85 R0 startup=0` / `p85stat`。如果镜像仍不匹配，继续检查启动选择和签名日志；若匹配但服务不响应，再定位 P85 初始化与 IPC。
5. 镜像匹配后通过真实 MsgBox 完成 HELLO/STATUS/DIAG，连续检查时钟推进、OFF、fault=0、CAN 发送计数为 0。`validate_off.py` 已准备好，但本次因第一条 HELLO 超时，25 轮采样未执行。
6. 软件运行验收后再做 CAN FD 第二节点/分析仪验证和电机调试。无电机时不能验收 ACK、反馈、转向、供电、TDC 或真实位置跟踪。

当时使用 Linux 启动 15 分钟后自动回退，只能覆盖 Linux 能启动的情况；不能替代无法启动时的外部恢复路径。该历史轮次结束时 timer/service 已禁用并清除标记；2026-11-17 的最新部署已通过验收并取消回退，见本文首节。

## 证据位置

本地（Git 忽略）：

```text
D:/20261022ubunturos_win/output/p85_r_core_reading/deployment-20261115/
  flash-before.bin / rt0-before.bin
  preflight.txt / install.txt / restore.txt
  rt0-after-reboot.bin / rt0-sram-trial.bin
  running-image-evidence.json
  running-p85-image-offline.json / running-p85-image-live.json
  off-validation.txt / final-board-state.txt
  official-r-core-guide.html / .txt / official-r-core-uart.txt
```

板上原备份及恢复脚本：`/var/lib/p85-rt0-deploy/20261115-01/`（root 所有）；工具、ELF 及无电机验收脚本：`/home/bst/p85_deploy/20261115-01/`。不在版本库中保存 SSH 口令、厂商签名材料或整片镜像。

官方资料留档来自 v2.0.x 的 R 核开发和 R 核 UART 调试页面。签名、布局和固件构建说明见 [写入流程](R0_FLASHING.md)、[完整构建](R0_BUILD.md) 和 [板上验证](VALIDATION.md)。

2026-11-16 本地补充证据（Git 忽略）：

```text
D:/20261022ubunturos_win/output/p85_r_core_reading/deployment-20261116/
  trial-20261116/result.json / install.txt / image-new.txt / restore.txt
  trial-final-state.txt
  aon-sram-state.json / aon-snapshot-20261116.bin
  aon-crosscheck-20261116.json
  sf-name-root-before.txt / sf-name-root-alternate-reset.txt / sf-name-root-after.txt
  boot-fdt-full.json
  linux-validation-fresh-6tests.txt
  sf-app/sf_appl/user/pmic_app/sys_pm.c
  sf-app/sf_appl/user/pwr_app/aon_sram/aon_sram_ops.c / .h
```
