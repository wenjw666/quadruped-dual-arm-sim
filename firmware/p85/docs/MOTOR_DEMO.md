# P85 实际电机控制与 R0 顺序演示

2026-11-20（Asia/Tokyo），分支 `codex/p85-rt0-motor-control`。本次在已验证的 CAN FD 基础上增加 R0 自主轨迹序列、阶段遥测和反馈稳定判定，并完成完整固件构建、R0 Flash 写入及整板冷启动验收。**当前新镜像已确认运行，控制器保持 OFF、fault=0；真实 P85 反馈、使能及运动尚未验证。**

## 对照说明书核查的协议

依据用户提供的《P85MAX-S关节电机协议手册-v1.0.pdf》：PDF 第 4–5 页为模式、总线和帧结构，第 6–10 页为指令，第 10–11 页为反馈和故障位。原始资料只用作技术依据，不将其中示例命令视作要求实际运行的指令。

| 项目 | 手册与本工程实现 |
| --- | --- |
| 运行模式 | 使能后为力位混合模式：`τ = Kp·(p_ref−p) + Kd·(v_ref−v) + τ_ff`；当前运行帧 `τ_ff=0` |
| 总线 | 标准 ID、CAN FD+BRS，仲裁 1 Mbps、数据 5 Mbps；R0 使用 CANFD0，控制周期 10 ms（100 Hz） |
| ID | 主机 ID `0x00`；关节 ID 1–6，手册默认 3，本次配置为 ID3 |
| 命令 | 64 字节：MSGID 2 字节、六个关节槽位各 10 字节、保留 2 字节；多字节字段小端 |
| ID3 槽位 | Byte22–Byte31，依次为位置、速度、Kp、Kd、前馈力矩，均为 uint16 LE |
| 编码范围 | 位置 ±12.5 rad、速度 ±65 rad/s、Kp 0–500、Kd 0–50、力矩 ±200 Nm；这些是编码范围，不是台架允许值 |
| 特殊命令 | 槽位前 9 字节 `FF`，末字节 `FC` 使能、`FD` 失能、`EF` 写机械零位；演示不写机械零位 |
| 反馈 | ID3、标准 FD、恰好 12 字节：MSGID、位置、速度、力矩、温度、状态、电压、保留 |
| 温度/电压 | 温度为 Byte8−40 °C；电压为 Byte10，单位 V |
| 状态 | bit0 使能；bit1–6 分别为过压、过流、过温、过速、编码器故障、欠压；错误掩码 `0x7E` |
| 数据有效性 | 回传 MSGID 必须来自近期驱动已接受的命令、未过期且顺序更新；重复、重排、错 ID/长度/格式不刷新反馈时间 |

手册同时写“0–65535 线性映射”与“0x8000 表示零”，二者存在半个 LSB 的中点差异。实现对对称字段的 `0x8000` 显式解为零，并对零指令编码为 `0x8000`；其他值按端点线性量化。使能与运行首帧的位置衔接、低速量化和实际方向仍要通过真实反馈核对。

当前单电机程序每帧显式失能另外五个槽位。同一总线不要并行运行其他独立关节控制进程。完整协议见 [PROTOCOL.md](PROTOCOL.md)。

## 动作顺序与观察内容

R0 接收到一次 `demo` 业务请求后，在自己的 10 ms 任务内生成全部轨迹。Linux 负责续租、采样和监督；任意阶段主机失联仍触发失能保护。`p0` 是本次使能时由反馈取得的软件原点。

| 阶段 | 状态名称 | 首次示例的动作 |
| --- | --- | --- |
| 准备 | probing → disabled → enabling → hold | 先确认失能反馈，再使能并衔接当前位置 |
| 1 | demo_point_negative | 五次曲线到 `p0−0.01 rad`，规划 2 s |
| 2 | demo_point_center_out | 五次曲线回 `p0`，规划 2 s |
| 3 | demo_point_positive | 五次曲线到 `p0+0.01 rad`，规划 2 s |
| 4 | demo_point_center_return | 五次曲线回 `p0`，规划 2 s |
| 5 | demo_position_sine | 位置幅值 0.01 rad、周期 4 s、稳定段一周期；起止包络各 1 s，共 6 s |
| 6 | demo_speed_sine | 速度峰值 0.03 rad/s、周期 4 s、一周期后回到 `p0` |
| 循环/结束 | 再进入阶段 1，或 hold → disabling → disabled | 按 `--repeat` 重放；全部完成后主机等待保持并确认失能反馈 |

位置正弦幅值为 A，周期为 `2×move_duration`，单侧包络为 `0.5×move_duration`。速度正弦使用 `v_ref=V·sin(2πt/T)`，同时发送 `p_ref=p0+V·T/(2π)·(1−cos(2πt/T))`。因此输出位置在 `[p0, p0+V·T/π]` 内往返，完整周期末位置和速度参考均回到起点值；示例的最大正向位移约 0.0382 rad（2.19°）。它仍保留位置反馈项，**不是 Kp=0 的纯速度模式，也不测试独立力矩模式**。手册只列出一种混合运行模式，这里测试的是不同参考轨迹。

每段规划时间结束后，还要求反馈速度绝对值小于 0.03 rad/s，位置误差小于 `min(0.002, max(0.0008, 0.1×A)) rad`，连续稳定 200 ms。A=0.01 时容差为 0.001 rad。超过规划结束 3 s 仍不稳定则锁存 `settle_timeout` 并持续请求失能；不能仅按时间假定电机完成动作。

演示开始前校验绝对编码范围、相对原点行程、首段实际距离的峰值速度、位置正弦速度上界及速度积分位移。运行中继续执行反馈超时、主机租约、跟踪误差、速度、力矩、温度、电压、故障位和任务周期检查。软件估算力矩限制不等于电机驱动器硬限矩。

## 本次板端路径及启动验收

| 内容 | 路径或状态 |
| --- | --- |
| Linux 工程 | `/home/bst/p85_validation/demo-20261120/source` |
| Linux 构建 | `/home/bst/p85_validation/demo-20261120/build` |
| 实际 IPC 库 | `/home/bst/p85_validation/demo-20261120/build/libp85_msgbox.so` |
| 固件、备份及部署事务 | `/var/lib/p85-rt0-deploy/20261120-motor-demo-01`，root 所有 |
| 本次 boot ID | `6e983f2c-5dfc-4f5d-9fd1-ffb1cf5c35d4` |
| 自动验收 | `p85-motor-demo-accept.service`，Result=success、退出码 0；oneshot 完成后 inactive/dead 属正常 |
| 回退保护 | 已接受本次镜像，`armed.json` 已移除，`p85-motor-demo-rollback.timer` disabled；未执行回退 |

本次只写 R0 槽位，没有更新 R2。新固件关闭启动时 CAN 内部回环和双控制器链路自检，默认 OFF；仅在显式 probe/运行命令后发送电机协议帧。接电机前应拆除此前 CANFD0↔CANFD1 的测试跨接，使用 CANFD0 接 P85，并按 [BRINGUP.md](BRINGUP.md) 确认接线、电源和总线端接。

本次整板重新上电的验收已通过。报告位于 `/var/lib/p85-rt0-deploy/20261120-motor-demo-01/boot-acceptance-6e983f2c-5dfc-4f5d-9fd1-ffb1cf5c35d4.json`，其 SHA256 为 `c6fcab289cd3b3247ffb3ab0150616da2c5c928d7203e96630aceec03f240e4a` 的 R0 镜像在 SRAM 中抽查 54,159 字节代码/只读数据全部匹配，AON 显示 `realtime0=normal`、cold start。25 轮真实 STATUS/DIAG 全程 OFF/fault=0，R0 设备时钟推进 5,280 ms，最大 STATUS/DIAG 往返 19.831 ms，电机 CAN tx=0。成功后 `accepted.json` 已写入，回退标记清除且 timer disabled。该验收不使能电机、不发送 CAN 控制帧，也不代表真实 P85 已通过。

```sh
# 整板重新上电且 SSH 恢复后，只读查看自动验收
sudo systemctl status p85-motor-demo-accept.service --no-pager
sudo journalctl -u p85-motor-demo-accept.service -b --no-pager
sudo cat /var/lib/p85-rt0-deploy/20261120-motor-demo-01/accepted.json
```

验收要求：新 ELF 代码/只读数据与 R0 SRAM 匹配、AON 中 R0 为 normal、25 轮真实 STATUS/DIAG 全程 OFF/fault=0、设备时钟推进且电机 CAN 发送为零。成功后移除 armed 标记并取消回退。失败会保留保护，15 分钟到期恢复原 R0 Flash，**不会自动重启或替换 SRAM**。若 Linux 本身无法启动，该 timer 无法执行；完整备份仍在部署目录及工作站。


### 电脑上的日常命令入口

网线连接用于 SSH 进入 A 核 Linux；在登录后的板端 shell 运行 `p85ctl.py`，命令经 MsgBox 交给 R0，再由 R0 的 CANFD0 控制电机。串口（板上 Micro-B 调试口）用于观察启动/故障日志，不是运行本控制 CLI 的入口。电机接线使用 CANFD0；接电机前拆下此前 CANFD0↔CANFD1 的诊断跨接。

Windows PowerShell：

```powershell
ssh bst@192.168.1.3
```

登录目标板后：

```sh
cd /home/bst/p85_validation/demo-20261120/source
sudo python3 host/p85ctl.py --library ../build/libp85_msgbox.so status
```

`status` 只发 HELLO/STATUS/DIAG 查询，不使能电机。当前实测返回 `state_name=off`、`fault_name=ok`、`motor_status=0`，CAN 发送和接收计数为 0，`feedback_fresh=false`；因此目前只是确认控制器和 A/R 软件链路已运行，尚无电机反馈。确认真实电机已按 [BRINGUP.md](BRINGUP.md) 接好、ID/电源/端接均核对后，才运行会发送失能帧并等待实机反馈的 `probe`：

```sh
sudo python3 host/p85ctl.py --library ../build/libp85_msgbox.so probe
```

无新鲜失能反馈时，停止后续步骤并检查 ID、CANH/CANL、地线、电机独立供电、终端电阻和总线速率。`run-hold`、`run-move`、`run-demo` 会主动使能并运动；先按 BRINGUP 的顺序验证，Kp/Kd 必须由安全台架实测确定，不能照抄仿真增益。

## 接入电机后运行

先通过 [首次实机调试](BRINGUP.md) 中的反馈探测、小增益保持及单次小幅移动。电机需要固定，输出端留有运动余量，并保留独立断开电机电源的条件。Kp/Kd 需根据实际电机确认；仿真增益没有被当作台架推荐值。

```sh
cd /home/bst/p85_validation/demo-20261120/source
sudo python3 host/p85ctl.py --library ../build/libp85_msgbox.so status
# probe 会发送失能帧并要求真实失能反馈
sudo python3 host/p85ctl.py --library ../build/libp85_msgbox.so probe

# 在当前终端设置经过台架确认的 P85_KP 与 P85_KD，再执行
: "${P85_KP:?set a validated bench Kp}"
: "${P85_KD:?set a validated bench Kd}"
sudo python3 host/p85ctl.py --library ../build/libp85_msgbox.so \
  --csv /home/bst/p85-demo-new.csv \
  run-demo --kp "$P85_KP" --kd "$P85_KD" \
  --position-amplitude 0.01 --move-duration 2 \
  --speed-amplitude 0.03 --speed-period 4 --speed-cycles 1 --repeat 1 \
  --max-speed 0.2 --max-torque 2 --excursion 0.05
```

CSV 路径必须不存在，每次运行使用新文件。终端约每 100 ms 输出状态和反馈，阶段切换立即输出；约每秒输出诊断。CSV 目标采样间隔 10 ms，受 Linux/IPC 延迟影响；每行带最近一次诊断值，诊断量本身不是 100 Hz 独立采样。可用 `python3 tools/analyze_csv.py /home/bst/p85-demo-new.csv` 分析位置误差。`--repeat` 与 `--speed-cycles` 均为 1–100，首次建议 1。

Ctrl+C 尝试平滑停止并等待失能确认；看到 `disabled_confirmed=true` 才表示收到新鲜失能反馈。若通信中断，主机不能证明已经停机，需要按现场条件处理。

## 构建、测试与写入证据

| 检查 | 本次结果及边界 |
| --- | --- |
| WSL 软件 CTest | 9/9 通过，包括核心协议、状态机、主机流程及诊断解析 |
| 板端 Linux 软件 CTest | 11/11 通过，额外验证官方 MsgBox/SDK 头文件适配；不代表电机机械动作 |
| 演示回归 | 全六阶段、两次循环、速度正负变化、位移范围、故障失能；新鲜但不动的反馈会触发 settle_timeout |
| 边界回归 | 绝对位置上下界、首段实际距离峰值速度、速度积分行程越界 |
| 完整 R0 固件 | 固定 19 项 SDK revision，Cortex-R5 hard-float 编译、链接、签名成功；SRAM 余量 397056 字节 |
| 启动自检配置 | boot_loopback=false，boot_can_link=false，默认 OFF |
| 写入 | `/dev/mtdblock0` 的 `0xB00000` 起始 1 MiB |
| 独立回读 | 下载完整 64 MiB，R0 逐字节等于候选；分区外所有字节与备份一致，R1/R2 未改变 |
| 旧运行镜像负向检查 | 自动验收 --check-only 在本次旧 boot 上因 SRAM 镜像不符失败，accepted=false；未将旧固件误验收为新固件 |
| 待完成 | 真实 P85 探测、反馈、使能、低速轨迹与故障响应实测 |

可在仓库查看[本轮精简证据清单](evidence/motor-demo-20261120.json)，其中记录 Flash、整板冷启动验收、当前 OFF 状态和产物/源码哈希。源码和最终构建产物通过 build-report 的 SHA256 关联。SDK 原有 newlib syscall 桩、RWX LOAD 段及旧签名工具弃用告警仍存在，完整日志已保留；本次 P85 源码未见编译告警。板端墙钟落后，首次解包/构建出现未来时间戳告警；使用全新构建目录完成全部目标，随后已校正该暂存源码目录的文件时间戳。

```text
签名包：BST-REALTIME0-RELEASE-C1200-1-3-0.bin（1048576 字节）
SHA256 c6fcab289cd3b3247ffb3ab0150616da2c5c928d7203e96630aceec03f240e4a
ELF SHA256 3fed2354ea79c025e8fde182416b5441aee230792fa559d91400af124fd060b0
写入前全片 SHA256 235ebdfd03cb5be6dd19dd5eb986dcec521948ba105f1d97a911cbb8d1eb302e
写入后全片 SHA256 424e16efae3e2d2da9a7636dbd0d974a6a987bd65cc8fa62a8d350b09a3086e2
写入时 boot ID 8c2102a9-c60c-438a-a8aa-057cedd88710
```

工作站证据位于 `output/p85_r_core_reading/p85-r0-demo-20261120-final/` 与 `output/p85_r_core_reading/p85-r0-demo-20261120-deployment/`：build-report、完整日志、板端软件测试、Flash 前后备份及 workstation-readback-verification.json。SDK、签名材料、原始 PDF、Flash 镜像和二进制产物均保留在 Git 忽略目录，不上传到分支。

当前实现入口为 `src/p85_control.c`、`src/p85_wire.c`、`rt0/p85_runtime.c` 和 `host/p85ctl.py`。A/R 新增操作码 13，原 0–12 操作码与 0–9 状态编号保持兼容，演示阶段状态为 10–15；新主机演示要求运行本轮或更新的 R0 镜像。
