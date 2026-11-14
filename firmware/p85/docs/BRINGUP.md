# 首次单电机接线与调试

**前置条件：匹配 R0 固件完成构建、部署，A/R OFF 状态握手通过，CAN 分析仪已核对 1/5 Mbps、FD+BRS 和 64 字节发送。当前工程尚未达到这些条件。** 下面是后续验收步骤，不表示当前连接电机即可运行。

## 连接关系

电脑到板子的网线继续用于 SSH、文件传输与查看数据；板子到电机另接 CAN FD 和电机电源。板载收发器正确接通时，不需要让电脑承担 USB-CAN 实时控制；USB-CAN FD 分析仪可用于独立检查和定位问题。

```text
电脑 ── Ethernet/SSH ── AURA Linux A 核 ── MsgBox ── R0 ── 板载收发器 ── CANH/CANL ── P85
                                                                                 ↑
                                                                        独立 24–60 V 电源
```

| 信号 | P85 图纸线色 | 板侧 |
| --- | --- | --- |
| CANH | 绿 | RT_CAN_FD0_H / RT_CANFD0_H |
| CANL | 白 | RT_CAN_FD0_L / RT_CANFD0_L |
| GND | 黑 | 与板侧信号地共地，并按电机电源要求回流 |
| VBUS | 红 | 电机电源 24–60 V；不是板上逻辑电源针脚 |

官方 AURA 套件供电资料为 12–20 V，电机为 24–60 V。不能把电机电源直接接开发板输入，也不能把排针的 1.8/3.3/5 V 作为电机 VBUS。按实物电源标签确认范围。

使用短双绞线，CAN 总线两端各 120 Ω。先检查载板与电机是否已装终端，断电测量确认，避免重复并联；两端均有 120 Ω 时通常测得约 60 Ω。仅接线但没有 ACK 对端，不能用持续 TX 验证通过。

### 官方 V1.2 载板图纸的针脚线索

已查看官方 `QSC801XP-AP_EVB_SCH_V1.2_20260430.pdf`，PDF 第 8 页的 26PIN Header `J0801`：

| J0801 针脚 | 图纸信号 |
| --- | --- |
| 7 | RT_CANFD0_L |
| 9 | RT_CANFD0_H |
| 5、6、19 | GND |
| 8、10 | SW_CANFD0_L/H，属于另一组，不能混接 |

来源仓库 `bst-sesamex/sesamex-hardware-docs`，查询时 commit `30add796af81663bd2b5f5836a00c779785a45d5`；图纸 SHA256 `5ff633bda60de4d8a9b966bd7da6c64b42afdd2c12e92c68b9545e343a16f6aa`。本地只读副本位于项目忽略目录 `output/p85_r_core_reading/hardware/`。另已核对官方载板规格书 `Aura DevKit Carrier Board Specification_DX-5000-D_V1.2.5.pdf` 第 15 页针脚图（7=L、9=H）和第 16 页 Pin1 方向图，与该信号表一致。

**尚未确认用户实物载板的型号、修订版与针脚 1 方向。只有丝印/版本与该图纸一致且针脚方向已核实，才可使用此表接线。** 图纸中的编号不等于从任意观察方向数的第几个针。官方载板规格书中的布局图也应与实物对照。

## 0. 台架准备

确认实物为 P85MAX-S CAN FD 版、实际 ID 和电机固件；默认 ID3 尚未读到实机验证。首阶段只接一个电机，避免本工程给其他槽位的失能影响其他设备。固定电机、移除机械负载并留出小角度余量，设置可独立断开电机电源的条件。初次不写机械零位，不测试大范围或四足承载动作。

增益必须依据该电机的实际参考和台架结果设置；仿真示例的 Kp=5、Kd=0.1 不构成硬件推荐。软件力矩检查不会修改驱动器的硬限矩参数。

## 1. 先验证 A/R，不使能

```sh
cd /home/bst/p85_control
sudo ./build/p85_ipc_probe
sudo python3 host/p85ctl.py status
```

必须得到本工程版本 1 的 HELLO 和诊断应答。查询结果 2 只代表 RT0 在线；`-110` 表示 P85 业务应答超时；不要继续运行轨迹。OFF 时应无 CAN TX。若找不到 `libp85_msgbox.so`，先安装/核对官方 MsgBox 包并构建，不能用仿真替代硬件验收。

## 2. 失能探测

```sh
sudo python3 host/p85ctl.py probe
```

程序发 64 字节失能帧，等待 ID3 的 12 字节反馈且状态 bit0=0。检查 `feedback_fresh=true`、温度/电压合理、无故障；CAN 分析仪核对真实 MSGID 和小端解码。没有反馈时应报 `ack_timeout`，这是失败而非“电机未动所以成功”。ID 不符要修改 R 配置重建，不允许放宽过滤或伪造反馈。

## 3. 首次保持

先将经确认的台架增益写入当前终端变量 `P85_KP` 和 `P85_KD`；下列参数检查会在变量缺失时中止：

```sh
: "${P85_KP:?set a validated bench Kp}"
: "${P85_KD:?set a validated bench Kd}"
sudo python3 host/p85ctl.py --csv /home/bst/p85-validation-hold-new.csv \
  run-hold --kp "$P85_KP" --kd "$P85_KD" --hold-seconds 1 \
  --max-speed 0.2 --max-torque 2 --excursion 0.05
```

观察从探测到使能、保持、停止、失能的完整状态转换和 `disabled_confirmed=true`。使能时以反馈位置衔接，不改变机械零位。检查无跳变、振荡、过流和异常温升。若任何保护触发，保留日志、断开危险运动条件并查原因；不要提高阈值掩盖问题。

## 4. 小幅移动，再做正弦

前一步稳定通过后，才执行如下示例。CSV 路径必须是新文件。

```sh
sudo python3 host/p85ctl.py --csv /home/bst/p85-validation-move-new.csv \
  run-move --kp "$P85_KP" --kd "$P85_KD" --offset 0.01 --duration 2 \
  --max-speed 0.2 --max-torque 2 --excursion 0.05

sudo python3 host/p85ctl.py --csv /home/bst/p85-validation-sine-new.csv \
  run-sine --kp "$P85_KP" --kd "$P85_KD" --amplitude 0.01 --period 4 \
  --cycles 3 --ramp 1 --max-speed 0.2 --max-torque 2 --excursion 0.05

python3 tools/analyze_csv.py /home/bst/p85-validation-sine-new.csv
```

位置偏移相对每次使能时的位置，单位 rad；本例 0.01 rad 约 0.57°。正弦稳定段 12 s，启动/退出包络另计。只有实机方向、比例与轨迹一致后再逐步增加幅值。RMSE 没有用户指定的验收阈值，不能把仿真误差作为实机通过标准。

## 5. 停止、故障和失联

- 正常结束及一次 Ctrl+C：主机先请求平滑停止、等待低速保持，再失能并查新鲜反馈。看到 `disabled_confirmed=true` 才是反馈确认；发送成功或进程退出本身不代表电机停机。
- 失去主机续租超过 300 ms、反馈超过 100 ms、驱动故障等，R0 状态机锁存并持续请求失能。实机验证应先在无负载台架进行。
- R0 自身停摆、电源故障或断开的 CAN 线会使软件无法发出有效失能；不要把这些情况等同于已验证的停止。厂家断流行为尚待确认。
- 故障原因排除，且已有健康新鲜的失能反馈，才使用 `sudo python3 host/p85ctl.py clear`；无反馈不会清除保护。
- `sudo python3 host/p85ctl.py disable` 请求立即失能，用于已确认可释放负载的情形；承载机构需要先处置负载。

## 后续四足扩展

一个 CAN 主机帧包含六个关节槽位。当前程序明确失能非目标槽位，不能通过“为每个电机启动一个进程”扩展。后续需统一的多关节帧所有者、每关节反馈与保护、控制周期预算、总线负载测量及独立急停设计。
