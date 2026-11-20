# 完整 P85 R0 固件构建记录

**最新进度（2026-11-20）：** 单电机顺序演示固件完整编译、链接及签名通过，SRAM 剩余 397056 字节，已写入 R0 并完成全片独立回读。新镜像待整板冷启动验收；以下为 11 月 15 日首次构建的历史记录。最新产物哈希、测试及动作说明见 [MOTOR_DEMO.md](MOTOR_DEMO.md)。

2026-11-15（Asia/Tokyo），分支 `codex/p85-rt0-motor-control`。公开 Hanhai R0 SDK 的 P85 接入已完成，使用当前接入脚本从新的隔离目录完成编译、链接、objcopy 和签名包生成。构建后的试烧已完成局部写入、回读和 Linux 重启，但 R0 SRAM 仍是原镜像；已恢复原 Flash，未向电机发送命令。具体见 [实板部署记录](R0_DEPLOYMENT.md)。

## 本次结果

| 项目 | 结果 |
| --- | --- |
| SDK | `C1200SW_V1.6_RC0930` / `realtime_freertos_core0.xml`，19 项 commit 已固定并检查 |
| 编译器 | Arm GNU 12.2.MPACBTI-Rel1，`arm-none-eabi-gcc 12.2.1` |
| ABI | Cortex-R5 / ARM / hard-float / VFPv3-D16 / small enum |
| 构建宿主 | WSL `MediaOps-Ubuntu`，Linux x86-64；GNU Make 4.3 |
| ELF 入口/加载起点 | `0x04000000` |
| text / data / bss | 285439 / 8796 / 352192 字节 |
| ELF LOAD 区间（含对齐、堆/栈） | `[0x04000000, 0x0409ddc0)`，646592 字节 |
| 1 MiB SRAM 余量 | 401984 字节，约 392.6 KiB |
| 原始固件 | 294244 字节 |
| 签名发布包 | 1048576 字节；SDK 随附签名工具生成 |
| 目标板 Linux 软件测试 | CTest 5/5 通过，未访问真实 CAN |

签名工具生成文件不证明当前板启动链接受该签名，也不证明新旧 BSP 互通。保留的 newlib 系统调用桩告警、RWX LOAD 段告警和旧签名工具 Python 弃用提示来自 SDK/工具链；P85 源文件未出现编译告警。未将这些告警描述成零告警构建。

## 文件归档

本次最终产物目录：

```text
D:/20261022ubunturos_win/output/p85_r_core_reading/p85-r0-integration-20261115-final/
```

| 文件 | 字节 | SHA256 |
| --- | --- | --- |
| `realtime_freertos_core0.elf` | 3141732 | `6c43b128ece5b4d2b66e6ea784b3bfc8194a29274ff2164fc86659c9639d13ae` |
| `realtime_freertos_core0.map` | 2580397 | `3e05d386f6f579136d034de66c706566a02660f173fb5fe807b06ce939418a3e` |
| `realtime_freertos_core0.bin` | 294244 | `d7df18d4737c466ff051cba9d885cd9343b719584921f6728f920fa2afcfa6e1` |
| `realtime_freertos_core0.hex` | 827685 | `1ca80ba705fb8f4ac65849836436ad142a95cad6a1d668c51a4223129900975a` |
| `BST-REALTIME0-RELEASE-C1200-1-3-0.bin` | 1048576 | `57c2d22f38a82cf2e2526ef28825f197ee6d6880e34fa74ea411a7341c624c07` |

同目录还包含 `build.log`、`build-report.json`、`elf-attributes.txt`、实际使用的 `p85.mk`、`p85-sdk-lock.json` 和 `linux-tests/` 中的配置/编译/CTest/LastTest 日志。报告记录源码哈希、工具版本、链接符号、SRAM 区间及告警。

最初的候选构建保存在没有 `-final` 后缀的目录；应以这里的最终目录为准。构建时间戳、绝对调试路径和签名随机量可能改变二进制哈希；这里的“可复现”指可重复执行固定底座上的完整构建流程，不宣称按位相同。

## 复现构建

输入必须是完整的 19 项 manifest 检出，保留各子项目 `.git`，不能混合 V1.7/V1.8 的项目。当前原始检出保存在：

```text
D:/20261022ubunturos_win/output/p85_r_core_reading/hanhai-v16-core0/realtime/
```

脚本检查全部项目 HEAD 和 8 个待补丁文件的 LF 归一化哈希，复制到新目录后接入 P85；不会修改原始 SDK 或访问开发板。工作目录与产物目录必须不存在。Linux/WSL 需要 Python3、git、bash、make、ARM 工具链，以及 SDK 自带 Linux x86-64 IDL/签名工具所需运行库。

本次实际命令（工作目录和产物目录已存在，重跑时换新名字）：

```sh
python3 /mnt/d/20261022ubunturos_win/firmware/p85/tools/build_hanhai_sdk.py \
  --sdk /tmp/p85-hanhai-build/realtime \
  --work /tmp/p85-hanhai-final-20261115-v2 \
  --artifacts /mnt/d/20261022ubunturos_win/output/p85_r_core_reading/p85-r0-integration-20261115-final \
  --toolchain-bin /tmp/p85-r5-toolchain-12.2/arm-gnu-toolchain-12.2.mpacbti-rel1-x86_64-arm-none-eabi/bin \
  --make-dir /tmp/p85-make-local/root/usr/bin \
  --jobs 4
```

`/tmp` 是本次临时环境。WSL 重建后可用 `/mnt/d/.../hanhai-v16-core0/realtime` 作为 `--sdk`，重新解压已留存的工具链并调整路径；系统 make 可用时省略 `--make-dir`。工具链归档位于 `output/p85_r_core_reading/toolchains/`。

`prepare_hanhai_sdk.py` 也可单独完成接入，再手动运行 SDK 的 `bash ./make.sh -j4`。推荐上面的总入口，它会记录退出码，并检查 ELF ABI/入口/LOAD 范围、P85 链接符号及全部输出，避免原 SDK 脚本隐藏失败。

## 板级实现

详细设计见 [R0 接入说明](../rt0/PORTING.md)。本次完成：

- 静态 FreeRTOS 控制/接收任务、10 ms 调度、浮点上下文和 IRQ 状态恢复。
- RT0/fid0/sid2/cmd0x85 MsgBox 服务和真实 SDK completed 队列接收流程；下一次 query 负责回收节点，业务层提前复制。
- CAN0 两层 1 Mbps / 5 Mbps、FD+BRS 配置及寄存器回读门控；HTH12、64 字节 TX 和保留真实 ID/FD/IDE 的 RX 入口。
- bus-off/错误被动/FIFO 溢出保护及只读 `p85stat`；初始 OFF、未设置增益，不主动发 CAN。
- 关闭冲突的 CAN/VCan/时间同步演示，隔离 P85 TX 确认，精简网络应用以满足 SRAM 容量。

## 软件测试证据

目标板原生 GCC 11.4 构建于独立目录 `/home/bst/p85_validation/integration-20261115`，沿用 `/usr/bst` 官方 MsgBox 头文件及库。5 组 CTest 通过：

| 组 | 结果 |
| --- | --- |
| core | 9905 检查，0 失败 |
| runtime | 406 检查，0 失败 |
| host | 6 个 Python 流程通过 |
| BST adapter | 46 检查，0 失败 |
| Hanhai receive | 35 检查，0 失败 |

新增接收测试覆盖多 packet、分片/空节点拒绝、状态/错误返回，以及节点回收后命令副本仍能产生正确 token/序号的回复；OFF 全程零 CAN 发送。这里使用假传输，不证明新 R0 固件在板运行；后续试烧也未通过实际启动验收。此前 ASan/UBSan 记录仍适用于原有测试；新增接收组本轮执行的是普通严格告警构建。

## 下一阶段的验收边界

现有板 BSP 名称为 V2.0.1，公开底座分支名为 V1.6；不能仅凭名称认定兼容或不兼容，需要启动和实际 IPC 验收。部署前核对签名/恢复通道和 RT0 槽位；已有布局证据为 Flash `0xB00000`、容量 1 MiB、SRAM `0x04000000`。然后依次完成：新 RT0 启动及 `p85stat`、OFF 状态 HELLO/STATUS/DIAG、CAN FD 分析仪/第二节点、单电机失能探测与低增益运行。

CAN 时序目前通过源码计算与寄存器配置检查，TDC offset12 尚未实测；实际引脚/收发器使能、波形、ACK、任务抖动和栈水位均未验收。完整编译成功不能替代这些测试，也不能保证目前“只接电机即可运行”。
