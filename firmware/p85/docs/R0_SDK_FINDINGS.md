# R 核源码、工具链与自行编译核查

核查日期：2026-11-15（Asia/Tokyo）。结论：AURA/C1200 支持自行编译 R5 固件；已在本机 WSL 中完成官方裸机示例构建和 P85 便携核心的 R5 对象编译。当前仍未取得能重建 CAN/IPC 固件的完整 `realtime_code` 工程，未生成可部署的 P85 RELEASE 镜像。

本次读取官网、附件和已有 Flash 备份，在隔离的本地输出目录执行编译。附件中的烧录、启动或电机指令只作为技术资料审阅，没有执行。

## 官网实际提供了什么

从官网站点地图找到此前遗漏的《R 核子系统开发快速上手指南》。v2.0.x 和当前 v2.1.0 两版正文一致，仅版本标识不同。指南明确说明：

- 源码来自 BST SDK 的 `opensource_code/realtime_code.zip`，解压后有 `realtime_code_core0` 到 `realtime_code_core5`，各核独立构建。
- Linux 和 WSL2 支持完整编译、签名流程；Windows/MSYS2 只能完成部分流程。
- 工具为 `arm-none-eabi-gcc` 10.x 及以上、GNU Make 3.81 及以上、bash 4.0 及以上。
- `make.sh` 先调用 `bst_idl_code_gen`，再通过 `make_bstrtos.sh` 生成 `librtos.a`、`libthread.a`，编译 ELF/BIN/HEX，最后调用 `sign_package_open` 生成 RELEASE 镜像。

官方全量构建入口如下，**此命令尚未在完整 R 工程上执行**：

```sh
cd realtime_code_core0
sudo ./make.sh
```

| 输出 | 用途 |
| --- | --- |
| `out/realtime_freertos_core0.elf` | 调试用 ELF |
| `out/realtime_freertos_core0.bin` | 未签名的原始固件 |
| `out/realtime_freertos_core0.map` | 内存分布 |
| `out/BST-REALTIME0-RELEASE-C1200-1-3-0.bin` | 签名后的发布镜像，官方部署流程要求使用此类镜像 |

指南还提供了新增任务的示例，以及 FlashTool 单选 R0 和 A 核 Linux 局部刷写两种方法。因此，“没有公开找到完整源码包”不能表述成“R 核不允许或不能自行编译”。

## 已检查的公开下载内容

| 资源 | 核查方法与发现 |
| --- | --- |
| 官网 v2.0.x 下载页 | 将 Linux/MCAL 源码指向 SDK 的 `opensource_code/`，SDK 获取方式写为联系 BST FAE，没有给出 `realtime_code.zip` 直链 |
| 官网当前下载页及包服务器 | 有 V2.1.0 系统镜像、SDK Manager、FlashTool 等；公开 AURA 包目录只列 V2.1.0，未列独立 R 工程 |
| V2.1.0 镜像 ZIP | 通过 HTTP Range 读取 ZIP 中央目录：4,854,903,057 字节、167 个条目；顶层是 Images、元数据和 recovery-tools，没有独立的 `opensource_code/realtime_code.zip`。未下载或展开其中大型 rootfs 镜像，不能据此断言其内部所有文件均不存在源码 |
| `Hanhai-SDK-Manager-25.zip` | 已扫描容器归档全部 332,916 个条目，并检查选出的源码 DEB 内层文件；有 Linux AArch64 SDK、IPC 开发材料和 `home/r5baremetal.zip`，未找到完整 realtime/MCAL 工程、RTOS 库或签名工具 |
| 官方 GitHub 公开仓库 | 检查 `sesamex-img-sdk`、`sesamex-hardware-docs`、`sesamex_model_zoo` 的目录及发布入口；发现下载索引、硬件材料和视觉示例，未找到完整 R 工程 |

这里描述的是上述可访问资源的检查结果，不代表 BST 所有 SDK 发行包都不包含 R 源码。官方开发指南明确指明了应有的包名。

## 用户目录中可复用的内容

原始目录：

```text
D:/1110实验课任务划分/1110实验课任务划分/
```

已递归登记 135 个文件，并检查相关压缩包的文件清单和实验文档。

| 文件/目录 | 对 P85 工程的价值 | 尚缺什么 |
| --- | --- | --- |
| `canfd通信位置跟踪/（参考）达妙电机canfd通信实验/canfd驱动电机.patch` | 最有价值的 R 端参考；新增 555 行 `motor_mit_test.c` 和 60 行头文件，并修改 Makefile、shell 命令、CanApp 和 CanIf 配置；展示 `Can_Write`、FD 标志、10 ms 任务及 RX 回调 | 它是依赖原始 R 工程的补丁，不包含启动、RTOS 和完整 MCAL；达妙 MIT 帧不能用于 P85 |
| `canfd通信位置跟踪/实验5-跨域通信实验（A核与R核）/sample/` | A 核 MsgBox 示例、R 工程接入补丁、已编译 RT0 镜像和局部刷写脚本 | `acore_ipc_server.c` 及完整底座未随补丁提供；现成 BIN 不是本工程 P85 服务 |
| `IMGS-BST-WD-Linux-V2.0.1-SDK.zip` 及解压目录 | 155 个 ZIP 条目；提供 R0–R5 发布镜像、SBL 设备树、FlashTool 元数据和恢复材料，可用于布局核对 | 这是预编译镜像包，未找到 `realtime_code_core0` 工程；`SourceUrl` 只有内核分支/提交说明，没有 R 工程下载地址 |
| `MIT电机模拟器/x86-c/` | PC/USB CAN 模拟参考 | 属于 Linux PC 侧，不是 R 固件 |
| 三份实验 DOCX、P85 协议和外形 PDF | 任务结构、刷写流程、物理连接和 P85 帧依据 | 不包含完整 R BSP |
| `logroutetool-v1.0.5.zip` | R 核日志分发/虚拟串口工具 | 不是编译器或 R 工程 |

同时只读查看了当前项目的 `compile_develop/` 文件清单，其中主要是硬件资料、FlashTool 和 YOLO 示例；没有改动该用户目录。

## 已完成的真实 R5 编译

### 官方裸机示例

来源为公开 SDK Manager 容器中的 `home/r5baremetal.zip`。源码包括 `boot.S`、`main.c`、`bst_cortex_r0.ld`、Makefile 和 readme.docx。

在 `MediaOps-Ubuntu` WSL 环境解压 ARM 官方 12.2.MPACBTI-Rel1 x86_64→arm-none-eabi 工具链。压缩包 SHA256 已与 ARM 提供的校验文件一致：

```text
17455a06c816031cc2c66243c117cba48463cd6a3a3fdfac7275b4e9c40eb314
```

WSL 原环境没有 GNU Make，本次以 Python 按 Makefile 中原有 CFLAGS/LDFLAGS 顺序执行两条编译、一条链接和一条 objcopy 命令，没有声称执行过 `make all`。

| 检查项 | 结果 |
| --- | --- |
| 编译器 | arm-none-eabi-gcc 12.2.1 20230214 |
| ELF | ELF32、little endian、ARM、EABI5、soft-float |
| 架构属性 | ARM v7 / Realtime profile |
| 加载和入口地址 | `0x04000000` |
| BIN 大小 | 66,888 字节 |
| text/data/bss | 64,900 / 1,984 / 12,640 字节 |
| BIN SHA256 | `864a9a733f1968a5ba2622b374ba56904b63e6606e5126c6508290966e9b8fd8` |

原示例能返回构建成功，但链接器告警 `cannot find entry symbol _reset`：链接脚本指定 `_reset`，汇编实际导出 `_boot`。在第二个隔离副本中仅将 `ENTRY(_reset)` 改成 `ENTRY(_boot)`，重建后该告警消失，原始 BIN 字节不变。仍有源函数原型、nosys 系统调用桩及 RWX 段告警；本次没有把这个旧裸机示例认定为可部署的生产固件。

该程序初始化 UART，打印 `StartOS`，启用 cache 后返回到启动代码中的循环。它没有 CAN、MsgBox 服务或周期控制任务，也没有官方指南所要求的 RELEASE 签名。readme 中旧 SBL 的 `rtburn0`/`rtrun0` 等命令未在当前板验证或执行。

### 本工程 P85 核心

新增可重复使用的 `firmware/p85/tools/check_r5_compile.py`，交叉编译这四个文件：

```text
src/p85_protocol.c
src/p85_control.c
src/p85_wire.c
rt0/p85_runtime.c
```

使用 Cortex-R5 / ARM / soft-float / small-enum 探测配置，并启用 `-Wall -Wextra -Werror -Wpedantic -Wshadow -Wstrict-prototypes`，四个对象均无告警通过，归档为 `libp85_r5.a`（94,472 字节）。命令、编译器版本、源码及归档哈希、ELF 属性和未解析符号均留档。

WSL 中可复现，输出目录须为新建或空目录：

```sh
python3 /mnt/d/20261022ubunturos_win/firmware/p85/tools/check_r5_compile.py \
  --toolchain-bin /tmp/p85-r5-toolchain-12.2/arm-gnu-toolchain-12.2.mpacbti-rel1-x86_64-arm-none-eabi/bin \
  --output /mnt/d/20261022ubunturos_win/output/p85-r5-compile-new
```

工具链归档保存在本地 `output/p85_r_core_reading/toolchains/`；上述 `/tmp/` 解压位置仅代表本次环境。

这个结果验证了目标架构编译，不包括 `p85_bst_msgbox.c`、`p85_mcal_tx.c` 的实际 R SDK 编译或完整固件链接。归档仍有 libc、libm、libgcc 依赖，探测浮点/枚举 ABI 还须与完整 SDK 保持一致；不能将 `.a` 写入 Flash。

## Flash 布局证据已经补强

从此前取得的 64 MiB 板上只读备份解析出 SBL DTB，位于 Flash `0x100000` 和 `0x114000`，两份均为 14,261 字节。它们与本地 V2.0.1 包的 `Images/sbl-dtb/BST-SBL-C1200-96_ADAS_DB.dtb` **SHA256 完全相同**：

```text
1b1a62bd2e1e2c36df4b73105fe29c535043fcd8c255f8a7df054da1e1b7dea4
```

| 核 | Flash 偏移 | 槽位容量 | SRAM 加载地址 |
| --- | --- | --- | --- |
| RT0 | `0xB00000` | `0x100000` | `0x04000000` |
| RT1 | `0xC00000` | `0x100000` | `0x04100000` |
| RT2 | `0xD00000` | `0x100000` | `0x04200000` |
| RT3 | `0xE00000` | `0x100000` | `0x04300000` |
| RT4 | `0xF00000` | `0x100000` | `0x04400000` |
| RT5 | `0x1000000` | `0x100000` | `0x04500000` |

这些值同时得到附件刷写脚本和新找到的官方 R 核开发指南支持。此前“偏移仅来自旧示例、布局未核实”的结论已过时；现在有当前板备份的配置依据。但布局一致不证明任意 BIN 的签名、版本和兼容性正确，本次没有执行擦除、写入或重启。

## 要完成 P85 R0 固件还需要什么

应补齐与板上 `V2.0.1_20260813-185450_276175` 相容的 **`opensource_code/realtime_code.zip`**，包括 `realtime_code_core0`、RTOS 构建入口、MsgBox、CAN MCAL/CanIf/时钟配置，并取得其构建所需的 `bst_idl_code_gen` 和 `sign_package_open` 等工具。工具的具体相对位置以完整包为准，不能只复制一个 Makefile 或现成 BIN。

取得后先原样构建基线，确认可重复生成签名发布镜像，再按 `../rt0/PORTING.md` 接入现有 P85 核心，借鉴达妙补丁的任务与 CAN 入口，完成 1 Mbps/5 Mbps FD+BRS、真实 RX 元数据、OFF 状态 A/R 握手，最后才进入 CAN 链路和电机调试。

在这条官方流程中，可以在本机 Linux/WSL 编译，经网线 SCP/SFTP 将签名镜像传到板上，再通过 SSH 执行已核对的局部刷写流程，重启后生效。FlashTool 是另一条官方方式，需要其 USB/串口接线。当前已具备网线/SSH 和本机 R5 编译能力，阻塞点是完整 CAN/IPC 工程及发布签名依赖。

## 来源与本地证据

官方主资料：

```text
https://documents.blacksesame.com/sesameX/sitemap.xml
https://documents.blacksesame.com/sesameX/aura/2.0.x/System_configuration/r-core/r-core-development-guide
https://documents.blacksesame.com/sesameX/aura/System_configuration/r-core/r-core-development-guide
https://documents.blacksesame.com/sesameX/aura/2.0.x/download
https://documents.blacksesame.com/sesameX/aura/download
https://pkgs.bstai.top/robot/sesamex-img-sdk/aura-nx-5000/V2.1.0/
https://github.com/bst-sesamex/sesamex-img-sdk
https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
```

全部本地证据在项目忽略的目录，未将厂商源码、镜像或大型工具链提交 Git：

```text
output/p85_r_core_reading/discovery-20261115/
  local-inventory.json
  official-r-core-pages-index.json
  v20-r-core-development-guide.html / .txt
  v21-r-core-development-guide.html / .txt
  acquisition-links.json
  remote_image_index.json
  board-flash-dtb-layout.json
  r5baremetal/                  # 提取原件
  r5baremetal-build/            # 原样构建、build-log.json、artifacts.json
  r5baremetal-entry-fixed/      # 入口修正副本及构建证据
  p85-r5-objects/               # .o/.a、compile-report.json、ELF 属性及未解析符号
output/p85_r_core_reading/sdk_scan_names.txt
output/p85_r_core_reading/sdk_scan_complete.log
output/p85_r_core_reading/sdk_selected/r5baremetal.zip
output/p85_r_core_reading/toolchains/
```
