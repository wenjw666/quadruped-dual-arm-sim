# R2 释放 CAN1 的配套固件

更新：2026-11-19（Asia/Tokyo）。R2 CAN1 release-v2 已通过实板冷启动：boot `3d8fbaa2-a575-4987-b452-19bb40c0ad6e` 中 ELF 内容匹配，心跳 2,153→2,181（间隔约 250 ms），stage=2、CAN1 inactive、CAN2/3 active 且 START 成功；R0 双向物理测试每方向 512 帧全部通过。配套诊断已按预设恢复原 Flash，当前 SRAM 仍运行候选，未接受为常驻基线。

## 来源与修改

`rt2/hanhai/sdk-lock.json` 固定 `C1200SW_V1.6_RC0930` 中的 8 个 Git revision，并记录待修改文件的 LF 归一化哈希。构建脚本只通过 `git archive` 导出指定对象，不复制已有工作树中的临时改动；源码不符合固定 revision、哈希或唯一替换锚点即失败。

R2 专有源码为 `rt-app-core2`、`rt-hal-core2`、`rt-mcal-cfg-core2`、`rt-radar-task`；BSW、OS、OS adapt 和 rlog 使用同 manifest 的固定版本。原始 SDK 与签名材料保存在 Git 忽略的 `output/`，不放入本工程源码。

变体将 hardware CAN1 的 activation 改为 FALSE，删除 `Can_SetControllerMode(CanController_1, STARTED)` 与 CAN1 中断注册，禁用原始 `can` 测试命令，并拒绝 `cansend` bus 1。CAN2/3 保持 active，按原任务流程启动。其余系统初始化保留；未修改 SBL 或 R1。

`p85_r2_handoff.[ch]` 发布 64 字节报告，包含实际配置 activation、初始化阶段、CAN2/3 START 结果和原有任务中的周期心跳。报告按 32 字节对齐，以 BSP cache clean/invalidate 与屏障发布；不读取 CAN1 MMIO。owner=R0 是配置声明，并非硬件总线仲裁测量。

## 可复现构建

构建入口为 `tools/build_r2_can1_handoff.py`，需 Linux/WSL、GNU Arm 12.2 工具链和固定源码。脚本不会连接板子、写 Flash 或复位。

本机 WSL 为 `MediaOps-Ubuntu`。进入 WSL 后，可用新的 work/artifacts 目录构建；这两个目录必须不存在且与源目录互不嵌套：

```sh
cd /mnt/d/20261022ubunturos_win
python3 firmware/p85/tools/build_r2_can1_handoff.py \
  --sources output/p85_r_core_reading/core2-ownership-source \
  --shared-sdk output/p85_r_core_reading/hanhai-v16-core0/realtime \
  --work /tmp/p85-r2-handoff-next \
  --artifacts output/p85_r_core_reading/physical-can-20261117/build-r2-handoff-next \
  --toolchain-bin /tmp/p85-r5-toolchain-12.2/arm-gnu-toolchain-12.2.mpacbti-rel1-x86_64-arm-none-eabi/bin \
  --jobs 8
```

工具链归档保存在 `output/p85_r_core_reading/toolchains/arm-gnu-toolchain-12.2.MPACBTI-Rel1-x86_64-arm-none-eabi.tar.xz`，SHA256 为 `17455a06c816031cc2c66243c117cba48463cd6a3a3fdfac7275b4e9c40eb314`。WSL `/tmp` 被清理后需先重新解压并确认实际 bin 路径。

本次成功结果为 `output/p85_r_core_reading/physical-can-20261117/build-r2-handoff-v2/`：ELF 入口 `0x04200000`，结束 `0x04262230`，剩余 646,608 字节；签名槽位 1 MiB，SHA256 `27eb3e398a68f81fb80a7692d7066e87953ab0b3e0814c806046788b1f4d2926`。构建报告记录所有产物哈希、固定源码、编译器和告警。已有 SDK/newlib 占位和 RWX LOAD 等告警仍存在。

## 执行验证

`tools/inspect_r2_handoff.py` 只映射已审核的 R2 1 MiB SRAM：`0x04200000..0x042FFFFF`。它匹配 ELF 的只读内容及关键函数，要求两份间隔快照均有正确 magic/version/core、CAN1 inactive、CAN2/3 active 且 START 成功，stage=2，心跳向前推进。心跳不变、倒退或镜像不匹配均拒绝。离线验证支持 `--snapshot` 与 `--second-snapshot`，两者必须一起提供。

以下只读检查适用于当前仍运行配套候选的 boot；再次完整上电后将运行恢复的基线，不能继续按候选 ELF 验收：

```sh
sudo -n python3 /var/lib/p85-rt0-deploy/20261117-can-handoff-01/inspect_r2_handoff.py \
  --elf /var/lib/p85-rt0-deploy/20261117-can-handoff-01/realtime_freertos_core2.elf
sudo -n journalctl -b -u p85-can-handoff-collect.service --no-pager
```

Linux 不应映射 CAN 控制器 MMIO。CAN 寄存器由 R0 读取并发布到其 SRAM 跟踪报告。R2 心跳和配置检查通过后，还必须核对 R0 的 CAN1 配置及真实物理收发，才能判断冲突已消除。

## 本次双槽位试烧与恢复

事务路径固定为 `/var/lib/p85-rt0-deploy/20261117-can-handoff-01`，仅修改 R0 offset `0xB00000` 和 R2 offset `0xD00000` 各 1 MiB。`can_handoff_flash_guard.py` 是模板，部署前将 `package-info.json` 的 SHA256 写入唯一占位符；metadata 固定候选、ELF、检查器、collector 和 unit 文件。render 后 guard 的哈希由独立上传清单保存，避免循环引用。

guard 的 `inspect` 和 `install` 要求同一个 origin boot、独立固定的完整基线、全部包哈希，以及 live full Flash 等于基线。marker 和下次启动服务先持久化，再先写 R2、后写 R0，最后全片读回比较；异常会尝试恢复基线。恢复仅依赖固定基线，不依赖候选文件或 metadata 完整性。R1 和槽位外变化会触发拒绝，guard 没有接受候选或自动重启操作。

`collect_can_handoff.py --check-only` 只采集，不写 Flash；正常模式拒绝 origin boot。本次诊断启动已保存同一份 R0 SRAM 的镜像/回环/物理/跟踪结果、两次 R2 SRAM、AON 结果，随后恢复原 R0/R2 两个 Flash 槽位并校验整片。15 分钟 timer 为采集异常提供后备恢复，部署时的 origin boot 只 enable，不 start；本次恢复后 timer 已禁用。恢复 Flash 不会立即替换 SRAM 中运行的候选；再次主电源启动才使用恢复后的镜像。

本次已实际完成写入和工作站独立读回比较：整片 SHA256 `d92aa47071b39e587eb7de875a2b21faa86f6c34e3fda963fd7d091552b00743`，R1 及槽位外逐字节一致，`accepted=false`。基线为 `235ebdfd03cb5be6dd19dd5eb986dcec521948ba105f1d97a911cbb8d1eb302e`。2026-11-19 用户已完成整板主电源重新上电，采集器全部报告退出码为 0，恢复成功，service/timer 已禁用，armed marker 已移除。

详细实测历史、CAN1 状态枚举修正、接线和通过条件见 [物理链路记录](PHYSICAL_CAN_VALIDATION.md)。
