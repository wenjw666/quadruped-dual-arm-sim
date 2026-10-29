# C1200 上 YOLO / SmolVLM 适配资料分析

## 1. 分析范围与结论

本报告只基于 `compile_develop/` 中现有资料进行分析，未执行资料中的刷机、登录、Docker 编译、ADB 推送或板端运行命令。资料中的命令和操作步骤属于设备与工具链的使用说明，不能替代本次研究任务本身。

当前资料已经覆盖了一条可复现的 C1200 YOLOv8s 部署链路：

```text
YOLOv8s（修改为 ReLU）
    -> 固定输入 640x640 的浮点 ONNX
    -> BSTNNX PTQ / INT8 编译
    -> C1200 UserlandStage 模型包
    -> BSNN C++ Runtime
    -> 板端后处理与结果绘制
```

SmolVLM 不能直接套用这条链路。现有材料没有提供 SmolVLM 模型、转换脚本、算子支持表、语言模型运行时或动态 KV-cache 示例。因此，当前最合理的研究路线是先把 SmolVLM 拆成视觉编码器、视觉语言投影层和语言解码器，分别判断哪些部分可以进入 C1200 NPU，哪些部分必须留在 CPU/GPU 或采用混合运行。

## 2. 资料组成

### 2.1 硬件和系统资料

- `compile_develop/第一部分 了解实验环境与软件工具.pdf`
  - 开发环境、串口、ADB、SSH、网络、刷机、Docker/BSTNNX 和 YOLO 实验说明。
- `compile_develop/硬件资料/Aura(NX-5000)产品参数及接口说明.pdf`
  - Aura NX-5000-S/D 模组规格和 DX-5000-S/D 载板接口。
- `compile_develop/硬件资料/原理图/RC12_CARRIER_V010_260812.pdf`
  - RC12 载板 20 页原理图，包含模组供电、USB、MIPI CSI、PCIe、以太网、UART 等连接。
- `compile_develop/硬件资料/原理图/RC12_EXT_V010_20260812_2.pdf`
  - RC12 外设扩展板 22 页原理图，包含外部电源、相机、IMU、MCU、隔离和执行器相关连接。
- `compile_develop/HanHai-flashtool-windows-4.0.2.0-robot/`
  - Windows 烧录工具及 C1200 相关 SBL、DTB、固件和工作流文件。
- `compile_develop/bsnn_tools_docker_8.3.1.tar.gz`
  - BSTNNX Docker 镜像归档，文件约 6.5 GB。

### 2.2 YOLO 样例

- `compile_develop/yolov8_bst_sample/train/`
  - 修改后的 Ultralytics YOLOv8 训练、导出、校准数据生成和精度验证代码。
- `compile_develop/yolov8_bst_sample/train/compiler/`
  - C1200/A1000 编译 YAML、YOLOv8s 浮点 ONNX、量化结果和编译结果目录。
- `compile_develop/yolov8_bst_sample/Deploy/code/`
  - C++17、OpenCV、BSNN Runtime 的板端推理程序和构建文件。
- `compile_develop/yolov8_bst_sample/Deploy/prebuild/`
  - C1200/A1000 可执行文件、模型包、输入/参考输出和启动脚本。
- `compile_develop/yolov8_bst_sample/runtime_envs.md`
  - 资料声明的 BSNN、BSTN、DSP RTOS 和编译器版本。

资料目录还包含完整的 Ultralytics 源码、预编译库、烧录程序和大量运行时文件。它们是实验输入或工具依赖，不能仅凭文件存在就视为已经验证可用。

## 3. C1200 / Aura 硬件能力

### 3.1 模组规格

`Aura(NX-5000)产品参数及接口说明.pdf` 对 NX-5000-D 给出的参数是：

| 项目 | 资料中的规格 |
|---|---|
| NPU AI 性能 | 70 TOPS |
| CPU | 8 核，150K DMIPS |
| MCU | 异构 MCU，42K+ DMIPS |
| GPU | Mali-G78AE，500+ GFLOPS |
| 内存 | 16 GB LPDDR5 |
| 存储 | 64 GB UFS 3.1 |
| 相机输入 | 3× MIPI CSI-2，共 12 lanes；支持 12 路 1080p@30 或 4 路 4K@30 |
| 显示 | 2× MIPI DSI TX，每端口 4 lanes，2.5 Gbps/lane |
| 扩展 | PCIe Gen4，1×4 或 2×2；USB 3.1×4 |
| 外设 | 10× UART、2× SPI、6× I2C、2× I3C、8× PWM、5× ADC、GPIO |
| CAN-FD | 1 路 Safety CAN-FD + 8 路常规 CAN-FD |
| 模组接口 | MXM 314-pin |

资料中的系统架构把 C1200 划分为 Safety、Realtime、Switch、Secure、AI 辅域和 AI 主域。AI 主域运行 Ubuntu/Linux、NPU 推理、SLAM 和目标检测。这个划分意味着部署程序应当以 AI 主域 Linux 用户态为主；刷写 SBL、DTB 或其他域固件不是模型适配的常规步骤。

### 3.2 载板和连接

开发环境资料给出的 DX-5000-D 载板是 120×113 mm，包含 2× USB-A、普通 USB-C、ADB 专用 USB-C、Debug Micro-B、2× RJ45、HDMI、M.2 Key M/E。资料建议：

- 串口 B 用于 Ubuntu 主系统控制台，波特率 115200、8N1、无流控。
- ADB 用于文件传输和 `adb shell`，应使用标注 ADB 的 Type-C 口。
- SSH 用于日常 Linux 管理和部署。
- 文档示例给出了 `192.168.34.200`/`192.168.35.200` 网口地址，但板端实际地址需要在串口或系统中确认。

RC12 载板原理图显示，外部输入为 22–55 V DC；P5V_STB 给 Aura 模组供电，P12V 连接 GMSL 相机 0–2，另外有多个 USB、以太网、MIPI CSI 和 PCIe 信号。原理图中出现了 “Aura 模组 C1236” 的标注，而产品资料和软件资料主要使用 “C1200 / Aura NX-5000-D”。这说明实际板卡/模组型号需要在接入硬件后用芯片 ID、启动日志和板卡铭牌再次确认，不能只根据目录名称判断。

## 4. 现有 YOLOv8 C1200 路线

### 4.1 模型侧适配

样例声明基于 Ultralytics，并针对 NPU 做了两项关键调整：

1. `train/ultralytics/cfg/models/v8/yolov8.yaml` 设置 `activation: nn.ReLU()`，覆盖普通 Conv 的默认激活，目标是避免 SiLU 带来的编译和硬件支持问题。
2. 模型导出时不把检测后处理放入 ONNX；解码、DFL、置信度处理和 NMS 留在主机/板端代码中。

样例包中的 `ultralytics.egg-info/PKG-INFO` 标记版本为 8.1.34。`train/yolov8.py` 通过修改后的 Ultralytics API 完成训练、ONNX 导出、验证和校准数据生成。训练环境文件固定了较早版本的 ONNX/ONNX Runtime/onnx-simplifier 依赖，后续复现时需要严格记录 Python、PyTorch、ONNX opset 和导出日志。

### 4.2 C1200 编译配置

`train/compiler/main_c1200.yaml` 的核心配置是：

- `device_engine: C1200`
- `model_path: ./yolov8s.onnx`
- `net_batch_size: 1`
- `orig_model_format: onnx`
- 输入预处理：BGR→RGB，均值 0，缩放 1/256
- 输出布局：`DWH`
- 编译阶段：预处理、图优化、模型固化、量化、图切分、绑定、代码生成、代码编译、硬件测试、Userland
- `enable_static_dag: true`

量化方法在 Userland 配置中记录为 `weight_perchannel_kl_easy_quant`。这是一条固定 batch、固定图和 INT8 校准驱动的流水线。配置文件包含 `/CE-data1/...`、`/media/shining/...` 等原始机器路径，不能直接当作当前主机可执行配置；在真正编译前必须改成容器内可见的绝对路径。

### 4.3 已生成的 C1200 模型包

`Deploy/prebuild/main_c1200_yolov8s/model/yolov8s.meta` 提供了比 README 更可靠的张量证据：

- 输入：1 个 `RGB888P`、`UINT8`、`WHD` 张量，尺寸 640×640×3×1。
- 输出：6 个 `INT8`、`DWH` 张量，3 个检测尺度各有回归 64 通道和分类 80 通道。
- 输出尺寸分别对应 80×80、40×40、20×20 三个尺度。
- 80 通道输出的 pitch 是 128，而实际通道数是 80，说明板端后处理必须遵守 tensor 属性中的 pitch，不能假定数据是紧凑排列。
- 元数据 `size` 字段记录 activation 约 15.85 MB、weight 约 11.23 MB、DAG 约 3.9 KB（这是模型包元数据中的尺寸字段，实际运行峰值仍需板端测量）。
- 模型运行依赖 `.lib`、`.meta`、`weights.bin`、加密权重/辅助文件和 `pil_test.rbf` 等专有产物，不能只复制 ONNX 到板端。

板端 C++ 程序通过 BSNN API 查询输入/输出 tensor 属性，读取 INT8 输出、按量化 scale 反量化，在 CPU 侧做 DFL、边框解码、Soft-NMS 和绘图。这说明 C1200 现有成功案例实际上是 “NPU 主干 + CPU 后处理”，而不是整条 YOLO 图完全在 NPU 内执行。

### 4.4 运行时和构建

`Deploy/code/CMakeLists.txt` 使用 C++17、OpenCV、`libbsnn.so`、pthread 和 `stdc++fs`。`Deploy/code/usr/` 已随资料提供 BSNN 头文件和 `libbsnn.so` 6.1.1 的多个链接名；`Deploy/code/build.sh` 和 `build_install.sh` 支持 `-C1200`、`-A1000` 和自定义 runtime 目录。

需要注意两个脚本行为：

- `build.sh` 的帮助文字说默认 C1200，但代码初始 `DEVICE` 为空；不显式传 `-C1200` 时会回退到源码目录作为 runtime 目录。研究时应显式指定 `-C1200` 和实际 runtime。
- `prebuild/start_yolov8s.sh` 在未指定设备时先检查 A1000 目录，再检查 C1200 目录。如果两个目录都存在，自动检测会选择 A1000。C1200 测试必须显式使用 `-C1200`。

### 4.5 版本证据和不一致

资料存在需要在实机前确认的版本差异：

- `runtime_envs.md` 声明 BSNN 6.1.1、BSTN 3.1.0、DSP RTOS 5.2.2、`bsnn_tools_v7.0.0`。
- 目录中的 Docker 归档命名为 `bsnn_tools_docker_8.3.1.tar.gz`。
- C1200 模型元数据里的 `net_fw_version` 是另一串 commit/hash 标识。

这些信息不能简单视为同一版本。应在 Docker 中执行 `bstnnx_run --help`/版本查询，在板端读取 `libbsnn.so` 和系统日志版本，并记录实际编译器、Net-FW、Runtime、固件的组合。

## 5. 对 SmolVLM 适配的判断

### 5.1 不能直接复用 YOLO 的原因

YOLO 样例是单输入、固定空间尺寸、单 batch、以卷积为主、输出数量固定的检测网络。SmolVLM 至少包含以下逻辑：

```text
图像预处理
  -> Vision Encoder
  -> 多模态投影 / image token 拼接
  -> Token Embedding + Language Decoder
  -> 自回归采样 / KV Cache
  -> Tokenizer 解码
```

资料只证明 BSTNNX 能处理一个已经适配过的 YOLO ONNX，并没有证明以下能力在 C1200/当前编译器上可用：动态序列长度、循环或逐 token 解码、KV-cache 的读写、RoPE、Multi-Head/GQA Attention、Gather/Scatter、Embedding、LayerNorm 的 NPU 路径、混合精度、FP16/BF16、动态 batch、语言模型的采样和停止条件。

### 5.2 推荐的分段部署路线

建议按风险从低到高推进：

1. **先验证视觉编码器**：导出固定图像尺寸、固定 batch 的视觉 encoder ONNX；先做主机 ONNX Runtime 浮点输出对齐，再尝试 ReLU/算子替换、静态 shape 和 INT8 编译。
2. **再验证投影层**：把视觉特征到语言 embedding 的 projector 单独导出，确认 MatMul/Linear、Reshape、Transpose、LayerNorm 等算子是否能编译；若不能，放在 CPU/GPU。
3. **语言解码器先走 CPU/GPU baseline**：在板端先确认 tokenizer、prompt 拼接、单步解码和输出质量，再考虑把可静态化的 decoder block 编译到 NPU。
4. **最后评估混合方案**：视觉 encoder 在 NPU，projector/decoder 在 CPU/GPU；或将 decoder 做成固定最大序列长度、固定一步输入的 NPU 子图，并由 CPU 管理 KV-cache 和循环。

这条路线可以先回答“视觉理解是否能利用 C1200 NPU”，再回答“完整 VLM 是否能在板端闭环”，避免一开始把模型、算子、量化和自回归运行时问题混在一起。

### 5.3 主要技术风险

| 风险 | 资料支持的判断 | 需要实测的内容 |
|---|---|---|
| 算子覆盖 | YOLO 只展示了卷积检测图；Userland 配置额外出现 LayerNorm/Softmax 转 DSP 选项 | Vision Transformer 的 Attention、RoPE、GELU/SiLU、Gather/Scatter、Embedding、LayerNorm 是否成功编译 |
| 动态 shape | YOLO 全部是 batch=1、640×640、静态 DAG | 图像尺寸、token 长度、decoder 单 token 输入、KV-cache shape 是否允许静态化 |
| 精度 | C1200 资料强调 INT8；YOLO 有校准和 per-channel 权重量化 | 视觉 encoder 和语言 decoder 的 INT8 敏感层、混合精度或 CPU fallback 能否使用 |
| 内存 | 模组有 16 GB LPDDR5，但模型包的实际内存由 activation、权重、缓存和系统占用决定 | SmolVLM 具体变体的权重、KV-cache、峰值 activation 和并发占用 |
| 运行时 | 现有 C++ 示例只覆盖 BSNN tensor I/O 和 YOLO 后处理 | 是否有适用于多输入、多输出、循环解码、缓存复用的 BSNN API 示例 |
| 版本 | 文档 v7.0.0、归档 v8.3.1、Runtime 6.1.1 信息并存 | 实际 Docker、Net-FW、Runtime、固件是否兼容 |

### 5.4 当前最重要的缺口

在开始修改代码前，还缺少以下确定性输入：

- 目标 SmolVLM 具体版本和规模，例如 256M/500M 或其他变体。
- 原始权重格式、Tokenizer、Processor、配置文件和许可证。
- 目标任务：图像问答、描述生成、视觉定位，还是仅做视觉编码/分类。
- 期望输入分辨率、最大 prompt 长度、最大输出 token 数、目标延迟/FPS。
- C1200 实机芯片 ID、系统镜像/固件版本、可用 `/userdata` 空间和实际 CPU/GPU/NPU 运行状态。
- 当前 BSTNNX 编译器的算子支持日志、失败模型的 compiler log，以及是否支持 CPU fallback 或多子图模型。

## 6. 下一步验证计划

### 阶段 A：建立可复现基线

1. 在不修改原资料的前提下确认 Docker 镜像实际 tag 和 `bstnnx_run` 版本。
2. 固定一份校准集、C1200 编译 YAML 和工具版本记录。
3. 在实机上显式运行 C1200 YOLO 样例，记录模型加载、推理、后处理和多次运行性能。
4. 用 `yolov8s.meta` 校验输入尺寸、输出顺序、pitch 和量化 scale，形成板端接口基线。

### 阶段 B：SmolVLM 主机侧拆图

1. 确定 SmolVLM 变体和完整 processor 行为。
2. 分别导出 Vision Encoder、projector、decoder 单步三个 ONNX/参考模型。
3. 生成每个子图的输入/输出、静态 shape、算子清单和浮点参考结果。
4. 先在 x86 主机上完成 PyTorch 与 ONNX Runtime 数值对齐，再进入 BSTNNX。

### 阶段 C：C1200 编译矩阵

对每个子图依次测试：原始 ONNX、静态 shape、替换激活、关闭模型内后处理、FP/INT8 可用性、LayerNorm/Softmax DSP 选项和不同 opset。每次测试保留 compiler log、失败节点、产物大小和输出误差。

### 阶段 D：板端混合运行

先实现“视觉 encoder NPU + decoder CPU/GPU”的闭环，再测端到端延迟、内存和输出质量；只有在混合方案稳定后，才评估 decoder block 或 KV-cache 的进一步 NPU 化。

## 7. 资料中的操作说明与本次请求的边界

资料中包含串口登录、SSH 默认凭据、ADB 推送、刷机、Docker 启动、编译、模型运行等操作步骤。这些内容是厂商/项目资料中的说明，属于后续实验的候选操作；本次用户请求是“先基于资料进行详细分析”。本轮没有执行刷机、重启设备、远程登录、文件推送、Docker 加载或模型编译，也没有把资料中的默认密码、IP 或路径当作当前设备事实。

## 8. 证据索引

- 硬件规格：`compile_develop/硬件资料/Aura(NX-5000)产品参数及接口说明.pdf`
- 系统和工具链说明：`compile_develop/第一部分 了解实验环境与软件工具.pdf`
- 载板电源/接口：`compile_develop/硬件资料/原理图/RC12_CARRIER_V010_260812.pdf`
- 外设扩展：`compile_develop/硬件资料/原理图/RC12_EXT_V010_20260812_2.pdf`
- YOLO 总览：`compile_develop/yolov8_bst_sample/use.md`
- 版本声明：`compile_develop/yolov8_bst_sample/runtime_envs.md`
- C1200 编译配置：`compile_develop/yolov8_bst_sample/train/compiler/main_c1200.yaml`
- C1200 Userland 配置：`compile_develop/yolov8_bst_sample/Deploy/prebuild/main_c1200_yolov8s/input_config.yaml`
- C1200 模型元数据：`compile_develop/yolov8_bst_sample/Deploy/prebuild/main_c1200_yolov8s/model/yolov8s.meta`
- 板端部署说明：`compile_develop/yolov8_bst_sample/Deploy/readme.md`、`Deploy/prebuild/README.md`
- 板端入口：`compile_develop/yolov8_bst_sample/Deploy/code/src/main.cpp`
- 编译和链接：`compile_develop/yolov8_bst_sample/Deploy/code/CMakeLists.txt`、`Deploy/code/build.sh`

## 9. 实机核验记录（192.168.1.100）

本轮通过 SSH 连接 `bst@192.168.1.100`，未刷机、未重启、未修改设备树或跨域配置。实机信息如下：

- 主机名：`c1200-ubuntu`
- 系统：Ubuntu 22.04.5，Linux 6.1.64-rt17，aarch64
- 设备树兼容串：`bst,c1200`
- 设备树型号：`BST C1200 CDCU RC12`
- CPU：8 核 Cortex-A78AE
- 内存：约 12 GiB 可见于当前 Linux 域，空闲约 11 GiB
- 已加载驱动：`bstnpu`、`bstn_driver`、`bst_lwnn_driver`、`bst_hwcv`、`system_heap_ipc`
- NPU/DSP 相关设备节点：`/dev/bstn0`、`/dev/bstdsp`、`/dev/bst_lwnn`、`/dev/bstipc0d`、`/dev/ipc_msgbox0`
- 内核接口：`/sys/kernel/bstn`、`/sys/kernel/bst_lwnn`、`/sys/kernel/bst-ipc`
- 当前可见网络：`eth3` 为 `192.168.1.100/24`；`can0`–`can3` 存在但当前均为 DOWN/STOPPED。

### 9.1 YOLO C1200 基线实测

已将资料中的 `Deploy/prebuild` 推送到 `/userdata/prebuild`，使用：

```bash
./app/yolov8 \
  -i ./datasets/000000000139.jpg \
  -m ./main_c1200_yolov8s/model \
  -o ./output_C1200/inferresult_test.jpg \
  -n 3 -v
```

结果：

- BSNN 模型成功加载，识别到 1 个输入张量和 6 个输出张量。
- 输入为 640×640×3 `UINT8 WHD`，输出为 3 个尺度的回归/分类张量。
- 3 次运行均检测到 15 个目标。
- 预处理平均 1.00 ms。
- NPU 推理平均 5.33 ms，最快 4 ms，最慢 8 ms。
- 后处理平均 1.00 ms。
- 程序统计的总耗时平均 7.34 ms，平均 FPS 136.30。

结果图已回传到工作区：[c1200_yolo_inferresult_test.jpg](../output/c1200_yolo_inferresult_test.jpg)。该结果证明当前板端的 C1200 BSNN Runtime、模型包、OpenCV 和资料中的 YOLO C++ 应用可以闭环运行；它还没有证明 SmolVLM 的算子和自回归运行时可用。

### 9.2 其他域的实际控制面

资料中的多域架构说明了职责划分，但没有给出一个面向应用开发者的“域控制 SDK”或控制协议。实机进一步看到的控制面可以分为四类：

1. **AI/NPU/DSP 域**：通过 BSNN/LWNN 用户库和 `/dev/bstn0`、`/dev/bstdsp`、`/dev/bst_lwnn` 使用；YOLO 已验证这条路径。`/sys/kernel/bstn/*` 主要是调试、加载和 profiling 状态接口，不应作为普通业务控制 API。
2. **Safety/Realtime/MCU 域**：设备树存在 `amba_apu@0/safety`，其兼容串为 `bst,safety_command`，状态为 `okay`，并有 `shmem` 子节点；内核还存在 `bst-ipc`、`ipc_msgbox0`、`/dev/bstipc0d` 和 `system_heap_ipc`。这些证据说明存在共享内存/消息盒子跨域通道，但资料没有提供消息结构、命令 ID、权限模型或用户态封装，因此目前不能安全地直接向 Safety/Realtime 域发控制命令。
3. **Switch/以太网域**：板端提供 `switch1_xpcs_tool`，可以读写 XPCS 间接寄存器、查询 link status 和 PCS mode；帮助信息明确它是 CANOE service 4 的工具。当前 `bst` 用户执行只读 status 时返回 `Permission denied`，说明它需要更高权限或专用服务授权，不能把它当作普通用户接口。
4. **CAN/外设路径**：系统存在 `can0`–`can3` 和 SocketCAN 工具（`cansend`、`candump`、`canfdtest` 等），内核还有 `adas2can`/`can2adas` 队列状态接口；但当前 CAN 接口全部停止，且资料没有提供 RC12 的 CAN ID、信号定义、波特率或 Safety 命令协议。连接真实执行器前必须取得 CAN 数据库/协议和安全约束。

系统还提供了 `/usr/local/bin/hwcv-cross-domain-on`、`hwcv-cross-domain-off` 和 `isp-use-ipc`。这些脚本不是运行时消息 API，而是修改启动分区上的 DTB/overlay：`hwcv-cross-domain-on` 会启用 HWCV cross-domain、修改 DMA ranges、禁用 `bstn`/`bst_lwnn` 节点并要求重启；`off` 执行反向修改；`isp-use-ipc` 修改 ISP overlay 使其走 IPC。它们会改变启动配置，当前研究阶段不应直接执行。

### 9.3 资料是否说明了其他域的利用方式

资料明确说明了以下内容：

- Safety 域负责信任根、SMMU 初始化和 SCMI 资源管理。
- Realtime 域负责底盘/执行器等确定性实时任务。
- Switch 域负责 TSN 以太网和 CAN 通信。
- Secure 域负责安全启动、加密和鉴权。
- AI 主域负责 Ubuntu/Linux、NPU 推理、SLAM 和目标检测。
- 刷机工具可以把各域固件、DTB 和系统镜像写入 UFS；这属于系统启动/固件管理，不是应用层调用。

资料没有提供以下关键内容：

- AI 主域到 Safety/Realtime/MCU 的消息格式和调用示例。
- Safety command 的命令表、共享内存布局和同步机制。
- Realtime 域任务加载、停止、状态读取或调试接口。
- Switch 域和 CAN 的具体业务报文定义。
- Secure 域的应用级鉴权 API。

因此，当前能够直接利用的只有 Linux 用户态、BSNN/LWNN、ISP/HWCV、SocketCAN 和已有系统工具；其他域只能通过已有的内核驱动/系统服务或厂商提供的协议继续补齐，不能根据架构图自行猜测寄存器或消息格式。
