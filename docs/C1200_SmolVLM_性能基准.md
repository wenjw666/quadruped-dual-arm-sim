# C1200 SmolVLM 性能与 GPU 可用性基线

本基线在当前板端常驻 `llama-server`、SmolVLM-500M-Instruct Q8 GGUF/mmproj、CPU-only llama.cpp 配置下采集。测试时间为 2026-10-29。

## 设备与常驻占用

| 项目 | 实测 |
|---|---:|
| CPU | 8 × Cortex-A78AE，最大 1.7 GHz |
| Linux 内存 | 12 GiB，总体可用约 10 GiB |
| Swap | 0 |
| llama-server 常驻 RSS | 约 1.11 GiB |
| llama-server VSZ | 约 1.92 GiB |
| llama-server 线程数 | 19 |
| 空闲 CPU 占用 | 约 9–10%（服务线程/HTTP/后台开销） |

## 推理占用与延迟

板端监控请求期间采样服务进程 `/proc/<pid>/stat` 和 RSS：

| 请求 | 延迟 | 峰值 RSS | 平均 CPU（按 8 核归一） |
|---:|---:|---:|---:|
| 1 | 1.08 s | 1111.7 MB | 89.8% |
| 2 | 1.54 s | 1114.3 MB | 86.7% |
| 3 | 1.16 s | 1116.8 MB | 88.4% |

另一次完整图像描述得到约 5.37 s：视觉 prompt 约 4.09 s，22 个输出 token 约 1.25 s，生成速度约 16.85 token/s。延迟会随图像尺寸、图像编码、prompt 缓存和输出长度变化；1 秒级结果来自缓存较热、短输出条件，不能当成固定实时指标。

在当前服务参数 `--parallel 1` 下，两个并发请求会排队执行：2 并发总墙时约 3.22 s，单请求延迟 1.42/3.01 s；4 并发总墙时约 6.23 s，单请求延迟 1.17/2.78/4.15/6.12 s。并发压测峰值 RSS 约 1134.9 MB（比单请求约多 20 MB），平均 CPU 约 79.6%/84.0%（按 8 核归一）。因此当前实例没有并行吞吐能力，增加请求只会增加队列等待；若要提高吞吐，应先独立评估 `--parallel`、上下文长度和多实例的内存代价。

当前结论是：板端 CPU 能够运行事件触发式 SmolVLM，内存余量充足，但连续高频视觉控制会把约 7 个 CPU 核长时间占满。模型输出适合状态查询、场景描述、任务语义转换和反馈审查，不适合直接放进高频控制环。

## Mali/GPU 检查

板端已经有：

- `/dev/mali0`、`/dev/dri/card0`；
- `libmali.so`、`libEGL_mali.so`、`libvulkan.so`、`libOpenCL.so.1`；
- Mali 内核日志识别为 Mali-G78AE，DDK `r47p0`。

但当前仍不能把这些条件当作可用的 SmolVLM GPU 后端：

- `llama-server --list-devices` 输出 `Available devices: (none)`；
- 当前 llama.cpp 是 CPU-only 构建，没有 `GGML_VULKAN`；
- 板端没有 `vulkaninfo`、`clinfo`；
- 没有 `/usr/include/vulkan` 或 OpenCL headers，无法直接在板上重新编译 Vulkan/OpenCL 后端；
- `dmesg` 显示 Mali 设备没有 OPP 表、没有可用时钟追踪和优先级管理配置。

OpenCL loader 可以加载 `clGetPlatformIDs`，但返回 `-1001 (CL_PLATFORM_NOT_FOUND_KHR)` 且 platform 数为 0；即使显式 `LD_PRELOAD=/usr/bst/lib/libmali.so` 仍然如此。当前镜像没有可用的 OpenCL ICD/platform 注册，不能把 `libOpenCL.so.1` 的存在当成 OpenCL 加速已经可用。

进一步的只读 Vulkan 探针发现，当前失败主要是 ICD 路径问题：`/usr/bst/lib/mali.json` 写入了不存在的 `/usr/lib/libMali.so`。临时复制一份 JSON 并把路径改为实际存在的 `/usr/bst/lib/libmali.so` 后，Vulkan loader 成功创建 instance，并枚举到 1 个物理设备：`Mali-G78AE`，vendor `0x13b5`，设备 API `1.3.269`。这证明 GPU 硬件和 vendor 驱动可以被 Vulkan 访问，正式方案需要由 BSP 修正 ICD 文件或提供等效环境变量配置，不能直接把临时文件当系统修复。

要把 Mali 纳入推理，需要修正正式 ICD、准备与系统 glibc/驱动匹配的 aarch64 Vulkan headers，并用 llama.cpp `GGML_VULKAN=ON` 构建。随后用 `--list-devices` 和小矩阵基准验证，再比较 mmproj offload、CPU 与 Vulkan 的延迟和功耗。仅替换一个预编译 llama.cpp 包会遇到当前系统 glibc 不兼容问题。

在板端实际运行 CMake 配置 `-DGGML_VULKAN=ON` 时，错误为 `Could NOT find Vulkan (missing: Vulkan_LIBRARY Vulkan_INCLUDE_DIR)`；当前用户属于 `video`、`render`、`graphics` 等组，`/dev/mali0` 和 `/dev/dri/card0` 权限可用，所以缺口是开发头文件/导入库与正式 ICD 配置，而不是设备节点权限。

仓库里保留了只读/候选验证工件：`scripts/c1200_vulkan_probe.c` 可在板端不依赖 Vulkan headers 地检查 loader 和物理设备，`scripts/c1200_opencl_probe.c` 可检查 OpenCL platform 数量，`scripts/c1200_mali_fixed.json` 是把 ICD 路径从错误的 `/usr/lib/libMali.so` 指向实际发现的 `/usr/bst/lib/libmali.so` 的候选 JSON。候选 JSON 尚未写入系统，也没有据此宣称 SmolVLM 已完成 GPU 推理；正式验证仍需 BSP/驱动提供方确认 ICD、权限、设备树 OPP 和 llama.cpp Vulkan 构建的组合。

## 对部署方案的影响

1. 第一版板端方案保持 CPU llama-server 常驻，ROS2 只做事件触发和语义结果传递。
2. YOLO、传统视觉和图像预处理可以继续利用 C1200 NPU/ISP/HWCV，避免 SmolVLM 占满 CPU。
3. 仿真主机负责产生相机、状态和任务消息；板端运行相机适配、SmolVLM、感知融合和安全网关，通过 DDS 局域网交互。
4. 需要实时性的控制环继续留在板端的确定性节点中，SmolVLM 只提供低频语义建议。
