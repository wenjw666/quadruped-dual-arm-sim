# C1200 SmolVLM 板端部署记录

本记录对应本轮在 `bst@192.168.1.100` 上完成的实际部署。目标是先把 SmolVLM 的图像理解链路稳定跑通，再把它接到现有 ROS2 语义网关。这里的 SmolVLM 是文本/图像模型，不包含 SmolVLA 的连续动作专家。

## 已部署内容

板端目录：

```text
/userdata/smolvlm/models/SmolVLM-500M-Instruct-Q8_0.gguf
/userdata/smolvlm/models/mmproj-SmolVLM-500M-Instruct-Q8_0.gguf
/userdata/smolvlm/build/bin/llama-server
/userdata/smolvlm/build/bin/llama-mtmd-cli
/userdata/smolvlm/ros_ws/install/c1200_smolvlm
```

模型和 mmproj 来自仓库已有的、经过本机验证的 SmolVLM-500M-Instruct Q8 资产。llama.cpp b11878 在 C1200 上按 ARM64 源码编译，使用板上 Ubuntu 22.04 的 glibc 2.35 和 GCC 11；官方 ARM64 预编译包要求 `GLIBC_2.38`/`GLIBCXX_3.4.32`，因此不能直接使用，也没有替换系统库。

常驻服务：

```bash
cd /userdata/smolvlm/build/bin
export LD_LIBRARY_PATH=$PWD:$LD_LIBRARY_PATH
./llama-server \
  -m /userdata/smolvlm/models/SmolVLM-500M-Instruct-Q8_0.gguf \
  --mmproj /userdata/smolvlm/models/mmproj-SmolVLM-500M-Instruct-Q8_0.gguf \
  --no-mmproj-offload --host 127.0.0.1 --port 8088 \
  -t 8 -c 2048 --parallel 1
```

`/health` 已返回 `{"status":"ok"}`。本轮直接向 OpenAI 兼容接口发送一张测试图和文本，实测一次响应约 5.6 秒；测试图像编码约 4.17 秒，22 个输出 token 约 1.39 秒，约 15 token/s。这是 CPU 参考基线，尚未使用 Mali 或 C1200 NPU。

## ROS2 接口

新增包：`src/c1200_smolvlm`。板上构建方式：

```bash
source /opt/ros/humble/setup.bash
cd /userdata/smolvlm/ros_ws
colcon build --packages-select c1200_smolvlm --symlink-install
source install/setup.bash
```

节点和话题：

| 接口 | 类型 | 用途 |
|---|---|---|
| `/smolvlm/image` | `sensor_msgs/Image` | 统一 RGB 输入 |
| `/smolvlm/prompt` | `std_msgs/String` | 一次自然语言查询 |
| `/smolvlm/response` | `std_msgs/String` | JSON 文本响应、帧序号和延迟 |
| `/smolvlm/status` | `std_msgs/String` | 心跳、图像接收、错误和完成状态 |

`vlm_node` 默认访问 `http://127.0.0.1:8088/v1/chat/completions`，只发布语义文本。它不直接发布关节、CAN 或 UART 指令，后续必须接已有的 `qda_brain` 约束、技能校验和安全门。

启动脚本：

```bash
bash /userdata/smolvlm/c1200_start_smolvlm.sh
```

脚本会检查并启动常驻 `llama-server`，再启动 ROS2 `vlm_node`。当前板端服务已经保持运行，可用下面命令发送查询：

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /smolvlm/prompt std_msgs/msg/String \
  "{data: 'Describe the current scene briefly.'}"
ros2 topic echo /smolvlm/response --once
```

## 相机接入状态

本轮用 ROS2 话题桥接验证了“相机话题 → `/smolvlm/image` → 常驻服务 → ROS 响应”。连续发布测试图像时，VLM 节点报告 `image_seq` 持续增加，随后返回 `backend=llama-server` 的响应。

真实相机仍有两个分支：

1. **GMSL/CSI/ISP**：RC12 的 MAX96712 → CSI → C1200 ISP → `/dev/video4` 等节点。用 `camera_source:=gstreamer` 和 `ispsrc` 接入；当前板上 GMSL 通道仍报告 `not linked`，所以需要先接好供电、同轴线和 serializer，再验证实际画面。生产路径应把 ISP 的 NV12/DMABUF 直接交给原生 C++ 适配器，Python 节点只是调试桥。
2. **USB/RealSense/UVC**：USB 主机 → UVC/HID/librealsense → 用户态 RGB。它不经过 RC12 的 MAX96712、CSI 或 ISP，适合先做 RGB 输入验证。当前板上没有 `pyrealsense2` 和 ROS RealSense 驱动，需要提供 aarch64 的 librealsense/ROS2 组件；在此之前可以用普通 UVC `/dev/videoN` 测试颜色流。

## 板端性能实测

在当前 C1200 上，`llama-server` 使用 8 个线程、CPU-only、`--no-mmproj-offload`：

| 指标 | 实测值 |
|---|---:|
| 常驻 RSS | 约 1.13 GiB |
| VSZ | 约 1.92 GiB |
| 线程数 | 19 |
| 单次完整图像请求 | 约 5.37 秒 |
| 视觉 prompt 阶段 | 约 4.09 秒 |
| 语言生成 | 约 16.85 token/s |
| 请求期间 CPU | 约占 8 个核心的 87–90% |
| 板端内存 | 12 GiB，无 swap |

连续请求的图像缓存命中后，延迟可降到约 1.1–1.5 秒；这不能作为冷启动或不同图像的保证值。当前服务适合事件触发语义理解，仍不适合 20–50 Hz 控制环。

## Mali GPU 缺口

板上存在 `/dev/mali0`、`/dev/dri/card0`、`libmali.so`、Vulkan/OpenCL loader，但当前 llama.cpp 构建结果为 `GGML_VULKAN=OFF`、`GGML_OPENCL=OFF`，`llama-server --list-devices` 没有可用设备，因此本轮没有 GPU 推理。

板端还缺少可直接用于重编译的 Vulkan/OpenCL headers 和完整验证工具（例如 `vulkaninfo`/`clinfo`）。进一步的只读探针发现 `/usr/bst/lib/mali.json` 指向错误的 `/usr/lib/libMali.so`；临时改为实际存在的 `/usr/bst/lib/libmali.so` 后，Vulkan 成功枚举到 1 个 `Mali-G78AE` 物理设备。因此 GPU 硬件和 vendor loader 可以访问，但当前 llama.cpp 仍未编译 Vulkan 后端。另有 `No OPPs found in device tree`，时钟和频率管理还未形成可验收条件。正式启用 GPU 仍需 BSP 修复 ICD、补齐 aarch64 headers、以当前 glibc 重新编译 `GGML_VULKAN=ON`，再逐项验证 `--list-devices`、小矩阵和 SmolVLM mmproj。

- 这是 SmolVLM 语义理解部署，不能当作 SmolVLA 连续动作策略。
- CPU 常驻服务已跑通，但约 5.6 秒/次不满足高频控制；适合事件触发、状态查询、任务语义转换和反馈审查。
- Mali 硬件已能通过临时 ICD 被 Vulkan 枚举，但当前 llama.cpp 构建没有 Vulkan 后端，仍不能解释成 SmolVLM 已完成 GPU 推理。下一步应先修复正式 ICD，再做板端 Vulkan 构建和小模型基准。
- C1200 NPU/BSNN 的 YOLO 路径已验证，SmolVLM 的 Transformer 视觉编码器尚未进入 BSTNNX 编译；应先固定输入、做 CPU/ONNX 数值参考，再逐层尝试 NPU 子图。
- 在真实执行器接入前，需要把文本响应限制为现有 `qda_brain` 合约允许的字段，并通过新鲜度、权限、动作范围、看门狗和急停检查。
