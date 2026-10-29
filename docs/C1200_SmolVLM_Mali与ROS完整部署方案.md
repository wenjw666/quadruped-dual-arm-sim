# C1200 上 SmolVLM、Mali GPU 与 ROS2 完整部署方案

## 1. 当前实机能力

当前 `BST C1200 CDCU RC12` 已确认具备：

- NPU：`/dev/bstn0`，BSNN Runtime，YOLO C1200 已实测运行；
- DSP/LWNN：`/dev/bstdsp`、`/dev/bst_lwnn`，4 路 LWNN DSP 固件；
- GPU：`/dev/mali0`、`/dev/dri/card0`；
- GPU 用户态库：`/usr/bst/lib/libmali.so`、`libEGL_mali.so`、Vulkan、OpenCL、GLES；
- 图像链路：`bst-isp`、GStreamer `ispsrc`、V4L2、HWCV；
- ROS2 Humble：`/opt/ros/humble`，加载 setup 后 `rclpy` 可用；
- OpenCV 4.11.0；
- 当前没有板端 PyTorch、Transformers、ONNX Runtime 或 `pyrealsense2`。

这意味着板端适合采用“BSNN NPU + C++ + GPU 原生算子 + ROS2”的部署方式。完整 Python Transformers 方案不应作为第一条路径。

## 2. 相机数据为什么有的经过 ISP，有的不经过

是否经过 ISP 由相机物理接口和 Linux 驱动路径决定，不是由 SmolVLM 决定。

### GMSL/CSI/ISP 路径

```text
GMSL Camera
  -> Serializer
  -> Coax / shielded cable
  -> MAX96712 Deserializer
  -> MIPI CSI receiver
  -> ISP firmware
  -> V4L2 / GStreamer ispsrc
  -> NV12/I420 DMABUF
  -> preprocess
  -> SmolVLM
```

这条路径使用 RC12 的 MAX96712、CSI、ISP 和 P12V/POC。ISP 负责或参与 Bayer/RAW 处理、去马赛克、曝光相关处理、颜色转换、缩放、裁剪和输出格式转换。当前 GStreamer `ispsrc` 支持 I420/NV12，默认 `memtype=4`，也就是 DMABUF 路径，适合减少 CPU 拷贝。

### USB RealSense/UVC 路径

```text
RealSense / UVC Camera
  -> USB 3.x xHCI
  -> USB driver
  -> V4L2/HID/librealsense
  -> user-space RGB/Depth/IR buffers
  -> GPU/CPU preprocess
  -> SmolVLM
```

这条路径不经过 MAX96712、MIPI CSI 或 RC12 ISP。原因是图像已经由 USB 设备输出为 USB 视频流；主机只需要接收、解码/转换和处理。RealSense 的深度、IR 和 IMU 还可能通过额外的 USB/HID endpoint 传输。

两条路径的差异：

| 项目 | GMSL/ISP | RealSense/USB |
|---|---|---|
| 物理链路 | 同轴/SerDes/MIPI CSI | USB 3.x |
| 主要驱动 | MAX96712、CSI、ISP、V4L2 | xHCI、UVC/HID、librealsense |
| 缓冲 | 可走 ISP DMABUF | 通常先进入用户态 buffer |
| 处理位置 | C1200 ISP + 硬件图像链路 | 相机内部 + CPU/GPU 用户态 |
| 适合场景 | 多相机、低延迟、量产 | 快速验证、RGB/Depth/IMU 一体 |
| 当前板端状态 | 驱动已在，但 GMSL 链路未连接 | USB 主机可用，librealsense 尚未安装 |

## 3. Mali GPU 应该怎么用

Mali GPU 不是 BSNN NPU 的替代品。它适合图形和通用并行计算，当前板端已经提供 Mali 设备节点和用户态库，但没有现成的 PyTorch/Transformers GPU 后端。

### 3.1 最适合的 GPU 工作

第一优先级是图像预处理：

- NV12/I420 → RGB；
- resize、crop、letterbox；
- mean/scale normalize；
- 多路相机颜色空间转换；
- 将 RealSense RGB/Depth 对齐或转换为模型输入。

第二优先级是轻量模型子图：

- 视觉 projector 的 Linear/MatMul；
- 语言 Decoder 中可被固定 shape 化的部分；
- attention 的矩阵乘法和 elementwise kernel。

第三优先级是显示和可视化：

- EGL/GLES/Vulkan 图像显示；
- 结果叠加和调试 UI。

### 3.2 当前可用的软件入口

实机有：

- `/dev/mali0`；
- `/usr/bst/lib/libmali.so`；
- `libEGL_mali.so`；
- `libOpenCL.so`；
- Vulkan 和 GLES 库；
- `/dev/dri/card0`；
- GStreamer 基础视频插件。

但当前没有：

- PyTorch ARM GPU 构建；
- Transformers；
- ONNX Runtime GPU 后端；
- 一套已经验证的 SmolVLM OpenCL/Vulkan kernel；
- GStreamer `glupload` 插件。

因此 Mali 的第一版使用方式应是写小型 C++ OpenCL/Vulkan/EGL 预处理模块，而不是把完整语言模型直接搬到 GPU。每个 kernel 都要用 CPU 参考实现校验输出和耗时。

## 4. SmolVLM 完整部署架构

建议的最终数据流：

```text
Camera Input
  -> Camera Adapter
  -> ISP/HWCV or GPU Preprocess
  -> C1200 NPU Vision Encoder
  -> Projector on CPU/GPU or NPU subgraph
  -> Language Decoder on CPU/GPU initially
  -> Tokenizer / stopping / safety filter
  -> ROS2 VLM Result
  -> Behavior Planner
  -> Safety Gateway
  -> CAN/UART/Realtime domain
```

必须把 Vision Encoder 和 Language Decoder 分开部署：

### 阶段 A：主机参考环境

在 x86 主机建立独立环境，锁定：

- SmolVLM 具体 checkpoint 和 commit；
- Transformers、PyTorch、Tokenizer、Processor；
- 输入图像尺寸和 patch/grid 规则；
- prompt 格式、最大输入 token、最大输出 token；
- PyTorch reference 输出；
- ONNX Runtime reference 输出；
- 一组固定图片和问题集合。

### 阶段 B：导出三个子图

分别导出：

1. Vision Encoder：固定 image shape、batch=1；
2. Projector：固定视觉 token 数和 hidden size；
3. Decoder step：固定一步 token 输入、past KV-cache shape 和输出 logits。

不要一开始导出包含动态循环、采样和 tokenizer 的完整 Python 图。

### 阶段 C：C1200 编译矩阵

对 Vision Encoder 先测试：

- 原始 ONNX；
- 固定 shape ONNX；
- FP32/FP16/INT8 可行性；
- LayerNorm、GELU/SiLU、RoPE、MatMul、Transpose、Reshape、Gather；
- 是否需要将 Softmax/LayerNorm 迁移到 DSP 或 CPU；
- 编译失败节点和 fallback 边界。

### 阶段 D：板端混合运行

第一版推荐：

```text
Vision Encoder -> C1200 NPU
Projector -> CPU 或 Mali OpenCL
Decoder -> CPU，必要时 Mali OpenCL/Vulkan
Tokenizer -> CPU
ROS2 orchestration -> CPU
```

只有在端到端结果正确后，才把 projector 或 decoder block 逐步放到 NPU。这样能把模型算子问题、量化问题和自回归控制问题分开定位。

### 阶段 E：性能优化顺序

1. 消除相机帧的重复拷贝；
2. ISP/HWCV/GPU 完成 resize 和颜色转换；
3. Vision Encoder 使用 NPU；
4. tokenizer 和 decoder 使用固定内存池；
5. KV-cache 复用；
6. 限制最大输出 token；
7. 最后再评估 decoder 的 GPU/NPU 化。

## 5. ROS2 在嵌入式设备上的部署方案

板端已经安装 ROS2 Humble，加载 `/opt/ros/humble/setup.bash` 后 `rclpy` 可用。但当前默认 shell 没有自动加载 ROS 环境，后续应建立 `/userdata/ros_ws` 或 `/home/bst/ros_ws`，并通过 systemd 或启动脚本加载环境。

### 5.1 推荐节点划分

```text
camera_gmsl_node / camera_realsense_node
        |
        v
image_preprocess_node
        |
        v
smolvlm_vision_node
        |
        +--> vlm_decoder_node
        |
        v
perception_fusion_node
        |
        v
behavior_planner_node
        |
        v
safety_gateway_node
        |
        v
can_gateway / uart_gateway / realtime interface
```

建议节点职责：

- `camera_gmsl_node`：使用 `ispsrc` 或 V4L2，输出 NV12/RGB 图像和 camera_info；
- `camera_realsense_node`：封装 librealsense，初期只发布 RGB，后续再加入 depth/IMU；
- `image_preprocess_node`：CPU/OpenCL/HWCV 做 resize、颜色转换和 normalize；
- `smolvlm_vision_node`：调用 BSNN C++ API 执行 Vision Encoder；
- `vlm_decoder_node`：维护 tokenizer、prompt、KV-cache 和逐 token 解码；
- `perception_fusion_node`：融合 YOLO、SmolVLM、深度、IMU 和机器人状态；
- `behavior_planner_node`：将语义结果转为受约束的行为意图；
- `safety_gateway_node`：限速、超时、碰撞区、急停和权限检查；
- `can_gateway`/`uart_gateway`：只通过确认过的 CAN/串口协议连接真实设备。

SmolVLM 不应直接发布执行器命令。它只能产生语义结果或高层意图，真实动作必须经过 planner 和 safety gateway。

### 5.2 推荐 ROS2 主题

```text
/camera/rgb/image_raw
/camera/camera_info
/camera/depth/image_raw              # RealSense 后续启用
/camera/imu                           # RealSense 后续启用
/smolvlm/query
/smolvlm/response
/perception/objects
/perception/scene_description
/behavior/intent
/cmd_vel_safe
/joint_states
/diagnostics
```

传感器图像建议使用 `KEEP_LAST=1`、低队列深度和 sensor-data QoS，避免延迟累积。ROS2 图像尽量在同一进程组合节点内传输，使用 loaned message 或共享内存；不要把高分辨率原始图像通过普通 DDS 网络反复复制。

### 5.3 真实硬件接入边界

真实执行器接入前必须先完成：

- CAN ID/DBC 或 UART 协议确认；
- Safety/Realtime 域控制协议确认；
- watchdog 和 heartbeat；
- 指令超时归零；
- 急停和限位；
- 仿真与实机 topic/接口一致性测试。

现有资料没有给出 Safety command 的消息结构，也没有给出执行器 CAN 协议，因此当前 ROS2 可以先搭建图像、NPU 推理、语义和仿真接口，不能直接猜测真实执行器命令。

## 6. 分阶段实施计划

### P0：环境和版本锁定

- 锁定 SmolVLM checkpoint、Processor、Tokenizer；
- 锁定 BSTNNX Docker、Net-FW、BSNN Runtime、DSP firmware；
- 在主机生成 reference outputs；
- 在板端确认 ROS2 Humble、Mali/OpenCL/Vulkan 和 GStreamer 环境。

### P1：离线 Vision Encoder

- 导出固定 shape ONNX；
- 主机 ONNX Runtime 对齐；
- BSTNNX C1200 编译；
- 板端 BSNN C++ 推理；
- 与 reference 特征做误差比较。

### P2：相机输入

- 先用图片文件；
- 再用 USB UVC/RealSense RGB；
- 最后接 GMSL Camera 0；
- 验证 ISP/HWCV/DMABUF/GPU 预处理。

### P3：Projector 和 Decoder

- Projector CPU baseline；
- Decoder CPU baseline；
- KV-cache 和 token loop 正确性测试；
- Mali/OpenCL 优化；
- 评估 NPU 固定 step 子图。

### P4：ROS2 集成

- 相机节点；
- 预处理节点；
- Vision Encoder 节点；
- Decoder 节点；
- perception/planner/safety gateway；
- 仿真和实机接口统一。

### P5：真实设备闭环

- 先只读传感器；
- 再低风险执行器；
- 最后接入 Safety/Realtime/CAN 控制；
- 全程保留 watchdog、急停和回滚机制。

## 7. 当前结论

现在可以立即开始 SmolVLM 的完整工程化准备，但第一版“完整部署”应定义为：Vision Encoder 在 C1200 NPU，Projector/Decoder 在 CPU 或 Mali GPU，ROS2 负责数据流和任务编排。将完整 Decoder 一次性编译到 NPU 需要后续算子、KV-cache 和动态 shape 实测，不能作为第一阶段前置条件。
