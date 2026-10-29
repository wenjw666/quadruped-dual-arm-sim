# C1200 SmolVLM ROS2 bridge

This package connects a C1200 camera stream to the existing SmolVLM GGUF and
mmproj assets through a board-native ARM64 llama.cpp runner. The default path
uses a persistent local `llama-server`; the CLI runner remains available as a
diagnostic fallback. It is an event-triggered semantic path, not a real-time
controller.

## Topics

- `/smolvlm/image` (`sensor_msgs/Image`): canonical RGB input.
- `/smolvlm/prompt` (`std_msgs/String`): one natural-language query.
- `/smolvlm/response` (`std_msgs/String`): JSON with `ok`, `text`, latency and frame sequence.
- `/smolvlm/status` (`std_msgs/String`): runner state and errors.

The response is semantic text only. It must go through the existing `qda_brain`
contract and safety/skill validation before any ROS, CAN or UART command.

## Board build

```bash
source /opt/ros/humble/setup.bash
cd /userdata/smolvlm/ros_ws
colcon build --packages-select c1200_smolvlm --symlink-install
source install/setup.bash
```

Use an existing ROS camera first:

```bash
ros2 launch c1200_smolvlm smolvlm_minimal.launch.py camera_source:=topic
ros2 topic pub --once /smolvlm/prompt std_msgs/msg/String "{data: 'Describe the scene and list the visible objects.'}"
```

For a UVC/RealSense color stream use `camera_source:=v4l2` and its color
device. For the C1200 GMSL/ISP path use `camera_source:=gstreamer`
`camera_device:=/dev/video4` after the selected channel is linked.
