from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("camera_source", default_value="topic"),
        DeclareLaunchArgument("camera_device", default_value="/dev/video4"),
        DeclareLaunchArgument("input_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("runner", default_value="/userdata/smolvlm/build/bin/llama-mtmd-cli"),
        DeclareLaunchArgument("backend", default_value="server"),
        DeclareLaunchArgument("server_url", default_value="http://127.0.0.1:8088/v1/chat/completions"),
        DeclareLaunchArgument("model", default_value="/userdata/smolvlm/models/SmolVLM-500M-Instruct-Q8_0.gguf"),
        DeclareLaunchArgument("mmproj", default_value="/userdata/smolvlm/models/mmproj-SmolVLM-500M-Instruct-Q8_0.gguf"),
        Node(
            package="c1200_smolvlm", executable="camera_node", name="c1200_smolvlm_camera",
            output="screen", parameters=[{
                "source": LaunchConfiguration("camera_source"),
                "device": LaunchConfiguration("camera_device"),
                "input_topic": LaunchConfiguration("input_topic"),
                "output_topic": "/smolvlm/image",
            }],
        ),
        Node(
            package="c1200_smolvlm", executable="vlm_node", name="c1200_smolvlm",
            output="screen", parameters=[{
                "runner": LaunchConfiguration("runner"),
                "backend": LaunchConfiguration("backend"),
                "server_url": LaunchConfiguration("server_url"),
                "model": LaunchConfiguration("model"),
                "mmproj": LaunchConfiguration("mmproj"),
            }],
        ),
    ])
