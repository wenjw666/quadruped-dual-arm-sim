#!/usr/bin/env bash
# Run in the ROS Humble container mounted at /ws. No cloud account activation.
set -euo pipefail
cd /ws
if [ "$#" -ne 2 ]; then
  echo 'Usage: bash scripts/setup_v2_release.sh /ws/output/qda-v0.2.0-runtime.tar.gz SHA256' >&2
  exit 2
fi
python3 scripts/release_runtime.py verify "$1" --sha256 "$2"
apt-get update
DEBIAN_FRONTEND=noninteractive apt-get install -y ros-humble-moveit-servo ros-humble-nav2-mppi-controller
python3 -m pip install --target /opt/qda-rl-v2 -r docker/requirements-rl-v2.txt
python3 scripts/release_runtime.py install "$1" --sha256 "$2"
set +u
source /opt/ros/humble/setup.bash
colcon build --base-paths src --symlink-install --packages-up-to qda_bringup qda_brain qda_locomotion qda_task qda_perception qda_moveit_config
source /ws/install/setup.bash
PYTHONPATH=/opt/qda-rl-v2:${PYTHONPATH:-} python3 scripts/check_v2_scene_assets.py
bash scripts/setup_v2_brain.sh
