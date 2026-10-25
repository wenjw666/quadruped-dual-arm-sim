#!/usr/bin/env bash
set -euo pipefail
apt-get update
DEBIAN_FRONTEND=noninteractive apt-get install -y ros-humble-moveit-servo
python3 -m pip install --timeout 300 --target /opt/qda-brain-python -r /ws/config/brain-requirements.lock.txt
python3 /ws/scripts/fetch_v2_brain_assets.py --models-only
python3 /ws/scripts/fetch_v2_brain_assets.py --asset llama-b11878-bin-ubuntu-x64.tar.gz
python3 /ws/scripts/fetch_v2_brain_assets.py --asset xiaozhi-in-rdk.tar.gz
python3 /ws/scripts/prepare_v2_brain.py
set +u
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
cd /ws
colcon build --symlink-install --packages-select qda_interfaces qda_kinematics qda_brain qda_mujoco qda_bringup
