#!/usr/bin/env bash
set -e

. /opt/ros/humble/setup.bash
. /userdata/smolvlm/ros_ws/install/setup.bash
export PYTHONUNBUFFERED=1

nohup ros2 run c1200_smolvlm vlm_node >/userdata/smolvlm/vlm.log 2>&1 </dev/null &
sleep 3
python3 /userdata/smolvlm/c1200_smolvlm_smoke.py /userdata/smolvlm/test.jpg \
  --prompt 'Describe the image briefly and list visible objects.'
rc=$?
pkill -f '/c1200_smolvlm/lib/c1200_smolvlm/vlm_node' 2>/dev/null || true
echo '--- VLM LOG ---'
cat /userdata/smolvlm/vlm.log || true
exit "$rc"
