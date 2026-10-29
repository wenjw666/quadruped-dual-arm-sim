#!/usr/bin/env bash
set -e

ROOT=/userdata/smolvlm
BIN="$ROOT/build/bin"
MODEL="$ROOT/models/SmolVLM-500M-Instruct-Q8_0.gguf"
MMPROJ="$ROOT/models/mmproj-SmolVLM-500M-Instruct-Q8_0.gguf"

. /opt/ros/humble/setup.bash
. "$ROOT/ros_ws/install/setup.bash"
mkdir -p "$ROOT/logs"

if ! pgrep -f "llama-server.*SmolVLM-500M-Instruct-Q8_0.gguf" >/dev/null; then
  export LD_LIBRARY_PATH="$BIN:${LD_LIBRARY_PATH:-}"
  nohup "$BIN/llama-server" \
    -m "$MODEL" --mmproj "$MMPROJ" --no-mmproj-offload \
    --host 127.0.0.1 --port 8088 -t 8 -c 2048 --parallel 1 \
    >"$ROOT/logs/llama-server.log" 2>&1 </dev/null &
fi

for _ in $(seq 1 60); do
  if curl -fsS http://127.0.0.1:8088/health >/dev/null 2>&1; then
    break
  fi
  sleep 1
done
curl -fsS http://127.0.0.1:8088/health >/dev/null

if ! pgrep -f "c1200_smolvlm/lib/c1200_smolvlm/vlm_node" >/dev/null; then
  nohup ros2 run c1200_smolvlm vlm_node \
    >"$ROOT/logs/vlm_node.log" 2>&1 </dev/null &
fi

echo "C1200_SMOLVLM_READY server=127.0.0.1:8088 ros_node=/c1200_smolvlm"
