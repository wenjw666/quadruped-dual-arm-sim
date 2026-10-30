#!/usr/bin/env bash
set -e

ROOT=/userdata/smolvlm
BACKEND="${C1200_SMOLVLM_BACKEND:-cpu}"
if [[ "$BACKEND" == "vulkan" ]]; then
  BIN="$ROOT/build-vulkan/bin"
  PORT="${C1200_SMOLVLM_PORT:-8089}"
  export LD_LIBRARY_PATH="/usr/bst/lib:${LD_LIBRARY_PATH:-}"
  export VK_ICD_FILENAMES="${VK_ICD_FILENAMES:-$ROOT/c1200_mali_fixed.json}"
  GPU_ARGS=(--device Vulkan0 -ngl 99 --mmproj-offload)
else
  BIN="$ROOT/build/bin"
  PORT="${C1200_SMOLVLM_PORT:-8088}"
  GPU_ARGS=(--no-mmproj-offload)
fi
MODEL="$ROOT/models/SmolVLM-500M-Instruct-Q8_0.gguf"
MMPROJ="$ROOT/models/mmproj-SmolVLM-500M-Instruct-Q8_0.gguf"
WARMUP_IMAGE="${C1200_SMOLVLM_WARMUP_IMAGE:-$ROOT/test_mujoco_overview.jpg}"
WARMUP_ENABLED="${C1200_SMOLVLM_WARMUP:-1}"

. /opt/ros/humble/setup.bash
. "$ROOT/ros_ws/install/setup.bash"
mkdir -p "$ROOT/logs"

if ! pgrep -f "$BIN/llama-server.*SmolVLM-500M-Instruct-Q8_0.gguf" >/dev/null; then
  export LD_LIBRARY_PATH="$BIN:${LD_LIBRARY_PATH:-}"
  nohup "$BIN/llama-server" \
    -m "$MODEL" --mmproj "$MMPROJ" \
    --host 127.0.0.1 --port "$PORT" -t 8 -c 2048 --parallel 1 \
    "${GPU_ARGS[@]}" \
    >"$ROOT/logs/llama-server.log" 2>&1 </dev/null &
fi

for _ in $(seq 1 60); do
  if curl -fsS "http://127.0.0.1:$PORT/health" >/dev/null 2>&1; then
    break
  fi
  sleep 1
done
curl -fsS "http://127.0.0.1:$PORT/health" >/dev/null

# The built-in llama.cpp empty warmup does not execute the multimodal image
# projector.  Run one real image request so the first ROS request does not pay
# the 40–50 second Vulkan shader/projector initialization cost.  The helper is
# optional so older board installs remain compatible.
WARMUP_HELPER="$ROOT/c1200_smolvlm_warmup.py"
if [[ "$WARMUP_ENABLED" != "0" && -f "$WARMUP_HELPER" && -f "$WARMUP_IMAGE" ]]; then
  python3 "$WARMUP_HELPER" "$WARMUP_IMAGE" \
    --url "http://127.0.0.1:$PORT/v1/chat/completions" \
    >"$ROOT/logs/llama-warmup.log" 2>&1 || true
fi

if ! pgrep -f "c1200_smolvlm/lib/c1200_smolvlm/vlm_node" >/dev/null; then
  nohup ros2 run c1200_smolvlm vlm_node \
    --ros-args -p "server_url:=http://127.0.0.1:$PORT/v1/chat/completions" \
    >"$ROOT/logs/vlm_node.log" 2>&1 </dev/null &
fi

echo "C1200_SMOLVLM_READY backend=$BACKEND server=127.0.0.1:$PORT ros_node=/c1200_smolvlm"
