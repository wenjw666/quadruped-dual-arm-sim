#!/usr/bin/env bash
set -euo pipefail
assets=/ws/output/v2_brain/assets
runtime=/opt/qda-brain-runtime/llama-b11878
test -f "$assets/SmolVLM-500M-Instruct-Q8_0.gguf"
test -f "$assets/mmproj-SmolVLM-500M-Instruct-Q8_0.gguf"
exec nice -n 10 "$runtime/llama-server" \
  -m "$assets/SmolVLM-500M-Instruct-Q8_0.gguf" \
  --mmproj "$assets/mmproj-SmolVLM-500M-Instruct-Q8_0.gguf" \
  --host 127.0.0.1 --port 8088 --ctx-size 2048 --threads 4 \
  --n-gpu-layers 0 --parallel 1
