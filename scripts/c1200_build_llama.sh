#!/usr/bin/env bash
set -euo pipefail

ROOT=/userdata/smolvlm
SRC="$ROOT/src/llama.cpp"
BUILD="$ROOT/build"

mkdir -p "$ROOT/src"
if [[ ! -d "$SRC" ]]; then
  tar -xzf "$ROOT/llama-b11878-source.tar.gz" -C "$ROOT/src"
  [[ -d "$ROOT/src/llama.cpp-b11878" ]] && mv "$ROOT/src/llama.cpp-b11878" "$SRC"
fi

cmake -S "$SRC" -B "$BUILD" \
  -DGGML_NATIVE=ON \
  -DGGML_OPENMP=ON \
  -DLLAMA_CURL=OFF \
  -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD" -j4 --target llama-mtmd-cli llama-server

echo "LLAMA_BUILD_OK $BUILD/bin/llama-mtmd-cli"
