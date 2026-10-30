#!/usr/bin/env bash
set -euo pipefail

# Build a board-local Vulkan llama.cpp variant without replacing the CPU build.
# The caller must provide Vulkan headers and glslc (normally from the BSP SDK or
# Ubuntu packages). The runtime ICD is selected through an environment variable.

ROOT="${C1200_SMOLVLM_ROOT:-/userdata/smolvlm}"
SRC="${C1200_LLAMA_SRC:-$ROOT/src/llama.cpp}"
BUILD="${C1200_LLAMA_VULKAN_BUILD:-$ROOT/build-vulkan}"
VULKAN_INCLUDE="${VULKAN_INCLUDE_DIR:-$ROOT/deps/vulkan-headers-1.4.350/include}"
SPIRV_HEADERS_INCLUDE="${SPIRV_HEADERS_INCLUDE_DIR:-$ROOT/deps/spirv-only/include}"
VULKAN_LIBRARY="${VULKAN_LIBRARY:-}"
ICD="${VK_ICD_FILENAMES:-$ROOT/c1200_mali_fixed.json}"

if [[ ! -f "$SRC/CMakeLists.txt" ]]; then
  echo "missing llama.cpp source: $SRC" >&2
  exit 2
fi
if [[ ! -f "$VULKAN_INCLUDE/vulkan/vulkan.h" ]]; then
  echo "missing Vulkan headers: $VULKAN_INCLUDE/vulkan/vulkan.h" >&2
  echo "install libvulkan-dev or set VULKAN_INCLUDE_DIR" >&2
  exit 2
fi
if ! command -v glslc >/dev/null 2>&1; then
  if [[ -x "$ROOT/deps/vulkan-sdk/usr/bin/glslc" ]]; then
    export PATH="$ROOT/deps/vulkan-sdk/usr/bin:$PATH"
  fi
fi
if ! command -v glslc >/dev/null 2>&1; then
  echo "missing glslc; install it or install c1200_glslc_compat.sh beside glslangValidator" >&2
  exit 2
fi
if [[ ! -f "$ICD" ]]; then
  echo "missing Vulkan ICD JSON: $ICD" >&2
  exit 2
fi

if [[ -z "$VULKAN_LIBRARY" ]]; then
  for candidate in \
    /usr/lib/aarch64-linux-gnu/libvulkan.so \
    /usr/lib/aarch64-linux-gnu/libvulkan.so.1 \
    /usr/lib/libvulkan.so \
    /usr/lib/libvulkan.so.1; do
    if [[ -e "$candidate" ]]; then
      VULKAN_LIBRARY="$candidate"
      break
    fi
  done
fi
if [[ -z "$VULKAN_LIBRARY" ]]; then
  echo "cannot locate libvulkan.so; set VULKAN_LIBRARY" >&2
  exit 2
fi

export LD_LIBRARY_PATH="/usr/bst/lib:${LD_LIBRARY_PATH:-}"
export VK_ICD_FILENAMES="$ICD"

# Ubuntu's arm64 package installs SPIR-V headers but not the CMake package
# config expected by llama.cpp. Provide a local interface target in the
# private SDK prefix; this does not modify /usr.
SPIRV_CONFIG_DIR="$ROOT/deps/vulkan-sdk/usr/lib/aarch64-linux-gnu/cmake/SPIRV-Headers"
if [[ ! -f "$SPIRV_CONFIG_DIR/SPIRV-HeadersConfig.cmake" ]]; then
  mkdir -p "$SPIRV_CONFIG_DIR"
  cat > "$SPIRV_CONFIG_DIR/SPIRV-HeadersConfig.cmake" <<EOF
set(SPIRV-Headers_FOUND TRUE)
if(NOT TARGET SPIRV-Headers::SPIRV-Headers)
  add_library(SPIRV-Headers::SPIRV-Headers INTERFACE IMPORTED)
  set_target_properties(SPIRV-Headers::SPIRV-Headers PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "$ROOT/deps/vulkan-sdk/usr/include")
endif()
EOF
fi

cmake -S "$SRC" -B "$BUILD" \
  -DCMAKE_BUILD_TYPE=Release \
  -DGGML_NATIVE=ON \
  -DGGML_OPENMP=ON \
  -DGGML_VULKAN=ON \
  -DLLAMA_CURL=OFF \
  -DCMAKE_PREFIX_PATH="$ROOT/deps/vulkan-sdk/usr" \
  -DCMAKE_CXX_FLAGS="-I$SPIRV_HEADERS_INCLUDE" \
  -DVulkan_INCLUDE_DIR="$VULKAN_INCLUDE" \
  -DVulkan_LIBRARY="$VULKAN_LIBRARY"

cmake --build "$BUILD" -j"${C1200_BUILD_JOBS:-4}" --target llama-mtmd-cli llama-server

echo "LLAMA_VULKAN_BUILD_OK $BUILD/bin/llama-server"
"$BUILD/bin/llama-server" --list-devices || true
