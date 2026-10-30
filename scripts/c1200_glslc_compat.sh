#!/usr/bin/env bash
set -euo pipefail

# Ubuntu 22.04 arm64 ships glslangValidator but may not ship glslc. llama.cpp
# only needs the glslc command-line surface for shader compilation. Translate
# the shader-stage spelling used by glslc and delegate the actual compilation.
args=()
skip_next=0
source_file=""
for arg in "$@"; do
  if [[ "$skip_next" == 1 ]]; then
    skip_next=0
    continue
  fi
  case "$arg" in
    -fshader-stage=compute) args+=("-S" "comp") ;;
    -fshader-stage=vertex) args+=("-S" "vert") ;;
    -fshader-stage=fragment) args+=("-S" "frag") ;;
    -fshader-stage=geometry) args+=("-S" "geom") ;;
    -fshader-stage=tesscontrol) args+=("-S" "tesc") ;;
    -fshader-stage=tesseval) args+=("-S" "tese") ;;
    --target-env=*) args+=("--target-env" "${arg#--target-env=}") ;;
    -O) ;;
    -MD) ;;
    -MF) skip_next=1 ;;
    *)
      args+=("$arg")
      if [[ "$arg" == *.comp ]]; then
        source_file="$arg"
      fi
      ;;
  esac
done

# glslc enables GL_GOOGLE_include_directive automatically; glslangValidator
# 11.x requires the extension declaration in the source. Inject it after the
# #version line while preserving the shader's include directory.
if [[ -n "$source_file" && -f "$source_file" ]]; then
  tmp="${TMPDIR:-/tmp}/c1200-glslc-$$.comp"
  awk 'NR == 1 && $0 ~ /^#version/ { print; print "#extension GL_GOOGLE_include_directive : enable"; next } { print }' \
    "$source_file" > "$tmp"
  for i in "${!args[@]}"; do
    if [[ "${args[$i]}" == "$source_file" ]]; then
      args[$i]="$tmp"
      break
    fi
  done
  args+=("-I$(dirname "$source_file")")
  trap 'rm -f "$tmp"' EXIT
fi
err="${TMPDIR:-/tmp}/c1200-glslc-$$.err"
trap 'rm -f "${tmp:-}" "$err"' EXIT
set +e
glslangValidator "${args[@]}" 2>"$err"
rc=$?
set -e
if [[ "$rc" != 0 ]]; then
  # CMake's extension probe recognizes glslc's diagnostic spelling. Make the
  # older glslangValidator failure equivalent so unsupported cooperative
  # matrix extensions are disabled instead of being compiled accidentally.
  if [[ -n "$source_file" && -f "$source_file" ]]; then
    if grep -q 'GL_KHR_cooperative_matrix' "$source_file"; then
      echo 'extension not supported: GL_KHR_cooperative_matrix' >&2
    fi
    if grep -q 'GL_NV_cooperative_matrix2' "$source_file"; then
      echo 'extension not supported: GL_NV_cooperative_matrix2' >&2
    fi
    if grep -q 'GL_NV_cooperative_matrix_decode_vector' "$source_file"; then
      echo 'extension not supported: GL_NV_cooperative_matrix_decode_vector' >&2
    fi
    if grep -q 'GL_EXT_integer_dot_product' "$source_file"; then
      echo 'extension not supported: GL_EXT_integer_dot_product' >&2
    fi
    if grep -q 'GL_EXT_bfloat16' "$source_file"; then
      echo 'extension not supported: GL_EXT_bfloat16' >&2
    fi
    if grep -q 'GL_EXT_float_e2m1' "$source_file"; then
      echo 'extension not supported: GL_EXT_float_e2m1' >&2
    fi
    if grep -q 'GL_EXT_float_e4m3' "$source_file"; then
      echo 'extension not supported: GL_EXT_float_e4m3' >&2
    fi
  fi
  cat "$err" >&2
fi
exit "$rc"
