#!/usr/bin/env bash
set -euo pipefail

ONNX_PATH="${1:-models/yolov5s_640.onnx}"
ENGINE_PATH="${2:-models/yolov5s_fp16.engine}"
DRY_RUN="${3:-}"
TRTEXEC="${TRTEXEC:-/usr/src/tensorrt/bin/trtexec}"
COMMAND=("${TRTEXEC}" --onnx="${ONNX_PATH}" --saveEngine="${ENGINE_PATH}" --fp16 --shapes=images:1x3x640x640 --skipInference)

if [[ "${DRY_RUN}" == "--dry-run" ]]; then
  printf '%q ' "${COMMAND[@]}"
  printf '\n'
  exit 0
fi

[[ "$(uname -m)" == "aarch64" ]] || { echo 'TensorRT Engine must be built on the target Jetson' >&2; exit 1; }
[[ -x "${TRTEXEC}" ]] || { echo "trtexec not executable: ${TRTEXEC}" >&2; exit 1; }
[[ -f "${ONNX_PATH}" ]] || { echo "missing ONNX model: ${ONNX_PATH}" >&2; exit 1; }
mkdir -p "$(dirname "${ENGINE_PATH}")"
"${COMMAND[@]}"
[[ -s "${ENGINE_PATH}" ]] || { echo "TensorRT Engine was not produced" >&2; exit 1; }
