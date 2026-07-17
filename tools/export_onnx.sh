#!/usr/bin/env bash
set -euo pipefail

WEIGHTS="${1:-models/yolov5s.pt}"
OUTPUT="${2:-models/yolov5s_640.onnx}"
DRY_RUN="${3:-}"
COMMAND=(python3 vendor/yolov5/export.py --weights "${WEIGHTS}" --imgsz 640 640 --batch-size 1 --opset 13 --include onnx)

if [[ "${DRY_RUN}" == "--dry-run" ]]; then
  printf '%q ' "${COMMAND[@]}"
  printf '\nvalidate with onnx.checker and copy to %s\n' "${OUTPUT}"
  exit 0
fi

[[ -f "${WEIGHTS}" ]] || { echo "missing weights: ${WEIGHTS}" >&2; exit 1; }
[[ -f vendor/yolov5/export.py ]] || { echo "missing pinned YOLOv5 checkout" >&2; exit 1; }
"${COMMAND[@]}"
GENERATED="${WEIGHTS%.*}.onnx"
[[ -f "${GENERATED}" ]] || { echo "export did not produce ${GENERATED}" >&2; exit 1; }
mkdir -p "$(dirname "${OUTPUT}")"
cp "${GENERATED}" "${OUTPUT}"
python3 -c "import onnx; model=onnx.load('${OUTPUT}'); onnx.checker.check_model(model); print('${OUTPUT}')"
