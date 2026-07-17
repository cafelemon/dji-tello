#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "${ROOT_DIR}/models"
curl -L --fail --retry 3 \
  -o "${ROOT_DIR}/models/yolov5s.pt" \
  "https://github.com/ultralytics/yolov5/releases/download/v6.1/yolov5s.pt"
echo "model saved to ${ROOT_DIR}/models/yolov5s.pt"
