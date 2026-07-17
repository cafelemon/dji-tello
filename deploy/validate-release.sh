#!/usr/bin/env bash
set -euo pipefail

RELEASE_DIR="${1:-.}"
for path in \
  README.md \
  install/setup.bash \
  deploy/start-runtime.sh \
  deploy/systemd/tello-edge.service \
  src/tello_bringup/config/jetson.yaml; do
  [[ -e "${RELEASE_DIR}/${path}" ]] || {
    echo "missing release artifact: ${path}" >&2
    exit 1
  }
done

[[ -s "${RELEASE_DIR}/models/yolov5s_fp16.engine" ]] || {
  echo 'missing or empty release artifact: models/yolov5s_fp16.engine' >&2
  exit 1
}

[[ ! -L "${RELEASE_DIR}/models/yolov5s_fp16.engine" ]] || {
  echo 'Engine must be a release-local file, not an external symlink' >&2
  exit 1
}

echo "release layout valid: ${RELEASE_DIR}"
