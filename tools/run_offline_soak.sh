#!/usr/bin/env bash
set -eo pipefail

DURATION_SECONDS="${1:-120}"
ARTIFACT_DIR="${2:-artifacts/offline-soak}"

source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

mkdir -p "${ARTIFACT_DIR}"
VIDEO_PATH="${ARTIFACT_DIR}/person_motion.mp4"
python3 tools/generate_offline_fixture.py --output "${VIDEO_PATH}" --duration 20 --fps 30 \
  >"${ARTIFACT_DIR}/fixture.log"
ros2 launch tello_bringup offline.launch.py \
  video_path:="${VIDEO_PATH}" auto_select_target:=true publish_rate_hz:=60.0 \
  >"${ARTIFACT_DIR}/launch.log" 2>&1 &
LAUNCH_PID=$!
cleanup() {
  kill -INT "${LAUNCH_PID}" 2>/dev/null || true
  for _ in {1..30}; do
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done
  kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
  wait "${LAUNCH_PID}" 2>/dev/null || true
}
trap cleanup EXIT

python3 tools/offline_soak_monitor.py \
  --duration "${DURATION_SECONDS}" \
  --launch-pid "${LAUNCH_PID}" \
  --output-dir "${ARTIFACT_DIR}"
