#!/usr/bin/env bash
set -eo pipefail

DURATION_SECONDS="${1:-120}"
ARTIFACT_DIR="${2:-artifacts/soak}"
mkdir -p "${ARTIFACT_DIR}"

source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

ros2 launch tello_bringup mock.launch.py >"${ARTIFACT_DIR}/mock.log" 2>&1 &
LAUNCH_PID=$!
cleanup() {
  kill -INT "${LAUNCH_PID}" 2>/dev/null || true
  for _ in {1..20}; do
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done
  kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
  wait "${LAUNCH_PID}" 2>/dev/null || true
}
trap cleanup EXIT

START="$(date +%s)"
echo "timestamp,rss_kb" >"${ARTIFACT_DIR}/rss.csv"
while kill -0 "${LAUNCH_PID}" 2>/dev/null; do
  NOW="$(date +%s)"
  if (( NOW - START >= DURATION_SECONDS )); then
    break
  fi
  RSS="$(ps -o rss= -p "${LAUNCH_PID}" | tr -d ' ')"
  echo "${NOW},${RSS:-0}" >>"${ARTIFACT_DIR}/rss.csv"
  sleep 5
done

if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
  echo "mock launch exited before soak duration" >&2
  exit 1
fi
