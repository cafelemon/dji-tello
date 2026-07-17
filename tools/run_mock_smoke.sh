#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

ARTIFACT_DIR="${1:-artifacts/mock-smoke}"
mkdir -p "${ARTIFACT_DIR}"
ros2 launch tello_bringup mock.launch.py >"${ARTIFACT_DIR}/launch.log" 2>&1 &
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

wait_for_service() {
  local service_name="$1"
  for _ in {1..40}; do
    if ros2 service list | grep -qx "${service_name}"; then
      return 0
    fi
    sleep 0.25
  done
  echo "service did not appear: ${service_name}" >&2
  return 1
}

wait_for_state() {
  local expected="$1"
  for _ in {1..40}; do
    if timeout 3 ros2 topic echo --once /flight/status 2>/dev/null | grep -q "state_name: ${expected}"; then
      return 0
    fi
    sleep 0.25
  done
  echo "flight state did not reach ${expected}" >&2
  return 1
}

wait_for_service /flight/takeoff
wait_for_service /mock/pause_state
wait_for_service /tello/execute_command
timeout 5 ros2 topic echo --once /tello/telemetry >"${ARTIFACT_DIR}/telemetry.txt"
wait_for_state READY

ros2 service call /mock/drop_ack std_srvs/srv/SetBool '{data: true}' >/dev/null
ros2 service call /tello/execute_command tello_interfaces/srv/TelloCommand '{command: 0}' \
  >"${ARTIFACT_DIR}/command-timeout.txt"
grep -q 'success=False\|success: false' "${ARTIFACT_DIR}/command-timeout.txt"
wait_for_state DISCONNECTED
ros2 service call /mock/reset_faults std_srvs/srv/Trigger '{}' >/dev/null
wait_for_state READY

ros2 service call /mock/error_ack std_srvs/srv/SetBool '{data: true}' >/dev/null
ros2 service call /tello/execute_command tello_interfaces/srv/TelloCommand '{command: 0}' \
  >"${ARTIFACT_DIR}/command-error-ack.txt"
grep -q 'success=False\|success: false' "${ARTIFACT_DIR}/command-error-ack.txt"
wait_for_state DISCONNECTED
ros2 service call /mock/reset_faults std_srvs/srv/Trigger '{}' >/dev/null
wait_for_state READY

FIRST_TAKEOFF="$(ros2 service call /flight/takeoff std_srvs/srv/Trigger '{}')"
echo "${FIRST_TAKEOFF}" >"${ARTIFACT_DIR}/takeoff-first.txt"
grep -q 'success=True\|success: true' "${ARTIFACT_DIR}/takeoff-first.txt"

SECOND_TAKEOFF="$(ros2 service call /flight/takeoff std_srvs/srv/Trigger '{}')"
echo "${SECOND_TAKEOFF}" >"${ARTIFACT_DIR}/takeoff-second.txt"
grep -q 'success=False\|success: false' "${ARTIFACT_DIR}/takeoff-second.txt"

ros2 service call /mock/pause_state std_srvs/srv/SetBool '{data: true}' \
  >"${ARTIFACT_DIR}/pause-state.txt"
wait_for_state LANDED

timeout 3 ros2 topic echo --once /tello/link_status >"${ARTIFACT_DIR}/link-status.txt"
grep -A1 'key: state_healthy' "${ARTIFACT_DIR}/link-status.txt" | grep -q "'false'"
echo "mock smoke passed"
