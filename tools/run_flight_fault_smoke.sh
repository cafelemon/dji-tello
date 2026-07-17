#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

ARTIFACT_DIR="${1:-artifacts/flight-fault-smoke}"
mkdir -p "${ARTIFACT_DIR}"

run_scenario() {
  local scenario="$1"
  local domain_id="$2"
  ROS_DOMAIN_ID="${domain_id}" ros2 launch tello_bringup fault_test.launch.py \
    >"${ARTIFACT_DIR}/${scenario}-launch.log" 2>&1 &
  local launch_pid=$!
  cleanup_scenario() {
    kill -INT "${launch_pid}" 2>/dev/null || true
    for _ in {1..20}; do
      if ! kill -0 "${launch_pid}" 2>/dev/null; then
        break
      fi
      sleep 0.1
    done
    kill -TERM "${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  }
  if ! ROS_DOMAIN_ID="${domain_id}" python3 tools/flight_fault_scenario.py "${scenario}" \
      >"${ARTIFACT_DIR}/${scenario}.log" 2>&1; then
    cleanup_scenario
    return 1
  fi
  cleanup_scenario
}

domain_id=70
for scenario in tracking video telemetry emergency; do
  run_scenario "${scenario}" "${domain_id}"
  domain_id=$((domain_id + 1))
done

echo "flight fault smoke passed"
