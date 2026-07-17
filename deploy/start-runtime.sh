#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
source /opt/tello-edge/current/install/setup.bash
exec ros2 launch tello_bringup "${TELLO_LAUNCH_FILE:-real.launch.py}"
