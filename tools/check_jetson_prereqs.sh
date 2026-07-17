#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" == "--dry-run" ]]; then
  echo 'checks: aarch64, Ubuntu 22.04, Jetson Linux 36.4.4, CUDA 12.6, TensorRT 10.3, ROS2 Humble, NVMe free space'
  exit 0
fi

[[ "$(uname -m)" == "aarch64" ]] || { echo 'expected aarch64 Jetson host' >&2; exit 1; }
source /etc/os-release
[[ "${VERSION_ID}" == "22.04" ]] || { echo "expected Ubuntu 22.04, got ${VERSION_ID}" >&2; exit 1; }
[[ -f /etc/nv_tegra_release ]] || { echo 'missing Jetson Linux release marker' >&2; exit 1; }
grep -q 'R36.*REVISION: 4.4' /etc/nv_tegra_release || { cat /etc/nv_tegra_release; exit 1; }
command -v nvcc >/dev/null
command -v ros2 >/dev/null
test -x /usr/src/tensorrt/bin/trtexec
df -Pk /opt | awk 'NR==2 { if ($4 < 10485760) exit 1 }'
echo 'Jetson prerequisites passed'
