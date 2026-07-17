#!/usr/bin/env bash
set -euo pipefail
TARGET_ROOT=/opt/tello-edge
[[ "${EUID}" -eq 0 ]] || { echo 'run as root' >&2; exit 1; }
PREVIOUS="$(readlink -f "${TARGET_ROOT}/previous" 2>/dev/null || true)"
[[ -n "${PREVIOUS}" && -d "${PREVIOUS}" ]] || { echo 'no previous release available' >&2; exit 1; }
CURRENT="$(readlink -f "${TARGET_ROOT}/current" 2>/dev/null || true)"
ln -sfn "${PREVIOUS}" "${TARGET_ROOT}/current"
[[ -z "${CURRENT}" ]] || ln -sfn "${CURRENT}" "${TARGET_ROOT}/previous"
systemctl restart tello-edge.service
