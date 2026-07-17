#!/usr/bin/env bash
set -euo pipefail
RELEASE_ID="${1:?usage: activate-release.sh RELEASE_ID}"
TARGET_ROOT=/opt/tello-edge
TARGET="${TARGET_ROOT}/releases/${RELEASE_ID}"
[[ "${EUID}" -eq 0 ]] || { echo 'run as root' >&2; exit 1; }
[[ -d "${TARGET}" ]] || { echo "missing release: ${TARGET}" >&2; exit 1; }
CURRENT="$(readlink -f "${TARGET_ROOT}/current" 2>/dev/null || true)"
[[ -z "${CURRENT}" ]] || ln -sfn "${CURRENT}" "${TARGET_ROOT}/previous"
ln -sfn "${TARGET}" "${TARGET_ROOT}/current"
systemctl enable --now tello-edge.service
