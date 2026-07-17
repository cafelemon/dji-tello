#!/usr/bin/env bash
set -euo pipefail

SOURCE_DIR="${1:?usage: install-release.sh SOURCE_DIR RELEASE_ID}"
RELEASE_ID="${2:?usage: install-release.sh SOURCE_DIR RELEASE_ID}"
TARGET_ROOT="/opt/tello-edge"
TARGET="${TARGET_ROOT}/releases/${RELEASE_ID}"
[[ "${EUID}" -eq 0 ]] || { echo 'run as root' >&2; exit 1; }
[[ -f "${SOURCE_DIR}/README.md" ]] || { echo 'invalid source directory' >&2; exit 1; }
id tello >/dev/null 2>&1 || useradd --system --create-home --shell /usr/sbin/nologin tello
install -d -o tello -g tello "${TARGET_ROOT}/releases" /etc/tello-edge
[[ ! -e "${TARGET}" ]] || { echo "release already exists: ${TARGET}" >&2; exit 1; }
cp -a "${SOURCE_DIR}" "${TARGET}"
chown -R tello:tello "${TARGET}"
ln -sfn "${TARGET}" "${TARGET_ROOT}/next"
install -m 0644 "${TARGET}/deploy/systemd/tello-edge.service" /etc/systemd/system/tello-edge.service
[[ -f /etc/tello-edge/tello-edge.env ]] || \
  install -m 0640 -o root -g tello "${TARGET}/deploy/tello-edge.env.example" /etc/tello-edge/tello-edge.env
systemctl daemon-reload
echo "installed ${TARGET}; run deploy/activate-release.sh ${RELEASE_ID}"
