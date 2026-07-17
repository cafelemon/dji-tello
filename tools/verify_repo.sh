#!/usr/bin/env bash
set -euo pipefail

if rg -n '/home/|your_package_name|TODO: License' README.md docs src tools deploy \
    --glob '!verify_repo.sh'; then
  echo "forbidden stale repository reference found" >&2
  exit 1
fi

python3 -m compileall -q src
python3 -m compileall -q tools
bash -n tools/*.sh deploy/*.sh
PYTHONPATH="src/tello_flight_manager:src/tello_mock:src/tello_vision${PYTHONPATH:+:${PYTHONPATH}}" \
python3 -m pytest -q \
  src/tello_flight_manager/test \
  src/tello_mock/test \
  src/tello_vision/test

echo "repository static checks passed"
