#!/usr/bin/env bash
set -eo pipefail
UR7E_PY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$UR7E_PY_ROOT/env.sh"
[[ "$(ros2 pkg prefix ur7e_nvblox_in_hand)" == "$UR7E_PY_ROOT/install/"* ]] || { echo 'Run ./build.sh first'; exit 1; }
exec ros2 launch ur7e_nvblox_in_hand bringup.launch.py "$@"
