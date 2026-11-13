#!/usr/bin/env bash
set -eo pipefail
UR7E_PY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$UR7E_PY_ROOT/env.sh"
exec ros2 launch ur7e_d435_gazebo scene.launch.py "$@"
