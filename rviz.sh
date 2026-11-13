#!/usr/bin/env bash
set -eo pipefail
UR7E_PY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$UR7E_PY_ROOT/env.sh"
UR7E_VIEW="${1:-diagnostic}"
case "$UR7E_VIEW" in mesh|diagnostic|2d) ;; *) echo 'Usage: ./rviz.sh [mesh|diagnostic|2d]'; exit 2;; esac
exec rviz2 -d "$UR7E_PY_ROOT/install/ur7e_nvblox_in_hand/share/ur7e_nvblox_in_hand/rviz/$UR7E_VIEW.rviz" --ros-args -p "use_sim_time:=${USE_SIM_TIME:-true}"
