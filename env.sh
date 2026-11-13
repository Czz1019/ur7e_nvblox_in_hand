#!/usr/bin/env bash
# Source this file in every terminal. No sibling workspace or Isaac ROS overlay.
UR7E_PY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH LD_PRELOAD
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-135}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11565}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_HOME="$UR7E_PY_ROOT/.ros"
export ROS_LOG_DIR="$UR7E_PY_ROOT/log/ros"
export CYCLONEDDS_URI="file://$UR7E_PY_ROOT/src/ur7e_nvblox_in_hand/config/cyclonedds_local.xml"
export UR7E_NVBLOX_BRIDGE="$UR7E_PY_ROOT/install_native/lib/libur7e_nvblox_bridge.so"
if [[ -f "$UR7E_PY_ROOT/install/local_setup.bash" ]]; then
  source "$UR7E_PY_ROOT/install/local_setup.bash"
fi
