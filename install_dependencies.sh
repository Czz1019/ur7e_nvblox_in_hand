#!/usr/bin/env bash
# Explicit installation command, never called automatically by build/run.
set -eo pipefail
UR7E_INSTALL_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == --system ]]; then
  sudo apt-get update
  sudo apt-get install -y build-essential cmake libeigen3-dev libgoogle-glog-dev libgflags-dev python3-colcon-common-extensions python3-rosdep \
    ros-humble-rmw-cyclonedds-cpp ros-humble-rosidl-default-generators ros-humble-rosidl-default-runtime \
    ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-ros2-controllers ros-humble-xacro ros-humble-rviz2 \
    ros-humble-tf2-ros-py ros-humble-cv-bridge ros-humble-image-transport ros-humble-point-cloud-transport libfcl-dev
elif [[ -n "${1:-}" ]]; then echo 'Usage: ./install_dependencies.sh [--system]'; exit 2; fi
# CUDA toolkit and compatible NVIDIA driver must be installed separately.
/usr/bin/python3 -m pip install --user 'setuptools<80' 'numpy==1.26.4' 'scipy>=1.10,<1.16' 'PyYAML>=6.0'
/usr/bin/python3 -m pip install --user --index-url https://download.pytorch.org/whl/cu128 'torch==2.9.1' 'torchvision==0.24.1'
/usr/bin/python3 -m pip install --user --no-deps 'https://github.com/nvidia-isaac/nvblox/releases/download/v0.0.10/nvblox_torch-0.0.10%2Bcu12ubuntu22-py3-none-linux_x86_64.whl'
/usr/bin/python3 -m pip install --user 'open3d==0.19.0' 'opencv-python>=4.8,<4.12' 'einops>=0.7' 'imageio>=2.30' 'matplotlib>=3.7' 'nvtx>=0.2' 'timm>=1.0' 'transforms3d>=0.4' trimesh
/usr/bin/python3 "$UR7E_INSTALL_ROOT/tools/check_wheel.py"
echo 'No Isaac ROS / NITROS / GXF / cuVSLAM installation requested.'
