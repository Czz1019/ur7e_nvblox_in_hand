#!/usr/bin/env bash
set -eo pipefail
UR7E_PY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$UR7E_PY_ROOT/env.sh"
cd "$UR7E_PY_ROOT"
/usr/bin/python3 tools/check_wheel.py
cmake -S native -B build_native -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES="${UR7E_CUDA_ARCH:-89}" -DCMAKE_CUDA_COMPILER="${CUDACXX:-/usr/local/cuda/bin/nvcc}" -DCMAKE_INSTALL_PREFIX="$UR7E_PY_ROOT/install_native"
cmake --build build_native --parallel "${UR7E_BUILD_JOBS:-1}"
cmake --install build_native
/usr/bin/colcon build --base-paths src --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
