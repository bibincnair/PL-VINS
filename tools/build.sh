#!/usr/bin/env bash
set -euo pipefail
PLVINS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PLVINS_ROOT"
set +u
source /opt/ros/jazzy/setup.bash
set -u
python3 tools/bootstrap_dependencies.py --jobs "${PLVINS_JOBS:-4}"
export AMENT_PREFIX_PATH="$PLVINS_ROOT/.deps/install:${AMENT_PREFIX_PATH:-}"
export CMAKE_PREFIX_PATH="$PLVINS_ROOT/.deps/install:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$PLVINS_ROOT/.deps/install/lib:${LD_LIBRARY_PATH:-}"
export CMAKE_BUILD_PARALLEL_LEVEL="${PLVINS_JOBS:-4}"
export MAKEFLAGS="-j${PLVINS_JOBS:-4}"
colcon build --parallel-workers 2 --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE=/usr/bin/python3
