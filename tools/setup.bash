# Source this file from bash after building PL-VINS.
PLVINS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$PLVINS_ROOT/install/setup.bash"
export AMENT_PREFIX_PATH="$PLVINS_ROOT/.deps/install:${AMENT_PREFIX_PATH:-}"
export CMAKE_PREFIX_PATH="$PLVINS_ROOT/.deps/install:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$PLVINS_ROOT/.deps/install/lib:${LD_LIBRARY_PATH:-}"
unset PLVINS_ROOT
