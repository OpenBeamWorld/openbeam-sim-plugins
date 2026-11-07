#!/usr/bin/env bash
# Builds the OpenBeam core and MuJoCo plugin on Linux (GCC or Clang + CMake),
# installs them to build/install and runs the core's tests. See README.md.
#
#   python3 tools/fetch_mujoco.py
#   ./build_linux.sh [--no-mujoco] [--no-tests]
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
build="${BUILD_DIR:-$repo/build}"
mujoco_root="${MUJOCO_ROOT:-$repo/third_party/mujoco-4.0.0}"
config="${CONFIG:-Release}"
mujoco=ON
tests=ON
for arg in "$@"; do
    case "$arg" in
        --no-mujoco) mujoco=OFF ;;
        --no-tests) tests=OFF ;;
        *) echo "unknown option $arg" >&2; exit 2 ;;
    esac
done

generator=()
command -v ninja >/dev/null && generator=(-G Ninja)
configure=(-S "$repo" -B "$build" "${generator[@]}" "-DCMAKE_BUILD_TYPE=$config" "-DOPENBEAM_BUILD_MUJOCO_PLUGIN=$mujoco")
if [ "$mujoco" = ON ]; then
    if [ ! -f "$mujoco_root/include/mujoco/mujoco.h" ]; then
        echo "No MuJoCo release at $mujoco_root: run python3 tools/fetch_mujoco.py, set MUJOCO_ROOT, or --no-mujoco" >&2
        exit 1
    fi
    configure+=("-DMUJOCO_ROOT=$mujoco_root")
fi

cmake "${configure[@]}"
cmake --build "$build" --config "$config" --parallel
cmake --install "$build" --config "$config" --prefix "$build/install"
if [ "$tests" = ON ]; then
    ctest --test-dir "$build" --output-on-failure -C "$config"
fi
