# OpenBeam simulator plugins: the soft-body core and its MuJoCo plugin.
# Build and run instructions: README.md.
cmake_minimum_required(VERSION 3.24)
project(openbeam_sim_plugins LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# OpenBeam's own sources that the core compiles, vendored by tools/vendor_openbeam.py.
set(OPENBEAM_ENGINE_MAIN "${CMAKE_CURRENT_SOURCE_DIR}/third_party/openbeam/engine/source/main" CACHE PATH
    "OpenBeam's BOARS engine sources (source/main)")
set(OPENBEAM_GAME_MAIN "${CMAKE_CURRENT_SOURCE_DIR}/third_party/openbeam/source/main" CACHE PATH
    "OpenBeam's game sources (source/main)")
if(NOT EXISTS "${OPENBEAM_ENGINE_MAIN}/physics/BeamAxialResponse.h"
   OR NOT EXISTS "${OPENBEAM_GAME_MAIN}/physics/ContactPenetration.h")
    message(FATAL_ERROR "The OpenBeam sources are missing from ${OPENBEAM_ENGINE_MAIN} and ${OPENBEAM_GAME_MAIN}. "
        "Check out this repository again, or run tools/vendor_openbeam.py --from <openbeam checkout>.")
endif()

option(OPENBEAM_BUILD_MUJOCO_PLUGIN "Build the MuJoCo engine plugin" ON)
set(MUJOCO_ROOT "" CACHE PATH "An unpacked MuJoCo release (include/, lib/); tools/fetch_mujoco.py fetches one")

if(MSVC)
    add_compile_options(/W3 /permissive- /utf-8 /bigobj /EHsc)
endif()

enable_testing()
add_subdirectory(openbeam_core)
if(OPENBEAM_BUILD_MUJOCO_PLUGIN)
    add_subdirectory(mujoco)
endif()
