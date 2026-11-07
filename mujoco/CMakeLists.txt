# SPDX-License-Identifier: GPL-3.0-or-later
# openbeam_mujoco: the MuJoCo engine plugin "openbeam.softbody", with the
# OpenBeam core linked in. Load it with mujoco.mj_loadPluginLibrary (Python)
# or drop it into a MuJoCo release's bin/mujoco_plugin/ for simulate.exe.

if(NOT MUJOCO_ROOT)
    message(FATAL_ERROR "Set MUJOCO_ROOT to an unpacked MuJoCo release, or OPENBEAM_BUILD_MUJOCO_PLUGIN=OFF")
endif()
find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h PATHS "${MUJOCO_ROOT}/include" NO_DEFAULT_PATH REQUIRED)
find_library(MUJOCO_LIBRARY NAMES mujoco PATHS "${MUJOCO_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)

add_library(openbeam_mujoco SHARED openbeam_mujoco_plugin.cpp $<TARGET_OBJECTS:openbeam_core_objects>)
target_include_directories(openbeam_mujoco PRIVATE "${MUJOCO_INCLUDE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/../openbeam_core/include")
# The core objects export the ob_* C API from this DLL as well.
target_compile_definitions(openbeam_mujoco PRIVATE OPENBEAM_CORE_BUILD_SHARED _CRT_SECURE_NO_WARNINGS)
target_link_libraries(openbeam_mujoco PRIVATE "${MUJOCO_LIBRARY}")

install(TARGETS openbeam_mujoco RUNTIME DESTINATION mujoco_plugin LIBRARY DESTINATION mujoco_plugin)
