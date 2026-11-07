# openbeam_core: the shared soft-body library (C ABI, openbeam_core.h).
#
# OpenBeam's engine and game law sources are compiled as they are, from
# third_party/openbeam: the JBeam parser, resolver and structural tier (BOARS
# resources/beamng), the beam laws and the calibrated material (BOARS physics),
# and the game's two contact-law headers (source/main/physics).

set(OPENBEAM_CORE_ENGINE_SOURCES
    "${OPENBEAM_ENGINE_MAIN}/resources/beamng/JBeamSyntax.cpp"
    "${OPENBEAM_ENGINE_MAIN}/resources/beamng/JBeamPartResolver.cpp"
    "${OPENBEAM_ENGINE_MAIN}/resources/beamng/JBeamExpressionEvaluator.cpp"
    "${OPENBEAM_ENGINE_MAIN}/resources/beamng/JBeamStructuralIR.cpp"
    "${OPENBEAM_ENGINE_MAIN}/physics/BeamAxialKinematics.cpp"
    "${OPENBEAM_ENGINE_MAIN}/physics/CalibratedBeamProductionStep.cpp")

add_library(openbeam_core_objects OBJECT
    src/JBeamStructure.cpp
    src/openbeam_core.cpp
    ${OPENBEAM_CORE_ENGINE_SOURCES})
target_include_directories(openbeam_core_objects
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
        "${OPENBEAM_ENGINE_MAIN}/physics"
        "${OPENBEAM_ENGINE_MAIN}/resources/beamng"
        "${OPENBEAM_GAME_MAIN}/physics")
target_compile_definitions(openbeam_core_objects PRIVATE OPENBEAM_CORE_BUILD_SHARED)

add_library(openbeam_core SHARED $<TARGET_OBJECTS:openbeam_core_objects>)
target_include_directories(openbeam_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

install(TARGETS openbeam_core RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
install(FILES include/openbeam_core.h DESTINATION include)

add_executable(openbeam_core_tests tests/openbeam_core_tests.cpp)
target_link_libraries(openbeam_core_tests PRIVATE openbeam_core)
add_test(NAME openbeam_core_tests COMMAND openbeam_core_tests)
if(WIN32)  # the test finds openbeam_core.dll on PATH; elsewhere the build's rpath finds it
    set_tests_properties(openbeam_core_tests PROPERTIES
        ENVIRONMENT "PATH=$<TARGET_FILE_DIR:openbeam_core>;$ENV{PATH}")
endif()
