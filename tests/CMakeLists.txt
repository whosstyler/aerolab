include(GoogleTest)

# One test executable per library keeps failures easy to localize.
function(aerolab_add_test name)
  cmake_parse_arguments(ARG "" "" "SOURCES;DEPS" ${ARGN})
  add_executable(${name} ${ARG_SOURCES})
  target_include_directories(${name} PRIVATE ${PROJECT_SOURCE_DIR}/src
                                             ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(${name} PRIVATE ${ARG_DEPS} GTest::gtest_main
                                        aerolab_warnings)
  aerolab_configure_executable(${name})
  gtest_discover_tests(${name} DISCOVERY_TIMEOUT 30)
endfunction()

aerolab_add_test(math_test
  SOURCES
    math/vec_test.cc
    math/mat_test.cc
    math/quat_test.cc
    math/transform_test.cc
    math/projection_test.cc
    math/intersection_test.cc
  DEPS aerolab_math)

aerolab_add_test(core_test
  SOURCES
    core/clock_test.cc
    core/fixed_timestep_test.cc
    core/input_test.cc
    core/log_test.cc
  DEPS aerolab_core)

aerolab_add_test(geometry_test
  SOURCES
    geometry/geometry_test.cc
  DEPS aerolab_geometry)

aerolab_add_test(sim_test
  SOURCES
    sim/force_model_test.cc
    sim/integrator_test.cc
  DEPS aerolab_sim)

aerolab_add_test(camera_test
  SOURCES
    camera/camera_test.cc
  DEPS aerolab_camera)

aerolab_add_test(scene_test
  SOURCES
    scene/scene_test.cc
  DEPS aerolab_scene)

aerolab_add_test(world_test
  SOURCES
    world/simulation_test.cc
    world/world_test.cc
  DEPS aerolab_world)

aerolab_add_test(app_options_test
  SOURCES
    app/options_test.cc
  DEPS aerolab_app_options)

# The scene presenter is plain C++ (no GL calls), so it is tested even in
# test-only configurations.
aerolab_add_test(scene_presenter_test
  SOURCES
    app/scene_presenter_test.cc
    ${PROJECT_SOURCE_DIR}/src/app/scene_presenter.cc
  DEPS aerolab_scene aerolab_world aerolab_geometry)

# Architectural guard: modules may only include what they depend on.
add_test(NAME module_dependencies
  COMMAND ${CMAKE_COMMAND} -DSOURCE_DIR=${PROJECT_SOURCE_DIR}/src
          -P ${PROJECT_SOURCE_DIR}/cmake/CheckModuleDependencies.cmake)

if(AEROLAB_BUILD_APP)
  aerolab_add_test(sdl_input_test
    SOURCES
      app/sdl_input_test.cc
      ${PROJECT_SOURCE_DIR}/src/app/sdl_input.cc
    DEPS aerolab_core SDL3::SDL3)

  # End-to-end smoke test: create the window and GL context, run a few frames
  # headless and exit cleanly. Needs a GPU; exclude with `ctest -LE gpu`.
  add_test(NAME app_smoke COMMAND aerolab --hidden --no-vsync --frames 10)
  set_tests_properties(app_smoke PROPERTIES LABELS gpu TIMEOUT 60)
endif()
