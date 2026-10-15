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

aerolab_add_test(core_test
  SOURCES
    core/clock_test.cc
    core/log_test.cc
  DEPS aerolab_core)

aerolab_add_test(app_options_test
  SOURCES
    app/options_test.cc
  DEPS aerolab_app_options)

# Architectural guard: modules may only include what they depend on.
add_test(NAME module_dependencies
  COMMAND ${CMAKE_COMMAND} -DSOURCE_DIR=${PROJECT_SOURCE_DIR}/src
          -P ${PROJECT_SOURCE_DIR}/cmake/CheckModuleDependencies.cmake)

if(AEROLAB_BUILD_APP)
  # End-to-end smoke test: create the window and GL context, run a few frames
  # headless and exit cleanly. Needs a GPU; exclude with `ctest -LE gpu`.
  add_test(NAME app_smoke COMMAND aerolab --hidden --no-vsync --frames 10)
  set_tests_properties(app_smoke PROPERTIES LABELS gpu TIMEOUT 60)
endif()
