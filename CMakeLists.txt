cmake_minimum_required(VERSION 3.25)

project(AeroLab
  VERSION 0.1.0
  DESCRIPTION "Educational real-time 3D simulation engine and editor"
  LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Keep the application and the test binaries side by side.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

option(AEROLAB_BUILD_APP "Build the interactive application" ON)
option(AEROLAB_BUILD_TESTS "Build the unit tests" ON)
option(AEROLAB_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(AerolabCompilerOptions)
include(AerolabDependencies)

add_subdirectory(src)

if(AEROLAB_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
