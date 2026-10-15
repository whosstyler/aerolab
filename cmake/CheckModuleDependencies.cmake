# Verifies that every first-party module only includes the modules it is
# allowed to depend on (see docs/ARCHITECTURE.md). Run as a test:
#   cmake -DSOURCE_DIR=<repo>/src -P CheckModuleDependencies.cmake
#
# Libraries share one include root, so the linker alone cannot stop, say, the
# simulation from including an OpenGL header. This script can.

if(NOT SOURCE_DIR)
  message(FATAL_ERROR "SOURCE_DIR is required")
endif()

# module -> first-party modules it may include (itself is always allowed).
set(allowed_math "")
set(allowed_core "math")
set(allowed_geometry "math")
set(allowed_sim "math")
set(allowed_camera "math;core")
set(allowed_scene "math;core;geometry;sim")
set(allowed_world "math;core;geometry;sim;scene;camera")
set(allowed_render "math;core;geometry")
set(allowed_editor "math;core;geometry;sim;scene;world;camera;render")
set(allowed_app "math;core;geometry;sim;scene;world;camera;render;editor;app")

# Third-party headers that only some modules may touch.
set(sdl_modules "app")
set(imgui_modules "editor;app")

set(violations "")
file(GLOB module_dirs LIST_DIRECTORIES true "${SOURCE_DIR}/*")
foreach(module_dir IN LISTS module_dirs)
  if(NOT IS_DIRECTORY "${module_dir}")
    continue()
  endif()
  get_filename_component(module "${module_dir}" NAME)
  if(NOT DEFINED allowed_${module})
    list(APPEND violations "unknown module '${module}' (add it to the table)")
    continue()
  endif()
  set(allowed "${allowed_${module}};${module}")

  file(GLOB_RECURSE sources "${module_dir}/*.h" "${module_dir}/*.cc")
  foreach(source IN LISTS sources)
    file(STRINGS "${source}" includes REGEX "^#include")
    file(RELATIVE_PATH relative "${SOURCE_DIR}" "${source}")
    foreach(line IN LISTS includes)
      if(line MATCHES "^#include \"([a-z_]+)/")
        set(target "${CMAKE_MATCH_1}")
        if(NOT target IN_LIST allowed)
          list(APPEND violations "${relative}: ${module} may not include ${target}/")
        endif()
      elseif(line MATCHES "^#include <SDL3/")
        if(NOT module IN_LIST sdl_modules)
          list(APPEND violations "${relative}: ${module} may not include SDL")
        endif()
      elseif(line MATCHES "^#include [<\"]imgui")
        if(NOT module IN_LIST imgui_modules)
          list(APPEND violations "${relative}: ${module} may not include ImGui")
        endif()
      endif()
    endforeach()
  endforeach()
endforeach()

if(violations)
  list(JOIN violations "\n  " report)
  message(FATAL_ERROR "Module dependency violations:\n  ${report}")
endif()
message(STATUS "Module dependencies OK")
