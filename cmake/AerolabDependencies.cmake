# Third-party dependencies, pinned to release archives verified by SHA-256.
#
# SDL publishes official release tarballs. Dear ImGui and GoogleTest only
# offer GitHub's generated tag archives; if GitHub ever regenerates those with
# different bytes, the hash check fails closed and the pin must be refreshed.

include(FetchContent)

if(AEROLAB_BUILD_APP)
  # --- SDL3 (static) --------------------------------------------------------
  set(SDL_SHARED OFF CACHE BOOL "" FORCE)
  set(SDL_STATIC ON CACHE BOOL "" FORCE)
  set(SDL_TEST_LIBRARY OFF CACHE BOOL "" FORCE)
  set(SDL_TESTS OFF CACHE BOOL "" FORCE)
  set(SDL_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(SDL_INSTALL OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(SDL3
    URL https://github.com/libsdl-org/SDL/releases/download/release-3.4.8/SDL3-3.4.8.tar.gz
    URL_HASH SHA256=e9fff7467fb60f037e6708da18b25560649e4c63edc2a69bb871b960d9cbfbba
    DOWNLOAD_EXTRACT_TIMESTAMP FALSE
    SYSTEM)
  FetchContent_MakeAvailable(SDL3)

  # --- Dear ImGui (docking branch) -----------------------------------------
  FetchContent_Declare(imgui
    URL https://github.com/ocornut/imgui/archive/refs/tags/v1.92.9b-docking.tar.gz
    URL_HASH SHA256=90ded916bd57db2e0e171b6b098940a47c6f5042725dcdc67fb19940ca8bfdcc
    DOWNLOAD_EXTRACT_TIMESTAMP FALSE
    SYSTEM)
  FetchContent_MakeAvailable(imgui)

  # Excluded from "all" so it only builds once something links it.
  add_library(imgui STATIC EXCLUDE_FROM_ALL
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl3.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp)
  target_include_directories(imgui SYSTEM PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends)
  target_compile_definitions(imgui PUBLIC IMGUI_DISABLE_OBSOLETE_FUNCTIONS)
  target_link_libraries(imgui PUBLIC SDL3::SDL3)

  # The UI font is compiled into the binary so the app has no runtime assets.
  set(AEROLAB_UI_FONT ${imgui_SOURCE_DIR}/misc/fonts/Roboto-Medium.ttf
    CACHE INTERNAL "TrueType font embedded for the editor UI")
endif()

if(AEROLAB_BUILD_TESTS)
  # --- GoogleTest -----------------------------------------------------------
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.tar.gz
    URL_HASH SHA256=65fab701d9829d38cb77c14acdc431d2108bfdbf8979e40eb8ae567edf10b27c
    DOWNLOAD_EXTRACT_TIMESTAMP FALSE
    SYSTEM)
  FetchContent_MakeAvailable(googletest)
endif()
