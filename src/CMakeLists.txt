# First-party libraries. Each directory is an independent static library; the
# allowed dependency direction is documented in docs/ARCHITECTURE.md and
# enforced by the module_dependencies test.

function(aerolab_add_library name)
  cmake_parse_arguments(ARG "" "" "SOURCES;DEPS" ${ARGN})
  add_library(${name} STATIC ${ARG_SOURCES})
  target_include_directories(${name} PUBLIC ${PROJECT_SOURCE_DIR}/src)
  target_link_libraries(${name} PUBLIC ${ARG_DEPS} PRIVATE aerolab_options)
endfunction()

aerolab_add_library(aerolab_math
  SOURCES
    math/mat3.cc
    math/mat4.cc
    math/quat.cc
    math/transform.cc
    math/projection.cc
    math/intersection.cc)

aerolab_add_library(aerolab_core
  SOURCES
    core/clock.cc
    core/event_log.cc
    core/fixed_timestep.cc
    core/frame_stats.cc
    core/input.cc
    core/log.cc
    core/string_util.cc
  DEPS aerolab_math)

aerolab_add_library(aerolab_geometry
  SOURCES
    geometry/debug_draw.cc
    geometry/mesh_data.cc
    geometry/mesh_library.cc
    geometry/primitives.cc
  DEPS aerolab_math)

aerolab_add_library(aerolab_sim
  SOURCES
    sim/flight_agent.cc
    sim/force_model.cc
    sim/integrator.cc
    sim/kinematic_path.cc
    sim/rigid_body.cc
  DEPS aerolab_math)

aerolab_add_library(aerolab_camera
  SOURCES
    camera/camera.cc
    camera/camera_controller.cc
  DEPS aerolab_math aerolab_core)

aerolab_add_library(aerolab_scene
  SOURCES
    scene/scene.cc
  DEPS aerolab_math aerolab_core aerolab_geometry aerolab_sim)

aerolab_add_library(aerolab_world
  SOURCES
    world/animation.cc
    world/autonomy.cc
    world/debug_visualizer.cc
    world/picking.cc
    world/presets.cc
    world/simulation.cc
    world/spawner.cc
  DEPS aerolab_scene aerolab_sim aerolab_geometry aerolab_core aerolab_math)

# Command-line parsing has no platform dependencies, so it is always built
# and unit tested, even in test-only configurations.
aerolab_add_library(aerolab_app_options
  SOURCES
    app/options.cc)

if(AEROLAB_BUILD_APP)
  aerolab_add_library(aerolab_render
    SOURCES
      render/gl.cc
      render/gpu_mesh.cc
      render/gpu_timer.cc
      render/readback.cc
      render/render_target.cc
      render/renderer.cc
      render/shader.cc
      render/shader_sources.cc
    DEPS aerolab_math aerolab_core aerolab_geometry)

  # The editor UI font is compiled into the binary (no runtime assets).
  include(AerolabEmbedFile)
  set(ui_font_source ${CMAKE_CURRENT_BINARY_DIR}/generated/ui_font_data.cc)
  aerolab_embed_file(
    INPUT ${AEROLAB_UI_FONT}
    OUTPUT ${ui_font_source}
    NAMESPACE aerolab::editor
    SYMBOL kUiFontData)

  aerolab_add_library(aerolab_editor
    SOURCES
      editor/actions.cc
      editor/editor.cc
      editor/hierarchy_panel.cc
      editor/inspector_panel.cc
      editor/log_panel.cc
      editor/performance_panel.cc
      editor/settings_panel.cc
      editor/telemetry_panel.cc
      editor/theme.cc
      editor/ui_font.cc
      editor/viewport_panel.cc
      editor/widgets.cc
      ${ui_font_source}
    DEPS aerolab_world aerolab_scene aerolab_camera aerolab_render
         aerolab_core aerolab_math imgui)

  add_executable(aerolab
    app/main.cc
    app/application.cc
    app/imgui_layer.cc
    app/scene_presenter.cc
    app/screenshot.cc
    app/sdl_input.cc
    app/sdl_window.cc)
  target_include_directories(aerolab PRIVATE ${PROJECT_SOURCE_DIR}/src)
  target_link_libraries(aerolab PRIVATE
    aerolab_app_options
    aerolab_camera
    aerolab_core
    aerolab_editor
    aerolab_geometry
    aerolab_render
    aerolab_scene
    aerolab_world
    SDL3::SDL3
    aerolab_options)
  aerolab_configure_executable(aerolab)
endif()
