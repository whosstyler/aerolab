#ifndef AEROLAB_EDITOR_EDITOR_H_
#define AEROLAB_EDITOR_EDITOR_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "camera/camera_controller.h"
#include "core/event_log.h"
#include "core/frame_stats.h"
#include "render/render_types.h"
#include "scene/scene.h"
#include "world/debug_visualizer.h"
#include "world/selection.h"
#include "world/simulation.h"

namespace aerolab::editor {

// Everything the editor reads or edits during one frame. The application
// owns all of it; pointers are only valid for the duration of Draw().
struct EditorFrame {
  Scene* scene = nullptr;
  SimulationSystem* simulation = nullptr;
  CameraRig* camera = nullptr;
  RenderSettings* render_settings = nullptr;
  DebugViewOptions* debug_options = nullptr;
  SelectionState* selection = nullptr;
  EventLog* event_log = nullptr;

  const FrameStats* frame_stats = nullptr;
  const RenderStats* render_stats = nullptr;

  // The resolved 3D viewport image and its size in pixels.
  std::uint64_t viewport_texture = 0;
  int texture_width = 1;
  int texture_height = 1;

  double wall_seconds = 0.0;
  bool vsync = true;
};

// What the viewport panel reported this frame. The application uses it on
// the next frame to size the render target and route camera input, so
// camera routing lags the UI by one frame (imperceptible in practice).
struct ViewportState {
  int pixel_width = 1280;  // Requested render target size.
  int pixel_height = 720;
  bool hovered = false;      // Pointer over the 3D image.
  bool focused = false;      // Viewport window has keyboard focus.
  bool camera_drag = false;  // A rotate/pan drag that began in the viewport.
  bool click_armed = false;  // Left button went down over the 3D image.
};

// Requests the editor makes of the application.
struct EditorRequests {
  bool quit = false;
  bool set_vsync = false;
  bool vsync = true;
};

// Per-panel UI state that persists between frames.
struct EditorState {
  bool show_hierarchy = true;
  bool show_inspector = true;
  bool show_settings = true;
  bool show_telemetry = true;
  bool show_log = true;
  bool show_performance = true;
  bool show_labels = true;
  bool show_hud = true;
  bool show_help = false;
  bool layout_initialized = false;
  // Selects the default tabs once all windows exist.
  bool default_focus_pending = true;

  // Event log panel.
  bool log_auto_scroll = true;
  bool log_show_debug = false;
  std::string log_filter;
  std::vector<std::size_t> log_rows;  // Reused: visible entry indices.

  // Hierarchy panel.
  std::string hierarchy_filter;
  bool hierarchy_filter_changed = false;

  // Reused per-frame scratch for plots, to avoid reallocating every frame.
  // Fixed-size containers: references into them stay valid while a panel
  // fills several series.
  std::array<std::vector<float>, 4> body_plot_scratch;
  std::vector<std::vector<float>> group_plot_scratch;  // Sized up front.
  std::vector<float> frame_time_scratch;
};

// The editor shell: menu bar, toolbar, docking layout and every panel.
class Editor {
 public:
  Editor() = default;

  // Builds this frame's UI. Call between ImGui::NewFrame() and
  // ImGui::Render().
  EditorRequests Draw(EditorFrame& frame);

  const ViewportState& viewport() const { return viewport_; }

  // Shows every panel again and rebuilds the default dock layout.
  void ResetLayout();

 private:
  void DrawMenuBar(EditorFrame& frame, EditorRequests& requests);
  void DrawToolbar(EditorFrame& frame, float y, float height);
  void BuildDefaultLayout(unsigned int dockspace_id, float width, float height);
  void HandleShortcuts(EditorFrame& frame);

  EditorState state_;
  ViewportState viewport_;
};

}  // namespace aerolab::editor

#endif  // AEROLAB_EDITOR_EDITOR_H_
