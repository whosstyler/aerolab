#ifndef AEROLAB_APP_APPLICATION_H_
#define AEROLAB_APP_APPLICATION_H_

#include <memory>
#include <vector>

#include "app/imgui_layer.h"
#include "app/options.h"
#include "app/sdl_window.h"
#include "camera/camera_controller.h"
#include "core/clock.h"
#include "core/event_log.h"
#include "core/frame_stats.h"
#include "core/input.h"
#include "editor/editor.h"
#include "geometry/debug_draw.h"
#include "math/quat.h"
#include "math/vec3.h"
#include "render/gpu_timer.h"
#include "render/render_target.h"
#include "render/render_types.h"
#include "render/renderer.h"
#include "scene/scene.h"
#include "world/autonomy.h"
#include "world/debug_visualizer.h"
#include "world/selection.h"
#include "world/simulation.h"
#include "world/viewpoint.h"

namespace aerolab {

// Owns every subsystem and drives the main loop:
//
//   events -> camera -> fixed-step simulation -> 3D render into the
//   viewport target -> editor UI (shows the viewport image) -> present.
class Application {
 public:
  // Returns nullptr (after logging) if a subsystem fails to initialize.
  static std::unique_ptr<Application> Create(const AppOptions& options);

  ~Application();
  Application(const Application&) = delete;
  Application& operator=(const Application&) = delete;

  // Runs the main loop and returns the process exit code.
  int Run();

 private:
  explicit Application(const AppOptions& options);

  bool Initialize();
  void BuildDefaultScene();
  void ProcessEvents();
  void RunFrame(double frame_dt);
  void UpdateCamera(double frame_dt);
  // Fills walk_blockers_ from the scene for first-person walking.
  void CollectWalkBlockers();
  // Moves the first-person camera to the observer if the observer was moved
  // by something other than the camera since the last frame.
  void FollowObserverEdits();
  // The observer being looked through, if any: not drawn, labelled,
  // overlaid or pickable.
  EntityId HiddenEntity() const;
  void UpdateSimulation(double frame_dt);
  void RenderScene();
  void RenderUi();
  bool SaveScreenshot();

  AppOptions options_;
  // Declaration order matters: SDL outlives the window, the window (and its
  // GL context) outlives ImGui and every GL resource declared after it.
  std::unique_ptr<SdlRuntime> sdl_;
  std::unique_ptr<SdlWindow> window_;
  std::unique_ptr<ImGuiLayer> imgui_;
  std::unique_ptr<Renderer> renderer_;
  RenderTarget viewport_target_;
  GpuTimer scene_gpu_timer_;

  RenderSettings render_settings_;
  InputState input_;
  CameraRig camera_rig_;
  Scene scene_;
  SimulationSystem simulation_;
  DebugViewOptions debug_options_;
  SelectionState selection_;
  ViewpointState viewpoint_;
  // The observer pose the camera last wrote, to notice outside edits.
  ViewpointSync viewpoint_sync_;
  // Per-frame scratch for first-person walking.
  std::vector<SceneObstacle> obstacle_scratch_;
  std::vector<WalkBlocker> walk_blockers_;
  EventLog event_log_;
  FrameStats frame_stats_;
  editor::Editor editor_;

  bool mouse_captured_ = false;
  std::vector<RenderItem> render_items_;
  DebugDrawList world_lines_;
  DebugDrawList overlay_lines_;

  Stopwatch wall_clock_;
  FrameClock frame_clock_;
  bool running_ = true;
};

}  // namespace aerolab

#endif  // AEROLAB_APP_APPLICATION_H_
