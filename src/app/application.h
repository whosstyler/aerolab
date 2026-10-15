#ifndef AEROLAB_APP_APPLICATION_H_
#define AEROLAB_APP_APPLICATION_H_

#include <memory>
#include <vector>

#include "app/options.h"
#include "app/scene_presenter.h"
#include "app/sdl_window.h"
#include "camera/camera_controller.h"
#include "core/clock.h"
#include "core/input.h"
#include "geometry/debug_draw.h"
#include "render/render_target.h"
#include "render/render_types.h"
#include "render/renderer.h"
#include "scene/scene.h"

namespace aerolab {

// Owns every subsystem and drives the main loop.
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
  void HandleShortcuts();
  void UpdateCamera(double frame_dt);
  void UpdatePicking();
  void Render();
  RenderView MakeRenderView(int width, int height) const;
  bool SaveScreenshot();

  AppOptions options_;
  // Declaration order matters: SDL must outlive the window and GL context,
  // and the context must outlive every GL resource declared after it.
  std::unique_ptr<SdlRuntime> sdl_;
  std::unique_ptr<SdlWindow> window_;
  std::unique_ptr<Renderer> renderer_;
  RenderTarget viewport_target_;

  RenderSettings render_settings_;
  InputState input_;
  CameraRig camera_rig_;
  Scene scene_;
  SelectionState selection_;

  // Set when a left press starts a potential selection click.
  bool click_armed_ = false;
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
