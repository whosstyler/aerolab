#include "app/application.h"

#include <SDL3/SDL_events.h>
#include <SDL3/SDL_mouse.h>
#include <SDL3/SDL_video.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "app/scene_presenter.h"
#include "app/screenshot.h"
#include "app/sdl_input.h"
#include "core/input.h"
#include "core/log.h"
#include "math/aabb.h"
#include "math/mat4.h"
#include "math/projection.h"
#include "math/quat.h"
#include "math/ray.h"
#include "math/vec3.h"
#include "render/gl.h"
#include "render/readback.h"
#include "scene/entity.h"
#include "world/debug_visualizer.h"
#include "world/picking.h"
#include "world/presets.h"
#include "world/simulation.h"

namespace aerolab {
namespace {

// A left press that moves less than this (in pixels) counts as a click.
constexpr double kClickSlop = 4.0;

GlProc LoadGlProc(const char* name) { return SDL_GL_GetProcAddress(name); }

const char* GlString(GLenum name) {
  const GLubyte* value = glGetString(name);
  return value != nullptr ? reinterpret_cast<const char*>(value) : "?";
}

}  // namespace

std::unique_ptr<Application> Application::Create(const AppOptions& options) {
  std::unique_ptr<Application> app(new Application(options));
  if (!app->Initialize()) return nullptr;
  return app;
}

Application::Application(const AppOptions& options) : options_(options) {}

Application::~Application() = default;

bool Application::Initialize() {
  sdl_ = SdlRuntime::Create();
  if (sdl_ == nullptr) return false;

  WindowConfig window_config;
  window_config.title = "AeroLab - 3D Simulation Sandbox";
  window_config.vsync = options_.vsync;
  window_config.hidden = options_.hidden_window;
  window_ = SdlWindow::Create(window_config);
  if (window_ == nullptr) return false;

  if (!LoadGlFunctions(&LoadGlProc)) {
    LogError("Failed to load the OpenGL 3.3 core function set");
    return false;
  }
  LogInfo("OpenGL {} | {} | {}", GlString(GL_VERSION), GlString(GL_RENDERER),
          GlString(GL_VENDOR));

  renderer_ = Renderer::Create();
  if (renderer_ == nullptr) return false;

  BuildDefaultScene();
  return true;
}

void Application::BuildDefaultScene() {
  BuildDemoScene(scene_);
  camera_rig_.LookAt({70.0, 38.0, 85.0}, {0.0, 12.0, 0.0});
  LogInfo("Demo scene: {} entities. Space = play/pause, . = step, R = reset",
          scene_.size());
}

int Application::Run() {
  int frame = 0;
  while (running_) {
    ProcessEvents();
    const double frame_dt = frame_clock_.Tick(wall_clock_.ElapsedSeconds());
    RunFrame(frame_dt);

    ++frame;
    if (options_.max_frames > 0 && frame >= options_.max_frames) {
      if (!options_.screenshot_path.empty() && !SaveScreenshot()) return 1;
      running_ = false;
    }
    window_->SwapBuffers();
  }
  return CheckGlErrors("shutdown") ? 0 : 1;
}

void Application::ProcessEvents() {
  input_.BeginFrame();
  const float density = SDL_GetWindowPixelDensity(window_->handle());
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    if (event.type == SDL_EVENT_QUIT ||
        event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED) {
      running_ = false;
    }
    ApplySdlEvent(event, density, input_);
  }
}

void Application::RunFrame(double frame_dt) {
  HandleShortcuts();
  UpdateCamera(frame_dt);
  simulation_.Update(scene_, frame_dt);
  for (const SimulationEvent& event : simulation_.TakeEvents()) {
    LogInfo("[t={:.2f}s] {}", event.time, event.message);
  }
  UpdatePicking();
  Render();
}

void Application::HandleShortcuts() {
  if (input_.WasPressed(Key::k1)) camera_rig_.SetMode(CameraMode::kFreeFly);
  if (input_.WasPressed(Key::k2)) camera_rig_.SetMode(CameraMode::kOrbit);
  if (input_.WasPressed(Key::k3)) camera_rig_.SetMode(CameraMode::kFollow);
  if (input_.WasPressed(Key::kSpace)) {
    simulation_.set_paused(!simulation_.paused());
  }
  if (input_.WasPressed(Key::kPeriod)) {
    simulation_.set_paused(true);
    simulation_.StepOnce(scene_);
  }
  if (input_.WasPressed(Key::kR)) simulation_.Reset(scene_);
  if (input_.WasPressed(Key::kEscape)) selection_.selected = kNoEntity;
  if (input_.WasPressed(Key::kF)) {
    if (const std::optional<Aabb> bounds = RenderWorldBounds(
            scene_, selection_.selected, simulation_.alpha())) {
      camera_rig_.Focus(Center(*bounds), BoundingRadius(*bounds));
    }
  }
  if (input_.WasPressed(Key::kDelete) && scene_.IsAlive(selection_.selected)) {
    const int removed = scene_.Destroy(selection_.selected);
    LogInfo("Removed {} entit{}", removed, removed == 1 ? "y" : "ies");
    selection_.selected = kNoEntity;
  }
}

void Application::UpdateCamera(double frame_dt) {
  // Capture the pointer while rotating so the view can spin indefinitely.
  const bool rotating = input_.IsDown(MouseButton::kRight);
  if (rotating != mouse_captured_ &&
      SDL_SetWindowRelativeMouseMode(window_->handle(), rotating)) {
    mouse_captured_ = rotating;
  }

  const CameraInput camera_input =
      CameraInputFromState(input_, /*use_mouse=*/true, /*use_keyboard=*/true,
                           static_cast<double>(window_->pixel_height()),
                           SDL_GetWindowPixelDensity(window_->handle()));
  std::optional<Vec3> follow_target;
  // Track the pose that is actually drawn, so the view does not jitter
  // against the interpolated body.
  if (const std::optional<Aabb> bounds =
          RenderWorldBounds(scene_, selection_.selected, simulation_.alpha())) {
    follow_target = Center(*bounds);
  }
  camera_rig_.Update(camera_input, frame_dt, follow_target);
}

void Application::UpdatePicking() {
  const int width = window_->pixel_width();
  const int height = window_->pixel_height();
  // A press that starts while the view is being rotated, or outside the
  // window, is never part of a selection click.
  if (input_.WasPressed(MouseButton::kLeft)) {
    click_armed_ = !mouse_captured_ && input_.mouse_inside();
  }
  const bool clicked =
      click_armed_ && input_.WasClicked(MouseButton::kLeft, kClickSlop);
  if (!input_.IsDown(MouseButton::kLeft)) click_armed_ = false;

  if (width <= 0 || height <= 0 || mouse_captured_ || !input_.mouse_inside()) {
    selection_.hovered = kNoEntity;
    return;
  }
  const RenderView view = MakeRenderView(width, height);
  const std::optional<Mat4> inverse = Inverse(view.projection * view.view);
  if (!inverse.has_value()) return;
  const ScreenRect rect{0.0, 0.0, static_cast<double>(width),
                        static_cast<double>(height)};
  const Ray ray = ScreenToWorldRay(*inverse, input_.mouse_position(), rect);
  const std::optional<PickHit> hit =
      PickEntity(scene_, ray, simulation_.alpha());
  selection_.hovered = hit.has_value() ? hit->entity : kNoEntity;

  if (clicked) {
    selection_.selected = selection_.hovered;
    if (const Entity* entity = scene_.Get(selection_.selected)) {
      LogInfo("Selected '{}'", entity->name);
    }
  }
}

RenderView Application::MakeRenderView(int width, int height) const {
  const Camera& camera = camera_rig_.camera();
  RenderView view;
  view.width = width;
  view.height = height;
  view.view = ViewMatrix(camera);
  view.projection = ProjectionMatrix(
      camera, static_cast<double>(width) / static_cast<double>(height));
  view.camera_position = camera.position;
  view.near_plane = camera.near_plane;
  view.far_plane = camera.far_plane;
  return view;
}

void Application::Render() {
  const int width = window_->pixel_width();
  const int height = window_->pixel_height();
  if (width <= 0 || height <= 0) return;  // Minimized.
  if (!viewport_target_.Resize(width, height, render_settings_.msaa_samples)) {
    return;
  }

  const double alpha = simulation_.alpha();
  debug_options_.prediction_step = simulation_.fixed_step();
  render_items_ = BuildRenderItems(scene_, selection_, alpha);
  world_lines_.Clear();
  overlay_lines_.Clear();
  AddDebugVisualization(scene_, debug_options_, alpha, selection_.selected,
                        world_lines_);
  AddSelectionOverlay(scene_, selection_, overlay_lines_, alpha);

  viewport_target_.BindForDrawing();
  renderer_->Render(MakeRenderView(width, height), render_items_, world_lines_,
                    overlay_lines_, render_settings_);
  viewport_target_.Resolve();
  viewport_target_.BlitToWindow(width, height);
}

bool Application::SaveScreenshot() {
  const int width = window_->pixel_width();
  const int height = window_->pixel_height();
  const std::vector<std::uint8_t> pixels =
      ReadFramebufferRgba(/*framebuffer=*/0, width, height);
  return WritePng(options_.screenshot_path, pixels, width, height);
}

}  // namespace aerolab
