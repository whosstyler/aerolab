#include "app/application.h"

#include <SDL3/SDL_events.h>
#include <SDL3/SDL_mouse.h>
#include <SDL3/SDL_video.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "app/scene_presenter.h"
#include "app/screenshot.h"
#include "app/sdl_input.h"
#include "core/log.h"
#include "math/aabb.h"
#include "math/vec3.h"
#include "render/gl.h"
#include "render/readback.h"
#include "scene/entity.h"
#include "world/presets.h"

namespace aerolab {
namespace {

GlProc LoadGlProc(const char* name) { return SDL_GL_GetProcAddress(name); }

const char* GlString(GLenum name) {
  const GLubyte* value = glGetString(name);
  return value != nullptr ? reinterpret_cast<const char*>(value) : "?";
}

LogLevel SimulationEventLevel(SimulationEvent::Kind kind) {
  switch (kind) {
    case SimulationEvent::Kind::kDivergence:
    case SimulationEvent::Kind::kOutOfBounds:
      return LogLevel::kWarning;
    case SimulationEvent::Kind::kBurnout:
    case SimulationEvent::Kind::kImpact:
    case SimulationEvent::Kind::kDecision:
      return LogLevel::kInfo;
  }
  return LogLevel::kInfo;
}

}  // namespace

std::unique_ptr<Application> Application::Create(const AppOptions& options) {
  std::unique_ptr<Application> app(new Application(options));
  if (!app->Initialize()) return nullptr;
  return app;
}

Application::Application(const AppOptions& options) : options_(options) {}

Application::~Application() {
  // The sink captures `this`; detach it before members are destroyed.
  SetLogSink(nullptr);
  if (mouse_captured_ && window_ != nullptr) {
    SDL_SetWindowRelativeMouseMode(window_->handle(), false);
  }
}

bool Application::Initialize() {
  // Mirror every log message into the editor's event log.
  SetLogSink([this](LogLevel level, std::string_view message) {
    event_log_.Add(level, EventSource::kApp, std::string(message),
                   wall_clock_.ElapsedSeconds());
  });

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

  imgui_ = ImGuiLayer::Create(window_->handle(), window_->gl_context(),
                              window_->ContentScale());
  if (imgui_ == nullptr) return false;

  renderer_ = Renderer::Create();
  if (renderer_ == nullptr) return false;

  BuildDefaultScene();
  return true;
}

void Application::BuildDefaultScene() {
  BuildDemoScene(scene_);
  camera_rig_.LookAt({48.0, 26.0, 60.0}, {0.0, 10.0, 0.0});
  LogInfo("Demo scene ready: {} entities. Press F1 for controls.",
          scene_.size());
  if (!options_.select_name.empty()) {
    for (EntityId id : scene_.roots()) {
      const Entity* entity = scene_.Get(id);
      if (entity == nullptr || entity->name != options_.select_name) continue;
      selection_.selected = id;
      camera_rig_.SetMode(CameraMode::kFollow);
      if (const std::optional<Aabb> bounds = scene_.WorldBounds(id)) {
        camera_rig_.Focus(Center(*bounds), BoundingRadius(*bounds) * 4.0);
      }
      break;  // Roots only; the first match wins.
    }
    if (!IsSet(selection_.selected)) {
      LogWarning("--select: no object named '{}'", options_.select_name);
    }
  }
}

int Application::Run() {
  int frame = 0;
  while (running_) {
    ProcessEvents();
    const double frame_dt = frame_clock_.Tick(wall_clock_.ElapsedSeconds());
    if (frame_clock_.raw_delta_seconds() > 0.0) {
      frame_stats_.Record(FrameTiming::kFrame,
                          frame_clock_.raw_delta_seconds() * 1000.0);
    }
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
    if (event.type == SDL_EVENT_WINDOW_DISPLAY_SCALE_CHANGED) {
      imgui_->SetDpiScale(window_->ContentScale());
    }
    imgui_->ProcessEvent(event);
    ApplySdlEvent(event, density, input_);
  }
}

void Application::RunFrame(double frame_dt) {
  // Simulation first, so the follow camera tracks the pose drawn this frame.
  UpdateSimulation(frame_dt);
  UpdateCamera(frame_dt);
  RenderScene();
  RenderUi();
}

void Application::UpdateCamera(double frame_dt) {
  const editor::ViewportState& viewport = editor_.viewport();
  // The camera only listens to the mouse over the viewport (or during a
  // drag that started there), and to the keyboard while the viewport is
  // in use and no text field is being edited.
  const bool use_mouse = viewport.hovered || viewport.camera_drag;
  const bool use_keyboard =
      (viewport.hovered || viewport.focused || viewport.camera_drag) &&
      !imgui_->WantsTextInput();

  // Capture the pointer while rotating so the view can spin indefinitely.
  const bool rotating =
      viewport.camera_drag && input_.IsDown(MouseButton::kRight);
  if (rotating != mouse_captured_ &&
      SDL_SetWindowRelativeMouseMode(window_->handle(), rotating)) {
    mouse_captured_ = rotating;
  }

  CameraInput camera_input =
      CameraInputFromState(input_, use_mouse, use_keyboard,
                           static_cast<double>(viewport.pixel_height),
                           SDL_GetWindowPixelDensity(window_->handle()));
  if (!viewport.hovered) camera_input.wheel = 0.0;

  // Track the pose that is actually drawn so the view does not jitter.
  std::optional<Vec3> follow_target;
  if (const std::optional<Aabb> bounds =
          RenderWorldBounds(scene_, selection_.selected, simulation_.alpha())) {
    follow_target = Center(*bounds);
  }
  camera_rig_.Update(camera_input, frame_dt, follow_target);
}

void Application::UpdateSimulation(double frame_dt) {
  simulation_.Update(scene_, frame_dt);
  frame_stats_.Record(FrameTiming::kSimulation,
                      simulation_.stats().step_milliseconds);
  for (SimulationEvent& event : simulation_.TakeEvents()) {
    // Mirror physics events to stderr too (agent decisions are frequent and
    // stay in the event log); log entries are tagged with sim time.
    if (event.kind != SimulationEvent::Kind::kDecision) {
      std::fprintf(stderr, "[sim t=%.2fs] %s\n", event.time,
                   event.message.c_str());
    }
    event_log_.Add(SimulationEventLevel(event.kind), EventSource::kSimulation,
                   std::move(event.message), wall_clock_.ElapsedSeconds(),
                   event.time);
  }
}

void Application::RenderScene() {
  Stopwatch watch;
  const editor::ViewportState& viewport = editor_.viewport();
  if (!viewport_target_.Resize(viewport.pixel_width, viewport.pixel_height,
                               render_settings_.msaa_samples)) {
    return;
  }
  const int width = viewport_target_.width();
  const int height = viewport_target_.height();
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

  const double alpha = simulation_.alpha();
  debug_options_.prediction_step = simulation_.fixed_step();
  render_items_ = BuildRenderItems(scene_, selection_, alpha);
  world_lines_.Clear();
  overlay_lines_.Clear();
  AddDebugVisualization(scene_, debug_options_, alpha, selection_.selected,
                        world_lines_);
  AddSelectionOverlay(scene_, selection_, overlay_lines_, alpha);

  scene_gpu_timer_.Begin();
  viewport_target_.BindForDrawing();
  renderer_->Render(view, render_items_, world_lines_, overlay_lines_,
                    render_settings_);
  viewport_target_.Resolve();
  scene_gpu_timer_.End();

  frame_stats_.Record(FrameTiming::kSceneCpu, watch.ElapsedMilliseconds());
  if (const std::optional<double> gpu = scene_gpu_timer_.PollMilliseconds()) {
    frame_stats_.Record(FrameTiming::kSceneGpu, *gpu);
  }
}

void Application::RenderUi() {
  Stopwatch watch;
  imgui_->BeginFrame();
  editor::EditorFrame frame;
  frame.scene = &scene_;
  frame.simulation = &simulation_;
  frame.camera = &camera_rig_;
  frame.render_settings = &render_settings_;
  frame.debug_options = &debug_options_;
  frame.selection = &selection_;
  frame.event_log = &event_log_;
  frame.frame_stats = &frame_stats_;
  frame.render_stats = &renderer_->stats();
  frame.viewport_texture = viewport_target_.color_texture();
  frame.texture_width = std::max(viewport_target_.width(), 1);
  frame.texture_height = std::max(viewport_target_.height(), 1);
  frame.wall_seconds = wall_clock_.ElapsedSeconds();
  frame.vsync = window_->vsync();
  const editor::EditorRequests requests = editor_.Draw(frame);
  if (requests.quit) running_ = false;
  if (requests.set_vsync) window_->SetVsync(requests.vsync);

  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glViewport(0, 0, window_->pixel_width(), window_->pixel_height());
  glClearColor(0.094f, 0.102f, 0.114f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);
  imgui_->Render();
  frame_stats_.Record(FrameTiming::kUi, watch.ElapsedMilliseconds());
}

bool Application::SaveScreenshot() {
  const int width = window_->pixel_width();
  const int height = window_->pixel_height();
  const std::vector<std::uint8_t> pixels =
      ReadFramebufferRgba(/*framebuffer=*/0, width, height);
  return WritePng(options_.screenshot_path, pixels, width, height);
}

}  // namespace aerolab
