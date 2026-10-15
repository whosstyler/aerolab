#include "app/application.h"

#include <SDL3/SDL_events.h>
#include <SDL3/SDL_video.h>

#include <cstdint>
#include <memory>
#include <vector>

#include "app/screenshot.h"
#include "core/log.h"
#include "render/gl.h"
#include "render/readback.h"

namespace aerolab {
namespace {

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
  return true;
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
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    if (event.type == SDL_EVENT_QUIT ||
        event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED) {
      running_ = false;
    }
  }
}

void Application::RunFrame(double frame_dt) {
  elapsed_seconds_ += frame_dt;
  Render();
}

void Application::Render() {
  glViewport(0, 0, window_->pixel_width(), window_->pixel_height());
  glClearColor(0.42f, 0.55f, 0.70f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

bool Application::SaveScreenshot() {
  const int width = window_->pixel_width();
  const int height = window_->pixel_height();
  const std::vector<std::uint8_t> pixels =
      ReadFramebufferRgba(/*framebuffer=*/0, width, height);
  return WritePng(options_.screenshot_path, pixels, width, height);
}

}  // namespace aerolab
