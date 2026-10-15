#include "app/sdl_window.h"

#include <SDL3/SDL_error.h>
#include <SDL3/SDL_init.h>
#include <SDL3/SDL_video.h>

#include <memory>

#include "core/log.h"

namespace aerolab {

std::unique_ptr<SdlRuntime> SdlRuntime::Create() {
  if (!SDL_Init(SDL_INIT_VIDEO)) {
    LogError("SDL_Init failed: {}", SDL_GetError());
    return nullptr;
  }
  return std::unique_ptr<SdlRuntime>(new SdlRuntime());
}

SdlRuntime::~SdlRuntime() { SDL_Quit(); }

std::unique_ptr<SdlWindow> SdlWindow::Create(const WindowConfig& config) {
  // OpenGL 3.3 core is the baseline: it has VAOs, framebuffer objects,
  // multisample renderbuffers and timer queries, and runs almost everywhere.
  const struct {
    SDL_GLAttr attribute;
    int value;
  } kAttributes[] = {
      {SDL_GL_CONTEXT_MAJOR_VERSION, 3},
      {SDL_GL_CONTEXT_MINOR_VERSION, 3},
      {SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE},
      {SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG},
      {SDL_GL_DOUBLEBUFFER, 1},
      {SDL_GL_DEPTH_SIZE, 24},
      {SDL_GL_STENCIL_SIZE, 8},
  };
  for (const auto& [attribute, value] : kAttributes) {
    if (!SDL_GL_SetAttribute(attribute, value)) {
      LogWarning("SDL_GL_SetAttribute({}, {}) failed: {}",
                 static_cast<int>(attribute), value, SDL_GetError());
    }
  }

  SDL_WindowFlags flags =
      SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIGH_PIXEL_DENSITY;
  if (config.hidden) flags |= SDL_WINDOW_HIDDEN;

  SDL_Window* window = SDL_CreateWindow(config.title.c_str(), config.width,
                                        config.height, flags);
  if (window == nullptr) {
    LogError("SDL_CreateWindow failed: {}", SDL_GetError());
    return nullptr;
  }

  SDL_GLContext context = SDL_GL_CreateContext(window);
  if (context == nullptr) {
    LogError("OpenGL 3.3 core context creation failed: {}", SDL_GetError());
    SDL_DestroyWindow(window);
    return nullptr;
  }
  if (!SDL_GL_MakeCurrent(window, context)) {
    LogError("SDL_GL_MakeCurrent failed: {}", SDL_GetError());
    SDL_GL_DestroyContext(context);
    SDL_DestroyWindow(window);
    return nullptr;
  }

  std::unique_ptr<SdlWindow> result(new SdlWindow(window, context));
  result->SetVsync(config.vsync);
  return result;
}

SdlWindow::SdlWindow(SDL_Window* window, SDL_GLContext context)
    : window_(window), context_(context) {}

SdlWindow::~SdlWindow() {
  SDL_GL_DestroyContext(context_);
  SDL_DestroyWindow(window_);
}

void SdlWindow::SwapBuffers() { SDL_GL_SwapWindow(window_); }

bool SdlWindow::SetVsync(bool enabled) {
  if (!SDL_GL_SetSwapInterval(enabled ? 1 : 0)) {
    LogWarning("Could not {} vsync: {}", enabled ? "enable" : "disable",
               SDL_GetError());
    return false;
  }
  vsync_ = enabled;
  return true;
}

int SdlWindow::pixel_width() const {
  int width = 0;
  SDL_GetWindowSizeInPixels(window_, &width, nullptr);
  return width;
}

int SdlWindow::pixel_height() const {
  int height = 0;
  SDL_GetWindowSizeInPixels(window_, nullptr, &height);
  return height;
}

float SdlWindow::ContentScale() const {
  const float scale = SDL_GetWindowDisplayScale(window_);
  return scale > 0.0f ? scale : 1.0f;
}

}  // namespace aerolab
