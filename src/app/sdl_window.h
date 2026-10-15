#ifndef AEROLAB_APP_SDL_WINDOW_H_
#define AEROLAB_APP_SDL_WINDOW_H_

#include <SDL3/SDL_video.h>

#include <memory>
#include <string>

namespace aerolab {

// RAII guard for SDL's video subsystem. Construct before any window and
// destroy after the last one.
class SdlRuntime {
 public:
  // Returns nullptr (after logging) if SDL cannot be initialized.
  static std::unique_ptr<SdlRuntime> Create();

  ~SdlRuntime();
  SdlRuntime(const SdlRuntime&) = delete;
  SdlRuntime& operator=(const SdlRuntime&) = delete;

 private:
  SdlRuntime() = default;
};

struct WindowConfig {
  std::string title = "AeroLab";
  int width = 1600;
  int height = 900;
  bool vsync = true;
  // Hidden windows are used by automated smoke tests.
  bool hidden = false;
};

// Owns an SDL window and its OpenGL 3.3 core-profile context.
class SdlWindow {
 public:
  // Returns nullptr (after logging) if the window or context cannot be made.
  static std::unique_ptr<SdlWindow> Create(const WindowConfig& config);

  ~SdlWindow();
  SdlWindow(const SdlWindow&) = delete;
  SdlWindow& operator=(const SdlWindow&) = delete;

  SDL_Window* handle() const { return window_; }
  SDL_GLContext gl_context() const { return context_; }

  void SwapBuffers();
  bool SetVsync(bool enabled);
  bool vsync() const { return vsync_; }

  // Size of the default framebuffer in pixels.
  int pixel_width() const;
  int pixel_height() const;

  // Ratio of UI pixels to logical points (1.0 on a standard-DPI display).
  float ContentScale() const;

 private:
  SdlWindow(SDL_Window* window, SDL_GLContext context);

  SDL_Window* window_;
  SDL_GLContext context_;
  bool vsync_ = false;
};

}  // namespace aerolab

#endif  // AEROLAB_APP_SDL_WINDOW_H_
