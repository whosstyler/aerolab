#ifndef AEROLAB_APP_APPLICATION_H_
#define AEROLAB_APP_APPLICATION_H_

#include <memory>

#include "app/options.h"
#include "app/sdl_window.h"
#include "core/clock.h"

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
  void ProcessEvents();
  void RunFrame(double frame_dt);
  void Render();
  bool SaveScreenshot();

  AppOptions options_;
  // Declaration order matters: SDL must outlive the window and GL context.
  std::unique_ptr<SdlRuntime> sdl_;
  std::unique_ptr<SdlWindow> window_;

  Stopwatch wall_clock_;
  FrameClock frame_clock_;
  double elapsed_seconds_ = 0.0;
  bool running_ = true;
};

}  // namespace aerolab

#endif  // AEROLAB_APP_APPLICATION_H_
