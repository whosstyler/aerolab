#ifndef AEROLAB_APP_IMGUI_LAYER_H_
#define AEROLAB_APP_IMGUI_LAYER_H_

#include <SDL3/SDL_events.h>
#include <SDL3/SDL_video.h>

#include <memory>

struct ImGuiContext;

namespace aerolab {

// Owns the Dear ImGui context and its SDL3 + OpenGL3 platform/renderer
// backends. Destroy it before the GL context it was created with.
class ImGuiLayer {
 public:
  // Returns nullptr (after logging) if a backend fails to initialize.
  static std::unique_ptr<ImGuiLayer> Create(SDL_Window* window,
                                            SDL_GLContext context,
                                            float dpi_scale);

  ~ImGuiLayer();
  ImGuiLayer(const ImGuiLayer&) = delete;
  ImGuiLayer& operator=(const ImGuiLayer&) = delete;

  // Forwards an SDL event to ImGui.
  void ProcessEvent(const SDL_Event& event);

  // Starts a UI frame; build widgets between BeginFrame() and Render().
  void BeginFrame();

  // Finalizes the UI and draws it into the currently bound framebuffer.
  void Render();

  // True while ImGui is editing text (keyboard shortcuts should yield).
  bool WantsTextInput() const;

  // Re-applies the theme and font scale, e.g. after the window moved to a
  // monitor with a different content scale.
  void SetDpiScale(float dpi_scale);

 private:
  ImGuiLayer() = default;

  ImGuiContext* context_ = nullptr;

  bool platform_initialized_ = false;
  bool renderer_initialized_ = false;
};

}  // namespace aerolab

#endif  // AEROLAB_APP_IMGUI_LAYER_H_
