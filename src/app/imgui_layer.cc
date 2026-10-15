#include "app/imgui_layer.h"

#include <memory>
#include <span>

#include "core/log.h"
#include "editor/theme.h"
#include "editor/ui_font.h"
#include "imgui.h"
#include "imgui_impl_opengl3.h"
#include "imgui_impl_sdl3.h"

namespace aerolab {
namespace {

constexpr float kBaseFontSize = 15.0f;

}  // namespace

std::unique_ptr<ImGuiLayer> ImGuiLayer::Create(SDL_Window* window,
                                               SDL_GLContext context,
                                               float dpi_scale) {
  IMGUI_CHECKVERSION();
  std::unique_ptr<ImGuiLayer> layer(new ImGuiLayer());
  layer->context_ = ImGui::CreateContext();

  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  // The default layout is rebuilt in code; nothing is written to disk.
  io.IniFilename = nullptr;
  io.ConfigWindowsMoveFromTitleBarOnly = true;

  layer->SetDpiScale(dpi_scale);

  const std::span<const unsigned char> font = editor::UiFontData();
  ImFontConfig config;
  // The data lives in the binary's read-only storage; the atlas must not
  // try to free it. ImGui only reads it, so dropping const is safe.
  config.FontDataOwnedByAtlas = false;
  config.OversampleH = 2;
  if (io.Fonts->AddFontFromMemoryTTF(const_cast<unsigned char*>(font.data()),
                                     static_cast<int>(font.size()),
                                     kBaseFontSize, &config) == nullptr) {
    LogWarning("Embedded UI font failed to load; using the default font");
  }

  layer->platform_initialized_ = ImGui_ImplSDL3_InitForOpenGL(window, context);
  if (!layer->platform_initialized_) {
    LogError("ImGui SDL3 backend failed to initialize");
    return nullptr;
  }
  layer->renderer_initialized_ = ImGui_ImplOpenGL3_Init("#version 330 core");
  if (!layer->renderer_initialized_) {
    LogError("ImGui OpenGL3 backend failed to initialize");
    return nullptr;
  }
  return layer;
}

ImGuiLayer::~ImGuiLayer() {
  if (renderer_initialized_) ImGui_ImplOpenGL3_Shutdown();
  if (platform_initialized_) ImGui_ImplSDL3_Shutdown();
  // Destroy our own context explicitly rather than whichever is current.
  ImGui::DestroyContext(context_);
}

void ImGuiLayer::SetDpiScale(float dpi_scale) {
  editor::ApplyTheme(dpi_scale);
  ImGui::GetStyle().FontSizeBase = kBaseFontSize;
}

void ImGuiLayer::ProcessEvent(const SDL_Event& event) {
  ImGui_ImplSDL3_ProcessEvent(&event);
}

void ImGuiLayer::BeginFrame() {
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplSDL3_NewFrame();
  ImGui::NewFrame();
}

void ImGuiLayer::Render() {
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

bool ImGuiLayer::WantsTextInput() const { return ImGui::GetIO().WantTextInput; }

}  // namespace aerolab
