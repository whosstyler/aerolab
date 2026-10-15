#include "editor/editor.h"

#include <algorithm>
#include <array>
#include <format>
#include <string>

#include "core/log.h"
#include "editor/panels.h"
#include "editor/theme.h"
#include "editor/widgets.h"
#include "imgui.h"
#include "imgui_internal.h"
#include "world/presets.h"

namespace aerolab::editor {
namespace {

constexpr ImGuiWindowFlags kFixedWindowFlags =
    ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
    ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoDocking |
    ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;

// Time scale presets offered in the toolbar dropdown.
constexpr std::array<double, 8> kTimeScales = {0.05, 0.1, 0.25, 0.5,
                                               1.0,  2.0, 4.0,  8.0};

void ToolbarSeparator() {
  ImGui::SameLine(0.0f, ImGui::GetStyle().ItemSpacing.x * 1.5f);
  const ImVec2 p = ImGui::GetCursorScreenPos();
  const float h = ImGui::GetFrameHeight();
  ImGui::GetWindowDrawList()->AddLine(ImVec2(p.x, p.y + 3),
                                      ImVec2(p.x, p.y + h - 3), theme::kBorder);
  ImGui::Dummy(ImVec2(1.0f, h));
  ImGui::SameLine(0.0f, ImGui::GetStyle().ItemSpacing.x * 1.5f);
}

void DrawHelpWindow(bool* open) {
  ImGui::SetNextWindowSize(ImVec2(460.0f * Dpi(), 0.0f), ImGuiCond_Appearing);
  if (!ImGui::Begin("Controls", open, ImGuiWindowFlags_NoDocking)) {
    ImGui::End();
    return;
  }
  constexpr std::array<std::array<const char*, 2>, 13> kRows = {{
      {"Right drag", "Orbit / look around"},
      {"Middle drag", "Pan"},
      {"Mouse wheel", "Zoom (orbit) / speed (free-fly)"},
      {"W A S D, Q E", "Move (free-fly) / slide pivot (orbit)"},
      {"Shift / Ctrl", "Faster / slower"},
      {"Left click", "Select object"},
      {"F", "Focus selection"},
      {"1 / 2 / 3", "Free-fly / orbit / follow camera"},
      {"Space", "Play / pause simulation"},
      {".", "Single step"},
      {"R", "Reset simulation"},
      {"Delete", "Remove selection"},
      {"Esc", "Clear selection"},
  }};
  if (ImGui::BeginTable("controls", 2, ImGuiTableFlags_RowBg)) {
    for (const auto& row : kRows) {
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::TextUnformatted(row[0]);
      ImGui::TableSetColumnIndex(1);
      ImGui::TextDisabled("%s", row[1]);
    }
    ImGui::EndTable();
  }
  ImGui::End();
}

}  // namespace

EditorRequests Editor::Draw(EditorFrame& frame) {
  EditorRequests requests;
  requests.vsync = frame.vsync;

  HandleShortcuts(frame);
  DrawMenuBar(frame, requests);

  const ImGuiViewport* main = ImGui::GetMainViewport();
  const float toolbar_height =
      ImGui::GetFrameHeight() + ImGui::GetStyle().WindowPadding.y * 2.0f;
  DrawToolbar(frame, main->WorkPos.y, toolbar_height);

  // Host window filling the rest of the screen with the dockspace.
  ImGui::SetNextWindowPos(
      ImVec2(main->WorkPos.x, main->WorkPos.y + toolbar_height));
  ImGui::SetNextWindowSize(
      ImVec2(main->WorkSize.x, main->WorkSize.y - toolbar_height));
  ImGui::SetNextWindowViewport(main->ID);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
  ImGui::Begin("##DockHost", nullptr, kFixedWindowFlags);
  ImGui::PopStyleVar(2);
  const ImGuiID dockspace_id = ImGui::GetID("MainDockSpace");
  if (!state_.layout_initialized) {
    BuildDefaultLayout(dockspace_id, ImGui::GetWindowWidth(),
                       ImGui::GetWindowHeight());
  }
  ImGui::DockSpace(dockspace_id, ImVec2(0, 0), ImGuiDockNodeFlags_None);
  ImGui::End();

  DrawViewportPanel(frame, state_, viewport_);
  if (state_.show_hierarchy) DrawHierarchyPanel(frame, state_);
  if (state_.show_inspector) DrawInspectorPanel(frame);
  if (state_.show_settings) DrawSettingsPanel(frame, state_);
  if (state_.show_telemetry) DrawTelemetryPanel(frame, state_);
  if (state_.show_log) DrawEventLogPanel(frame, state_);
  if (state_.show_performance) DrawPerformancePanel(frame, state_, requests);
  if (state_.show_help) DrawHelpWindow(&state_.show_help);
  if (state_.default_focus_pending) {
    // Bring the most useful tab of each dock node to the front, then give
    // the viewport keyboard focus.
    state_.default_focus_pending = false;
    ImGui::SetWindowFocus(kTelemetryWindow);
    ImGui::SetWindowFocus(kInspectorWindow);
    ImGui::SetWindowFocus(kViewportWindow);
  }
  return requests;
}

void Editor::ResetLayout() {
  state_.show_hierarchy = state_.show_inspector = state_.show_settings = true;
  state_.show_telemetry = state_.show_log = state_.show_performance = true;
  state_.layout_initialized = false;
}

void Editor::BuildDefaultLayout(unsigned int dockspace_id, float width,
                                float height) {
  state_.layout_initialized = true;
  ImGui::DockBuilderRemoveNode(dockspace_id);
  ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace);
  // Size from the host window (the screen minus menu bar and toolbar).
  ImGui::DockBuilderSetNodeSize(dockspace_id, ImVec2(width, height));

  ImGuiID center = dockspace_id;
  const ImGuiID left = ImGui::DockBuilderSplitNode(center, ImGuiDir_Left, 0.17f,
                                                   nullptr, &center);
  const ImGuiID right = ImGui::DockBuilderSplitNode(center, ImGuiDir_Right,
                                                    0.27f, nullptr, &center);
  const ImGuiID bottom = ImGui::DockBuilderSplitNode(center, ImGuiDir_Down,
                                                     0.36f, nullptr, &center);
  ImGui::DockBuilderDockWindow(kViewportWindow, center);
  ImGui::DockBuilderDockWindow(kHierarchyWindow, left);
  ImGui::DockBuilderDockWindow(kInspectorWindow, right);
  ImGui::DockBuilderDockWindow(kSettingsWindow, right);
  ImGui::DockBuilderDockWindow(kTelemetryWindow, bottom);
  ImGui::DockBuilderDockWindow(kLogWindow, bottom);
  ImGui::DockBuilderDockWindow(kPerformanceWindow, bottom);
  ImGui::DockBuilderFinish(dockspace_id);
  // The viewport hides its tab bar: it is the permanent centre of the UI.
  // (DockNode flags live in imgui_internal.h; revisit on ImGui upgrades.)
  if (ImGuiDockNode* node = ImGui::DockBuilderGetNode(center)) {
    node->LocalFlags |= ImGuiDockNodeFlags_AutoHideTabBar;
  }
  state_.default_focus_pending = true;
}

void Editor::HandleShortcuts(EditorFrame& frame) {
  const ImGuiIO& io = ImGui::GetIO();
  // Single-key shortcuts must not fire while typing, while a popup or
  // combo is open, while a widget is being dragged, or with modifiers held.
  if (io.WantTextInput || ImGui::IsAnyItemActive() ||
      ImGui::IsPopupOpen(nullptr, ImGuiPopupFlags_AnyPopupId |
                                      ImGuiPopupFlags_AnyPopupLevel) ||
      io.KeyCtrl || io.KeyAlt || io.KeySuper) {
    return;
  }
  const auto pressed = [](ImGuiKey key) {
    return ImGui::IsKeyPressed(key, /*repeat=*/false);
  };
  SimulationSystem& sim = *frame.simulation;
  if (pressed(ImGuiKey_Space)) sim.set_paused(!sim.paused());
  if (pressed(ImGuiKey_Period)) {
    sim.set_paused(true);
    sim.StepOnce(*frame.scene);
  }
  if (pressed(ImGuiKey_R)) ResetSimulation(frame);
  if (pressed(ImGuiKey_F)) FocusSelection(frame);
  if (pressed(ImGuiKey_Delete)) DeleteSelection(frame);
  if (pressed(ImGuiKey_Escape)) frame.selection->selected = kNoEntity;
  if (pressed(ImGuiKey_1)) frame.camera->SetMode(CameraMode::kFreeFly);
  if (pressed(ImGuiKey_2)) frame.camera->SetMode(CameraMode::kOrbit);
  if (pressed(ImGuiKey_3)) frame.camera->SetMode(CameraMode::kFollow);
  if (pressed(ImGuiKey_F1)) state_.show_help = !state_.show_help;
}

void Editor::DrawMenuBar(EditorFrame& frame, EditorRequests& requests) {
  if (!ImGui::BeginMainMenuBar()) return;
  if (ImGui::BeginMenu("File")) {
    if (ImGui::MenuItem("Reset simulation", "R")) ResetSimulation(frame);
    ImGui::Separator();
    if (ImGui::MenuItem("Quit", "Alt+F4")) requests.quit = true;
    ImGui::EndMenu();
  }
  if (ImGui::BeginMenu("Scene")) {
    if (ImGui::BeginMenu("Spawn")) {
      SpawnMenuItems(frame);
      ImGui::EndMenu();
    }
    if (ImGui::MenuItem("Integrator comparison")) {
      const Vec3 center = frame.camera->pivot() + Vec3{0.0, 8.0, 0.0};
      frame.selection->selected =
          SpawnIntegratorComparison(*frame.scene, center);
      LogInfo("Spawned an integrator comparison (one orbit per method)");
    }
    ImGui::Separator();
    if (ImGui::MenuItem("Load demo scene")) {
      frame.scene->Clear();
      BuildDemoScene(*frame.scene);
      ResetSimulation(frame);
      frame.selection->selected = kNoEntity;
      LogInfo("Loaded the demo scene");
    }
    if (ImGui::MenuItem("Clear scene")) {
      frame.scene->Clear();
      ResetSimulation(frame);
      frame.selection->selected = kNoEntity;
      LogInfo("Cleared the scene");
    }
    ImGui::Separator();
    if (ImGui::MenuItem("Delete selection", "Del", false,
                        frame.scene->IsAlive(frame.selection->selected))) {
      DeleteSelection(frame);
    }
    ImGui::EndMenu();
  }
  if (ImGui::BeginMenu("View")) {
    ImGui::MenuItem(kHierarchyWindow, nullptr, &state_.show_hierarchy);
    ImGui::MenuItem(kInspectorWindow, nullptr, &state_.show_inspector);
    ImGui::MenuItem(kSettingsWindow, nullptr, &state_.show_settings);
    ImGui::MenuItem(kTelemetryWindow, nullptr, &state_.show_telemetry);
    ImGui::MenuItem(kLogWindow, nullptr, &state_.show_log);
    ImGui::MenuItem(kPerformanceWindow, nullptr, &state_.show_performance);
    ImGui::Separator();
    ImGui::MenuItem("Object labels", nullptr, &state_.show_labels);
    ImGui::MenuItem("Viewport HUD", nullptr, &state_.show_hud);
    ImGui::Separator();
    if (ImGui::MenuItem("Focus selection", "F")) FocusSelection(frame);
    if (ImGui::MenuItem("Frame whole scene")) FrameWholeScene(frame);
    if (ImGui::MenuItem("Reset layout")) ResetLayout();
    ImGui::EndMenu();
  }
  if (ImGui::BeginMenu("Help")) {
    ImGui::MenuItem("Controls", "F1", &state_.show_help);
    ImGui::EndMenu();
  }
  ImGui::EndMainMenuBar();
}

void Editor::DrawToolbar(EditorFrame& frame, float y, float height) {
  const ImGuiViewport* main = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(ImVec2(main->WorkPos.x, y));
  ImGui::SetNextWindowSize(ImVec2(main->WorkSize.x, height));
  ImGui::SetNextWindowViewport(main->ID);
  ImGui::PushStyleColor(ImGuiCol_WindowBg, IM_COL32(24, 26, 29, 255));
  ImGui::Begin("##Toolbar", nullptr, kFixedWindowFlags);
  ImGui::PopStyleColor();

  SimulationSystem& sim = *frame.simulation;
  // Transport controls.
  if (IconButton("##play", sim.paused() ? Icon::kPlay : Icon::kPause,
                 sim.paused() ? "Play (Space)" : "Pause (Space)",
                 !sim.paused())) {
    sim.set_paused(!sim.paused());
  }
  ImGui::SameLine();
  if (IconButton("##step", Icon::kStep, "Single step (.)")) {
    sim.set_paused(true);
    sim.StepOnce(*frame.scene);
  }
  ImGui::SameLine();
  if (IconButton("##reset", Icon::kReset, "Reset to launch conditions (R)")) {
    ResetSimulation(frame);
  }
  ImGui::SameLine();
  ImGui::AlignTextToFramePadding();
  ImGui::Text("t = %8.2f s", sim.time());

  ToolbarSeparator();
  ImGui::TextDisabled("Speed");
  ImGui::SameLine();
  ImGui::SetNextItemWidth(78.0f * ImGui::GetStyle().FontScaleDpi);
  const std::string scale_label = std::format("x{:g}", sim.time_scale());
  if (ImGui::BeginCombo("##timescale", scale_label.c_str())) {
    for (double scale : kTimeScales) {
      if (ImGui::Selectable(std::format("x{:g}", scale).c_str(),
                            scale == sim.time_scale())) {
        sim.set_time_scale(scale);
      }
    }
    ImGui::EndCombo();
  }

  ToolbarSeparator();
  ImGui::TextDisabled("Camera");
  ImGui::SameLine();
  constexpr std::array<CameraMode, 3> kModes = {
      CameraMode::kOrbit, CameraMode::kFreeFly, CameraMode::kFollow};
  for (CameraMode mode : kModes) {
    bool active = frame.camera->mode() == mode;
    if (ToggleButton(CameraModeName(mode), &active,
                     mode == CameraMode::kFollow
                         ? "Orbit around the selected object as it moves (3)"
                         : nullptr)) {
      frame.camera->SetMode(mode);
    }
    ImGui::SameLine();
  }

  ToolbarSeparator();
  DebugViewOptions& d = *frame.debug_options;
  RenderSettings& r = *frame.render_settings;
  ToggleButton("Trails", &d.trails, "Motion trails");
  ImGui::SameLine();
  ToggleButton("Velocity", &d.velocity_vectors, "Velocity vectors (cyan)");
  ImGui::SameLine();
  ToggleButton("Accel", &d.acceleration_vectors,
               "Acceleration vectors (amber)");
  ImGui::SameLine();
  ToggleButton("Forces", &d.force_vectors,
               "Per-term force vectors of the selection");
  ImGui::SameLine();
  ToggleButton("Frames", &d.local_frames, "Local coordinate frames");
  ImGui::SameLine();
  ToggleButton("Bounds", &d.bounding_boxes, "Oriented bounding boxes");
  ImGui::SameLine();
  ToggleButton("Predict", &d.predicted_path, "Look-ahead of the selected body");
  ImGui::SameLine();
  ToggleButton("AI", &d.agent_plans,
               "Autonomous agents: destinations, flight area and what "
               "they are avoiding");
  ImGui::SameLine();
  ToggleButton("Grid", &r.show_grid, "World grid");

  // Right-aligned frame rate.
  const std::string fps =
      std::format("{:.0f} FPS  {:.2f} ms", frame.frame_stats->Fps(),
                  frame.frame_stats->Summarize(FrameTiming::kFrame).average);
  const float fps_width = ImGui::CalcTextSize(fps.c_str()).x;
  ImGui::SameLine();
  const float fps_x = ImGui::GetWindowWidth() - fps_width -
                      ImGui::GetStyle().WindowPadding.x * 2.0f;
  // Only when it fits; on narrow windows the toggles take priority.
  if (fps_x > ImGui::GetCursorPosX()) {
    ImGui::SetCursorPosX(fps_x);
    ImGui::AlignTextToFramePadding();
    ImGui::TextDisabled("%s", fps.c_str());
  }
  ImGui::End();
}

}  // namespace aerolab::editor
