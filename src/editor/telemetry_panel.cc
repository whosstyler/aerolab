// Telemetry: a simulation timeline with event markers, plus flight-data
// plots for the selected body (or for each body under a selected group,
// e.g. the integrator comparison).

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <format>
#include <limits>
#include <span>
#include <string>
#include <vector>

#include "core/event_log.h"
#include "core/log.h"
#include "editor/panels.h"
#include "editor/theme.h"
#include "editor/widgets.h"
#include "imgui.h"
#include "scene/entity.h"
#include "scene/scene.h"

namespace aerolab::editor {
namespace {

// Seconds of history shown on the timeline.
constexpr int kTimelineWindow = 60;

ImU32 ToImColor(const Color& c) {
  return ImGui::ColorConvertFloat4ToU32(ImVec4(c.r, c.g, c.b, 1.0f));
}

ImU32 LevelColor(LogLevel level) {
  switch (level) {
    case LogLevel::kError:
      return theme::kError;
    case LogLevel::kWarning:
      return theme::kWarning;
    default:
      return theme::kAccent;
  }
}

void DrawTimeline(const EditorFrame& frame) {
  const SimulationSystem& sim = *frame.simulation;
  const double now = sim.time();
  const double start = std::max(0.0, now - kTimelineWindow);
  const double end = std::max(start + kTimelineWindow, now);

  const float height = ImGui::GetFrameHeight() * 1.4f;
  const ImVec2 origin = ImGui::GetCursorScreenPos();
  const float width = std::max(ImGui::GetContentRegionAvail().x, 50.0f);
  ImGui::InvisibleButton("##timeline", ImVec2(width, height));
  const bool hovered = ImGui::IsItemHovered();
  ImDrawList* draw = ImGui::GetWindowDrawList();
  const ImVec2 max(origin.x + width, origin.y + height);
  draw->AddRectFilled(origin, max, IM_COL32(20, 22, 25, 255), 3.0f);
  draw->AddRect(origin, max, theme::kBorder, 3.0f);

  const auto x_of = [&](double t) {
    return origin.x + static_cast<float>((t - start) / (end - start)) * width;
  };
  // One tick per second, labelled every 10 s (labels that would overflow
  // the right edge are skipped).
  for (int second = static_cast<int>(std::ceil(start));
       second <= static_cast<int>(end); ++second) {
    const bool major = second % 10 == 0;
    const float x = x_of(second);
    draw->AddLine(ImVec2(x, max.y - (major ? 9.0f : 4.0f)), ImVec2(x, max.y),
                  major ? theme::kTextDim : theme::kBorder);
    if (!major) continue;
    const std::string label = std::format("{}s", second);
    if (x + 2.0f + ImGui::CalcTextSize(label.c_str()).x < max.x) {
      draw->AddText(ImVec2(x + 2, origin.y + 1), theme::kTextDim,
                    label.c_str());
    }
  }
  // Simulation events of the current run only: a reset starts a new run,
  // so markers from earlier runs never linger past the playhead.
  const EventLog& log = *frame.event_log;
  std::string tooltip;
  for (std::size_t i = 0; i < log.size(); ++i) {
    const EventEntry& entry = log[i];
    if (entry.run != log.current_run() || entry.sim_seconds < start ||
        entry.sim_seconds > end) {
      continue;
    }
    const float x = x_of(entry.sim_seconds);
    draw->AddLine(ImVec2(x, origin.y + 4), ImVec2(x, max.y - 4),
                  LevelColor(entry.level), 2.0f);
    if (hovered && std::abs(ImGui::GetIO().MousePos.x - x) < 4.0f * Dpi()) {
      tooltip +=
          std::format("t={:.2f}s  {}\n", entry.sim_seconds, entry.message);
    }
  }
  const float now_x = x_of(now);
  draw->AddLine(ImVec2(now_x, origin.y), ImVec2(now_x, max.y),
                sim.paused() ? theme::kWarning : theme::kText, 2.0f);
  if (!tooltip.empty()) ImGui::SetTooltip("%s", tooltip.c_str());
}

// Fills `out` (reusing its capacity) with one telemetry channel.
template <typename Getter>
void Channel(const BodyComponent& body, Getter getter,
             std::vector<float>& out) {
  out.clear();
  for (std::size_t i = 0; i < body.telemetry.size(); ++i) {
    out.push_back(static_cast<float>(getter(body.telemetry[i])));
  }
}

void DrawBodyPlots(const Entity& entity, EditorState& state) {
  const BodyComponent& body = *entity.body;
  const ImU32 color = entity.trail.has_value() ? ToImColor(entity.trail->color)
                                               : theme::kAccent;
  // Four plots side by side fit the default bottom dock.
  const float height = 76.0f * Dpi();
  const auto plot = [&](const char* title, const char* unit,
                        const std::vector<float>& values) {
    const PlotSeries series{title, values, color};
    MultiLinePlot(title, std::span<const PlotSeries>(&series, 1), height, unit);
  };
  std::vector<float>& speed = state.body_plot_scratch[0];
  std::vector<float>& altitude = state.body_plot_scratch[1];
  std::vector<float>& acceleration = state.body_plot_scratch[2];
  std::vector<float>& energy = state.body_plot_scratch[3];
  Channel(body, [](const TelemetrySample& s) { return s.speed; }, speed);
  Channel(body, [](const TelemetrySample& s) { return s.altitude; }, altitude);
  Channel(
      body, [](const TelemetrySample& s) { return s.acceleration; },
      acceleration);
  Channel(body, [](const TelemetrySample& s) { return s.energy; }, energy);

  if (ImGui::BeginTable("##plots", 4, ImGuiTableFlags_SizingStretchSame)) {
    ImGui::TableNextColumn();
    plot("Speed", "m/s", speed);
    ImGui::TableNextColumn();
    plot("Altitude", "m", altitude);
    ImGui::TableNextColumn();
    plot("Acceleration", "m/s^2", acceleration);
    ImGui::TableNextColumn();
    if (body.motion != MotionType::kKinematic) {
      plot("Energy", "J", energy);
    } else {
      HelpText("Kinematic paths are scripted: no energy is tracked.");
    }
    ImGui::EndTable();
  }
}

// Overlays relative energy drift of every body under `group`, to compare
// integrators (or any set of bodies) on one chart.
void DrawGroupPlots(const Scene& scene, const Entity& group,
                    EditorState& state) {
  // Size the scratch before taking references so none can dangle.
  if (state.group_plot_scratch.size() < group.children.size()) {
    state.group_plot_scratch.resize(group.children.size());
  }
  std::vector<PlotSeries> series;
  std::size_t used = 0;
  for (EntityId child : group.children) {
    const Entity* entity = scene.Get(child);
    if (entity == nullptr || !entity->body.has_value() ||
        entity->body->telemetry.empty()) {
      continue;
    }
    const BodyComponent& body = *entity->body;
    const double e0 = body.telemetry[0].energy;
    std::vector<float>& drift = state.group_plot_scratch[used++];
    Channel(
        body,
        [e0](const TelemetrySample& s) {
          return std::abs(e0) > 1e-9 ? (s.energy - e0) / std::abs(e0) * 100.0
                                     : std::numeric_limits<double>::quiet_NaN();
        },
        drift);
    series.push_back({entity->name, drift,
                      entity->trail.has_value()
                          ? ToImColor(entity->trail->color)
                          : theme::kAccent});
  }
  if (series.empty()) {
    HelpText("Run the simulation to compare the bodies in this group.");
    return;
  }
  ImGui::TextDisabled("Energy drift relative to start (%%)");
  MultiLinePlot("##drift", series, 120.0f * Dpi(), "%");
  HelpText(
      "A conservative orbit should keep its energy constant. Explicit "
      "Euler gains energy and spirals out; RK4 and Verlet stay close.");
}

}  // namespace

void DrawTelemetryPanel(EditorFrame& frame, EditorState& state) {
  if (!ImGui::Begin(kTelemetryWindow)) {
    ImGui::End();
    return;
  }
  SimulationSystem& sim = *frame.simulation;
  DrawTimeline(frame);
  ImGui::Text("t = %.2f s", sim.time());
  ImGui::SameLine();
  const std::string status =
      std::format("|  {} steps  |  dt {:.2f} ms  |  x{:g}  |  {}",
                  sim.step_count(), sim.fixed_step() * 1000.0, sim.time_scale(),
                  sim.paused() ? "paused" : "running");
  ImGui::TextDisabled("%s", status.c_str());
  ImGui::Separator();

  const Entity* entity = frame.scene->Get(frame.selection->selected);
  if (entity == nullptr) {
    HelpText(
        "Select a simulated object to plot its speed, altitude, "
        "acceleration and energy. Select the 'Integrator comparison' "
        "group to compare integrators.");
  } else if (entity->body.has_value()) {
    ImGui::TextUnformatted(entity->name.c_str());
    DrawBodyPlots(*entity, state);
  } else {
    ImGui::TextUnformatted(entity->name.c_str());
    DrawGroupPlots(*frame.scene, *entity, state);
  }
  ImGui::End();
}

}  // namespace aerolab::editor
