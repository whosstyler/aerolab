// Performance: frame rate, per-stage CPU/GPU timings and renderer counters.

#include <cstddef>
#include <format>
#include <span>
#include <string>
#include <vector>

#include "core/frame_stats.h"
#include "core/ring_buffer.h"
#include "editor/panels.h"
#include "editor/theme.h"
#include "editor/widgets.h"
#include "imgui.h"
#include "scene/entity.h"

namespace aerolab::editor {

void DrawPerformancePanel(EditorFrame& frame, EditorState& state,
                          EditorRequests& requests) {
  if (!ImGui::Begin(kPerformanceWindow)) {
    ImGui::End();
    return;
  }
  const FrameStats& stats = *frame.frame_stats;
  const FrameStats::Summary frame_summary =
      stats.Summarize(FrameTiming::kFrame);

  ImGui::Text("%.0f FPS", stats.Fps());
  ImGui::SameLine();
  ImGui::TextDisabled("%.2f ms avg   %.2f min   %.2f max",
                      frame_summary.average, frame_summary.minimum,
                      frame_summary.maximum);
  ImGui::SameLine();
  bool vsync = frame.vsync;
  if (ImGui::Checkbox("VSync", &vsync)) {
    requests.set_vsync = true;
    requests.vsync = vsync;
  }

  // Frame-time graph (reuses a scratch buffer instead of allocating).
  const RingBuffer<float>& history = stats.history(FrameTiming::kFrame);
  if (state.plot_scratch.empty()) state.plot_scratch.resize(1);
  std::vector<float>& values = state.plot_scratch[0];
  values.clear();
  for (std::size_t i = 0; i < history.size(); ++i) values.push_back(history[i]);
  const PlotSeries series{"frame ms", values, theme::kAccent};
  MultiLinePlot("##frametime", std::span<const PlotSeries>(&series, 1),
                72.0f * Dpi(), "ms");

  if (ImGui::BeginTable(
          "##timings", 4,
          ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV)) {
    ImGui::TableSetupColumn("Stage");
    ImGui::TableSetupColumn("avg ms");
    ImGui::TableSetupColumn("min");
    ImGui::TableSetupColumn("max");
    ImGui::TableHeadersRow();
    for (int i = 0; i < kFrameTimingCount; ++i) {
      const auto timing = static_cast<FrameTiming>(i);
      const FrameStats::Summary s = stats.Summarize(timing);
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      const std::string name(FrameTimingName(timing));
      ImGui::TextUnformatted(name.c_str());
      ImGui::TableSetColumnIndex(1);
      ImGui::Text("%.3f", s.average);
      ImGui::TableSetColumnIndex(2);
      ImGui::TextDisabled("%.3f", s.minimum);
      ImGui::TableSetColumnIndex(3);
      ImGui::TextDisabled("%.3f", s.maximum);
    }
    ImGui::EndTable();
  }

  const RenderStats& render = *frame.render_stats;
  const SimulationStats& sim = frame.simulation->stats();
  int bodies = 0;
  frame.scene->ForEachEntity([&bodies](EntityId, const Entity& entity) {
    if (entity.body.has_value()) ++bodies;
  });
  if (BeginProperties("##counters")) {
    PropertyText(
        "Viewport",
        std::format("{} x {} px, {}x MSAA", frame.texture_width,
                    frame.texture_height, frame.render_settings->msaa_samples));
    PropertyText("Draw calls", std::format("{}", render.draw_calls));
    PropertyText("Triangles", std::format("{}", render.triangles));
    PropertyText("Debug lines", std::format("{}", render.line_segments));
    PropertyText("Entities",
                 std::format("{} ({} simulated)", frame.scene->size(), bodies));
    PropertyText("Steps / frame", std::format("{}", sim.steps_last_frame));
    PropertyText("Step cost", std::format("{:.3f} ms", sim.step_milliseconds));
    PropertyText("Dropped steps", std::format("{}", sim.dropped_steps));
    EndProperties();
  }
  ImGui::End();
}

}  // namespace aerolab::editor
