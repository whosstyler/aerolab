// Event log: everything logged by the app and the simulation, filterable.

#include <array>
#include <cstddef>
#include <string_view>

#include "core/event_log.h"
#include "core/log.h"
#include "core/string_util.h"
#include "editor/panels.h"
#include "editor/theme.h"
#include "imgui.h"

namespace aerolab::editor {
namespace {

ImU32 LevelColor(LogLevel level) {
  switch (level) {
    case LogLevel::kDebug:
      return theme::kTextDim;
    case LogLevel::kInfo:
      return theme::kText;
    case LogLevel::kWarning:
      return theme::kWarning;
    case LogLevel::kError:
      return theme::kError;
  }
  return theme::kText;
}

}  // namespace

void DrawEventLogPanel(EditorFrame& frame, EditorState& state) {
  if (!ImGui::Begin(kLogWindow)) {
    ImGui::End();
    return;
  }
  EventLog& log = *frame.event_log;
  if (ImGui::Button("Clear")) log.Clear();
  ImGui::SameLine();
  ImGui::Checkbox("Auto-scroll", &state.log_auto_scroll);
  ImGui::SameLine();
  ImGui::Checkbox("Debug", &state.log_show_debug);
  ImGui::SameLine();
  std::array<char, 128> buffer{};
  state.log_filter.copy(buffer.data(), buffer.size() - 1);
  ImGui::SetNextItemWidth(-1.0f);
  if (ImGui::InputTextWithHint("##filter", "Filter", buffer.data(),
                               buffer.size())) {
    state.log_filter = buffer.data();
  }

  // Collect the visible rows once (into a reused vector), then let the
  // clipper lay out only the rows that are actually on screen.
  state.log_rows.clear();
  for (std::size_t i = 0; i < log.size(); ++i) {
    const EventEntry& entry = log[i];
    if (entry.level == LogLevel::kDebug && !state.log_show_debug) continue;
    if (!ContainsIgnoringCase(entry.message, state.log_filter)) continue;
    state.log_rows.push_back(i);
  }

  constexpr ImGuiTableFlags kFlags =
      ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY |
      ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_Resizable;
  if (ImGui::BeginTable("##log", 4, kFlags)) {
    ImGui::TableSetupScrollFreeze(0, 1);
    ImGui::TableSetupColumn("Sim t", ImGuiTableColumnFlags_WidthFixed,
                            64.0f * Dpi());
    ImGui::TableSetupColumn("Wall t", ImGuiTableColumnFlags_WidthFixed,
                            64.0f * Dpi());
    ImGui::TableSetupColumn("Source", ImGuiTableColumnFlags_WidthFixed,
                            56.0f * Dpi());
    ImGui::TableSetupColumn("Message", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableHeadersRow();
    ImGuiListClipper clipper;
    clipper.Begin(static_cast<int>(state.log_rows.size()));
    while (clipper.Step()) {
      for (int row = clipper.DisplayStart; row < clipper.DisplayEnd; ++row) {
        const EventEntry& entry =
            log[state.log_rows[static_cast<std::size_t>(row)]];
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        if (entry.sim_seconds >= 0.0) {
          ImGui::Text("%.2f s", entry.sim_seconds);
        } else {
          ImGui::TextDisabled("-");
        }
        ImGui::TableSetColumnIndex(1);
        ImGui::TextDisabled("%.1f s", entry.wall_seconds);
        ImGui::TableSetColumnIndex(2);
        const std::string_view source = EventSourceName(entry.source);
        ImGui::PushStyleColor(ImGuiCol_Text, theme::kTextDim);
        ImGui::TextUnformatted(source.data(), source.data() + source.size());
        ImGui::PopStyleColor();
        ImGui::TableSetColumnIndex(3);
        ImGui::PushStyleColor(ImGuiCol_Text, LevelColor(entry.level));
        ImGui::TextUnformatted(entry.message.c_str());
        ImGui::PopStyleColor();
      }
    }
    if (state.log_auto_scroll &&
        ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 4.0f) {
      ImGui::SetScrollHereY(1.0f);
    }
    ImGui::EndTable();
  }
  ImGui::End();
}

}  // namespace aerolab::editor
