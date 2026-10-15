#include "editor/widgets.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <limits>
#include <span>
#include <string>
#include <string_view>

#include "editor/theme.h"
#include "imgui.h"
#include "math/scalar.h"

namespace aerolab::editor {
namespace {

void Tooltip(const char* text) {
  if (text != nullptr && ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
    ImGui::SetTooltip("%s", text);
  }
}

void DrawIcon(ImDrawList* draw, Icon icon, ImVec2 min, ImVec2 max,
              ImU32 color) {
  const ImVec2 c((min.x + max.x) * 0.5f, (min.y + max.y) * 0.5f);
  const float r = (max.x - min.x) * 0.26f;
  switch (icon) {
    case Icon::kPlay:
      draw->AddTriangleFilled(ImVec2(c.x - r * 0.75f, c.y - r),
                              ImVec2(c.x - r * 0.75f, c.y + r),
                              ImVec2(c.x + r, c.y), color);
      break;
    case Icon::kPause: {
      const float w = r * 0.55f;
      draw->AddRectFilled(ImVec2(c.x - r * 0.85f, c.y - r),
                          ImVec2(c.x - r * 0.85f + w, c.y + r), color, 1.0f);
      draw->AddRectFilled(ImVec2(c.x + r * 0.85f - w, c.y - r),
                          ImVec2(c.x + r * 0.85f, c.y + r), color, 1.0f);
      break;
    }
    case Icon::kStep:
      draw->AddTriangleFilled(ImVec2(c.x - r, c.y - r),
                              ImVec2(c.x - r, c.y + r),
                              ImVec2(c.x + r * 0.45f, c.y), color);
      draw->AddRectFilled(ImVec2(c.x + r * 0.55f, c.y - r),
                          ImVec2(c.x + r, c.y + r), color, 1.0f);
      break;
    case Icon::kReset: {
      // Three-quarter circle with an arrow head at its end.
      constexpr float kPiF = static_cast<float>(kPi);
      draw->PathArcTo(c, r, kPiF * 0.2f, kPiF * 1.75f, 16);
      draw->PathStroke(color, r * 0.32f);
      const float end = kPiF * 1.75f;
      const ImVec2 tip(c.x + std::cos(end) * r, c.y + std::sin(end) * r);
      draw->AddTriangleFilled(ImVec2(tip.x - r * 0.55f, tip.y - r * 0.1f),
                              ImVec2(tip.x + r * 0.35f, tip.y - r * 0.45f),
                              ImVec2(tip.x + r * 0.2f, tip.y + r * 0.45f),
                              color);
      break;
    }
  }
}

}  // namespace

bool IconButton(const char* id, Icon icon, const char* tooltip,
                bool highlighted) {
  const float size = ImGui::GetFrameHeight();
  if (highlighted) {
    ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetColorU32(ImGuiCol_Header));
  }
  const bool pressed = ImGui::Button(id, ImVec2(size, size));
  if (highlighted) ImGui::PopStyleColor();
  DrawIcon(ImGui::GetWindowDrawList(), icon, ImGui::GetItemRectMin(),
           ImGui::GetItemRectMax(), theme::kText);
  Tooltip(tooltip);
  return pressed;
}

bool ToggleButton(const char* label, bool* value, const char* tooltip) {
  const bool on = *value;
  if (on) {
    ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetColorU32(ImGuiCol_Header));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
                          ImGui::GetColorU32(ImGuiCol_HeaderActive));
  }
  const bool pressed = ImGui::Button(label);
  if (on) ImGui::PopStyleColor(2);
  if (pressed) *value = !*value;
  Tooltip(tooltip);
  return pressed;
}

bool BeginProperties(const char* id) {
  if (!ImGui::BeginTable(id, 2, ImGuiTableFlags_SizingStretchProp)) {
    return false;
  }
  ImGui::TableSetupColumn("name", ImGuiTableColumnFlags_WidthStretch, 0.38f);
  ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthStretch, 0.62f);
  return true;
}

void EndProperties() { ImGui::EndTable(); }

void PropertyLabel(const char* label, const char* tooltip) {
  ImGui::TableNextRow();
  ImGui::TableSetColumnIndex(0);
  ImGui::AlignTextToFramePadding();
  ImGui::TextUnformatted(label);
  Tooltip(tooltip);
  ImGui::TableSetColumnIndex(1);
  ImGui::SetNextItemWidth(-std::numeric_limits<float>::min());
}

bool PropertyVec3(const char* label, Vec3* value, float speed,
                  const char* format, const char* tooltip) {
  PropertyLabel(label, tooltip);
  ImGui::PushID(label);
  std::array<double, 3> components = {value->x, value->y, value->z};
  const bool changed =
      ImGui::DragScalarN("##v", ImGuiDataType_Double, components.data(), 3,
                         speed, nullptr, nullptr, format);
  ImGui::PopID();
  if (changed) *value = {components[0], components[1], components[2]};
  return changed;
}

bool PropertyRotation(const char* label, Quat* rotation, const char* tooltip) {
  PropertyLabel(label, tooltip != nullptr
                           ? tooltip
                           : "Yaw (about +Y), pitch (nose up), roll, degrees");
  ImGui::PushID(label);
  // While a drag is in progress, edit a cached Euler triple instead of
  // re-deriving it from the quaternion every frame; otherwise the other two
  // angles can jump near pitch +/-90 degrees (gimbal lock).
  const ImGuiID key = ImGui::GetID("##euler");
  ImGuiStorage* storage = ImGui::GetStateStorage();
  std::array<double, 3> components;
  if (storage->GetBool(key, false)) {
    for (int i = 0; i < 3; ++i) {
      components[i] = storage->GetFloat(key + 1 + static_cast<ImGuiID>(i));
    }
  } else {
    const EulerAngles euler = ToEuler(*rotation);
    components = {Degrees(euler.yaw), Degrees(euler.pitch),
                  Degrees(euler.roll)};
  }
  const bool changed =
      ImGui::DragScalarN("##r", ImGuiDataType_Double, components.data(), 3,
                         0.5f, nullptr, nullptr, "%.1f deg");
  const bool active = ImGui::IsItemActive();
  storage->SetBool(key, active);
  if (active) {
    for (int i = 0; i < 3; ++i) {
      storage->SetFloat(key + 1 + static_cast<ImGuiID>(i),
                        static_cast<float>(components[i]));
    }
  }
  ImGui::PopID();
  if (changed) {
    *rotation = FromEuler({.yaw = Radians(components[0]),
                           .pitch = Radians(components[1]),
                           .roll = Radians(components[2])});
  }
  return changed;
}

bool PropertyDouble(const char* label, double* value, float speed, double min,
                    double max, const char* format, const char* tooltip) {
  PropertyLabel(label, tooltip);
  ImGui::PushID(label);
  const bool changed =
      ImGui::DragScalar("##d", ImGuiDataType_Double, value, speed, &min, &max,
                        format, ImGuiSliderFlags_AlwaysClamp);
  ImGui::PopID();
  return changed;
}

bool PropertyBool(const char* label, bool* value, const char* tooltip) {
  PropertyLabel(label, tooltip);
  ImGui::PushID(label);
  const bool changed = ImGui::Checkbox("##b", value);
  ImGui::PopID();
  return changed;
}

bool PropertyColor(const char* label, Color* color, const char* tooltip) {
  PropertyLabel(label, tooltip);
  ImGui::PushID(label);
  std::array<float, 3> rgb = {color->r, color->g, color->b};
  const bool changed = ImGui::ColorEdit3(
      "##c", rgb.data(),
      ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_NoLabel);
  ImGui::PopID();
  if (changed) {
    color->r = rgb[0];
    color->g = rgb[1];
    color->b = rgb[2];
  }
  return changed;
}

void PropertyText(const char* label, std::string_view text,
                  const char* tooltip) {
  PropertyLabel(label, tooltip);
  ImGui::TextUnformatted(text.data(), text.data() + text.size());
}

void PropertyVec3ReadOnly(const char* label, const Vec3& value,
                          const char* unit, const char* tooltip) {
  PropertyText(
      label,
      std::format("{:.2f}, {:.2f}, {:.2f} {}", value.x, value.y, value.z, unit),
      tooltip);
}

void MultiLinePlot(const char* id, std::span<const PlotSeries> series,
                   float height, const char* unit) {
  ImGui::PushID(id);
  const float width = ImGui::GetContentRegionAvail().x;
  const ImVec2 origin = ImGui::GetCursorScreenPos();
  const ImVec2 size(std::max(width, 50.0f), height);
  ImGui::InvisibleButton("##plot", size);
  ImDrawList* draw = ImGui::GetWindowDrawList();
  const ImVec2 end(origin.x + size.x, origin.y + size.y);
  draw->AddRectFilled(origin, end, IM_COL32(20, 22, 25, 255), 3.0f);
  draw->AddRect(origin, end, theme::kBorder, 3.0f);

  // Shared auto-scaled range.
  float lo = std::numeric_limits<float>::max();
  float hi = std::numeric_limits<float>::lowest();
  std::size_t longest = 0;
  for (const PlotSeries& s : series) {
    longest = std::max(longest, s.values.size());
    for (float v : s.values) {
      if (!std::isfinite(v)) continue;
      lo = std::min(lo, v);
      hi = std::max(hi, v);
    }
  }
  if (longest < 2 || lo > hi) {
    draw->AddText(ImVec2(origin.x + 8, origin.y + 8), theme::kTextDim,
                  "No data yet - run the simulation");
    ImGui::PopID();
    return;
  }
  if (hi - lo < 1e-6f) {
    hi += 0.5f;
    lo -= 0.5f;
  }
  const float pad = (hi - lo) * 0.08f;
  lo -= pad;
  hi += pad;

  const float left = origin.x + 4;
  const float right = end.x - 4;
  const float top = origin.y + 4;
  const float bottom = end.y - 4;
  const auto map_y = [&](float v) {
    return bottom - (v - lo) / (hi - lo) * (bottom - top);
  };
  // Zero line when the range crosses it.
  if (lo < 0.0f && hi > 0.0f) {
    draw->AddLine(ImVec2(left, map_y(0.0f)), ImVec2(right, map_y(0.0f)),
                  IM_COL32(255, 255, 255, 30));
  }
  for (const PlotSeries& s : series) {
    const std::size_t n = s.values.size();
    if (n < 2) continue;
    // Right-align so all series end at "now".
    const float step = (right - left) / static_cast<float>(longest - 1);
    const float x0 = right - step * static_cast<float>(n - 1);
    for (std::size_t i = 1; i < n; ++i) {
      const float a = s.values[i - 1];
      const float b = s.values[i];
      if (!std::isfinite(a) || !std::isfinite(b)) continue;
      draw->AddLine(ImVec2(x0 + step * static_cast<float>(i - 1), map_y(a)),
                    ImVec2(x0 + step * static_cast<float>(i), map_y(b)),
                    s.color, 1.5f);
    }
  }
  // Axis extremes and legend.
  const std::string top_label = std::format("{:.3g} {}", hi, unit);
  const std::string bottom_label = std::format("{:.3g} {}", lo, unit);
  const auto axis_label = [&](float y, const std::string& text) {
    const ImVec2 size = ImGui::CalcTextSize(text.c_str());
    draw->AddRectFilled(ImVec2(left, y), ImVec2(left + size.x + 4, y + size.y),
                        IM_COL32(20, 22, 25, 200), 2.0f);
    draw->AddText(ImVec2(left + 2, y), theme::kTextDim, text.c_str());
  };
  float legend_y = top;
  float legend_left = right;  // Left edge of the first legend row.
  for (const PlotSeries& s : series) {
    float latest = std::numeric_limits<float>::quiet_NaN();
    for (std::size_t i = s.values.size(); i-- > 0;) {
      if (std::isfinite(s.values[i])) {
        latest = s.values[i];
        break;
      }
    }
    const std::string label = !std::isfinite(latest)
                                  ? std::string(s.label)
                                  : std::format("{}  {:.4g}", s.label, latest);
    const ImVec2 text_size = ImGui::CalcTextSize(label.c_str());
    const float x = right - text_size.x - 4;
    // A backing plate keeps the legend readable over the curves.
    draw->AddRectFilled(ImVec2(x - 18, legend_y),
                        ImVec2(right, legend_y + text_size.y),
                        IM_COL32(20, 22, 25, 200), 2.0f);
    draw->AddRectFilled(ImVec2(x - 14, legend_y + 4),
                        ImVec2(x - 6, legend_y + 12), s.color, 2.0f);
    draw->AddText(ImVec2(x, legend_y), theme::kText, label.c_str());
    if (legend_y == top) legend_left = x - 18;
    legend_y += ImGui::GetTextLineHeight();
  }
  // The top label moves below the legend when the plot is too narrow for
  // both on one line.
  const float top_label_right =
      left + ImGui::CalcTextSize(top_label.c_str()).x + 8;
  axis_label(top_label_right < legend_left ? top : legend_y, top_label);
  axis_label(bottom - ImGui::GetTextLineHeight(), bottom_label);
  ImGui::PopID();
}

void HelpText(std::string_view text) {
  ImGui::PushStyleColor(ImGuiCol_Text,
                        ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
  ImGui::PushTextWrapPos(0.0f);
  ImGui::TextUnformatted(text.data(), text.data() + text.size());
  ImGui::PopTextWrapPos();
  ImGui::PopStyleColor();
}

}  // namespace aerolab::editor
