#ifndef AEROLAB_EDITOR_WIDGETS_H_
#define AEROLAB_EDITOR_WIDGETS_H_

#include <span>
#include <string_view>

#include "geometry/color.h"
#include "imgui.h"
#include "math/quat.h"
#include "math/vec3.h"

namespace aerolab::editor {

// Small, reusable ImGui building blocks with a consistent look.

enum class Icon { kPlay, kPause, kStep, kReset };

// Square button with a vector-drawn icon (no icon font needed).
bool IconButton(const char* id, Icon icon, const char* tooltip,
                bool highlighted = false);

// Button that renders as "pressed" while `*value` is true and flips it.
bool ToggleButton(const char* label, bool* value, const char* tooltip);

// Two-column property table helpers: BeginProperties() opens a table,
// PropertyLabel() writes the left column and moves to the right one.
bool BeginProperties(const char* id);
void EndProperties();
void PropertyLabel(const char* label, const char* tooltip = nullptr);

// Editable double-precision vector in a property row. Returns true when
// changed.
bool PropertyVec3(const char* label, Vec3* value, float speed = 0.05f,
                  const char* format = "%.3f", const char* tooltip = nullptr);

// Orientation edited as yaw/pitch/roll in degrees.
bool PropertyRotation(const char* label, Quat* rotation,
                      const char* tooltip = nullptr);

bool PropertyDouble(const char* label, double* value, float speed, double min,
                    double max, const char* format = "%.3f",
                    const char* tooltip = nullptr);
bool PropertyBool(const char* label, bool* value,
                  const char* tooltip = nullptr);
bool PropertyColor(const char* label, Color* color,
                   const char* tooltip = nullptr);

// Read-only rows.
void PropertyText(const char* label, std::string_view text,
                  const char* tooltip = nullptr);
void PropertyVec3ReadOnly(const char* label, const Vec3& value,
                          const char* unit = "", const char* tooltip = nullptr);

// One series of a multi-line plot.
struct PlotSeries {
  std::string_view label;
  std::span<const float> values;  // Oldest first; NaN entries are skipped.
  ImU32 color;
};

// Line plot of several series sharing one auto-scaled Y axis, drawn with the
// window draw list. Shows a legend and the latest value of each series.
void MultiLinePlot(const char* id, std::span<const PlotSeries> series,
                   float height, const char* unit);

// Dimmed helper text that wraps.
void HelpText(std::string_view text);

}  // namespace aerolab::editor

#endif  // AEROLAB_EDITOR_WIDGETS_H_
