#ifndef AEROLAB_EDITOR_THEME_H_
#define AEROLAB_EDITOR_THEME_H_

#include "imgui.h"

namespace aerolab::editor {

// Restrained dark palette shared by the editor's custom widgets.
namespace theme {
inline constexpr ImU32 kAccent = IM_COL32(77, 143, 204, 255);
inline constexpr ImU32 kAccentDim = IM_COL32(77, 143, 204, 90);
inline constexpr ImU32 kText = IM_COL32(220, 223, 228, 255);
inline constexpr ImU32 kTextDim = IM_COL32(140, 146, 156, 255);
inline constexpr ImU32 kPanel = IM_COL32(30, 32, 36, 255);
inline constexpr ImU32 kBorder = IM_COL32(56, 60, 68, 255);
inline constexpr ImU32 kWarning = IM_COL32(245, 165, 36, 255);
inline constexpr ImU32 kError = IM_COL32(229, 72, 77, 255);
inline constexpr ImU32 kSuccess = IM_COL32(70, 167, 88, 255);
}  // namespace theme

// Applies the editor style: dark neutral greys, one blue accent, subtle
// borders, small rounding and consistent 8 px spacing, scaled for DPI.
void ApplyTheme(float dpi_scale);

}  // namespace aerolab::editor

#endif  // AEROLAB_EDITOR_THEME_H_
