#ifndef AEROLAB_EDITOR_UI_FONT_H_
#define AEROLAB_EDITOR_UI_FONT_H_

#include <cstddef>
#include <span>

namespace aerolab::editor {

// TrueType data of the editor's UI font (Roboto Medium, Apache 2.0, shipped
// with Dear ImGui), compiled into the binary at build time.
std::span<const unsigned char> UiFontData();

}  // namespace aerolab::editor

#endif  // AEROLAB_EDITOR_UI_FONT_H_
