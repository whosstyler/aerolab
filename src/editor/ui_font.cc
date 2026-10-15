#include "editor/ui_font.h"

#include <cstddef>
#include <span>

namespace aerolab::editor {

// Defined in the generated ui_font_data.cc.
extern const unsigned char kUiFontData[];
extern const std::size_t kUiFontDataSize;

std::span<const unsigned char> UiFontData() {
  return {kUiFontData, kUiFontDataSize};
}

}  // namespace aerolab::editor
