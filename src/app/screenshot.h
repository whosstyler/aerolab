#ifndef AEROLAB_APP_SCREENSHOT_H_
#define AEROLAB_APP_SCREENSHOT_H_

#include <cstdint>
#include <string>
#include <vector>

namespace aerolab {

// Encodes tightly packed, top-to-bottom RGBA8 pixels as a PNG at `path`.
// Returns false and logs on failure.
bool WritePng(const std::string& path, const std::vector<std::uint8_t>& rgba,
              int width, int height);

}  // namespace aerolab

#endif  // AEROLAB_APP_SCREENSHOT_H_
