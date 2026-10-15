#include "render/readback.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "core/log.h"

namespace aerolab {

std::vector<std::uint8_t> ReadFramebufferRgba(GLuint framebuffer, int width,
                                              int height) {
  if (width <= 0 || height <= 0 || width > kMaxReadbackDimension ||
      height > kMaxReadbackDimension) {
    LogError("Readback rejected: framebuffer is {}x{}", width, height);
    return {};
  }
  const std::size_t row_bytes = static_cast<std::size_t>(width) * 4;
  const std::size_t rows = static_cast<std::size_t>(height);
  std::vector<std::uint8_t> pixels(row_bytes * rows);

  GLint previous_alignment = 4;
  glGetIntegerv(GL_PACK_ALIGNMENT, &previous_alignment);
  glBindFramebuffer(GL_READ_FRAMEBUFFER, framebuffer);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
  glPixelStorei(GL_PACK_ALIGNMENT, previous_alignment);
  if (!CheckGlErrors("ReadFramebufferRgba")) return {};

  // OpenGL rows start at the bottom; image files start at the top.
  std::vector<std::uint8_t> flipped(pixels.size());
  for (std::size_t y = 0; y < rows; ++y) {
    std::memcpy(flipped.data() + y * row_bytes,
                pixels.data() + (rows - 1 - y) * row_bytes, row_bytes);
  }
  // The default framebuffer may lack an alpha channel; images are opaque.
  for (std::size_t i = 3; i < flipped.size(); i += 4) flipped[i] = 255;
  return flipped;
}

}  // namespace aerolab
