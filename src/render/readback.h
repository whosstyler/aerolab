#ifndef AEROLAB_RENDER_READBACK_H_
#define AEROLAB_RENDER_READBACK_H_

#include <cstdint>
#include <vector>

#include "render/gl.h"

namespace aerolab {

// Maximum readback edge length; guards size arithmetic against bogus input.
inline constexpr int kMaxReadbackDimension = 16384;

// Reads the colour buffer of `framebuffer` (0 = the window's back buffer)
// into tightly packed RGBA8 rows ordered top to bottom, with alpha forced to
// opaque. Restores the pixel-pack state it touches. Returns an empty vector
// (after logging) on invalid dimensions or a GL error.
std::vector<std::uint8_t> ReadFramebufferRgba(GLuint framebuffer, int width,
                                              int height);

}  // namespace aerolab

#endif  // AEROLAB_RENDER_READBACK_H_
