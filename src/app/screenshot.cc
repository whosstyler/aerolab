#include "app/screenshot.h"

#include <SDL3/SDL_error.h>
#include <SDL3/SDL_pixels.h>
#include <SDL3/SDL_surface.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "core/log.h"

namespace aerolab {

bool WritePng(const std::string& path, const std::vector<std::uint8_t>& rgba,
              int width, int height) {
  if (width <= 0 || height <= 0 ||
      rgba.size() != static_cast<std::size_t>(width) *
                         static_cast<std::size_t>(height) * 4) {
    LogError("Screenshot skipped: no pixel data for {}x{}", width, height);
    return false;
  }
  // SDL only reads from the buffer; the const_cast is required by its C API.
  SDL_Surface* surface =
      SDL_CreateSurfaceFrom(width, height, SDL_PIXELFORMAT_RGBA32,
                            const_cast<std::uint8_t*>(rgba.data()), width * 4);
  if (surface == nullptr) {
    LogError("Screenshot surface creation failed: {}", SDL_GetError());
    return false;
  }
  const bool saved = SDL_SavePNG(surface, path.c_str());
  SDL_DestroySurface(surface);
  if (!saved) {
    LogError("Could not write screenshot '{}': {}", path, SDL_GetError());
    return false;
  }
  LogInfo("Saved screenshot to {}", path);
  return true;
}

}  // namespace aerolab
