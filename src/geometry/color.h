#ifndef AEROLAB_GEOMETRY_COLOR_H_
#define AEROLAB_GEOMETRY_COLOR_H_

#include <cstdint>

namespace aerolab {

// sRGB-encoded RGBA with components in [0, 1]. Shaders convert to linear
// space where lighting needs it.
struct Color {
  float r = 1.0f;
  float g = 1.0f;
  float b = 1.0f;
  float a = 1.0f;

  constexpr bool operator==(const Color&) const = default;
};

// Builds a colour from a 0xRRGGBB literal, e.g. FromHex(0x4D8FCC).
constexpr Color FromHex(std::uint32_t rgb, float alpha = 1.0f) {
  return {static_cast<float>((rgb >> 16) & 0xFF) / 255.0f,
          static_cast<float>((rgb >> 8) & 0xFF) / 255.0f,
          static_cast<float>(rgb & 0xFF) / 255.0f, alpha};
}

constexpr Color WithAlpha(const Color& c, float alpha) {
  return {c.r, c.g, c.b, alpha};
}

constexpr Color Lerp(const Color& a, const Color& b, float t) {
  return {a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t,
          a.a + (b.a - a.a) * t};
}

// Palette shared by debug visualizations so colours mean the same thing
// everywhere: X/Y/Z axes are red/green/blue.
namespace colors {
inline constexpr Color kWhite{1.0f, 1.0f, 1.0f, 1.0f};
inline constexpr Color kBlack{0.0f, 0.0f, 0.0f, 1.0f};
inline constexpr Color kAxisX = FromHex(0xE5484D);
inline constexpr Color kAxisY = FromHex(0x46A758);
inline constexpr Color kAxisZ = FromHex(0x3E7BFA);
inline constexpr Color kVelocity = FromHex(0x38BDF8);
inline constexpr Color kAcceleration = FromHex(0xF5A524);
inline constexpr Color kSelection = FromHex(0xFFD166);
inline constexpr Color kHover = FromHex(0xA0AEC0);
inline constexpr Color kAccent = FromHex(0x4D8FCC);
}  // namespace colors

}  // namespace aerolab

#endif  // AEROLAB_GEOMETRY_COLOR_H_
