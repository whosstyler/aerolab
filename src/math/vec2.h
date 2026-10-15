#ifndef AEROLAB_MATH_VEC2_H_
#define AEROLAB_MATH_VEC2_H_

#include <cmath>

#include "math/scalar.h"

namespace aerolab {

// Two-component vector, used for screen-space positions and 2D deltas.
struct Vec2 {
  double x = 0.0;
  double y = 0.0;

  constexpr bool operator==(const Vec2&) const = default;
};

constexpr Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
constexpr Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
constexpr Vec2 operator-(Vec2 v) { return {-v.x, -v.y}; }
constexpr Vec2 operator*(Vec2 v, double s) { return {v.x * s, v.y * s}; }
constexpr Vec2 operator*(double s, Vec2 v) { return {v.x * s, v.y * s}; }
constexpr Vec2 operator/(Vec2 v, double s) { return {v.x / s, v.y / s}; }

constexpr Vec2& operator+=(Vec2& a, Vec2 b) { return a = a + b; }
constexpr Vec2& operator-=(Vec2& a, Vec2 b) { return a = a - b; }
constexpr Vec2& operator*=(Vec2& v, double s) { return v = v * s; }

constexpr double Dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
constexpr double LengthSquared(Vec2 v) { return Dot(v, v); }
inline double Length(Vec2 v) { return std::sqrt(LengthSquared(v)); }

// Returns the zero vector when `v` is too short to normalize.
inline Vec2 Normalized(Vec2 v) {
  const double length = Length(v);
  return length > kEpsilon ? v / length : Vec2{};
}

inline bool NearlyEqual(Vec2 a, Vec2 b, double tolerance = kEpsilon) {
  return NearlyEqual(a.x, b.x, tolerance) && NearlyEqual(a.y, b.y, tolerance);
}

}  // namespace aerolab

#endif  // AEROLAB_MATH_VEC2_H_
