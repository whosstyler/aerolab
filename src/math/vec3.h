#ifndef AEROLAB_MATH_VEC3_H_
#define AEROLAB_MATH_VEC3_H_

#include <algorithm>
#include <cmath>

#include "math/scalar.h"

namespace aerolab {

// Three-component vector of doubles: positions, directions, velocities.
struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  static constexpr Vec3 Zero() { return {0.0, 0.0, 0.0}; }
  static constexpr Vec3 One() { return {1.0, 1.0, 1.0}; }
  static constexpr Vec3 UnitX() { return {1.0, 0.0, 0.0}; }
  static constexpr Vec3 UnitY() { return {0.0, 1.0, 0.0}; }
  static constexpr Vec3 UnitZ() { return {0.0, 0.0, 1.0}; }

  // Component access by index (0 = x, 1 = y, 2 = z) for axis loops.
  constexpr double operator[](int i) const {
    return i == 0 ? x : (i == 1 ? y : z);
  }
  constexpr double& operator[](int i) { return i == 0 ? x : (i == 1 ? y : z); }

  constexpr bool operator==(const Vec3&) const = default;
};

constexpr Vec3 operator+(const Vec3& a, const Vec3& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}
constexpr Vec3 operator-(const Vec3& a, const Vec3& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}
constexpr Vec3 operator-(const Vec3& v) { return {-v.x, -v.y, -v.z}; }
constexpr Vec3 operator*(const Vec3& v, double s) {
  return {v.x * s, v.y * s, v.z * s};
}
constexpr Vec3 operator*(double s, const Vec3& v) { return v * s; }
constexpr Vec3 operator/(const Vec3& v, double s) {
  return {v.x / s, v.y / s, v.z / s};
}

constexpr Vec3& operator+=(Vec3& a, const Vec3& b) { return a = a + b; }
constexpr Vec3& operator-=(Vec3& a, const Vec3& b) { return a = a - b; }
constexpr Vec3& operator*=(Vec3& v, double s) { return v = v * s; }
constexpr Vec3& operator/=(Vec3& v, double s) { return v = v / s; }

constexpr double Dot(const Vec3& a, const Vec3& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

// Right-handed cross product.
constexpr Vec3 Cross(const Vec3& a, const Vec3& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

// Component-wise product.
constexpr Vec3 Hadamard(const Vec3& a, const Vec3& b) {
  return {a.x * b.x, a.y * b.y, a.z * b.z};
}

constexpr double LengthSquared(const Vec3& v) { return Dot(v, v); }
inline double Length(const Vec3& v) { return std::sqrt(LengthSquared(v)); }
inline double Distance(const Vec3& a, const Vec3& b) { return Length(a - b); }

// Returns `fallback` when `v` is too short to normalize reliably.
inline Vec3 NormalizedOr(const Vec3& v, const Vec3& fallback) {
  const double length = Length(v);
  return length > kEpsilon ? v / length : fallback;
}

// Returns the zero vector when `v` is too short to normalize.
inline Vec3 Normalized(const Vec3& v) { return NormalizedOr(v, Vec3::Zero()); }

constexpr Vec3 Lerp(const Vec3& a, const Vec3& b, double t) {
  return a + (b - a) * t;
}

inline Vec3 Min(const Vec3& a, const Vec3& b) {
  return {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)};
}
inline Vec3 Max(const Vec3& a, const Vec3& b) {
  return {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)};
}
inline Vec3 Abs(const Vec3& v) {
  return {std::abs(v.x), std::abs(v.y), std::abs(v.z)};
}
inline double MaxComponent(const Vec3& v) { return std::max({v.x, v.y, v.z}); }

// Component of `v` along `onto` (zero if `onto` is degenerate).
inline Vec3 Project(const Vec3& v, const Vec3& onto) {
  const double denom = LengthSquared(onto);
  return denom > kEpsilon * kEpsilon ? onto * (Dot(v, onto) / denom)
                                     : Vec3::Zero();
}

// Component of `v` perpendicular to `onto`.
inline Vec3 Reject(const Vec3& v, const Vec3& onto) {
  return v - Project(v, onto);
}

// Reflects `v` about the plane with unit `normal`.
constexpr Vec3 Reflect(const Vec3& v, const Vec3& normal) {
  return v - normal * (2.0 * Dot(v, normal));
}

// Unsigned angle between two vectors in radians, in [0, pi].
inline double AngleBetween(const Vec3& a, const Vec3& b) {
  // atan2 of |cross| and dot is accurate for both tiny and near-pi angles.
  return std::atan2(Length(Cross(a, b)), Dot(a, b));
}

// Any unit vector perpendicular to the unit vector `v`.
inline Vec3 AnyPerpendicular(const Vec3& v) {
  const Vec3 helper = std::abs(v.x) < 0.9 ? Vec3::UnitX() : Vec3::UnitY();
  return Normalized(Cross(v, helper));
}

inline bool IsFinite(const Vec3& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

inline bool NearlyEqual(const Vec3& a, const Vec3& b,
                        double tolerance = kEpsilon) {
  return NearlyEqual(a.x, b.x, tolerance) && NearlyEqual(a.y, b.y, tolerance) &&
         NearlyEqual(a.z, b.z, tolerance);
}

}  // namespace aerolab

#endif  // AEROLAB_MATH_VEC3_H_
