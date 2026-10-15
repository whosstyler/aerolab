#ifndef AEROLAB_MATH_VEC4_H_
#define AEROLAB_MATH_VEC4_H_

#include "math/scalar.h"
#include "math/vec3.h"

namespace aerolab {

// Homogeneous coordinates (w = 1 for points, w = 0 for directions) and RGBA.
struct Vec4 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 0.0;

  constexpr double operator[](int i) const {
    return i == 0 ? x : (i == 1 ? y : (i == 2 ? z : w));
  }
  constexpr double& operator[](int i) {
    return i == 0 ? x : (i == 1 ? y : (i == 2 ? z : w));
  }

  constexpr bool operator==(const Vec4&) const = default;
};

constexpr Vec4 MakePoint(const Vec3& p) { return {p.x, p.y, p.z, 1.0}; }
constexpr Vec4 MakeDirection(const Vec3& d) { return {d.x, d.y, d.z, 0.0}; }
constexpr Vec3 Xyz(const Vec4& v) { return {v.x, v.y, v.z}; }

constexpr Vec4 operator+(const Vec4& a, const Vec4& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w};
}
constexpr Vec4 operator-(const Vec4& a, const Vec4& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w};
}
constexpr Vec4 operator*(const Vec4& v, double s) {
  return {v.x * s, v.y * s, v.z * s, v.w * s};
}
constexpr Vec4 operator*(double s, const Vec4& v) { return v * s; }

constexpr double Dot(const Vec4& a, const Vec4& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

inline bool NearlyEqual(const Vec4& a, const Vec4& b,
                        double tolerance = kEpsilon) {
  return NearlyEqual(a.x, b.x, tolerance) && NearlyEqual(a.y, b.y, tolerance) &&
         NearlyEqual(a.z, b.z, tolerance) && NearlyEqual(a.w, b.w, tolerance);
}

}  // namespace aerolab

#endif  // AEROLAB_MATH_VEC4_H_
