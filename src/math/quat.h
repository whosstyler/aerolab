#ifndef AEROLAB_MATH_QUAT_H_
#define AEROLAB_MATH_QUAT_H_

#include "math/mat3.h"
#include "math/scalar.h"
#include "math/vec3.h"

namespace aerolab {

// Unit quaternion representing a rotation, Hamilton convention:
// q = w + xi + yj + zk, rotating a vector as v' = q v q*.
// Composition q2 * q1 applies q1 first, then q2 (same order as matrices).
struct Quat {
  double w = 1.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  static constexpr Quat Identity() { return {1.0, 0.0, 0.0, 0.0}; }

  constexpr bool operator==(const Quat&) const = default;
};

// Intrinsic Tait-Bryan angles in radians, applied yaw (about +Y), then pitch
// (about the rotated +X), then roll (about the rotated +Z). Fields are
// declared in application order; prefer designated initializers, e.g.
// EulerAngles{.pitch = 0.2}.
// Positive pitch raises the nose (-Z forward tilts toward +Y); positive yaw
// turns counter-clockwise seen from above; positive roll banks right-wing up.
struct EulerAngles {
  double yaw = 0.0;
  double pitch = 0.0;
  double roll = 0.0;
};

constexpr Quat operator*(const Quat& a, const Quat& b) {
  return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
          a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
          a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}
constexpr Quat operator*(const Quat& q, double s) {
  return {q.w * s, q.x * s, q.y * s, q.z * s};
}
constexpr Quat operator+(const Quat& a, const Quat& b) {
  return {a.w + b.w, a.x + b.x, a.y + b.y, a.z + b.z};
}
constexpr Quat operator-(const Quat& q) { return {-q.w, -q.x, -q.y, -q.z}; }

constexpr double Dot(const Quat& a, const Quat& b) {
  return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}
constexpr Quat Conjugate(const Quat& q) { return {q.w, -q.x, -q.y, -q.z}; }
double Length(const Quat& q);

// Returns the identity for a degenerate (near-zero) quaternion.
Quat Normalized(const Quat& q);

// Multiplicative inverse; equals the conjugate for unit quaternions.
Quat Inverse(const Quat& q);

// Rotation of `angle` radians about `axis` (need not be normalized).
// A degenerate axis yields the identity.
Quat FromAxisAngle(const Vec3& axis, double angle);

// Rotation by the vector `rotation_vector` (axis * angle), i.e. the
// exponential map. Exact for any magnitude; used to integrate angular
// velocity: q(t + dt) = FromRotationVector(omega * dt) * q(t).
Quat FromRotationVector(const Vec3& rotation_vector);

// Inverse of FromRotationVector: returns axis * angle with angle in [0, pi].
Vec3 ToRotationVector(const Quat& q);

Quat FromEuler(const EulerAngles& angles);
EulerAngles ToEuler(const Quat& q);

// Rotation matrix with the same effect as the unit quaternion `q`.
Mat3 ToMat3(const Quat& q);

// Quaternion of an orthonormal rotation matrix (Shepperd's method).
Quat FromMat3(const Mat3& m);

// Rotates `v` by the unit quaternion `q`.
Vec3 Rotate(const Quat& q, const Vec3& v);

// Shortest-arc rotation taking direction `from` onto direction `to`.
Quat FromTo(const Vec3& from, const Vec3& to);

// Orientation whose local -Z axis points along `forward` and whose local +Y
// is as close as possible to `up`. Handles `forward` parallel to `up`.
Quat LookRotation(const Vec3& forward, const Vec3& up);

// Spherical linear interpolation along the shortest path.
Quat Slerp(const Quat& a, const Quat& b, double t);

// Smallest angle in radians (0..pi) rotating `a` onto `b`. Accurate for
// arbitrarily small angles.
double AngleBetween(const Quat& a, const Quat& b);

// Time derivative of orientation for world-frame angular velocity `omega`:
// dq/dt = 0.5 * (0, omega) * q.
Quat Derivative(const Quat& q, const Vec3& omega);

// True when the rotations differ by at most `tolerance` radians (q and -q
// are equivalent).
bool SameRotation(const Quat& a, const Quat& b, double tolerance = 1e-9);

bool NearlyEqual(const Quat& a, const Quat& b, double tolerance = kEpsilon);

}  // namespace aerolab

#endif  // AEROLAB_MATH_QUAT_H_
