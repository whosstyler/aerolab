#include "math/quat.h"

#include <algorithm>
#include <cmath>

namespace aerolab {

double Length(const Quat& q) { return std::sqrt(Dot(q, q)); }

Quat Normalized(const Quat& q) {
  const double length = Length(q);
  if (length < kEpsilon) return Quat::Identity();
  return q * (1.0 / length);
}

Quat Inverse(const Quat& q) {
  const double norm_squared = Dot(q, q);
  if (norm_squared < kEpsilon * kEpsilon) return Quat::Identity();
  return Conjugate(q) * (1.0 / norm_squared);
}

Quat FromAxisAngle(const Vec3& axis, double angle) {
  const double length = Length(axis);
  if (length < kEpsilon) return Quat::Identity();
  const Vec3 unit = axis / length;
  const double half = 0.5 * angle;
  const double s = std::sin(half);
  return {std::cos(half), unit.x * s, unit.y * s, unit.z * s};
}

Quat FromRotationVector(const Vec3& rotation_vector) {
  const double angle = Length(rotation_vector);
  if (angle == 0.0) return Quat::Identity();
  // Scale the vector directly instead of normalizing an axis first, so even
  // tiny per-step rotations (omega * dt ~ 1e-12 rad) are represented exactly.
  const double half = 0.5 * angle;
  const double s = std::sin(half) / angle;
  return {std::cos(half), rotation_vector.x * s, rotation_vector.y * s,
          rotation_vector.z * s};
}

Vec3 ToRotationVector(const Quat& q) {
  // Use the hemisphere with w >= 0 so the angle lies in [0, pi].
  const Quat p = q.w < 0.0 ? -q : q;
  const Vec3 v{p.x, p.y, p.z};
  const double sin_half = Length(v);
  if (sin_half < 1e-12) return v * 2.0;
  const double angle = 2.0 * std::atan2(sin_half, p.w);
  return v * (angle / sin_half);
}

Quat FromEuler(const EulerAngles& angles) {
  const Quat yaw = FromAxisAngle(Vec3::UnitY(), angles.yaw);
  const Quat pitch = FromAxisAngle(Vec3::UnitX(), angles.pitch);
  const Quat roll = FromAxisAngle(Vec3::UnitZ(), angles.roll);
  // Intrinsic Y-X'-Z'': the right-most factor is applied first in body axes.
  return yaw * pitch * roll;
}

EulerAngles ToEuler(const Quat& q) {
  // For R = Ry(yaw) * Rx(pitch) * Rz(roll):
  //   m12 = -sin(pitch), m02 = sin(yaw)cos(pitch), m22 = cos(yaw)cos(pitch),
  //   m10 = cos(pitch)sin(roll), m11 = cos(pitch)cos(roll).
  const Mat3 m = ToMat3(Normalized(q));
  EulerAngles angles;
  const double sin_pitch = std::clamp(-m(1, 2), -1.0, 1.0);
  angles.pitch = std::asin(sin_pitch);
  if (std::abs(sin_pitch) < 0.9999999) {
    angles.yaw = std::atan2(m(0, 2), m(2, 2));
    angles.roll = std::atan2(m(1, 0), m(1, 1));
  } else {
    // Gimbal lock: yaw and roll share an axis. Attribute it all to yaw.
    angles.yaw = std::atan2(-m(2, 0), m(0, 0));
    angles.roll = 0.0;
  }
  return angles;
}

Mat3 ToMat3(const Quat& q) {
  const double xx = q.x * q.x;
  const double yy = q.y * q.y;
  const double zz = q.z * q.z;
  const double xy = q.x * q.y;
  const double xz = q.x * q.z;
  const double yz = q.y * q.z;
  const double wx = q.w * q.x;
  const double wy = q.w * q.y;
  const double wz = q.w * q.z;
  return Mat3::FromRows(
      {1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)},
      {2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)},
      {2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)});
}

Quat FromMat3(const Mat3& m) {
  const double trace = m(0, 0) + m(1, 1) + m(2, 2);
  Quat q;
  // Pick the largest of w, x, y, z to divide by, for numerical stability.
  if (trace > 0.0) {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    q = {0.25 * s, (m(2, 1) - m(1, 2)) / s, (m(0, 2) - m(2, 0)) / s,
         (m(1, 0) - m(0, 1)) / s};
  } else if (m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2)) {
    const double s = std::sqrt(1.0 + m(0, 0) - m(1, 1) - m(2, 2)) * 2.0;
    q = {(m(2, 1) - m(1, 2)) / s, 0.25 * s, (m(0, 1) + m(1, 0)) / s,
         (m(0, 2) + m(2, 0)) / s};
  } else if (m(1, 1) > m(2, 2)) {
    const double s = std::sqrt(1.0 + m(1, 1) - m(0, 0) - m(2, 2)) * 2.0;
    q = {(m(0, 2) - m(2, 0)) / s, (m(0, 1) + m(1, 0)) / s, 0.25 * s,
         (m(1, 2) + m(2, 1)) / s};
  } else {
    const double s = std::sqrt(1.0 + m(2, 2) - m(0, 0) - m(1, 1)) * 2.0;
    q = {(m(1, 0) - m(0, 1)) / s, (m(0, 2) + m(2, 0)) / s,
         (m(1, 2) + m(2, 1)) / s, 0.25 * s};
  }
  return Normalized(q);
}

Vec3 Rotate(const Quat& q, const Vec3& v) {
  // v' = v + 2w(u x v) + 2u x (u x v), with u the vector part of q.
  const Vec3 u{q.x, q.y, q.z};
  const Vec3 t = Cross(u, v) * 2.0;
  return v + t * q.w + Cross(u, t);
}

Quat FromTo(const Vec3& from, const Vec3& to) {
  const Vec3 a = Normalized(from);
  const Vec3 b = Normalized(to);
  const double d = Dot(a, b);
  const Vec3 c = Cross(a, b);
  if (LengthSquared(c) < kDegenerateLengthSquared) {
    // Parallel: no rotation. Opposite: the axis is undefined, so rotate pi
    // about any perpendicular axis.
    return d > 0.0 ? Quat::Identity()
                   : FromAxisAngle(AnyPerpendicular(a), kPi);
  }
  // Axis-angle with atan2 stays accurate near 0 and near pi, unlike the
  // half-way trick (1 + d, c) which cancels catastrophically as d -> -1.
  return FromAxisAngle(c, std::atan2(Length(c), d));
}

Quat LookRotation(const Vec3& forward, const Vec3& up) {
  const Vec3 f = NormalizedOr(forward, -Vec3::UnitZ());
  Vec3 right = Cross(f, up);
  if (LengthSquared(right) < 1e-12) {
    // Forward is parallel to up; any perpendicular reference works.
    right = Cross(f, AnyPerpendicular(f));
  }
  right = Normalized(right);
  const Vec3 true_up = Cross(right, f);
  // Local axes: +X -> right, +Y -> up, -Z -> forward.
  return FromMat3(Mat3::FromColumns(right, true_up, -f));
}

Quat Slerp(const Quat& a, const Quat& b, double t) {
  Quat end = b;
  double cos_theta = Dot(a, b);
  if (cos_theta < 0.0) {
    end = -b;
    cos_theta = -cos_theta;
  }
  if (cos_theta > 0.9995) {
    // Nearly parallel: normalized lerp avoids dividing by sin(theta) ~ 0.
    return Normalized(a * (1.0 - t) + end * t);
  }
  const double theta = std::acos(std::clamp(cos_theta, -1.0, 1.0));
  const double sin_theta = std::sin(theta);
  const double wa = std::sin((1.0 - t) * theta) / sin_theta;
  const double wb = std::sin(t * theta) / sin_theta;
  return Normalized(a * wa + end * wb);
}

double AngleBetween(const Quat& a, const Quat& b) {
  // The relative rotation's half-angle via atan2 stays accurate for tiny
  // angles, unlike acos(dot) which loses about half the significant digits.
  const Quat delta = Conjugate(Normalized(a)) * Normalized(b);
  const double sin_half = Length(Vec3{delta.x, delta.y, delta.z});
  return 2.0 * std::atan2(sin_half, std::abs(delta.w));
}

Quat Derivative(const Quat& q, const Vec3& omega) {
  return Quat{0.0, omega.x, omega.y, omega.z} * q * 0.5;
}

bool SameRotation(const Quat& a, const Quat& b, double tolerance) {
  return AngleBetween(a, b) <= tolerance;
}

bool NearlyEqual(const Quat& a, const Quat& b, double tolerance) {
  return NearlyEqual(a.w, b.w, tolerance) && NearlyEqual(a.x, b.x, tolerance) &&
         NearlyEqual(a.y, b.y, tolerance) && NearlyEqual(a.z, b.z, tolerance);
}

}  // namespace aerolab
