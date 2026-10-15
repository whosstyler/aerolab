#include "math/transform.h"

namespace aerolab {

Mat4 ToMatrix(const Transform& t) {
  return Mat4::Trs(t.position, t.rotation, t.scale);
}

Transform FromMatrix(const Mat4& m) {
  Transform t;
  t.position = m.TranslationPart();
  const Mat3 linear = m.UpperLeft3x3();
  const Vec3 c0 = linear.Column(0);
  const Vec3 c1 = linear.Column(1);
  const Vec3 c2 = linear.Column(2);
  t.scale = {Length(c0), Length(c1), Length(c2)};
  const Vec3 safe_scale = Max(t.scale, Vec3{kEpsilon, kEpsilon, kEpsilon});
  t.rotation = FromMat3(Mat3::FromColumns(c0 / safe_scale.x, c1 / safe_scale.y,
                                          c2 / safe_scale.z));
  return t;
}

Vec3 TransformPoint(const Transform& t, const Vec3& local_point) {
  return t.position + Rotate(t.rotation, Hadamard(t.scale, local_point));
}

Vec3 TransformDirection(const Transform& t, const Vec3& local_direction) {
  return Rotate(t.rotation, local_direction);
}

Vec3 InverseTransformPoint(const Transform& t, const Vec3& parent_point) {
  const Vec3 unrotated =
      Rotate(Conjugate(t.rotation), parent_point - t.position);
  const Vec3 safe_scale = Max(Abs(t.scale), Vec3{kEpsilon, kEpsilon, kEpsilon});
  return {unrotated.x / safe_scale.x, unrotated.y / safe_scale.y,
          unrotated.z / safe_scale.z};
}

Vec3 InverseTransformDirection(const Transform& t,
                               const Vec3& parent_direction) {
  return Rotate(Conjugate(t.rotation), parent_direction);
}

Vec3 Forward(const Transform& t) { return Rotate(t.rotation, -Vec3::UnitZ()); }
Vec3 Right(const Transform& t) { return Rotate(t.rotation, Vec3::UnitX()); }
Vec3 Up(const Transform& t) { return Rotate(t.rotation, Vec3::UnitY()); }

Transform Interpolate(const Transform& a, const Transform& b, double alpha) {
  return {Lerp(a.position, b.position, alpha),
          Slerp(a.rotation, b.rotation, alpha), Lerp(a.scale, b.scale, alpha)};
}

bool NearlyEqual(const Transform& a, const Transform& b, double tolerance) {
  return NearlyEqual(a.position, b.position, tolerance) &&
         SameRotation(a.rotation, b.rotation, tolerance) &&
         NearlyEqual(a.scale, b.scale, tolerance);
}

}  // namespace aerolab
