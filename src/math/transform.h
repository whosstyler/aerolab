#ifndef AEROLAB_MATH_TRANSFORM_H_
#define AEROLAB_MATH_TRANSFORM_H_

#include "math/mat4.h"
#include "math/quat.h"
#include "math/scalar.h"
#include "math/vec3.h"

namespace aerolab {

// Translation-rotation-scale transform mapping local space into its parent
// space: p_parent = position + rotation * (scale * p_local).
//
// Local axes follow the engine convention: +X right, +Y up, -Z forward.
struct Transform {
  Vec3 position;
  Quat rotation;
  Vec3 scale = Vec3::One();
};

// Matrix form: Translation * Rotation * Scale.
Mat4 ToMatrix(const Transform& t);

// Decomposes an affine matrix without shear into translation, rotation and
// positive scale. Mirrored (negative determinant) inputs are not supported.
Transform FromMatrix(const Mat4& m);

// Local point -> parent space.
Vec3 TransformPoint(const Transform& t, const Vec3& local_point);

// Local direction -> parent space (rotation only; length is preserved).
Vec3 TransformDirection(const Transform& t, const Vec3& local_direction);

// Parent-space point -> local space.
Vec3 InverseTransformPoint(const Transform& t, const Vec3& parent_point);

// Parent-space direction -> local space (rotation only).
Vec3 InverseTransformDirection(const Transform& t,
                               const Vec3& parent_direction);

// Local basis vectors expressed in parent space.
Vec3 Forward(const Transform& t);  // local -Z
Vec3 Right(const Transform& t);    // local +X
Vec3 Up(const Transform& t);       // local +Y

// Interpolates position/scale linearly and rotation spherically. Used to
// blend simulation states for rendering between fixed steps.
Transform Interpolate(const Transform& a, const Transform& b, double alpha);

bool NearlyEqual(const Transform& a, const Transform& b,
                 double tolerance = kEpsilon);

}  // namespace aerolab

#endif  // AEROLAB_MATH_TRANSFORM_H_
