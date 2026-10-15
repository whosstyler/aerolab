#include "math/projection.h"

#include <cassert>
#include <cmath>
#include <optional>

#include "math/scalar.h"
#include "math/vec4.h"

namespace aerolab {

Mat4 Perspective(double vertical_fov_radians, double aspect, double near_plane,
                 double far_plane) {
  assert(vertical_fov_radians > 0.0 && vertical_fov_radians < kPi);
  assert(aspect > 0.0);
  assert(near_plane > 0.0 && near_plane < far_plane);
  const double f = 1.0 / std::tan(0.5 * vertical_fov_radians);
  const double depth = near_plane - far_plane;
  Mat4 m = Mat4::Zero();
  m(0, 0) = f / aspect;
  m(1, 1) = f;
  m(2, 2) = (far_plane + near_plane) / depth;
  m(2, 3) = 2.0 * far_plane * near_plane / depth;
  m(3, 2) = -1.0;
  return m;
}

Mat4 Orthographic(double left, double right, double bottom, double top,
                  double near_plane, double far_plane) {
  assert(left != right && bottom != top && near_plane != far_plane);
  Mat4 m;
  m(0, 0) = 2.0 / (right - left);
  m(1, 1) = 2.0 / (top - bottom);
  m(2, 2) = -2.0 / (far_plane - near_plane);
  m(0, 3) = -(right + left) / (right - left);
  m(1, 3) = -(top + bottom) / (top - bottom);
  m(2, 3) = -(far_plane + near_plane) / (far_plane - near_plane);
  return m;
}

Mat4 LookAt(const Vec3& eye, const Vec3& target, const Vec3& up) {
  const Vec3 f = NormalizedOr(target - eye, -Vec3::UnitZ());
  Vec3 r = Cross(f, up);
  if (LengthSquared(r) < 1e-12) r = AnyPerpendicular(f);
  r = Normalized(r);
  const Vec3 u = Cross(r, f);
  // Rows are the camera basis; the translation moves the eye to the origin.
  return Mat4::FromColumns({r.x, u.x, -f.x, 0.0}, {r.y, u.y, -f.y, 0.0},
                           {r.z, u.z, -f.z, 0.0},
                           {-Dot(r, eye), -Dot(u, eye), Dot(f, eye), 1.0});
}

Vec2 ScreenToNdc(const Vec2& screen, const ScreenRect& rect) {
  return {2.0 * (screen.x - rect.x) / rect.width - 1.0,
          1.0 - 2.0 * (screen.y - rect.y) / rect.height};
}

Vec2 NdcToScreen(const Vec2& ndc, const ScreenRect& rect) {
  return {rect.x + (ndc.x + 1.0) * 0.5 * rect.width,
          rect.y + (1.0 - ndc.y) * 0.5 * rect.height};
}

std::optional<Vec2> WorldToScreen(const Mat4& view_projection,
                                  const Vec3& world, const ScreenRect& rect) {
  const Vec4 clip = view_projection * MakePoint(world);
  if (clip.w <= kEpsilon) return std::nullopt;
  return NdcToScreen({clip.x / clip.w, clip.y / clip.w}, rect);
}

Ray ScreenToWorldRay(const Mat4& inverse_view_projection, const Vec2& screen,
                     const ScreenRect& rect) {
  const Vec2 ndc = ScreenToNdc(screen, rect);
  const Vec3 near_point =
      TransformPointProjective(inverse_view_projection, {ndc.x, ndc.y, -1.0});
  const Vec3 far_point =
      TransformPointProjective(inverse_view_projection, {ndc.x, ndc.y, 1.0});
  return {near_point, NormalizedOr(far_point - near_point, -Vec3::UnitZ())};
}

}  // namespace aerolab
