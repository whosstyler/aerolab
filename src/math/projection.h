#ifndef AEROLAB_MATH_PROJECTION_H_
#define AEROLAB_MATH_PROJECTION_H_

#include <optional>

#include "math/mat4.h"
#include "math/ray.h"
#include "math/vec2.h"
#include "math/vec3.h"

namespace aerolab {

// Right-handed perspective projection into OpenGL clip space: the camera
// looks down -Z and NDC depth spans [-1, 1].
// Requires 0 < vertical_fov_radians < pi, aspect > 0 and
// 0 < near_plane < far_plane.
Mat4 Perspective(double vertical_fov_radians, double aspect, double near_plane,
                 double far_plane);

// Right-handed orthographic projection into OpenGL clip space.
// Requires left != right, bottom != top and near_plane != far_plane.
Mat4 Orthographic(double left, double right, double bottom, double top,
                  double near_plane, double far_plane);

// World -> camera (view) matrix for a camera at `eye` looking at `target`.
Mat4 LookAt(const Vec3& eye, const Vec3& target, const Vec3& up);

// A pixel rectangle with a top-left origin (the UI/window convention).
// Width and height must be positive.
struct ScreenRect {
  double x = 0.0;
  double y = 0.0;
  double width = 1.0;
  double height = 1.0;
};

// Pixel position (top-left origin) -> normalized device coordinates.
Vec2 ScreenToNdc(const Vec2& screen, const ScreenRect& rect);

// Normalized device coordinates -> pixel position (top-left origin).
Vec2 NdcToScreen(const Vec2& ndc, const ScreenRect& rect);

// Projects a world-space point to pixels. Returns nullopt for points at or
// behind the camera plane, which have no meaningful screen position.
std::optional<Vec2> WorldToScreen(const Mat4& view_projection,
                                  const Vec3& world, const ScreenRect& rect);

// Ray from the near plane through the pixel `screen`, in world space.
// `inverse_view_projection` is Inverse(projection * view).
Ray ScreenToWorldRay(const Mat4& inverse_view_projection, const Vec2& screen,
                     const ScreenRect& rect);

}  // namespace aerolab

#endif  // AEROLAB_MATH_PROJECTION_H_
