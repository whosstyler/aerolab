#include "math/intersection.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>

#include "math/scalar.h"

namespace aerolab {
namespace {

// A ray direction component below this is treated as parallel to the slab;
// it is far below any meaningful direction yet keeps 1/d finite.
constexpr double kAxisParallelTolerance = 1e-15;

// |n . d| below this means the ray runs parallel to the plane.
constexpr double kPlaneParallelTolerance = 1e-12;

}  // namespace

std::optional<double> IntersectRayAabb(const Ray& ray, const Aabb& box) {
  if (IsEmpty(box)) return std::nullopt;
  double t_enter = -kMaxDouble;
  double t_exit = kMaxDouble;
  // Slab method: clip the ray's parameter range against each axis' slab.
  for (int axis = 0; axis < 3; ++axis) {
    const double origin = ray.origin[axis];
    const double direction = ray.direction[axis];
    if (std::abs(direction) < kAxisParallelTolerance) {
      if (origin < box.min[axis] || origin > box.max[axis]) return std::nullopt;
      continue;
    }
    const double inv = 1.0 / direction;
    double t0 = (box.min[axis] - origin) * inv;
    double t1 = (box.max[axis] - origin) * inv;
    if (t0 > t1) std::swap(t0, t1);
    t_enter = std::max(t_enter, t0);
    t_exit = std::min(t_exit, t1);
    if (t_enter > t_exit) return std::nullopt;
  }
  if (t_exit < 0.0) return std::nullopt;
  return std::max(t_enter, 0.0);
}

std::optional<double> IntersectRaySphere(const Ray& ray, const Vec3& center,
                                         double radius) {
  const Vec3 offset = ray.origin - center;
  const double a = LengthSquared(ray.direction);
  if (a < kDegenerateLengthSquared) return std::nullopt;
  const double half_b = Dot(offset, ray.direction);
  const double c = LengthSquared(offset) - radius * radius;
  if (c <= 0.0) return 0.0;  // Origin inside the sphere.
  const double discriminant = half_b * half_b - a * c;
  if (discriminant < 0.0) return std::nullopt;
  const double t = (-half_b - std::sqrt(discriminant)) / a;
  if (t < 0.0) return std::nullopt;
  return t;
}

std::optional<double> IntersectRayPlane(const Ray& ray, const Vec3& point,
                                        const Vec3& normal) {
  const double denom = Dot(normal, ray.direction);
  if (std::abs(denom) < kPlaneParallelTolerance) return std::nullopt;
  const double t = Dot(point - ray.origin, normal) / denom;
  if (t < 0.0) return std::nullopt;
  return t;
}

std::optional<double> IntersectRayOrientedBox(const Ray& world_ray,
                                              const Aabb& local_box,
                                              const Mat4& local_to_world) {
  const std::optional<Mat4> world_to_local = AffineInverse(local_to_world);
  if (!world_to_local.has_value()) return std::nullopt;
  // The direction is deliberately not renormalized: an affine map preserves
  // the ray parameter, so the local t equals the world t.
  const Ray local_ray{TransformPoint(*world_to_local, world_ray.origin),
                      TransformDirection(*world_to_local, world_ray.direction)};
  return IntersectRayAabb(local_ray, local_box);
}

Aabb TransformAabb(const Aabb& local_box, const Mat4& m) {
  if (IsEmpty(local_box)) return local_box;
  const Vec3 translation = m.TranslationPart();
  Aabb result{translation, translation};
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      const double a = m(row, col) * local_box.min[col];
      const double b = m(row, col) * local_box.max[col];
      result.min[row] += std::min(a, b);
      result.max[row] += std::max(a, b);
    }
  }
  return result;
}

}  // namespace aerolab
