#ifndef AEROLAB_MATH_INTERSECTION_H_
#define AEROLAB_MATH_INTERSECTION_H_

#include <optional>

#include "math/aabb.h"
#include "math/mat4.h"
#include "math/ray.h"
#include "math/vec3.h"

namespace aerolab {

// Ray parameter t of the first intersection with the box, or 0 if the ray
// starts inside it. Nullopt when the ray misses.
std::optional<double> IntersectRayAabb(const Ray& ray, const Aabb& box);

// Ray parameter t of the first intersection with the sphere, or 0 if the ray
// starts inside it. Nullopt when the ray misses.
std::optional<double> IntersectRaySphere(const Ray& ray, const Vec3& center,
                                         double radius);

// Ray parameter t where the ray crosses the plane through `point` with
// `normal`. Nullopt when parallel or when the crossing is behind the origin.
std::optional<double> IntersectRayPlane(const Ray& ray, const Vec3& point,
                                        const Vec3& normal);

// Intersects a world-space ray with an oriented box given by `local_box` and
// its local-to-world matrix. The returned t is measured along the world ray.
std::optional<double> IntersectRayOrientedBox(const Ray& world_ray,
                                              const Aabb& local_box,
                                              const Mat4& local_to_world);

// Smallest world-space AABB enclosing `local_box` transformed by the affine
// matrix `m` (Arvo's method).
Aabb TransformAabb(const Aabb& local_box, const Mat4& m);

}  // namespace aerolab

#endif  // AEROLAB_MATH_INTERSECTION_H_
