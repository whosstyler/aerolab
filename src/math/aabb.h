#ifndef AEROLAB_MATH_AABB_H_
#define AEROLAB_MATH_AABB_H_

#include "math/scalar.h"
#include "math/vec3.h"

namespace aerolab {

// Axis-aligned bounding box. A default-constructed box is empty (min > max)
// so that expanding it by a first point yields that point.
struct Aabb {
  Vec3 min{kMaxDouble, kMaxDouble, kMaxDouble};
  Vec3 max{-kMaxDouble, -kMaxDouble, -kMaxDouble};
};

inline Aabb MakeAabb(const Vec3& min, const Vec3& max) { return {min, max}; }

inline Aabb MakeAabbFromCenter(const Vec3& center, const Vec3& half_extents) {
  return {center - half_extents, center + half_extents};
}

inline bool IsEmpty(const Aabb& box) {
  return box.min.x > box.max.x || box.min.y > box.max.y ||
         box.min.z > box.max.z;
}

inline Vec3 Center(const Aabb& box) { return (box.min + box.max) * 0.5; }
inline Vec3 HalfExtents(const Aabb& box) { return (box.max - box.min) * 0.5; }

inline Aabb Expanded(const Aabb& box, const Vec3& point) {
  return {Min(box.min, point), Max(box.max, point)};
}

inline Aabb Union(const Aabb& a, const Aabb& b) {
  if (IsEmpty(a)) return b;
  if (IsEmpty(b)) return a;
  return {Min(a.min, b.min), Max(a.max, b.max)};
}

inline bool Contains(const Aabb& box, const Vec3& point) {
  return point.x >= box.min.x && point.x <= box.max.x && point.y >= box.min.y &&
         point.y <= box.max.y && point.z >= box.min.z && point.z <= box.max.z;
}

// Radius of the sphere centred on the box that encloses it.
inline double BoundingRadius(const Aabb& box) {
  return IsEmpty(box) ? 0.0 : Length(HalfExtents(box));
}

}  // namespace aerolab

#endif  // AEROLAB_MATH_AABB_H_
