#ifndef AEROLAB_MATH_RAY_H_
#define AEROLAB_MATH_RAY_H_

#include "math/vec3.h"

namespace aerolab {

// Half-line origin + t * direction for t >= 0. `direction` is normally unit
// length, in which case t is a distance.
struct Ray {
  Vec3 origin;
  Vec3 direction = -Vec3::UnitZ();
};

constexpr Vec3 PointAt(const Ray& ray, double t) {
  return ray.origin + ray.direction * t;
}

}  // namespace aerolab

#endif  // AEROLAB_MATH_RAY_H_
