#ifndef AEROLAB_SIM_HEADING_H_
#define AEROLAB_SIM_HEADING_H_

#include <cmath>

#include "math/vec3.h"

namespace aerolab::sim {

// The heading convention shared by the flight agents and the autopilots:
// heading 0 faces -Z and positive angles turn left (toward -X), seen from
// above. Angles wrap with WrapAngle from math/scalar.h.

inline Vec3 HeadingVector(double heading) {
  return {-std::sin(heading), 0.0, -std::cos(heading)};
}

// Heading of the horizontal part of `v`.
inline double HeadingOf(const Vec3& v) { return std::atan2(-v.x, -v.z); }

inline double HorizontalLength(const Vec3& v) { return std::hypot(v.x, v.z); }

}  // namespace aerolab::sim

#endif  // AEROLAB_SIM_HEADING_H_
