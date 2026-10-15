#include "sim/kinematic_path.h"

#include <algorithm>
#include <cmath>
#include <string_view>

#include "math/scalar.h"
#include "sim/force_model.h"

namespace aerolab::sim {
std::string_view PathShapeName(PathShape shape) {
  switch (shape) {
    case PathShape::kCircle:
      return "Circle";
    case PathShape::kFigureEight:
      return "Figure eight";
    case PathShape::kHelix:
      return "Helix";
    case PathShape::kCount:
      break;
  }
  return "Unknown";
}

PathSample SamplePath(const KinematicPath& path, double time) {
  const double omega =
      (path.clockwise ? -1.0 : 1.0) * kTwoPi / std::max(path.period, 1e-3);
  const double theta = omega * time;
  const double r = path.radius;
  const double s = std::sin(theta);
  const double c = std::cos(theta);

  PathSample out;
  switch (path.shape) {
    case PathShape::kFigureEight: {
      // x = r sin(t), z = (r/2) sin(2t); derivatives by the chain rule.
      const double s2 = std::sin(2.0 * theta);
      const double c2 = std::cos(2.0 * theta);
      out.position = {r * s, 0.0, 0.5 * r * s2};
      out.velocity = {r * omega * c, 0.0, r * omega * c2};
      out.acceleration = {-r * omega * omega * s, 0.0,
                          -2.0 * r * omega * omega * s2};
      break;
    }
    case PathShape::kHelix:
    case PathShape::kCircle:
    case PathShape::kCount: {
      // Counter-clockwise from above: +X toward -Z.
      out.position = {r * c, 0.0, -r * s};
      out.velocity = {-r * omega * s, 0.0, -r * omega * c};
      out.acceleration = {-r * omega * omega * c, 0.0, r * omega * omega * s};
      if (path.shape == PathShape::kHelix) {
        out.position.y = path.climb_rate * time;
        out.velocity.y = path.climb_rate;
      }
      break;
    }
  }
  out.position += path.center;

  // Coordinated turn: the felt "up" is the specific force a - g.
  const Vec3 apparent_up = out.acceleration + Vec3{0.0, kStandardGravity, 0.0};
  out.orientation = LookRotation(out.velocity, apparent_up);
  return out;
}

Vec3 PathCenterForStart(const KinematicPath& path, const Vec3& start) {
  KinematicPath at_origin = path;
  at_origin.center = Vec3::Zero();
  return start - SamplePath(at_origin, 0.0).position;
}

}  // namespace aerolab::sim
