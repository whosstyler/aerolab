#include "sim/rigid_body.h"

namespace aerolab::sim {

MassProperties BoxMassProperties(double mass, const Vec3& size) {
  const double k = mass / 12.0;
  return {mass,
          {k * (size.y * size.y + size.z * size.z),
           k * (size.x * size.x + size.z * size.z),
           k * (size.x * size.x + size.y * size.y)}};
}

MassProperties SphereMassProperties(double mass, double radius) {
  const double i = 0.4 * mass * radius * radius;
  return {mass, {i, i, i}};
}

Transform InterpolatePose(const RigidBodyState& previous,
                          const RigidBodyState& current, double alpha) {
  Transform pose;
  pose.position = Lerp(previous.position, current.position, alpha);
  pose.rotation = Slerp(previous.orientation, current.orientation, alpha);
  return pose;
}

}  // namespace aerolab::sim
