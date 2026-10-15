#ifndef AEROLAB_SIM_RIGID_BODY_H_
#define AEROLAB_SIM_RIGID_BODY_H_

#include "math/quat.h"
#include "math/transform.h"
#include "math/vec3.h"

namespace aerolab::sim {

// Kinematic state of a rigid body. All vectors are in world space;
// `orientation` maps body space to world space (body forward is -Z).
struct RigidBodyState {
  Vec3 position;
  Vec3 velocity;          // m/s
  Quat orientation;       // body -> world
  Vec3 angular_velocity;  // rad/s, world frame
};

// Mass and principal moments of inertia (body frame, diagonal tensor).
struct MassProperties {
  double mass = 1.0;                      // kg
  Vec3 principal_inertia{0.1, 0.1, 0.1};  // kg m^2 about body X, Y, Z
};

// Inertia of a solid box with full side lengths `size`.
MassProperties BoxMassProperties(double mass, const Vec3& size);

// Inertia of a solid sphere.
MassProperties SphereMassProperties(double mass, double radius);

// Pose blended between two states for rendering between fixed steps.
Transform InterpolatePose(const RigidBodyState& previous,
                          const RigidBodyState& current, double alpha);

}  // namespace aerolab::sim

#endif  // AEROLAB_SIM_RIGID_BODY_H_
