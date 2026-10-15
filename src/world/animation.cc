#include "world/animation.h"

#include <cmath>

#include "math/quat.h"
#include "math/scalar.h"
#include "scene/entity.h"

namespace aerolab {
namespace {

// Keeps angles small without breaking interpolation: both angles are
// shifted by the same whole number of turns.
constexpr double kRewrapThreshold = 1000.0;

}  // namespace

void AdvanceSpinners(Scene& scene, double dt) {
  scene.ForEachEntity([dt](EntityId, Entity& entity) {
    if (!entity.spin.has_value()) return;
    SpinComponent& spin = *entity.spin;
    spin.previous_angle = spin.angle;
    spin.angle += spin.rate * dt;
    if (std::abs(spin.angle) > kRewrapThreshold) {
      const double turns = std::trunc(spin.angle / kTwoPi) * kTwoPi;
      spin.angle -= turns;
      spin.previous_angle -= turns;
    }
    entity.local.rotation = InterpolatedSpinRotation(spin, 1.0);
  });
}

void ResetSpinners(Scene& scene) {
  scene.ForEachEntity([](EntityId, Entity& entity) {
    if (!entity.spin.has_value()) return;
    entity.spin->angle = entity.spin->previous_angle = 0.0;
    entity.local.rotation = entity.spin->rest_rotation;
  });
}

Quat InterpolatedSpinRotation(const SpinComponent& spin, double alpha) {
  const double angle = Lerp(spin.previous_angle, spin.angle, alpha);
  // Post-multiplying applies the spin about the part's own local axis.
  return Normalized(spin.rest_rotation * FromAxisAngle(spin.axis, angle));
}

}  // namespace aerolab
