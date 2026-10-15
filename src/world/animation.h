#ifndef AEROLAB_WORLD_ANIMATION_H_
#define AEROLAB_WORLD_ANIMATION_H_

#include "math/quat.h"
#include "scene/entity.h"
#include "scene/scene.h"

namespace aerolab {

// Advances every SpinComponent by `dt` seconds and updates each entity's
// local rotation. Purely kinematic (no forces).
void AdvanceSpinners(Scene& scene, double dt);

// Returns every spinner to its rest pose (angle zero).
void ResetSpinners(Scene& scene);

// Local rotation of a spinning part at the spin angle interpolated by
// `alpha` between its last two steps.
Quat InterpolatedSpinRotation(const SpinComponent& spin, double alpha);

}  // namespace aerolab

#endif  // AEROLAB_WORLD_ANIMATION_H_
