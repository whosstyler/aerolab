#ifndef AEROLAB_WORLD_ANIMATION_H_
#define AEROLAB_WORLD_ANIMATION_H_

#include "scene/scene.h"

namespace aerolab {

// Advances every SpinComponent by `dt` seconds, rotating each entity's local
// transform about its local spin axis. Purely kinematic (no forces).
void AdvanceSpinners(Scene& scene, double dt);

}  // namespace aerolab

#endif  // AEROLAB_WORLD_ANIMATION_H_
