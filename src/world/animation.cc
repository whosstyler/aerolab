#include "world/animation.h"

#include "math/quat.h"
#include "scene/entity.h"

namespace aerolab {

void AdvanceSpinners(Scene& scene, double dt) {
  for (EntityId id : scene.AllEntities()) {
    Entity* entity = scene.Get(id);
    if (!entity->spin.has_value() || entity->spin->rate == 0.0) continue;
    // Post-multiplying applies the step about the entity's own local axis.
    const Quat step =
        FromAxisAngle(entity->spin->axis, entity->spin->rate * dt);
    entity->local.rotation = Normalized(entity->local.rotation * step);
  }
}

}  // namespace aerolab
