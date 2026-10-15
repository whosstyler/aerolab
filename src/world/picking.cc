#include "world/picking.h"

#include <optional>

#include "geometry/mesh_library.h"
#include "math/intersection.h"

namespace aerolab {

std::optional<PickHit> PickEntity(const Scene& scene, const Ray& ray) {
  std::optional<PickHit> best;
  for (EntityId id : scene.AllEntities()) {
    const Entity* entity = scene.Get(id);
    if (!entity->render.has_value() || !entity->render->visible) continue;
    const std::optional<double> t = IntersectRayOrientedBox(
        ray, MeshLocalBounds(entity->render->mesh), scene.WorldMatrix(id));
    if (!t.has_value() || (best.has_value() && *t >= best->distance)) {
      continue;
    }
    EntityId target = id;
    if (entity->render->pick_parent && scene.IsAlive(entity->parent)) {
      target = entity->parent;
    }
    best = PickHit{target, *t, PointAt(ray, *t)};
  }
  return best;
}

}  // namespace aerolab
