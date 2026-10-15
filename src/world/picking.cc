#include "world/picking.h"

#include <optional>

#include "geometry/mesh_library.h"
#include "math/intersection.h"
#include "world/simulation.h"

namespace aerolab {

std::optional<PickHit> PickEntity(const Scene& scene, const Ray& ray,
                                  double alpha, EntityId hidden) {
  std::optional<PickHit> best;
  scene.ForEachEntity([&](EntityId id, const Entity& entity_ref) {
    const Entity* entity = &entity_ref;
    if (!entity->render.has_value() || !entity->render->visible) return;
    if (IsSet(hidden) && scene.IsAncestorOrSelf(hidden, id)) return;
    const std::optional<double> t =
        IntersectRayOrientedBox(ray, MeshLocalBounds(entity->render->mesh),
                                RenderWorldMatrix(scene, id, alpha));
    if (!t.has_value() || (best.has_value() && *t >= best->distance)) {
      return;
    }
    EntityId target = id;
    if (entity->render->pick_parent && scene.IsAlive(entity->parent)) {
      target = entity->parent;
    }
    best = PickHit{target, *t, PointAt(ray, *t)};
  });
  return best;
}

}  // namespace aerolab
