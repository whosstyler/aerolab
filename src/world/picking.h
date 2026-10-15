#ifndef AEROLAB_WORLD_PICKING_H_
#define AEROLAB_WORLD_PICKING_H_

#include <optional>

#include "math/ray.h"
#include "math/vec3.h"
#include "scene/entity.h"
#include "scene/scene.h"

namespace aerolab {

struct PickHit {
  EntityId entity;
  double distance = 0.0;  // Along the (unit) ray.
  Vec3 point;
};

// Casts `ray` against the oriented bounding boxes of every visible mesh and
// returns the nearest hit. Hits on parts marked `pick_parent` resolve to the
// part's parent so clicking a rotor selects the whole vehicle. Objects are
// tested at their rendered pose (`alpha`, see RenderWorldMatrix) so picks
// match what is on screen.
std::optional<PickHit> PickEntity(const Scene& scene, const Ray& ray,
                                  double alpha = 1.0);

}  // namespace aerolab

#endif  // AEROLAB_WORLD_PICKING_H_
