#include "world/viewpoint.h"

#include "math/quat.h"
#include "math/vec3.h"

namespace aerolab {
namespace {

// Tolerance for "unchanged": the camera's own write is read back exactly,
// so anything beyond rounding is an outside edit.
constexpr double kSyncTolerance = 1e-9;

}  // namespace

void RecordViewpointSync(const Scene& scene, EntityId observer,
                         ViewpointSync& sync) {
  const Entity* entity = scene.Get(observer);
  sync.observer = entity != nullptr ? observer : kNoEntity;
  if (entity != nullptr) sync.local = entity->local;
}

bool ObserverMovedSinceSync(const Scene& scene, EntityId observer,
                            const ViewpointSync& sync) {
  const Entity* entity = scene.Get(observer);
  if (entity == nullptr || observer != sync.observer) return false;
  const Transform& now = entity->local;
  return !NearlyEqual(now.position, sync.local.position, kSyncTolerance) ||
         !NearlyEqual(now.scale, sync.local.scale, kSyncTolerance) ||
         !SameRotation(now.rotation, sync.local.rotation, kSyncTolerance);
}

}  // namespace aerolab
