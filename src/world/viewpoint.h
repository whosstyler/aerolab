#ifndef AEROLAB_WORLD_VIEWPOINT_H_
#define AEROLAB_WORLD_VIEWPOINT_H_

#include "math/transform.h"
#include "scene/entity.h"
#include "scene/scene.h"

namespace aerolab {

// What the first-person camera last wrote to the observer it looks
// through, so a change made by anything else (an inspector edit) can be
// told apart from the camera's own updates and followed.
struct ViewpointSync {
  EntityId observer;
  Transform local;  // The observer's local transform after the write.
};

// Records `observer`'s current local transform in `sync`.
void RecordViewpointSync(const Scene& scene, EntityId observer,
                         ViewpointSync& sync);

// True when `observer` is the one recorded in `sync` and its local
// transform has changed since. Local (not world) transforms are compared,
// so a moving or non-uniformly scaled parent does not count as an edit.
bool ObserverMovedSinceSync(const Scene& scene, EntityId observer,
                            const ViewpointSync& sync);

}  // namespace aerolab

#endif  // AEROLAB_WORLD_VIEWPOINT_H_
