#ifndef AEROLAB_WORLD_SELECTION_H_
#define AEROLAB_WORLD_SELECTION_H_

#include "scene/entity.h"

namespace aerolab {

// Which entity the user has selected and which one is under the pointer.
// Shared by the editor (which changes it) and the presenter (which
// highlights it).
struct SelectionState {
  EntityId selected;
  EntityId hovered;
};

// The observer, if any, that the first-person camera is looking through.
// While in use its figure is hidden from its own view and ignored by
// picking, and it follows the camera around.
struct ViewpointState {
  EntityId observer;
};

}  // namespace aerolab

#endif  // AEROLAB_WORLD_SELECTION_H_
