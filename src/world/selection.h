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

}  // namespace aerolab

#endif  // AEROLAB_WORLD_SELECTION_H_
