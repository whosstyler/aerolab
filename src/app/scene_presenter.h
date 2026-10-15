#ifndef AEROLAB_APP_SCENE_PRESENTER_H_
#define AEROLAB_APP_SCENE_PRESENTER_H_

#include <vector>

#include "geometry/debug_draw.h"
#include "render/render_types.h"
#include "scene/entity.h"
#include "scene/scene.h"

namespace aerolab {

struct SelectionState {
  EntityId selected;
  EntityId hovered;
};

// Translates the scene into the renderer's flat input: one RenderItem per
// visible mesh, with selection/hover highlighting. This is the only place
// where scene and renderer types meet.
std::vector<RenderItem> BuildRenderItems(const Scene& scene,
                                         const SelectionState& selection);

// Adds bounding boxes and the local coordinate frame of the selected entity,
// and a lighter box around the hovered one.
void AddSelectionOverlay(const Scene& scene, const SelectionState& selection,
                         DebugDrawList& lines);

}  // namespace aerolab

#endif  // AEROLAB_APP_SCENE_PRESENTER_H_
