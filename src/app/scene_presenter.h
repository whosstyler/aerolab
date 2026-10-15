#ifndef AEROLAB_APP_SCENE_PRESENTER_H_
#define AEROLAB_APP_SCENE_PRESENTER_H_

#include <vector>

#include "geometry/debug_draw.h"
#include "render/render_types.h"
#include "scene/entity.h"
#include "scene/scene.h"
#include "world/selection.h"

namespace aerolab {

// Translates the scene into the renderer's flat input: one RenderItem per
// visible mesh, with selection/hover highlighting. This is the only place
// where scene and renderer types meet. Simulated bodies are placed at their
// pose interpolated by `alpha` between the last two fixed steps. `hidden`
// and its descendants are left out (the observer the camera is inside).
std::vector<RenderItem> BuildRenderItems(const Scene& scene,
                                         const SelectionState& selection,
                                         double alpha = 1.0,
                                         EntityId hidden = kNoEntity);

// Adds bounding boxes and the local coordinate frame of the selected entity,
// and a lighter box around the hovered one.
void AddSelectionOverlay(const Scene& scene, const SelectionState& selection,
                         DebugDrawList& lines, double alpha = 1.0,
                         EntityId hidden = kNoEntity);

}  // namespace aerolab

#endif  // AEROLAB_APP_SCENE_PRESENTER_H_
