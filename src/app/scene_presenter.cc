#include "app/scene_presenter.h"

#include <algorithm>
#include <vector>

#include "geometry/color.h"
#include "geometry/mesh_library.h"
#include "math/aabb.h"
#include "math/mat4.h"
#include "world/simulation.h"

namespace aerolab {
namespace {

// True if `id` is `root` or one of its descendants.
bool InSubtree(const Scene& scene, EntityId root, EntityId id) {
  return IsSet(root) && scene.IsAncestorOrSelf(root, id);
}

}  // namespace

std::vector<RenderItem> BuildRenderItems(const Scene& scene,
                                         const SelectionState& selection,
                                         double alpha, EntityId hidden) {
  std::vector<RenderItem> items;
  items.reserve(scene.size());
  for (EntityId id : scene.AllEntities()) {
    const Entity* entity = scene.Get(id);
    if (!entity->render.has_value() || !entity->render->visible) continue;
    if (InSubtree(scene, hidden, id)) continue;
    RenderItem item;
    item.mesh = entity->render->mesh;
    item.model = RenderWorldMatrix(scene, id, alpha);
    item.material = entity->render->material;
    item.cast_shadow = entity->render->cast_shadow;
    if (InSubtree(scene, selection.selected, id)) {
      item.highlight = 1.0f;
      item.highlight_color = colors::kSelection;
    } else if (InSubtree(scene, selection.hovered, id)) {
      item.highlight = 0.45f;
      item.highlight_color = colors::kHover;
    }
    items.push_back(item);
  }
  return items;
}

void AddSelectionOverlay(const Scene& scene, const SelectionState& selection,
                         DebugDrawList& lines, double alpha, EntityId hidden) {
  if (const Entity* hovered = scene.Get(selection.hovered);
      hovered != nullptr && hovered->render.has_value() &&
      selection.hovered != selection.selected &&
      !InSubtree(scene, hidden, selection.hovered)) {
    lines.AddOrientedBox(MeshLocalBounds(hovered->render->mesh),
                         RenderWorldMatrix(scene, selection.hovered, alpha),
                         WithAlpha(colors::kHover, 0.6f));
  }
  const Entity* selected = scene.Get(selection.selected);
  if (selected == nullptr || InSubtree(scene, hidden, selection.selected)) {
    return;
  }
  const Mat4 world = RenderWorldMatrix(scene, selection.selected, alpha);
  double frame_size = 1.0;
  if (selected->render.has_value()) {
    const Aabb& bounds = MeshLocalBounds(selected->render->mesh);
    lines.AddOrientedBox(bounds, world, colors::kSelection);
    frame_size = std::max(BoundingRadius(bounds) * 1.4, 0.5);
  }
  // The local frame shows the object's own X/Y/Z axes (forward is -Z).
  lines.AddAxes(world, frame_size);
}

}  // namespace aerolab
