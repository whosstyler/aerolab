#ifndef AEROLAB_RENDER_AXIS_GIZMO_H_
#define AEROLAB_RENDER_AXIS_GIZMO_H_

#include <algorithm>

namespace aerolab {

// Placement of the corner axis gizmo inside a viewport, in pixels with a
// bottom-left origin (GL convention). The gizmo shows unit axes under an
// orthographic projection spanning [-kExtent, kExtent]. Shared by the
// renderer (which draws it) and the editor (which labels its axes), and
// free of GL so it can be unit tested.
struct AxisGizmoLayout {
  static constexpr double kExtent = 1.35;
  static constexpr int kMinSize = 64;
  static constexpr int kMaxSize = 140;
  int x = 0;
  int y = 0;
  int size = 0;
};

constexpr AxisGizmoLayout ComputeAxisGizmoLayout(int viewport_width,
                                                 int viewport_height) {
  AxisGizmoLayout layout;
  layout.size =
      std::clamp(std::min(viewport_width, viewport_height) / 7,
                 AxisGizmoLayout::kMinSize, AxisGizmoLayout::kMaxSize);
  layout.x = layout.size / 6;
  layout.y = layout.size / 6;
  return layout;
}

}  // namespace aerolab

#endif  // AEROLAB_RENDER_AXIS_GIZMO_H_
