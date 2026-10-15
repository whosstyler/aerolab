// The central 3D viewport: shows the rendered scene, routes camera input,
// performs picking, and draws screen-space overlays (labels, HUD).

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <optional>
#include <string>

#include "camera/camera.h"
#include "editor/panels.h"
#include "editor/theme.h"
#include "imgui.h"
#include "math/aabb.h"
#include "math/mat3.h"
#include "math/mat4.h"
#include "math/projection.h"
#include "math/ray.h"
#include "math/scalar.h"
#include "math/vec2.h"
#include "render/axis_gizmo.h"
#include "scene/entity.h"
#include "world/picking.h"
#include "world/simulation.h"

namespace aerolab::editor {
namespace {

// Maximum pointer travel, in pixels, for a press-release to count as a click.
constexpr float kClickSlop = 4.0f;
// Labels are drawn only for objects closer than this (metres).
constexpr double kLabelRange = 600.0;

// Maps between texture pixels (top-left origin) and screen points.
struct ImageMapping {
  ImVec2 min;            // Screen position of the image's top-left corner.
  ImVec2 size;           // Image size in screen points.
  double texture_width;  // Texture size in pixels.
  double texture_height;

  ImVec2 ToScreen(const Vec2& pixel) const {
    return ImVec2(
        min.x + static_cast<float>(pixel.x / texture_width) * size.x,
        min.y + static_cast<float>(pixel.y / texture_height) * size.y);
  }
  Vec2 ToPixel(const ImVec2& screen) const {
    return {(screen.x - min.x) / size.x * texture_width,
            (screen.y - min.y) / size.y * texture_height};
  }
  bool Contains(const ImVec2& screen) const {
    return screen.x >= min.x && screen.y >= min.y &&
           screen.x <= min.x + size.x && screen.y <= min.y + size.y;
  }
};

void DrawLabel(ImDrawList* draw, ImVec2 anchor, const std::string& text,
               bool selected) {
  // Slightly smaller than body text so labels stay out of the way.
  ImFont* font = ImGui::GetFont();
  const float font_size = ImGui::GetFontSize() * 0.85f;
  const ImVec2 size = font->CalcTextSizeA(font_size, 1e9f, 0.0f, text.c_str());
  const ImVec2 pad(5.0f * Dpi(), 2.0f * Dpi());
  const ImVec2 min(anchor.x - size.x * 0.5f - pad.x,
                   anchor.y - size.y - pad.y * 2.0f - 4.0f * Dpi());
  const ImVec2 max(min.x + size.x + pad.x * 2.0f, min.y + size.y + pad.y * 2);
  draw->AddRectFilled(min, max, IM_COL32(16, 18, 21, 190), 3.0f);
  if (selected) draw->AddRect(min, max, theme::kAccent, 3.0f);
  draw->AddText(font, font_size, ImVec2(min.x + pad.x, min.y + pad.y),
                selected ? theme::kText : IM_COL32(200, 205, 212, 230),
                text.c_str());
}

void DrawLabels(const EditorFrame& frame, const Mat4& view_projection,
                const ImageMapping& image, ImDrawList* draw) {
  const Scene& scene = *frame.scene;
  const ScreenRect rect{0.0, 0.0, image.texture_width, image.texture_height};
  const Vec3 eye = frame.camera->camera().position;
  const double alpha = frame.simulation->alpha();
  for (EntityId id : scene.roots()) {
    // Label moving objects; static props only when selected or hovered.
    const Entity* entity = scene.Get(id);
    if (entity == nullptr || id == frame.hidden_entity) continue;
    const bool emphasized =
        id == frame.selection->selected || id == frame.selection->hovered;
    if (!entity->body.has_value() && !emphasized) continue;
    const std::optional<Aabb> bounds = RenderWorldBounds(scene, id, alpha);
    if (!bounds.has_value()) continue;
    const Vec3 top{Center(*bounds).x, bounds->max.y, Center(*bounds).z};
    if (Distance(top, eye) > kLabelRange) continue;
    const std::optional<Vec2> pixel = WorldToScreen(view_projection, top, rect);
    if (!pixel.has_value()) continue;
    const ImVec2 screen = image.ToScreen(*pixel);
    if (!image.Contains(screen)) continue;
    DrawLabel(draw, screen, entity->name, id == frame.selection->selected);
  }
}

void DrawHud(const EditorFrame& frame, const ImageMapping& image,
             const std::optional<PickHit>& hover, ImDrawList* draw) {
  const Camera& camera = frame.camera->camera();
  const SimulationSystem& sim = *frame.simulation;
  std::string text = std::format(
      "{} camera   pos {:.1f} {:.1f} {:.1f}   fov {:.0f} deg\n"
      "{}   t = {:.2f} s   x{:g}   step {:.2f} ms",
      CameraModeName(frame.camera->mode()), camera.position.x,
      camera.position.y, camera.position.z,
      Degrees(camera.vertical_fov_radians), sim.paused() ? "PAUSED" : "RUNNING",
      sim.time(), sim.time_scale(), sim.fixed_step() * 1000.0);
  if (frame.camera->mode() == CameraMode::kFreeFly) {
    text += std::format("   cruise {:.1f} m/s",
                        frame.camera->settings().move_speed);
  } else if (frame.camera->mode() == CameraMode::kFirstPerson) {
    text +=
        std::format("   walk {:.1f} m/s", frame.camera->settings().walk_speed);
  }
  if (hover.has_value()) {
    if (const Entity* entity = frame.scene->Get(hover->entity)) {
      text +=
          std::format("\nhover: {}   world {:.2f} {:.2f} {:.2f}", entity->name,
                      hover->point.x, hover->point.y, hover->point.z);
    }
  }
  const ImVec2 size = ImGui::CalcTextSize(text.c_str());
  const float s = Dpi();
  const ImVec2 min(image.min.x + 10 * s, image.min.y + 10 * s);
  draw->AddRectFilled(min,
                      ImVec2(min.x + size.x + 16 * s, min.y + size.y + 10 * s),
                      IM_COL32(16, 18, 21, 170), 4.0f * s);
  draw->AddText(ImVec2(min.x + 8 * s, min.y + 5 * s),
                sim.paused() ? theme::kWarning : theme::kText, text.c_str());
}

// Places X/Y/Z letters at the tips of the renderer's corner axis gizmo.
void DrawGizmoLabels(const Mat4& view, const ImageMapping& image,
                     ImDrawList* draw) {
  const AxisGizmoLayout layout =
      ComputeAxisGizmoLayout(static_cast<int>(image.texture_width),
                             static_cast<int>(image.texture_height));
  const Mat3 rotation = view.UpperLeft3x3();
  const std::array<const char*, 3> names = {"X", "Y", "Z"};
  const std::array<ImU32, 3> colors = {IM_COL32(229, 72, 77, 255),
                                       IM_COL32(70, 167, 88, 255),
                                       IM_COL32(62, 123, 250, 255)};
  for (int axis = 0; axis < 3; ++axis) {
    Vec3 unit;
    unit[axis] = 1.0;
    const Vec3 v = rotation * unit * 1.22;
    // Gizmo NDC -> texture pixels (bottom-left origin) -> top-left origin.
    const double scale = 0.5 * layout.size / AxisGizmoLayout::kExtent;
    const double gx = layout.x + 0.5 * layout.size + v.x * scale;
    const double gy = layout.y + 0.5 * layout.size + v.y * scale;
    const ImVec2 screen = image.ToScreen({gx, image.texture_height - gy});
    const ImVec2 text_size = ImGui::CalcTextSize(names[axis]);
    draw->AddText(
        ImVec2(screen.x - text_size.x * 0.5f, screen.y - text_size.y * 0.5f),
        colors[axis], names[axis]);
  }
}

}  // namespace

void DrawViewportPanel(EditorFrame& frame, const EditorState& state,
                       ViewportState& viewport) {
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  const bool visible = ImGui::Begin(kViewportWindow, nullptr,
                                    ImGuiWindowFlags_NoScrollbar |
                                        ImGuiWindowFlags_NoScrollWithMouse |
                                        ImGuiWindowFlags_NoCollapse);
  ImGui::PopStyleVar();
  if (!visible) {
    viewport.hovered = viewport.focused = viewport.camera_drag = false;
    viewport.click_armed = false;
    ImGui::End();
    return;
  }

  const ImGuiIO& io = ImGui::GetIO();
  const ImVec2 avail(std::max(ImGui::GetContentRegionAvail().x, 1.0f),
                     std::max(ImGui::GetContentRegionAvail().y, 1.0f));
  viewport.pixel_width = std::max(
      1, static_cast<int>(std::lround(avail.x * io.DisplayFramebufferScale.x)));
  viewport.pixel_height = std::max(
      1, static_cast<int>(std::lround(avail.y * io.DisplayFramebufferScale.y)));

  const ImVec2 image_min = ImGui::GetCursorScreenPos();
  // GL textures start at the bottom row; flip vertically for display.
  ImGui::Image(ImTextureRef(static_cast<ImTextureID>(frame.viewport_texture)),
               avail, ImVec2(0, 1), ImVec2(1, 0));
  viewport.hovered = ImGui::IsItemHovered();
  viewport.focused = ImGui::IsWindowFocused();

  // A right/middle drag that starts on the image belongs to the camera
  // until the button is released, even if the pointer leaves the panel.
  if (viewport.hovered && (ImGui::IsMouseClicked(ImGuiMouseButton_Right) ||
                           ImGui::IsMouseClicked(ImGuiMouseButton_Middle))) {
    viewport.camera_drag = true;
    ImGui::SetWindowFocus();
  }
  if (!ImGui::IsMouseDown(ImGuiMouseButton_Right) &&
      !ImGui::IsMouseDown(ImGuiMouseButton_Middle)) {
    viewport.camera_drag = false;
  }

  const ImageMapping image{image_min, avail,
                           static_cast<double>(frame.texture_width),
                           static_cast<double>(frame.texture_height)};
  const Camera& camera = frame.camera->camera();
  const Mat4 view = ViewMatrix(camera);
  const Mat4 view_projection =
      ProjectionMatrix(camera, image.texture_width / image.texture_height) *
      view;

  // Hover and click picking at the rendered (interpolated) poses.
  std::optional<PickHit> hover;
  if (viewport.hovered && !viewport.camera_drag) {
    if (const std::optional<Mat4> inverse = Inverse(view_projection)) {
      const ScreenRect rect{0.0, 0.0, image.texture_width,
                            image.texture_height};
      const Ray ray =
          ScreenToWorldRay(*inverse, image.ToPixel(io.MousePos), rect);
      hover = PickEntity(*frame.scene, ray, frame.simulation->alpha(),
                         frame.hidden_entity);
    }
  }
  frame.selection->hovered = hover.has_value() ? hover->entity : kNoEntity;
  // Only presses that start on the image can select, so releasing a panel
  // splitter drag over the viewport does not change the selection.
  if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
    viewport.click_armed = viewport.hovered;
  }
  if (viewport.click_armed && ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
    viewport.click_armed = false;
    const ImVec2 drag = ImGui::GetMouseDragDelta(ImGuiMouseButton_Left, 0.0f);
    if (viewport.hovered && std::hypot(drag.x, drag.y) < kClickSlop * Dpi()) {
      frame.selection->selected = frame.selection->hovered;
    }
  }

  ImDrawList* draw = ImGui::GetWindowDrawList();
  draw->PushClipRect(image_min,
                     ImVec2(image_min.x + avail.x, image_min.y + avail.y));
  if (state.show_labels) DrawLabels(frame, view_projection, image, draw);
  if (frame.render_settings->show_axis_gizmo) {
    DrawGizmoLabels(view, image, draw);
  }
  if (state.show_hud) DrawHud(frame, image, hover, draw);
  draw->PopClipRect();
  ImGui::End();
}

}  // namespace aerolab::editor
