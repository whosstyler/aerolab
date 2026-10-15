// Scene hierarchy: a tree of every entity with selection, filtering,
// drag-and-drop re-parenting and a context menu.

#include <algorithm>
#include <array>
#include <cstring>
#include <optional>
#include <string>
#include <string_view>

#include "core/log.h"
#include "core/string_util.h"
#include "editor/panels.h"
#include "editor/widgets.h"
#include "imgui.h"
#include "scene/entity.h"
#include "scene/scene.h"

namespace aerolab::editor {
namespace {

constexpr const char* kEntityPayload = "AEROLAB_ENTITY";

// Reads an EntityId from a drag-and-drop payload of the expected size.
std::optional<EntityId> PayloadEntity(const ImGuiPayload* payload) {
  if (payload == nullptr ||
      payload->DataSize != static_cast<int>(sizeof(EntityId))) {
    return std::nullopt;
  }
  EntityId id;
  std::memcpy(&id, payload->Data, sizeof(EntityId));
  return id;
}

// True if the entity or any descendant matches the filter.
bool MatchesFilter(const Scene& scene, EntityId id, std::string_view filter) {
  const Entity* entity = scene.Get(id);
  if (entity == nullptr) return false;
  if (ContainsIgnoringCase(entity->name, filter)) return true;
  return std::any_of(
      entity->children.begin(), entity->children.end(),
      [&](EntityId child) { return MatchesFilter(scene, child, filter); });
}

const char* TypeTag(const Entity& entity) {
  if (entity.body.has_value()) {
    return entity.body->motion == MotionType::kKinematic ? "kinematic"
                                                         : "dynamic";
  }
  if (entity.spin.has_value()) return "spinner";
  if (entity.render.has_value()) return "static";
  return "group";
}

struct TreeContext {
  EditorFrame* frame;
  std::string_view filter;
  EntityId pending_delete;
  EntityId pending_unparent;
  // Re-parenting is deferred until after the tree is drawn.
  EntityId reparent_child;
  EntityId reparent_parent;
  bool reparent_requested = false;
  bool filter_changed = false;
};

void DrawNode(TreeContext& context, EntityId id) {
  const Scene& scene = *context.frame->scene;
  const Entity* entity = scene.Get(id);
  if (entity == nullptr || !MatchesFilter(scene, id, context.filter)) return;

  ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow |
                             ImGuiTreeNodeFlags_OpenOnDoubleClick |
                             ImGuiTreeNodeFlags_SpanAvailWidth;
  if (entity->children.empty()) flags |= ImGuiTreeNodeFlags_Leaf;
  if (id == context.frame->selection->selected) {
    flags |= ImGuiTreeNodeFlags_Selected;
  }
  // Expand matching branches when the filter changes, but let the user
  // collapse them afterwards.
  if (context.filter_changed && !context.filter.empty()) {
    ImGui::SetNextItemOpen(true);
  }

  ImGui::PushID(static_cast<int>(id.index));
  // Window-local x of the row's right edge, for the right-aligned tag.
  const float right_edge =
      ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x;
  const bool open =
      ImGui::TreeNodeEx("##node", flags, "%s", entity->name.c_str());
  if (ImGui::IsItemClicked(ImGuiMouseButton_Left) &&
      !ImGui::IsItemToggledOpen()) {
    context.frame->selection->selected = id;
  }
  if (ImGui::IsItemHovered()) context.frame->selection->hovered = id;

  // Drag an entity onto another to make it a child.
  if (ImGui::BeginDragDropSource()) {
    ImGui::SetDragDropPayload(kEntityPayload, &id, sizeof(EntityId));
    ImGui::Text("Move '%s'", entity->name.c_str());
    ImGui::EndDragDropSource();
  }
  if (ImGui::BeginDragDropTarget()) {
    if (const std::optional<EntityId> dragged =
            PayloadEntity(ImGui::AcceptDragDropPayload(kEntityPayload))) {
      context.reparent_child = *dragged;
      context.reparent_parent = id;
      context.reparent_requested = true;
    }
    ImGui::EndDragDropTarget();
  }

  if (ImGui::BeginPopupContextItem("##context")) {
    context.frame->selection->selected = id;
    if (ImGui::MenuItem("Focus", "F")) FocusSelection(*context.frame);
    if (ImGui::MenuItem("Unparent", nullptr, false,
                        scene.IsAlive(entity->parent))) {
      context.pending_unparent = id;
    }
    ImGui::Separator();
    if (ImGui::MenuItem("Delete", "Del")) context.pending_delete = id;
    ImGui::EndPopup();
  }

  // Right-aligned type tag.
  const char* tag = TypeTag(*entity);
  const float tag_width = ImGui::CalcTextSize(tag).x;
  ImGui::SameLine();
  ImGui::SetCursorPosX(
      std::max(right_edge - tag_width, ImGui::GetCursorPosX()));
  ImGui::TextDisabled("%s", tag);

  if (open) {
    for (EntityId child : entity->children) DrawNode(context, child);
    ImGui::TreePop();
  }
  ImGui::PopID();
}

}  // namespace

void DrawHierarchyPanel(EditorFrame& frame, EditorState& state) {
  if (!ImGui::Begin(kHierarchyWindow)) {
    ImGui::End();
    return;
  }
  Scene& scene = *frame.scene;

  if (ImGui::Button("+ Add")) ImGui::OpenPopup("##spawn");
  if (ImGui::BeginPopup("##spawn")) {
    SpawnMenuItems(frame);
    ImGui::EndPopup();
  }
  ImGui::SameLine();
  ImGui::SetNextItemWidth(-1.0f);
  std::array<char, 128> buffer{};
  state.hierarchy_filter.copy(buffer.data(), buffer.size() - 1);
  if (ImGui::InputTextWithHint("##filter", "Filter by name", buffer.data(),
                               buffer.size())) {
    state.hierarchy_filter = buffer.data();
    state.hierarchy_filter_changed = true;
  }
  ImGui::Separator();

  TreeContext context{.frame = &frame,
                      .filter = state.hierarchy_filter,
                      .pending_delete = kNoEntity,
                      .pending_unparent = kNoEntity,
                      .reparent_child = kNoEntity,
                      .reparent_parent = kNoEntity,
                      .reparent_requested = false,
                      .filter_changed = state.hierarchy_filter_changed};
  state.hierarchy_filter_changed = false;
  ImGui::BeginChild("##tree");
  // Structural edits are deferred until after drawing, so the root list is
  // stable while we iterate it.
  for (EntityId root : scene.roots()) DrawNode(context, root);

  // Dropping onto empty space makes the dragged entity a root.
  ImGui::Dummy(ImVec2(ImGui::GetContentRegionAvail().x,
                      std::max(ImGui::GetContentRegionAvail().y, 24.0f)));
  if (ImGui::BeginDragDropTarget()) {
    if (const std::optional<EntityId> dragged =
            PayloadEntity(ImGui::AcceptDragDropPayload(kEntityPayload))) {
      context.reparent_child = *dragged;
      context.reparent_parent = kNoEntity;
      context.reparent_requested = true;
    }
    ImGui::EndDragDropTarget();
  }
  if (ImGui::IsItemClicked()) frame.selection->selected = kNoEntity;
  ImGui::EndChild();

  ImGui::TextDisabled("%zu entities", scene.size());

  // Apply deferred structural edits now that drawing is done.
  if (context.reparent_requested && scene.IsAlive(context.reparent_child) &&
      context.reparent_child != context.reparent_parent) {
    if (!scene.SetParent(context.reparent_child, context.reparent_parent)) {
      LogWarning("Cannot re-parent: {}",
                 scene.IsAncestorOrSelf(context.reparent_child,
                                        context.reparent_parent)
                     ? "an object cannot become its own descendant"
                     : "the new parent has a zero scale");
    }
  }
  if (scene.IsAlive(context.pending_unparent)) {
    scene.SetParent(context.pending_unparent, kNoEntity);
  }
  if (scene.IsAlive(context.pending_delete)) {
    frame.selection->selected = context.pending_delete;
    DeleteSelection(frame);
  }
  ImGui::End();
}

}  // namespace aerolab::editor
