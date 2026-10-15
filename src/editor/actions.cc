// Editor actions shared by menus, toolbar, panels and shortcuts.

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>

#include "camera/camera.h"
#include "core/log.h"
#include "editor/panels.h"
#include "imgui.h"
#include "math/aabb.h"
#include "math/quat.h"
#include "math/scalar.h"
#include "world/presets.h"
#include "world/simulation.h"
#include "world/spawner.h"

namespace aerolab::editor {
namespace {

// Launch pitch that makes each preset look sensible when spawned.
double LaunchPitch(std::optional<MotionPreset> preset) {
  if (!preset.has_value()) return 0.0;
  switch (*preset) {
    case MotionPreset::kBallistic:
      return Radians(40.0);
    case MotionPreset::kPowered:
      return Radians(70.0);
    default:
      return 0.0;
  }
}

// Height to spawn at so the motion has room to develop.
double SpawnHeight(std::optional<MotionPreset> preset) {
  if (!preset.has_value()) return 0.5;
  switch (*preset) {
    case MotionPreset::kBallistic:
    case MotionPreset::kPowered:
      return 1.0;
    case MotionPreset::kGlider:
      return 30.0;
    default:
      return 12.0;
  }
}

}  // namespace

float Dpi() { return ImGui::GetStyle().FontScaleDpi; }

void ResetSimulation(EditorFrame& frame) {
  frame.simulation->Reset(*frame.scene);
  frame.event_log->BeginRun();
}

EntityId SpawnObject(EditorFrame& frame, Archetype archetype,
                     std::optional<MotionPreset> preset) {
  const Camera& camera = frame.camera->camera();
  const Vec3 forward = CameraForward(camera);
  const Vec3 flat =
      NormalizedOr(Vec3{forward.x, 0.0, forward.z}, -Vec3::UnitZ());
  Vec3 position = frame.camera->mode() == CameraMode::kFreeFly
                      ? camera.position + forward * 15.0
                      : frame.camera->pivot();
  position.y = SpawnHeight(preset);
  // Face away from the camera so launches travel into the scene.
  const double yaw = std::atan2(-flat.x, -flat.z);
  const Quat orientation =
      FromEuler({.yaw = yaw, .pitch = LaunchPitch(preset), .roll = 0.0});
  const EntityId id = Spawn(*frame.scene, archetype, position, orientation);
  if (preset.has_value()) ApplyMotionPreset(*frame.scene, id, *preset);
  frame.selection->selected = id;
  if (const Entity* entity = frame.scene->Get(id)) {
    LogInfo("Spawned '{}'{}", entity->name,
            preset.has_value()
                ? " (" + std::string(MotionPresetName(*preset)) + ")"
                : std::string());
  }
  return id;
}

void SpawnMenuItems(EditorFrame& frame) {
  for (Archetype archetype : AllArchetypes()) {
    const std::string name(ArchetypeName(archetype));
    if (!ImGui::BeginMenu(name.c_str())) continue;
    if (ImGui::MenuItem("Static")) {
      SpawnObject(frame, archetype, std::nullopt);
    }
    ImGui::Separator();
    for (MotionPreset preset : AllMotionPresets()) {
      const std::string label(MotionPresetName(preset));
      if (ImGui::MenuItem(label.c_str())) {
        SpawnObject(frame, archetype, preset);
      }
      if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
        const std::string description(MotionPresetDescription(preset));
        ImGui::SetTooltip("%s", description.c_str());
      }
    }
    ImGui::EndMenu();
  }
}

void FocusSelection(EditorFrame& frame) {
  const std::optional<Aabb> bounds = RenderWorldBounds(
      *frame.scene, frame.selection->selected, frame.simulation->alpha());
  if (bounds.has_value()) {
    frame.camera->Focus(Center(*bounds), BoundingRadius(*bounds));
  }
}

void FrameWholeScene(EditorFrame& frame) {
  Aabb all;
  const double alpha = frame.simulation->alpha();
  frame.scene->ForEachEntity([&](EntityId id, const Entity&) {
    if (const std::optional<Aabb> bounds =
            RenderWorldBounds(*frame.scene, id, alpha)) {
      all = Union(all, *bounds);
    }
  });
  if (!IsEmpty(all)) frame.camera->Focus(Center(all), BoundingRadius(all));
}

void DeleteSelection(EditorFrame& frame) {
  const EntityId id = frame.selection->selected;
  const Entity* entity = frame.scene->Get(id);
  if (entity == nullptr) return;
  const std::string name = entity->name;
  const int removed = frame.scene->Destroy(id);
  frame.selection->selected = kNoEntity;
  frame.selection->hovered = kNoEntity;
  LogInfo("Deleted '{}' ({} entit{})", name, removed,
          removed == 1 ? "y" : "ies");
}

}  // namespace aerolab::editor
