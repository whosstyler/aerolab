// Editor actions shared by menus, toolbar, panels and shortcuts.

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>

#include "camera/camera.h"
#include "camera/camera_controller.h"
#include "core/log.h"
#include "editor/panels.h"
#include "geometry/mesh_library.h"
#include "imgui.h"
#include "math/aabb.h"
#include "math/quat.h"
#include "math/scalar.h"
#include "math/transform.h"
#include "math/vec3.h"
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

// Placement of first-person observers.
constexpr double kMaxPlaceDistance = 80.0;  // m along the view ray.
constexpr double kPlaceAhead = 10.0;        // m ahead when not looking down.
constexpr double kSpawnAhead = 15.0;  // m in front of an eye-driven camera.

// The spot on the ground the camera is looking at: where the view ray meets
// the ground if that is within reach, else under the orbit pivot (orbit and
// follow) or a few metres ahead.
Vec3 GroundPointInView(const CameraRig& rig) {
  const Camera& camera = rig.camera();
  const Vec3 forward = CameraForward(camera);
  if (forward.y < -0.02 && camera.position.y > 0.0) {
    const double distance = camera.position.y / -forward.y;
    if (distance <= kMaxPlaceDistance) {
      const Vec3 hit = camera.position + forward * distance;
      return {hit.x, 0.0, hit.z};
    }
  }
  if (rig.mode() == CameraMode::kOrbit || rig.mode() == CameraMode::kFollow) {
    return {rig.pivot().x, 0.0, rig.pivot().z};
  }
  const Vec3 flat =
      NormalizedOr(Vec3{forward.x, 0.0, forward.z}, -Vec3::UnitZ());
  const Vec3 ahead = camera.position + flat * kPlaceAhead;
  return {ahead.x, 0.0, ahead.z};
}

// Heading of a rotation's forward (-Z) axis about +Y; 0 faces -Z.
double YawOf(const Quat& rotation) {
  const Vec3 forward = Rotate(rotation, -Vec3::UnitZ());
  if (std::hypot(forward.x, forward.z) < 1e-9) return 0.0;
  return std::atan2(-forward.x, -forward.z);
}

}  // namespace

// Where new objects appear: the orbit pivot, or 15 m in front of the eye
// (level in first person, so looking up or down does not drop things on
// the walker).
Vec3 SpawnPoint(const CameraRig& rig) {
  const Camera& camera = rig.camera();
  const Vec3 forward = CameraForward(camera);
  switch (rig.mode()) {
    case CameraMode::kFreeFly:
      return camera.position + forward * kSpawnAhead;
    case CameraMode::kFirstPerson:
      return camera.position +
             NormalizedOr(Vec3{forward.x, 0.0, forward.z}, -Vec3::UnitZ()) *
                 kSpawnAhead;
    case CameraMode::kOrbit:
    case CameraMode::kFollow:
      break;
  }
  return rig.pivot();
}

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
  Vec3 position = SpawnPoint(*frame.camera);
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
  if (ImGui::MenuItem("Observer (first person)")) SpawnObserver(frame);
  if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
    ImGui::SetTooltip(
        "Places a standing figure on the ground where you are looking and "
        "views the scene through its eyes: WASD walks, right-drag looks.");
  }
  ImGui::Separator();
  for (Archetype archetype : AllArchetypes()) {
    if (archetype == Archetype::kObserver) continue;  // Placed above.
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
  // Focusing on the figure you stand in would only stare at your feet.
  if (frame.selection->selected == frame.hidden_entity) return;
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
  if (frame.viewpoint != nullptr &&
      !frame.scene->IsAlive(frame.viewpoint->observer)) {
    frame.viewpoint->observer = kNoEntity;  // Its figure was deleted.
  }
  LogInfo("Deleted '{}' ({} entit{})", name, removed,
          removed == 1 ? "y" : "ies");
}

bool IsObserver(const Entity& entity) {
  return entity.render.has_value() &&
         entity.render->mesh == MeshKind::kObserver;
}

EntityId SpawnObserver(EditorFrame& frame) {
  const EntityId id = Spawn(
      *frame.scene, Archetype::kObserver, GroundPointInView(*frame.camera),
      FromEuler({.yaw = frame.camera->yaw(), .pitch = 0.0, .roll = 0.0}));
  if (const Entity* entity = frame.scene->Get(id)) {
    LogInfo("Placed '{}'; looking through it (WASD walks, right-drag looks)",
            entity->name);
  }
  LookThrough(frame, id);
  return id;
}

void LookThrough(EditorFrame& frame, EntityId observer) {
  if (!frame.scene->IsAlive(observer)) return;
  // Already looking through it: nothing to do (and no view snap).
  if (frame.camera->mode() == CameraMode::kFirstPerson &&
      frame.viewpoint != nullptr && frame.viewpoint->observer == observer) {
    return;
  }
  const Transform world = frame.scene->WorldTransform(observer);
  frame.camera->EnterFirstPerson(world.position, YawOf(world.rotation));
  if (frame.viewpoint != nullptr) frame.viewpoint->observer = observer;
  // The figure you stand in is hidden, so do not leave it selected.
  if (frame.selection->selected == observer) {
    frame.selection->selected = kNoEntity;
  }
}

void EnterFirstPersonView(EditorFrame& frame) {
  const EntityId selected = frame.selection->selected;
  const Entity* entity = frame.scene->Get(selected);
  const bool observer_selected = entity != nullptr && IsObserver(*entity);
  const EntityId current =
      frame.viewpoint != nullptr &&
              frame.scene->IsAlive(frame.viewpoint->observer)
          ? frame.viewpoint->observer
          : kNoEntity;
  if (frame.camera->mode() == CameraMode::kFirstPerson) {
    // Already walking: only switch to a different, selected observer.
    if (observer_selected && selected != current) {
      LookThrough(frame, selected);
    } else if (!IsSet(current) && frame.viewpoint != nullptr) {
      // The figure was deleted: stand a new one right here.
      frame.viewpoint->observer = Spawn(
          *frame.scene, Archetype::kObserver, frame.camera->feet(),
          FromEuler({.yaw = frame.camera->yaw(), .pitch = 0.0, .roll = 0.0}));
    }
    return;
  }
  if (observer_selected) {
    LookThrough(frame, selected);
  } else if (IsSet(current)) {
    LookThrough(frame, current);
  } else {
    SpawnObserver(frame);
  }
}

}  // namespace aerolab::editor
