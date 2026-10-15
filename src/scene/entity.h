#ifndef AEROLAB_SCENE_ENTITY_H_
#define AEROLAB_SCENE_ENTITY_H_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "geometry/material.h"
#include "geometry/mesh_library.h"
#include "math/transform.h"
#include "math/vec3.h"

namespace aerolab {

// Generational handle to a scene entity. A handle to a destroyed entity
// never aliases a newer entity that reuses the same slot.
struct EntityId {
  static constexpr std::uint32_t kInvalidIndex = 0xFFFFFFFFu;

  std::uint32_t index = kInvalidIndex;
  std::uint32_t generation = 0;

  constexpr bool operator==(const EntityId&) const = default;
};

inline constexpr EntityId kNoEntity{};

constexpr bool IsSet(EntityId id) {
  return id.index != EntityId::kInvalidIndex;
}

// Visual representation of an entity.
struct RenderComponent {
  MeshKind mesh = MeshKind::kCube;
  Material material;
  bool visible = true;
  bool cast_shadow = true;
  // Clicks on this entity select its parent instead (e.g. rotor blades).
  bool pick_parent = false;
};

// Constant-rate rotation of the local transform about a local axis; a
// purely kinematic animation used for spinning parts.
struct SpinComponent {
  Vec3 axis = Vec3::UnitY();
  double rate = 0.0;  // rad/s
};

// A node of the scene graph. `local` is relative to the parent (or to the
// world for root entities).
struct Entity {
  std::string name;
  Transform local;
  // Maintained by Scene; use Scene::SetParent() to change.
  EntityId parent;
  std::vector<EntityId> children;

  std::optional<RenderComponent> render;
  std::optional<SpinComponent> spin;
};

}  // namespace aerolab

#endif  // AEROLAB_SCENE_ENTITY_H_
