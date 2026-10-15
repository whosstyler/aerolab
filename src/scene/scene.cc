#include "scene/scene.h"

#include <algorithm>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "geometry/mesh_library.h"
#include "math/intersection.h"

namespace aerolab {
namespace {

void EraseId(std::vector<EntityId>& ids, EntityId id) {
  ids.erase(std::remove(ids.begin(), ids.end(), id), ids.end());
}

}  // namespace

EntityId Scene::Create(std::string name, EntityId parent) {
  std::uint32_t index;
  if (!free_slots_.empty()) {
    index = free_slots_.back();
    free_slots_.pop_back();
  } else {
    index = static_cast<std::uint32_t>(slots_.size());
    slots_.emplace_back();
  }
  Slot& slot = slots_[index];
  slot.alive = true;
  slot.entity = Entity();
  slot.entity.name = std::move(name);
  const EntityId id{index, slot.generation};
  ++alive_count_;

  if (IsAlive(parent)) {
    slot.entity.parent = parent;
    Get(parent)->children.push_back(id);
  } else {
    roots_.push_back(id);
  }
  return id;
}

bool Scene::IsAlive(EntityId id) const {
  return id.index < slots_.size() && slots_[id.index].alive &&
         slots_[id.index].generation == id.generation;
}

Entity* Scene::Get(EntityId id) {
  return IsAlive(id) ? &slots_[id.index].entity : nullptr;
}

const Entity* Scene::Get(EntityId id) const {
  return IsAlive(id) ? &slots_[id.index].entity : nullptr;
}

void Scene::DetachFromParent(EntityId id) {
  Entity* entity = Get(id);
  if (entity == nullptr) return;
  if (Entity* parent = Get(entity->parent)) {
    EraseId(parent->children, id);
  } else {
    EraseId(roots_, id);
  }
  entity->parent = kNoEntity;
}

void Scene::DestroyRecursive(EntityId id, int& count) {
  Entity* entity = Get(id);
  if (entity == nullptr) return;
  // Copy: destroying children edits this entity's child list.
  const std::vector<EntityId> children = entity->children;
  for (EntityId child : children) DestroyRecursive(child, count);
  DetachFromParent(id);
  Slot& slot = slots_[id.index];
  slot.alive = false;
  slot.entity = Entity();
  ++slot.generation;  // Invalidates every outstanding handle.
  free_slots_.push_back(id.index);
  --alive_count_;
  ++count;
}

int Scene::Destroy(EntityId id) {
  int count = 0;
  DestroyRecursive(id, count);
  return count;
}

void Scene::Clear() {
  const std::vector<EntityId> roots = roots_;
  for (EntityId root : roots) Destroy(root);
}

bool Scene::IsAncestorOrSelf(EntityId ancestor, EntityId id) const {
  for (EntityId current = id; IsAlive(current);
       current = Get(current)->parent) {
    if (current == ancestor) return true;
  }
  return false;
}

bool Scene::SetParent(EntityId child, EntityId new_parent) {
  if (!IsAlive(child)) return false;
  if (IsSet(new_parent) && !IsAlive(new_parent)) return false;
  if (IsAlive(new_parent) && IsAncestorOrSelf(child, new_parent)) return false;

  // Validate before touching the hierarchy: a singular parent (zero scale)
  // cannot express the child's current world transform.
  const Mat4 world = WorldMatrix(child);
  const std::optional<Mat4> parent_inverse = AffineInverse(
      IsAlive(new_parent) ? WorldMatrix(new_parent) : Mat4::Identity());
  if (!parent_inverse.has_value()) return false;

  DetachFromParent(child);
  Entity* entity = Get(child);
  if (IsAlive(new_parent)) {
    entity->parent = new_parent;
    Get(new_parent)->children.push_back(child);
  } else {
    roots_.push_back(child);
  }
  // Re-express the unchanged world transform relative to the new parent.
  entity->local = FromMatrix(*parent_inverse * world);
  return true;
}

Mat4 Scene::WorldMatrix(EntityId id) const {
  const Entity* entity = Get(id);
  if (entity == nullptr) return Mat4::Identity();
  return ParentWorldMatrix(id) * ToMatrix(entity->local);
}

Mat4 Scene::ParentWorldMatrix(EntityId id) const {
  const Entity* entity = Get(id);
  if (entity == nullptr || !IsAlive(entity->parent)) return Mat4::Identity();
  return WorldMatrix(entity->parent);
}

Transform Scene::WorldTransform(EntityId id) const {
  return FromMatrix(WorldMatrix(id));
}

void Scene::SetWorldPose(EntityId id, const Vec3& position,
                         const Quat& rotation) {
  Entity* entity = Get(id);
  if (entity == nullptr) return;
  const Mat4 parent_world = ParentWorldMatrix(id);
  const std::optional<Mat4> parent_inverse = AffineInverse(parent_world);
  if (!parent_inverse.has_value()) return;
  const Transform parent = FromMatrix(parent_world);
  entity->local.position = TransformPoint(*parent_inverse, position);
  entity->local.rotation = Normalized(Conjugate(parent.rotation) * rotation);
}

std::optional<Aabb> Scene::WorldBounds(EntityId id) const {
  const Entity* entity = Get(id);
  if (entity == nullptr || !entity->render.has_value()) return std::nullopt;
  return TransformAabb(MeshLocalBounds(entity->render->mesh), WorldMatrix(id));
}

std::vector<EntityId> Scene::AllEntities() const {
  std::vector<EntityId> ids;
  ids.reserve(alive_count_);
  for (std::uint32_t i = 0; i < slots_.size(); ++i) {
    if (slots_[i].alive) ids.push_back({i, slots_[i].generation});
  }
  return ids;
}

std::string Scene::MakeUniqueName(const std::string& base) const {
  const auto taken = [this](const std::string& name) {
    for (const Slot& slot : slots_) {
      if (slot.alive && slot.entity.name == name) return true;
    }
    return false;
  };
  if (!taken(base)) return base;
  for (int n = 2;; ++n) {
    std::string candidate = base + " " + std::to_string(n);
    if (!taken(candidate)) return candidate;
  }
}

}  // namespace aerolab
