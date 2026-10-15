#ifndef AEROLAB_SCENE_SCENE_H_
#define AEROLAB_SCENE_SCENE_H_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "math/aabb.h"
#include "math/mat4.h"
#include "math/quat.h"
#include "math/transform.h"
#include "math/vec3.h"
#include "scene/entity.h"

namespace aerolab {

// Owns all entities and their parent/child relationships.
//
// Entities live in a slot array addressed by generational EntityIds, so
// handles stay cheap to copy and never dangle silently. World transforms are
// computed on demand by composing local transforms up the parent chain.
//
// Invariants and limits:
//  * `Entity::parent` and `Entity::children` are maintained by the scene;
//    change them only through Create(), SetParent() and Destroy().
//  * Create() may reallocate storage, invalidating `Entity*` pointers
//    obtained earlier (EntityIds stay valid).
//  * Hierarchy traversal is recursive; it is meant for editor-scale trees
//    (tens of levels), not arbitrarily deep chains.
//  * Transforms are translation-rotation-scale. A rotated child under a
//    non-uniformly scaled parent would need shear, which a Transform cannot
//    hold, so SetParent() then approximates the world transform.
class Scene {
 public:
  Scene() = default;

  // Creates an entity, optionally as a child of `parent` (local transform is
  // then relative to it). An invalid parent creates a root entity.
  EntityId Create(std::string name, EntityId parent = kNoEntity);

  // Destroys the entity and its whole subtree. Returns how many entities
  // were destroyed (0 if `id` was not alive).
  int Destroy(EntityId id);

  void Clear();

  bool IsAlive(EntityId id) const;
  Entity* Get(EntityId id);
  const Entity* Get(EntityId id) const;

  // Re-parents `child` under `new_parent` (kNoEntity makes it a root). The
  // child's world transform is preserved. Fails, leaving the scene
  // unchanged, if either id is dead or the change would create a cycle.
  bool SetParent(EntityId child, EntityId new_parent);

  // True if `ancestor` is `id` itself or any of its parents.
  bool IsAncestorOrSelf(EntityId ancestor, EntityId id) const;

  // Local -> world matrix of the entity (identity for dead ids).
  Mat4 WorldMatrix(EntityId id) const;
  // World matrix of the entity's parent (identity for roots).
  Mat4 ParentWorldMatrix(EntityId id) const;
  // World-space pose decomposed from WorldMatrix().
  Transform WorldTransform(EntityId id) const;

  // Sets the local transform so the entity ends up at the given world pose,
  // keeping its local scale.
  void SetWorldPose(EntityId id, const Vec3& position, const Quat& rotation);

  // World-space AABB of the entity's own mesh, if it has one.
  std::optional<Aabb> WorldBounds(EntityId id) const;

  // Root entities in creation order.
  const std::vector<EntityId>& roots() const { return roots_; }

  // Every live entity, in slot order.
  std::vector<EntityId> AllEntities() const;

  // Calls fn(EntityId, Entity&) for every live entity in slot order without
  // allocating. `fn` must not create or destroy entities.
  template <typename Fn>
  void ForEachEntity(Fn&& fn) {
    for (std::uint32_t i = 0; i < slots_.size(); ++i) {
      if (slots_[i].alive) {
        fn(EntityId{i, slots_[i].generation}, slots_[i].entity);
      }
    }
  }
  template <typename Fn>
  void ForEachEntity(Fn&& fn) const {
    for (std::uint32_t i = 0; i < slots_.size(); ++i) {
      if (slots_[i].alive) {
        fn(EntityId{i, slots_[i].generation},
           static_cast<const Entity&>(slots_[i].entity));
      }
    }
  }

  std::size_t size() const { return alive_count_; }

  // Unique, human-friendly name such as "Dart 3" based on `base`.
  std::string MakeUniqueName(const std::string& base) const;

 private:
  struct Slot {
    std::uint32_t generation = 0;
    bool alive = false;
    Entity entity;
  };

  void DetachFromParent(EntityId id);
  void DestroyRecursive(EntityId id, int& count);

  std::vector<Slot> slots_;
  std::vector<std::uint32_t> free_slots_;
  std::vector<EntityId> roots_;
  std::size_t alive_count_ = 0;
};

}  // namespace aerolab

#endif  // AEROLAB_SCENE_SCENE_H_
