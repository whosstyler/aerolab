#ifndef AEROLAB_WORLD_SPAWNER_H_
#define AEROLAB_WORLD_SPAWNER_H_

#include <array>
#include <string_view>

#include "math/quat.h"
#include "math/vec3.h"
#include "scene/entity.h"
#include "scene/scene.h"

namespace aerolab {

// Ready-made object types that can be spawned into a scene. Each produces a
// root entity (plus child parts for composite objects).
enum class Archetype {
  kCube,
  kSphere,
  kDart,
  kFixedWing,
  kQuadrotor,
  kObserver,  // A standing figure to look through in first person.
  kCount,
};

inline constexpr int kArchetypeCount = static_cast<int>(Archetype::kCount);

std::string_view ArchetypeName(Archetype archetype);

constexpr std::array<Archetype, kArchetypeCount> AllArchetypes() {
  std::array<Archetype, kArchetypeCount> all{};
  for (int i = 0; i < kArchetypeCount; ++i) all[i] = static_cast<Archetype>(i);
  return all;
}

// Spawns `archetype` at a world pose and returns its root entity. Names are
// made unique ("Dart", "Dart 2", ...).
EntityId Spawn(Scene& scene, Archetype archetype, const Vec3& position,
               const Quat& orientation = Quat::Identity());

}  // namespace aerolab

#endif  // AEROLAB_WORLD_SPAWNER_H_
