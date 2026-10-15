#ifndef AEROLAB_GEOMETRY_MESH_LIBRARY_H_
#define AEROLAB_GEOMETRY_MESH_LIBRARY_H_

#include <array>
#include <string_view>

#include "geometry/mesh_data.h"
#include "math/aabb.h"

namespace aerolab {

// Built-in procedural meshes. Scene objects reference meshes by kind, so the
// scene never depends on the renderer; the renderer uploads one GPU mesh per
// kind.
//
// Vehicle-like models face local -Z (forward) with +Y up and are generic
// shapes chosen for readability in the viewport.
enum class MeshKind {
  kCube,
  kSphere,
  kCylinder,
  kCone,
  kDart,       // Finned slender body, ~2 m long.
  kFixedWing,  // Small fixed-wing airframe, ~2.4 m wingspan.
  kQuadrotor,  // Multirotor frame (rotors are separate child objects).
  kRotor,      // Two-blade rotor disc, spins about local +Y.
  kObserver,   // Human-scale figure, ~1.7 m tall, feet at the origin.
  kCount,
};

inline constexpr int kMeshKindCount = static_cast<int>(MeshKind::kCount);

std::string_view MeshKindName(MeshKind kind);

// Builds the local-space mesh for `kind`. Deterministic: the same kind always
// yields identical data.
MeshData BuildMesh(MeshKind kind);

// Local-space bounds of BuildMesh(kind), computed once and cached. Used for
// picking, selection boxes and camera framing without touching the GPU.
const Aabb& MeshLocalBounds(MeshKind kind);

// All kinds, for iteration in UIs and renderers.
constexpr std::array<MeshKind, kMeshKindCount> AllMeshKinds() {
  std::array<MeshKind, kMeshKindCount> kinds{};
  for (int i = 0; i < kMeshKindCount; ++i) kinds[i] = static_cast<MeshKind>(i);
  return kinds;
}

}  // namespace aerolab

#endif  // AEROLAB_GEOMETRY_MESH_LIBRARY_H_
