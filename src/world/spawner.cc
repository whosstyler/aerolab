#include "world/spawner.h"

#include <array>
#include <cstddef>
#include <string>
#include <string_view>

#include "geometry/color.h"
#include "geometry/mesh_library.h"
#include "math/scalar.h"

namespace aerolab {
namespace {

RenderComponent MakeRender(MeshKind mesh, const Color& color) {
  RenderComponent render;
  render.mesh = mesh;
  render.material.base_color = color;
  return render;
}

EntityId CreateRoot(Scene& scene, std::string_view base_name,
                    const Vec3& position, const Quat& orientation) {
  const EntityId id =
      scene.Create(scene.MakeUniqueName(std::string(base_name)));
  Entity* entity = scene.Get(id);
  entity->local.position = position;
  entity->local.rotation = Normalized(orientation);
  return id;
}

void AddRotors(Scene& scene, EntityId frame) {
  // Rotor hubs sit above the motors at the ends of the "X" arms.
  constexpr double kArm = 0.36 * 0.70710678118654752;
  const std::array<Vec3, 4> offsets = {
      Vec3{kArm, 0.065, -kArm}, Vec3{-kArm, 0.065, -kArm},
      Vec3{-kArm, 0.065, kArm}, Vec3{kArm, 0.065, kArm}};
  const std::array<const char*, 4> names = {"Rotor FR", "Rotor FL", "Rotor RL",
                                            "Rotor RR"};
  for (std::size_t i = 0; i < offsets.size(); ++i) {
    const EntityId rotor = scene.Create(names[i], frame);
    Entity* entity = scene.Get(rotor);
    entity->local.position = offsets[i];
    RenderComponent render = MakeRender(MeshKind::kRotor, colors::kWhite);
    render.cast_shadow = false;
    render.pick_parent = true;
    entity->render = render;
    // Diagonal rotors counter-rotate, as on a real multirotor.
    SpinComponent spin;
    spin.axis = Vec3::UnitY();
    spin.rate = (i % 2 == 0 ? 1.0 : -1.0) * kTwoPi * 6.0;
    spin.rest_rotation = entity->local.rotation;
    entity->spin = spin;
  }
}

}  // namespace

std::string_view ArchetypeName(Archetype archetype) {
  switch (archetype) {
    case Archetype::kCube:
      return "Cube";
    case Archetype::kSphere:
      return "Sphere";
    case Archetype::kDart:
      return "Dart";
    case Archetype::kFixedWing:
      return "Fixed wing";
    case Archetype::kQuadrotor:
      return "Quadrotor";
    case Archetype::kObserver:
      return "Observer";
    case Archetype::kCount:
      break;
  }
  return "Object";
}

EntityId Spawn(Scene& scene, Archetype archetype, const Vec3& position,
               const Quat& orientation) {
  const EntityId id =
      CreateRoot(scene, ArchetypeName(archetype), position, orientation);
  Entity* entity = scene.Get(id);
  switch (archetype) {
    case Archetype::kCube:
      entity->render = MakeRender(MeshKind::kCube, FromHex(0x5B8DEF));
      break;
    case Archetype::kSphere:
      entity->render = MakeRender(MeshKind::kSphere, FromHex(0xE5674D));
      break;
    case Archetype::kDart:
      entity->render = MakeRender(MeshKind::kDart, colors::kWhite);
      break;
    case Archetype::kFixedWing:
      entity->render = MakeRender(MeshKind::kFixedWing, colors::kWhite);
      break;
    case Archetype::kQuadrotor:
      entity->render = MakeRender(MeshKind::kQuadrotor, colors::kWhite);
      AddRotors(scene, id);
      break;
    case Archetype::kObserver:
      entity->render = MakeRender(MeshKind::kObserver, colors::kWhite);
      break;
    case Archetype::kCount:
      break;
  }
  return id;
}

}  // namespace aerolab
