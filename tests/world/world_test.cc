#include <gtest/gtest.h>

#include <optional>

#include "math/ray.h"
#include "scene/scene.h"
#include "test_util.h"
#include "world/animation.h"
#include "world/picking.h"
#include "world/spawner.h"

namespace aerolab {
namespace {

TEST(SpawnerTest, EveryArchetypeCreatesRenderableRoot) {
  Scene scene;
  for (Archetype archetype : AllArchetypes()) {
    const EntityId id = Spawn(scene, archetype, {1, 2, 3});
    const Entity* entity = scene.Get(id);
    ASSERT_NE(entity, nullptr) << ArchetypeName(archetype);
    EXPECT_TRUE(entity->render.has_value());
    EXPECT_EQ(entity->parent, kNoEntity);
    EXPECT_VEC3_NEAR(entity->local.position, (Vec3{1, 2, 3}), 1e-12);
  }
  EXPECT_EQ(scene.roots().size(), static_cast<std::size_t>(kArchetypeCount));
}

TEST(SpawnerTest, QuadrotorHasSpinningRotorChildren) {
  Scene scene;
  const EntityId quad = Spawn(scene, Archetype::kQuadrotor, Vec3::Zero());
  const Entity* entity = scene.Get(quad);
  ASSERT_EQ(entity->children.size(), 4u);
  double total_spin = 0.0;
  for (EntityId rotor : entity->children) {
    const Entity* part = scene.Get(rotor);
    ASSERT_TRUE(part->spin.has_value());
    ASSERT_TRUE(part->render.has_value());
    EXPECT_TRUE(part->render->pick_parent);
    total_spin += part->spin->rate;
  }
  // Counter-rotating pairs cancel.
  EXPECT_NEAR(total_spin, 0.0, 1e-12);
}

TEST(SpawnerTest, NamesAreUnique) {
  Scene scene;
  const EntityId a = Spawn(scene, Archetype::kDart, Vec3::Zero());
  const EntityId b = Spawn(scene, Archetype::kDart, Vec3::Zero());
  EXPECT_EQ(scene.Get(a)->name, "Dart");
  EXPECT_EQ(scene.Get(b)->name, "Dart 2");
}

TEST(PickingTest, PicksNearestHit) {
  Scene scene;
  const EntityId near_cube = Spawn(scene, Archetype::kCube, {0, 0, -5});
  Spawn(scene, Archetype::kCube, {0, 0, -10});
  const std::optional<PickHit> hit =
      PickEntity(scene, Ray{Vec3::Zero(), -Vec3::UnitZ()});
  ASSERT_TRUE(hit.has_value());
  EXPECT_EQ(hit->entity, near_cube);
  EXPECT_NEAR(hit->distance, 4.5, 1e-9);
  EXPECT_VEC3_NEAR(hit->point, (Vec3{0, 0, -4.5}), 1e-9);
}

TEST(PickingTest, MissesAndInvisibleObjects) {
  Scene scene;
  const EntityId cube = Spawn(scene, Archetype::kCube, {0, 0, -5});
  EXPECT_FALSE(PickEntity(scene, Ray{Vec3::Zero(), Vec3::UnitX()}));
  scene.Get(cube)->render->visible = false;
  EXPECT_FALSE(PickEntity(scene, Ray{Vec3::Zero(), -Vec3::UnitZ()}));
}

TEST(PickingTest, RayStartingInsideAnObjectHitsIt) {
  Scene scene;
  const EntityId cube = Spawn(scene, Archetype::kCube, Vec3::Zero());
  const std::optional<PickHit> hit =
      PickEntity(scene, Ray{Vec3::Zero(), Vec3::UnitX()});
  ASSERT_TRUE(hit.has_value());
  EXPECT_EQ(hit->entity, cube);
  EXPECT_EQ(hit->distance, 0.0);
}

TEST(AnimationTest, SpinnersRotateAboutTheirLocalAxis) {
  Scene scene;
  const EntityId quad = Spawn(scene, Archetype::kQuadrotor, Vec3::Zero());
  const EntityId rotor = scene.Get(quad)->children[0];
  const double rate = scene.Get(rotor)->spin->rate;
  AdvanceSpinners(scene, 0.01);
  EXPECT_SAME_ROTATION(scene.Get(rotor)->local.rotation,
                       FromAxisAngle(Vec3::UnitY(), rate * 0.01), 1e-12);
  // The vehicle frame itself has no spin component and stays put.
  EXPECT_SAME_ROTATION(scene.Get(quad)->local.rotation, Quat::Identity(),
                       1e-15);
}

TEST(AnimationTest, SpinInterpolatesAndResets) {
  Scene scene;
  const EntityId quad = Spawn(scene, Archetype::kQuadrotor, Vec3::Zero());
  const EntityId rotor = scene.Get(quad)->children[0];
  const SpinComponent& spin = *scene.Get(rotor)->spin;
  AdvanceSpinners(scene, 0.01);
  EXPECT_SAME_ROTATION(InterpolatedSpinRotation(spin, 0.5),
                       FromAxisAngle(Vec3::UnitY(), spin.rate * 0.005), 1e-12);
  ResetSpinners(scene);
  EXPECT_EQ(scene.Get(rotor)->spin->angle, 0.0);
  EXPECT_SAME_ROTATION(scene.Get(rotor)->local.rotation, Quat::Identity(),
                       1e-15);
}

TEST(PickingTest, RotorHitsSelectTheVehicle) {
  Scene scene;
  const EntityId quad = Spawn(scene, Archetype::kQuadrotor, {0, 0, 0});
  // Aim straight down at the front-right rotor hub, from above.
  const Entity* rotor = scene.Get(scene.Get(quad)->children[0]);
  const Vec3 hub = rotor->local.position;
  const std::optional<PickHit> hit =
      PickEntity(scene, Ray{hub + Vec3{0, 5, 0}, -Vec3::UnitY()});
  ASSERT_TRUE(hit.has_value());
  EXPECT_EQ(hit->entity, quad);
}

}  // namespace
}  // namespace aerolab
