#include "scene/scene.h"

#include <gtest/gtest.h>

#include "math/quat.h"
#include "test_util.h"

namespace aerolab {
namespace {

TEST(SceneTest, CreateAndDestroyWithGenerationalHandles) {
  Scene scene;
  const EntityId a = scene.Create("A");
  EXPECT_TRUE(scene.IsAlive(a));
  EXPECT_EQ(scene.size(), 1u);
  EXPECT_EQ(scene.Get(a)->name, "A");
  EXPECT_EQ(scene.Destroy(a), 1);
  EXPECT_FALSE(scene.IsAlive(a));
  EXPECT_EQ(scene.Get(a), nullptr);

  // The slot is reused, but the stale handle must not see the new entity.
  const EntityId b = scene.Create("B");
  EXPECT_EQ(b.index, a.index);
  EXPECT_NE(b.generation, a.generation);
  EXPECT_FALSE(scene.IsAlive(a));
  EXPECT_EQ(scene.Destroy(a), 0);
  EXPECT_TRUE(scene.IsAlive(b));
}

TEST(SceneTest, InvalidHandlesAreSafe) {
  Scene scene;
  EXPECT_FALSE(scene.IsAlive(kNoEntity));
  EXPECT_EQ(scene.Get(kNoEntity), nullptr);
  EXPECT_TRUE(NearlyEqual(scene.WorldMatrix(kNoEntity), Mat4::Identity()));
  EXPECT_FALSE(scene.WorldBounds(kNoEntity).has_value());
  EXPECT_FALSE(scene.SetParent(kNoEntity, kNoEntity));
}

TEST(SceneTest, DestroyRemovesWholeSubtree) {
  Scene scene;
  const EntityId root = scene.Create("Root");
  const EntityId child = scene.Create("Child", root);
  const EntityId grandchild = scene.Create("Grandchild", child);
  const EntityId other = scene.Create("Other");
  EXPECT_EQ(scene.Destroy(child), 2);
  EXPECT_FALSE(scene.IsAlive(grandchild));
  EXPECT_TRUE(scene.Get(root)->children.empty());
  EXPECT_EQ(scene.size(), 2u);
  scene.Clear();
  EXPECT_EQ(scene.size(), 0u);
  EXPECT_FALSE(scene.IsAlive(other));
  EXPECT_TRUE(scene.roots().empty());
}

TEST(SceneTest, WorldMatrixComposesLocalTransforms) {
  Scene scene;
  const EntityId parent = scene.Create("Parent");
  scene.Get(parent)->local.position = {10, 0, 0};
  scene.Get(parent)->local.rotation = FromAxisAngle(Vec3::UnitY(), kHalfPi);
  const EntityId child = scene.Create("Child", parent);
  scene.Get(child)->local.position = {0, 0, -2};  // In front of the parent.
  EXPECT_VEC3_NEAR(scene.WorldTransform(child).position, (Vec3{8, 0, 0}),
                   1e-12);
}

TEST(SceneTest, ReparentingPreservesWorldTransform) {
  Scene scene;
  const EntityId a = scene.Create("A");
  scene.Get(a)->local.position = {5, 1, 0};
  scene.Get(a)->local.rotation = FromAxisAngle(Vec3::UnitZ(), 0.7);
  const EntityId b = scene.Create("B");
  scene.Get(b)->local.position = {-2, 3, 4};
  scene.Get(b)->local.rotation = FromAxisAngle(Vec3::UnitX(), -0.4);

  const Transform before = scene.WorldTransform(b);
  ASSERT_TRUE(scene.SetParent(b, a));
  EXPECT_EQ(scene.Get(b)->parent, a);
  EXPECT_EQ(scene.roots().size(), 1u);
  EXPECT_TRUE(NearlyEqual(scene.WorldTransform(b), before, 1e-9));

  ASSERT_TRUE(scene.SetParent(b, kNoEntity));
  EXPECT_EQ(scene.roots().size(), 2u);
  EXPECT_TRUE(NearlyEqual(scene.WorldTransform(b), before, 1e-9));
}

TEST(SceneTest, ReparentingRejectsCycles) {
  Scene scene;
  const EntityId a = scene.Create("A");
  const EntityId b = scene.Create("B", a);
  const EntityId c = scene.Create("C", b);
  EXPECT_FALSE(scene.SetParent(a, c));
  EXPECT_FALSE(scene.SetParent(a, a));
  EXPECT_EQ(scene.Get(a)->parent, kNoEntity);
  EXPECT_TRUE(scene.IsAncestorOrSelf(a, c));
  EXPECT_FALSE(scene.IsAncestorOrSelf(c, a));
}

TEST(SceneTest, SetWorldPoseUnderRotatedParent) {
  Scene scene;
  const EntityId parent = scene.Create("Parent");
  scene.Get(parent)->local.position = {1, 2, 3};
  scene.Get(parent)->local.rotation = FromAxisAngle(Vec3{1, 1, 0}, 0.9);
  const EntityId child = scene.Create("Child", parent);
  const Quat target_rotation = FromAxisAngle(Vec3::UnitY(), -0.3);
  scene.SetWorldPose(child, {7, -1, 2}, target_rotation);
  const Transform world = scene.WorldTransform(child);
  EXPECT_VEC3_NEAR(world.position, (Vec3{7, -1, 2}), 1e-9);
  EXPECT_SAME_ROTATION(world.rotation, target_rotation, 1e-9);
}

TEST(SceneTest, WorldBoundsUseMeshAndTransform) {
  Scene scene;
  const EntityId cube = scene.Create("Cube");
  scene.Get(cube)->render = RenderComponent{};  // Unit cube mesh.
  scene.Get(cube)->local.position = {0, 5, 0};
  const std::optional<Aabb> bounds = scene.WorldBounds(cube);
  ASSERT_TRUE(bounds.has_value());
  EXPECT_VEC3_NEAR(bounds->min, (Vec3{-0.5, 4.5, -0.5}), 1e-6);
  const EntityId empty = scene.Create("Empty");
  EXPECT_FALSE(scene.WorldBounds(empty).has_value());
}

TEST(SceneTest, ReparentUnderSingularParentFails) {
  Scene scene;
  const EntityId flat = scene.Create("Flat");
  scene.Get(flat)->local.scale = {1.0, 0.0, 1.0};
  const EntityId child = scene.Create("Child");
  scene.Get(child)->local.position = {1, 2, 3};
  EXPECT_FALSE(scene.SetParent(child, flat));
  EXPECT_EQ(scene.Get(child)->parent, kNoEntity);
  EXPECT_VEC3_NEAR(scene.Get(child)->local.position, (Vec3{1, 2, 3}), 1e-12);
}

TEST(SceneTest, ReparentFailureModesLeaveSceneUnchanged) {
  Scene scene;
  const EntityId a = scene.Create("A");
  const EntityId b = scene.Create("B");
  EXPECT_FALSE(scene.SetParent(a, a));  // Self.
  const EntityId dead = scene.Create("Dead");
  scene.Destroy(dead);
  EXPECT_FALSE(scene.SetParent(dead, a));  // Dead child.
  EXPECT_FALSE(scene.SetParent(a, dead));  // Dead parent.
  EXPECT_EQ(scene.roots().size(), 2u);
  EXPECT_TRUE(scene.SetParent(b, a));
  EXPECT_FALSE(scene.SetParent(a, b));  // Cycle.
  EXPECT_EQ(scene.Get(a)->parent, kNoEntity);
}

TEST(SceneTest, CreateUnderDeadParentMakesRoot) {
  Scene scene;
  const EntityId gone = scene.Create("Gone");
  scene.Destroy(gone);
  const EntityId orphan = scene.Create("Orphan", gone);
  EXPECT_EQ(scene.Get(orphan)->parent, kNoEntity);
  EXPECT_EQ(scene.roots().size(), 1u);
  EXPECT_FALSE(scene.SetParent(orphan, gone));
}

TEST(SceneTest, UniqueNames) {
  Scene scene;
  EXPECT_EQ(scene.MakeUniqueName("Dart"), "Dart");
  scene.Create("Dart");
  EXPECT_EQ(scene.MakeUniqueName("Dart"), "Dart 2");
  scene.Create("Dart 2");
  EXPECT_EQ(scene.MakeUniqueName("Dart"), "Dart 3");
}

TEST(SceneTest, AllEntitiesListsOnlyLiveOnes) {
  Scene scene;
  const EntityId a = scene.Create("A");
  const EntityId b = scene.Create("B");
  scene.Destroy(a);
  const std::vector<EntityId> all = scene.AllEntities();
  ASSERT_EQ(all.size(), 1u);
  EXPECT_EQ(all[0], b);
}

}  // namespace
}  // namespace aerolab
