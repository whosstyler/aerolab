#include "math/intersection.h"

#include <gtest/gtest.h>

#include <optional>

#include "math/aabb.h"
#include "math/mat4.h"
#include "math/quat.h"
#include "math/ray.h"
#include "math/vec3.h"
#include "test_util.h"

namespace aerolab {
namespace {

const Aabb kUnitBox = MakeAabb({-1, -1, -1}, {1, 1, 1});

TEST(AabbTest, EmptyBoxExpandsToPoint) {
  Aabb box;
  EXPECT_TRUE(IsEmpty(box));
  box = Expanded(box, {1, 2, 3});
  EXPECT_FALSE(IsEmpty(box));
  EXPECT_EQ(box.min, (Vec3{1, 2, 3}));
  EXPECT_EQ(box.max, (Vec3{1, 2, 3}));
  box = Expanded(box, {-1, 0, 5});
  EXPECT_EQ(Center(box), (Vec3{0, 1, 4}));
  EXPECT_EQ(HalfExtents(box), (Vec3{1, 1, 1}));
  EXPECT_TRUE(Contains(box, {0, 1, 4}));
  EXPECT_FALSE(Contains(box, {0, 3, 4}));
}

TEST(AabbTest, UnionIgnoresEmptyBoxes) {
  const Aabb a = MakeAabb({0, 0, 0}, {1, 1, 1});
  EXPECT_EQ(Union(a, Aabb{}).max, a.max);
  EXPECT_EQ(Union(Aabb{}, a).min, a.min);
  const Aabb b = MakeAabbFromCenter({5, 0, 0}, {1, 1, 1});
  EXPECT_EQ(Union(a, b).max, (Vec3{6, 1, 1}));
  EXPECT_NEAR(BoundingRadius(kUnitBox), std::sqrt(3.0), 1e-12);
}

TEST(IntersectionTest, RayHitsBoxFrontFace) {
  const Ray ray{{0, 0, 5}, {0, 0, -1}};
  const std::optional<double> t = IntersectRayAabb(ray, kUnitBox);
  ASSERT_TRUE(t.has_value());
  EXPECT_NEAR(*t, 4.0, 1e-12);
}

TEST(IntersectionTest, RayMissesBoxAndIgnoresBoxBehind) {
  EXPECT_FALSE(IntersectRayAabb({{0, 3, 5}, {0, 0, -1}}, kUnitBox).has_value());
  EXPECT_FALSE(IntersectRayAabb({{0, 0, 5}, {0, 0, 1}}, kUnitBox).has_value());
}

TEST(IntersectionTest, RayAxisParallelHandling) {
  // Direction has zero components; origin inside the x/y slabs.
  EXPECT_TRUE(IntersectRayAabb({{0.5, 0.5, 5}, {0, 0, -1}}, kUnitBox));
  // Origin outside a slab with zero direction along that axis misses.
  EXPECT_FALSE(IntersectRayAabb({{1.5, 0.5, 5}, {0, 0, -1}}, kUnitBox));
}

TEST(IntersectionTest, RayStartingInsideBoxReportsZero) {
  const std::optional<double> t =
      IntersectRayAabb({{0, 0, 0}, Normalized(Vec3{1, 1, 0})}, kUnitBox);
  ASSERT_TRUE(t.has_value());
  EXPECT_EQ(*t, 0.0);
}

TEST(IntersectionTest, RayGrazingAnEdgeCountsAsHit) {
  // Travels exactly along the top-front edge line of the box.
  const std::optional<double> t =
      IntersectRayAabb({{-5, 1, 1}, {1, 0, 0}}, kUnitBox);
  ASSERT_TRUE(t.has_value());
  EXPECT_NEAR(*t, 4.0, 1e-12);
}

TEST(IntersectionTest, RayOriginOnFacePointingInwardReportsZero) {
  const std::optional<double> t =
      IntersectRayAabb({{0, 0, 1}, {0, 0, -1}}, kUnitBox);
  ASSERT_TRUE(t.has_value());
  EXPECT_EQ(*t, 0.0);
}

TEST(IntersectionTest, UnnormalizedDirectionScalesParameter) {
  const std::optional<double> t =
      IntersectRayAabb({{0, 0, 5}, {0, 0, -2}}, kUnitBox);
  ASSERT_TRUE(t.has_value());
  EXPECT_NEAR(*t, 2.0, 1e-12);  // Hit point is origin + 2 * (0, 0, -2).
}

TEST(IntersectionTest, RaySphere) {
  const std::optional<double> hit =
      IntersectRaySphere({{-10, 0, 0}, {1, 0, 0}}, {0, 0, 0}, 2.0);
  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(*hit, 8.0, 1e-12);
  EXPECT_FALSE(IntersectRaySphere({{-10, 3, 0}, {1, 0, 0}}, {0, 0, 0}, 2.0));
  EXPECT_FALSE(IntersectRaySphere({{10, 0, 0}, {1, 0, 0}}, {0, 0, 0}, 2.0));
  EXPECT_EQ(*IntersectRaySphere({{0.5, 0, 0}, {1, 0, 0}}, {0, 0, 0}, 2.0), 0.0);
}

TEST(IntersectionTest, RayPlane) {
  const Ray down{{0, 10, 0}, Normalized(Vec3{1, -1, 0})};
  const std::optional<double> t =
      IntersectRayPlane(down, Vec3::Zero(), Vec3::UnitY());
  ASSERT_TRUE(t.has_value());
  EXPECT_VEC3_NEAR(PointAt(down, *t), (Vec3{10, 0, 0}), 1e-9);
  EXPECT_FALSE(
      IntersectRayPlane({{0, 10, 0}, {1, 0, 0}}, Vec3::Zero(), Vec3::UnitY()));
  EXPECT_FALSE(
      IntersectRayPlane({{0, 10, 0}, {0, 1, 0}}, Vec3::Zero(), Vec3::UnitY()));
}

TEST(IntersectionTest, OrientedBoxUsesWorldDistance) {
  // A box rotated 45 degrees and stretched along X, centred at (10, 0, 0).
  const Mat4 local_to_world = Mat4::Trs(
      {10, 0, 0}, FromAxisAngle(Vec3::UnitY(), kPi / 4.0), {3.0, 1.0, 1.0});
  const Ray ray{{10, 0, 20}, {0, 0, -1}};
  const std::optional<double> t =
      IntersectRayOrientedBox(ray, kUnitBox, local_to_world);
  ASSERT_TRUE(t.has_value());
  // Cross-check against the hit point lying on the box surface in local space.
  const Vec3 hit_local =
      TransformPoint(*AffineInverse(local_to_world), PointAt(ray, *t));
  EXPECT_NEAR(MaxComponent(Abs(hit_local)), 1.0, 1e-9);
  EXPECT_FALSE(IntersectRayOrientedBox({{30, 0, 20}, {0, 0, -1}}, kUnitBox,
                                       local_to_world));
}

TEST(IntersectionTest, OrientedBoxWithSingularTransformMisses) {
  const Mat4 flattened = Mat4::Scale({1.0, 0.0, 1.0});
  EXPECT_FALSE(
      IntersectRayOrientedBox({{0, 5, 0}, {0, -1, 0}}, kUnitBox, flattened));
}

TEST(IntersectionTest, TransformAabbHandlesMirroring) {
  const Aabb box = MakeAabb({1, 2, 3}, {2, 4, 6});
  const Aabb mirrored = TransformAabb(box, Mat4::Scale({-1, 1, -2}));
  EXPECT_VEC3_NEAR(mirrored.min, (Vec3{-2, 2, -12}), 1e-12);
  EXPECT_VEC3_NEAR(mirrored.max, (Vec3{-1, 4, -6}), 1e-12);
}

TEST(IntersectionTest, TransformAabbEnclosesRotatedCorners) {
  const Mat4 m =
      Mat4::Trs({1, 2, 3}, FromAxisAngle(Vec3::UnitZ(), kPi / 4.0), {1, 1, 1});
  const Aabb world = TransformAabb(kUnitBox, m);
  const double r = std::sqrt(2.0);
  EXPECT_VEC3_NEAR(world.min, (Vec3{1 - r, 2 - r, 2}), 1e-12);
  EXPECT_VEC3_NEAR(world.max, (Vec3{1 + r, 2 + r, 4}), 1e-12);
  EXPECT_TRUE(IsEmpty(TransformAabb(Aabb{}, m)));
}

}  // namespace
}  // namespace aerolab
