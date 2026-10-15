#include "math/projection.h"

#include <gtest/gtest.h>

#include <optional>

#include "math/mat4.h"
#include "math/ray.h"
#include "math/vec2.h"
#include "math/vec3.h"
#include "math/vec4.h"
#include "test_util.h"

namespace aerolab {
namespace {

TEST(ProjectionTest, PerspectiveMapsNearAndFarPlanesToNdcDepth) {
  const Mat4 p = Perspective(Radians(60.0), 16.0 / 9.0, 0.1, 1000.0);
  EXPECT_NEAR(TransformPointProjective(p, {0, 0, -0.1}).z, -1.0, 1e-9);
  EXPECT_NEAR(TransformPointProjective(p, {0, 0, -1000.0}).z, 1.0, 1e-9);
  // Clip w equals the view-space distance in front of the camera.
  EXPECT_NEAR((p * Vec4{0, 0, -5, 1}).w, 5.0, 1e-12);
}

TEST(ProjectionTest, PerspectiveFieldOfViewHitsFrustumEdges) {
  const double fov = Radians(90.0);
  const Mat4 p = Perspective(fov, 2.0, 0.1, 100.0);
  // With a 90 degree vertical FOV, y == -z lies on the top edge.
  EXPECT_NEAR(TransformPointProjective(p, {0, 3, -3}).y, 1.0, 1e-12);
  // The horizontal extent is scaled by the aspect ratio.
  EXPECT_NEAR(TransformPointProjective(p, {6, 0, -3}).x, 1.0, 1e-12);
}

TEST(ProjectionTest, OrthographicMapsBoxToCube) {
  const Mat4 o = Orthographic(-2, 2, -1, 1, 0.5, 10.0);
  EXPECT_VEC3_NEAR(TransformPoint(o, {2, 1, -0.5}), (Vec3{1, 1, -1}), 1e-12);
  EXPECT_VEC3_NEAR(TransformPoint(o, {-2, -1, -10}), (Vec3{-1, -1, 1}), 1e-12);
}

TEST(ProjectionTest, LookAtPutsTargetOnNegativeZAxis) {
  const Vec3 eye{5.0, 3.0, 8.0};
  const Vec3 target{-1.0, 0.5, 2.0};
  const Mat4 view = LookAt(eye, target, Vec3::UnitY());
  const Vec3 in_view = TransformPoint(view, target);
  EXPECT_NEAR(in_view.x, 0.0, 1e-12);
  EXPECT_NEAR(in_view.y, 0.0, 1e-12);
  EXPECT_NEAR(in_view.z, -Distance(eye, target), 1e-12);
  EXPECT_VEC3_NEAR(TransformPoint(view, eye), Vec3::Zero(), 1e-12);
}

TEST(ProjectionTest, LookAtStraightDownIsStillValid) {
  const Mat4 view = LookAt({0, 10, 0}, Vec3::Zero(), Vec3::UnitY());
  EXPECT_NEAR(Determinant(view), 1.0, 1e-12);
  EXPECT_NEAR(TransformPoint(view, Vec3::Zero()).z, -10.0, 1e-12);
}

TEST(ProjectionTest, ScreenNdcConversionsRoundTrip) {
  const ScreenRect rect{100.0, 50.0, 800.0, 600.0};
  // Top-left pixel is NDC (-1, +1); the centre is the origin.
  EXPECT_TRUE(NearlyEqual(ScreenToNdc({100.0, 50.0}, rect), Vec2{-1.0, 1.0}));
  EXPECT_TRUE(NearlyEqual(ScreenToNdc({500.0, 350.0}, rect), Vec2{0.0, 0.0}));
  const Vec2 pixel{321.0, 123.0};
  EXPECT_TRUE(NearlyEqual(NdcToScreen(ScreenToNdc(pixel, rect), rect), pixel));
}

TEST(ProjectionTest, WorldToScreenAndBackAgree) {
  const ScreenRect rect{0.0, 0.0, 1280.0, 720.0};
  const Mat4 view = LookAt({3, 4, 12}, {0, 1, 0}, Vec3::UnitY());
  const Mat4 proj =
      Perspective(Radians(55.0), rect.width / rect.height, 0.1, 500.0);
  const Mat4 view_proj = proj * view;
  const Vec3 world{1.5, 2.0, -1.0};
  const std::optional<Vec2> pixel = WorldToScreen(view_proj, world, rect);
  ASSERT_TRUE(pixel.has_value());

  const Ray ray = ScreenToWorldRay(*Inverse(view_proj), *pixel, rect);
  // The ray through the projected pixel must pass through the world point.
  const Vec3 to_point = world - ray.origin;
  EXPECT_NEAR(Length(Reject(to_point, ray.direction)), 0.0, 1e-9);
  EXPECT_GT(Dot(to_point, ray.direction), 0.0);
  EXPECT_NEAR(Length(ray.direction), 1.0, 1e-12);
}

TEST(ProjectionTest, OrthographicRaysAreParallel) {
  const ScreenRect rect{0.0, 0.0, 400.0, 200.0};
  const Mat4 view_proj = Orthographic(-20, 20, -10, 10, 0.1, 100.0) *
                         LookAt({0, 0, 50}, Vec3::Zero(), Vec3::UnitY());
  const Mat4 inverse = *Inverse(view_proj);
  const Ray center = ScreenToWorldRay(inverse, {200.0, 100.0}, rect);
  const Ray corner = ScreenToWorldRay(inverse, {0.0, 0.0}, rect);
  EXPECT_VEC3_NEAR(center.direction, -Vec3::UnitZ(), 1e-12);
  EXPECT_VEC3_NEAR(corner.direction, -Vec3::UnitZ(), 1e-12);
  // The top-left pixel starts at the top-left corner of the view volume.
  EXPECT_NEAR(corner.origin.x, -20.0, 1e-9);
  EXPECT_NEAR(corner.origin.y, 10.0, 1e-9);
}

TEST(ProjectionTest, PointsBehindCameraHaveNoScreenPosition) {
  const ScreenRect rect{0.0, 0.0, 640.0, 480.0};
  const Mat4 view_proj = Perspective(Radians(60.0), 4.0 / 3.0, 0.1, 100.0) *
                         LookAt(Vec3::Zero(), {0, 0, -1}, Vec3::UnitY());
  EXPECT_FALSE(WorldToScreen(view_proj, {0, 0, 5}, rect).has_value());
  EXPECT_TRUE(WorldToScreen(view_proj, {0, 0, -5}, rect).has_value());
}

}  // namespace
}  // namespace aerolab
