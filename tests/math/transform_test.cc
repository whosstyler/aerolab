#include "math/transform.h"

#include <gtest/gtest.h>

#include "math/mat4.h"
#include "math/quat.h"
#include "math/vec3.h"
#include "test_util.h"

namespace aerolab {
namespace {

Transform SampleTransform() {
  Transform t;
  t.position = {4.0, 1.0, -2.0};
  t.rotation = FromEuler({.yaw = -1.1, .pitch = 0.3, .roll = 0.4});
  t.scale = {2.0, 0.5, 3.0};
  return t;
}

TEST(TransformTest, DefaultIsIdentity) {
  const Transform t;
  const Vec3 p{1.0, 2.0, 3.0};
  EXPECT_EQ(TransformPoint(t, p), p);
  EXPECT_TRUE(NearlyEqual(ToMatrix(t), Mat4::Identity()));
}

TEST(TransformTest, PointTransformMatchesMatrix) {
  const Transform t = SampleTransform();
  const Vec3 p{-0.5, 1.5, 2.0};
  EXPECT_VEC3_NEAR(TransformPoint(t, p), TransformPoint(ToMatrix(t), p), 1e-12);
}

TEST(TransformTest, InversePointRoundTrip) {
  const Transform t = SampleTransform();
  const Vec3 p{7.0, -3.0, 0.25};
  EXPECT_VEC3_NEAR(InverseTransformPoint(t, TransformPoint(t, p)), p, 1e-12);
  const Vec3 d = Normalized(Vec3{1.0, 1.0, 0.0});
  EXPECT_VEC3_NEAR(InverseTransformDirection(t, TransformDirection(t, d)), d,
                   1e-12);
}

TEST(TransformTest, BasisVectorsFollowEngineConvention) {
  Transform t;
  EXPECT_VEC3_NEAR(Forward(t), -Vec3::UnitZ(), 1e-15);
  EXPECT_VEC3_NEAR(Right(t), Vec3::UnitX(), 1e-15);
  EXPECT_VEC3_NEAR(Up(t), Vec3::UnitY(), 1e-15);
  t.rotation = FromAxisAngle(Vec3::UnitY(), kHalfPi);
  // Yawing +90 degrees turns forward (-Z) to -X.
  EXPECT_VEC3_NEAR(Forward(t), -Vec3::UnitX(), 1e-12);
  EXPECT_NEAR(Dot(Forward(t), Right(t)), 0.0, 1e-12);
  EXPECT_VEC3_NEAR(Cross(Right(t), Up(t)), -Forward(t), 1e-12);
}

TEST(TransformTest, MatrixDecompositionRoundTrip) {
  const Transform t = SampleTransform();
  const Transform back = FromMatrix(ToMatrix(t));
  EXPECT_TRUE(NearlyEqual(back, t, 1e-12));
}

TEST(TransformTest, HierarchyComposesThroughMatrices) {
  Transform parent;
  parent.position = {10.0, 0.0, 0.0};
  parent.rotation = FromAxisAngle(Vec3::UnitY(), kHalfPi);
  Transform child;
  child.position = {0.0, 0.0, -2.0};  // Two metres in front of the parent.
  const Mat4 world = ToMatrix(parent) * ToMatrix(child);
  // Parent faces -X, so "in front" is at x = 8.
  EXPECT_VEC3_NEAR(world.TranslationPart(), (Vec3{8.0, 0.0, 0.0}), 1e-12);
}

TEST(TransformTest, InterpolateBlendsAllComponents) {
  Transform a;
  Transform b;
  b.position = {2.0, 4.0, 6.0};
  b.rotation = FromAxisAngle(Vec3::UnitZ(), 1.0);
  b.scale = {3.0, 3.0, 3.0};
  const Transform mid = Interpolate(a, b, 0.5);
  EXPECT_VEC3_NEAR(mid.position, (Vec3{1.0, 2.0, 3.0}), 1e-12);
  EXPECT_SAME_ROTATION(mid.rotation, FromAxisAngle(Vec3::UnitZ(), 0.5), 1e-12);
  EXPECT_VEC3_NEAR(mid.scale, (Vec3{2.0, 2.0, 2.0}), 1e-12);
  EXPECT_TRUE(NearlyEqual(Interpolate(a, b, 1.0), b, 1e-12));
}

}  // namespace
}  // namespace aerolab
