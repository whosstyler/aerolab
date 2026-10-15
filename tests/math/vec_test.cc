#include <gtest/gtest.h>

#include <cmath>

#include "math/scalar.h"
#include "math/vec2.h"
#include "math/vec3.h"
#include "math/vec4.h"
#include "test_util.h"

namespace aerolab {
namespace {

TEST(ScalarTest, AngleConversionsRoundTrip) {
  EXPECT_DOUBLE_EQ(Radians(180.0), kPi);
  EXPECT_DOUBLE_EQ(Degrees(kHalfPi), 90.0);
  EXPECT_NEAR(Degrees(Radians(37.5)), 37.5, 1e-12);
}

TEST(ScalarTest, WrapAngleStaysInHalfOpenRange) {
  EXPECT_NEAR(WrapAngle(3.0 * kPi), kPi, 1e-12);
  EXPECT_NEAR(WrapAngle(-3.0 * kPi), kPi, 1e-12);
  EXPECT_NEAR(WrapAngle(0.25), 0.25, 1e-12);
  EXPECT_NEAR(WrapAngle(kTwoPi + 0.5), 0.5, 1e-12);
}

TEST(ScalarTest, SmoothingFactorIsFrameRateIndependent) {
  // Two half steps must cover the same fraction as one full step.
  const double full = SmoothingFactor(8.0, 0.1);
  const double half = SmoothingFactor(8.0, 0.05);
  const double remaining_after_two_halves = (1.0 - half) * (1.0 - half);
  EXPECT_NEAR(1.0 - remaining_after_two_halves, full, 1e-12);
}

TEST(ScalarTest, NearlyEqualUsesRelativeToleranceForLargeValues) {
  EXPECT_TRUE(NearlyEqual(1e12, 1e12 + 1.0, 1e-9));
  EXPECT_FALSE(NearlyEqual(1.0, 1.0 + 1e-6, 1e-9));
}

TEST(Vec2Test, ArithmeticAndLength) {
  const Vec2 v = Vec2{3.0, 4.0} * 2.0 - Vec2{1.0, 1.0};
  EXPECT_EQ(v, (Vec2{5.0, 7.0}));
  EXPECT_DOUBLE_EQ(Length(Vec2{3.0, 4.0}), 5.0);
  EXPECT_TRUE(NearlyEqual(Normalized(Vec2{0.0, 2.0}), Vec2{0.0, 1.0}));
  EXPECT_EQ(Normalized(Vec2{}), Vec2{});
}

TEST(Vec3Test, BasicArithmetic) {
  const Vec3 a{1.0, 2.0, 3.0};
  const Vec3 b{4.0, -5.0, 6.0};
  EXPECT_EQ(a + b, (Vec3{5.0, -3.0, 9.0}));
  EXPECT_EQ(a - b, (Vec3{-3.0, 7.0, -3.0}));
  EXPECT_EQ(-a, (Vec3{-1.0, -2.0, -3.0}));
  EXPECT_EQ(a * 2.0, (Vec3{2.0, 4.0, 6.0}));
  EXPECT_EQ(2.0 * a, a * 2.0);
  EXPECT_EQ(b / 2.0, (Vec3{2.0, -2.5, 3.0}));
  EXPECT_EQ(Hadamard(a, b), (Vec3{4.0, -10.0, 18.0}));

  Vec3 c = a;
  c += b;
  c -= a;
  c *= 3.0;
  c /= 3.0;
  EXPECT_EQ(c, b);
}

TEST(Vec3Test, IndexAccessMatchesNamedComponents) {
  Vec3 v{7.0, 8.0, 9.0};
  EXPECT_EQ(v[0], 7.0);
  EXPECT_EQ(v[1], 8.0);
  EXPECT_EQ(v[2], 9.0);
  v[1] = -1.0;
  EXPECT_EQ(v.y, -1.0);
}

TEST(Vec3Test, DotAndCrossFollowRightHandRule) {
  EXPECT_EQ(Cross(Vec3::UnitX(), Vec3::UnitY()), Vec3::UnitZ());
  EXPECT_EQ(Cross(Vec3::UnitY(), Vec3::UnitZ()), Vec3::UnitX());
  EXPECT_EQ(Cross(Vec3::UnitZ(), Vec3::UnitX()), Vec3::UnitY());
  const Vec3 a{1.0, 2.0, 3.0};
  const Vec3 b{-2.0, 0.5, 4.0};
  const Vec3 c = Cross(a, b);
  EXPECT_NEAR(Dot(c, a), 0.0, 1e-12);
  EXPECT_NEAR(Dot(c, b), 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(Dot(a, b), -2.0 + 1.0 + 12.0);
}

TEST(Vec3Test, NormalizationHandlesDegenerateInput) {
  EXPECT_VEC3_NEAR(Normalized(Vec3{0.0, 3.0, 4.0}), (Vec3{0.0, 0.6, 0.8}),
                   1e-12);
  EXPECT_EQ(Normalized(Vec3{}), Vec3::Zero());
  EXPECT_EQ(NormalizedOr(Vec3{1e-12, 0.0, 0.0}, Vec3::UnitY()), Vec3::UnitY());
  EXPECT_DOUBLE_EQ(Length(Normalized(Vec3{-3.0, 2.0, 9.0})), 1.0);
}

TEST(Vec3Test, ProjectRejectReflect) {
  const Vec3 v{3.0, 4.0, 5.0};
  const Vec3 axis{0.0, 2.0, 0.0};
  EXPECT_VEC3_NEAR(Project(v, axis), (Vec3{0.0, 4.0, 0.0}), 1e-12);
  EXPECT_VEC3_NEAR(Reject(v, axis), (Vec3{3.0, 0.0, 5.0}), 1e-12);
  EXPECT_VEC3_NEAR(Project(v, Vec3::Zero()), Vec3::Zero(), 1e-12);
  EXPECT_VEC3_NEAR(Reflect(Vec3{1.0, -1.0, 0.0}, Vec3::UnitY()),
                   (Vec3{1.0, 1.0, 0.0}), 1e-12);
}

TEST(Vec3Test, AngleBetweenIsAccurateAtExtremes) {
  EXPECT_NEAR(AngleBetween(Vec3::UnitX(), Vec3::UnitY()), kHalfPi, 1e-12);
  EXPECT_NEAR(AngleBetween(Vec3::UnitX(), -Vec3::UnitX()), kPi, 1e-12);
  EXPECT_NEAR(AngleBetween(Vec3::UnitX(), Vec3{1.0, 1e-9, 0.0}), 1e-9, 1e-15);
}

TEST(Vec3Test, AnyPerpendicularIsUnitAndOrthogonal) {
  for (const Vec3& v : {Vec3::UnitX(), Vec3::UnitY(), Vec3::UnitZ(),
                        Normalized(Vec3{1.0, 1.0, 1.0})}) {
    const Vec3 p = AnyPerpendicular(v);
    EXPECT_NEAR(Length(p), 1.0, 1e-12);
    EXPECT_NEAR(Dot(p, v), 0.0, 1e-12);
  }
}

TEST(Vec3Test, MinMaxAbsLerp) {
  const Vec3 a{1.0, -2.0, 3.0};
  const Vec3 b{-1.0, 5.0, 2.0};
  EXPECT_EQ(Min(a, b), (Vec3{-1.0, -2.0, 2.0}));
  EXPECT_EQ(Max(a, b), (Vec3{1.0, 5.0, 3.0}));
  EXPECT_EQ(Abs(a), (Vec3{1.0, 2.0, 3.0}));
  EXPECT_EQ(MaxComponent(a), 3.0);
  EXPECT_VEC3_NEAR(Lerp(a, b, 0.5), (Vec3{0.0, 1.5, 2.5}), 1e-12);
  EXPECT_TRUE(IsFinite(a));
  EXPECT_FALSE(IsFinite(Vec3{std::nan(""), 0.0, 0.0}));
}

TEST(Vec4Test, HomogeneousHelpers) {
  const Vec3 p{1.0, 2.0, 3.0};
  EXPECT_EQ(MakePoint(p), (Vec4{1.0, 2.0, 3.0, 1.0}));
  EXPECT_EQ(MakeDirection(p), (Vec4{1.0, 2.0, 3.0, 0.0}));
  EXPECT_EQ(Xyz(MakePoint(p)), p);
  EXPECT_DOUBLE_EQ(Dot((Vec4{1, 2, 3, 4}), (Vec4{4, 3, 2, 1})), 20.0);
  EXPECT_EQ((Vec4{1, 2, 3, 4})[3], 4.0);
}

}  // namespace
}  // namespace aerolab
