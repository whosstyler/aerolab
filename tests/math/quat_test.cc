#include "math/quat.h"

#include <gtest/gtest.h>

#include <cmath>

#include "math/mat3.h"
#include "math/scalar.h"
#include "math/vec3.h"
#include "test_util.h"

namespace aerolab {
namespace {

TEST(QuatTest, IdentityLeavesVectorsUnchanged) {
  const Vec3 v{1.5, -2.0, 0.25};
  EXPECT_VEC3_NEAR(Rotate(Quat::Identity(), v), v, 1e-15);
  EXPECT_EQ(Quat{}, Quat::Identity());
}

TEST(QuatTest, AxisAngleRotationsFollowRightHandRule) {
  // +90 degrees about +Z takes +X to +Y.
  EXPECT_VEC3_NEAR(Rotate(FromAxisAngle(Vec3::UnitZ(), kHalfPi), Vec3::UnitX()),
                   Vec3::UnitY(), 1e-12);
  // +90 degrees about +Y takes +Z to +X.
  EXPECT_VEC3_NEAR(Rotate(FromAxisAngle(Vec3::UnitY(), kHalfPi), Vec3::UnitZ()),
                   Vec3::UnitX(), 1e-12);
  // +90 degrees about +X takes +Y to +Z.
  EXPECT_VEC3_NEAR(Rotate(FromAxisAngle(Vec3::UnitX(), kHalfPi), Vec3::UnitY()),
                   Vec3::UnitZ(), 1e-12);
  EXPECT_EQ(FromAxisAngle(Vec3::Zero(), 1.0), Quat::Identity());
}

TEST(QuatTest, CompositionAppliesRightOperandFirst) {
  const Quat first = FromAxisAngle(Vec3::UnitZ(), kHalfPi);
  const Quat second = FromAxisAngle(Vec3::UnitX(), kHalfPi);
  const Vec3 v = Vec3::UnitX();
  EXPECT_VEC3_NEAR(Rotate(second * first, v), Rotate(second, Rotate(first, v)),
                   1e-12);
}

TEST(QuatTest, MatrixRoundTripAcrossAllBranches) {
  // Angles near 0, pi/2 and pi exercise every branch of Shepperd's method.
  const Vec3 axes[] = {Vec3::UnitX(), Vec3::UnitY(), Vec3::UnitZ(),
                       Normalized(Vec3{1.0, -2.0, 0.5})};
  const double angles[] = {0.0, 0.3, kHalfPi, 2.9, kPi};
  for (const Vec3& axis : axes) {
    for (double angle : angles) {
      const Quat q = FromAxisAngle(axis, angle);
      EXPECT_SAME_ROTATION(FromMat3(ToMat3(q)), q, 1e-12) << "angle " << angle;
    }
  }
}

TEST(QuatTest, MatrixAndQuaternionRotateIdentically) {
  const Quat q = FromAxisAngle(Vec3{0.3, 1.0, -0.4}, 1.234);
  const Vec3 v{-0.7, 2.1, 0.9};
  EXPECT_VEC3_NEAR(ToMat3(q) * v, Rotate(q, v), 1e-12);
  EXPECT_NEAR(Determinant(ToMat3(q)), 1.0, 1e-12);
}

TEST(QuatTest, InverseUndoesRotation) {
  const Quat q = FromAxisAngle(Vec3{1.0, 1.0, 0.0}, 0.77);
  const Vec3 v{3.0, -1.0, 2.0};
  EXPECT_VEC3_NEAR(Rotate(Inverse(q), Rotate(q, v)), v, 1e-12);
  EXPECT_SAME_ROTATION(q * Conjugate(q), Quat::Identity(), 1e-12);
  // A non-unit quaternion's inverse is still a true multiplicative inverse.
  const Quat scaled = q * 3.0;
  EXPECT_TRUE(NearlyEqual(scaled * Inverse(scaled), Quat::Identity(), 1e-12));
}

TEST(QuatTest, EulerRoundTripAwayFromGimbalLock) {
  const EulerAngles cases[] = {
      {.yaw = 0.2, .pitch = 0.1, .roll = 0.3},
      {.yaw = 2.5, .pitch = -0.5, .roll = -1.0},
      {.yaw = -3.0, .pitch = 1.2, .roll = 2.0},
      {.yaw = kPi * 0.9},
  };
  for (const EulerAngles& angles : cases) {
    const EulerAngles back = ToEuler(FromEuler(angles));
    EXPECT_SAME_ROTATION(FromEuler(back), FromEuler(angles), 1e-12);
    EXPECT_NEAR(back.pitch, angles.pitch, 1e-9);
    EXPECT_NEAR(back.yaw, angles.yaw, 1e-9);
    EXPECT_NEAR(back.roll, angles.roll, 1e-9);
  }
}

TEST(QuatTest, EulerAtGimbalLockStillRepresentsTheRotation) {
  const EulerAngles locked{.yaw = 0.4, .pitch = kHalfPi, .roll = 0.3};
  const EulerAngles back = ToEuler(FromEuler(locked));
  EXPECT_SAME_ROTATION(FromEuler(back), FromEuler(locked), 1e-7);
  EXPECT_NEAR(back.roll, 0.0, 1e-12);
}

TEST(QuatTest, EulerSignConventions) {
  const Vec3 forward = -Vec3::UnitZ();
  // Positive pitch raises the nose.
  EXPECT_GT(Rotate(FromEuler({.pitch = 0.3}), forward).y, 0.0);
  // Positive yaw turns the nose toward -X (counter-clockwise from above).
  EXPECT_LT(Rotate(FromEuler({.yaw = 0.3}), forward).x, 0.0);
  // Positive roll lifts the right wing.
  EXPECT_GT(Rotate(FromEuler({.roll = 0.3}), Vec3::UnitX()).y, 0.0);
}

TEST(QuatTest, RotationVectorRoundTrip) {
  const Vec3 rv{0.4, -1.1, 0.25};
  EXPECT_VEC3_NEAR(ToRotationVector(FromRotationVector(rv)), rv, 1e-12);
  EXPECT_SAME_ROTATION(FromRotationVector(Vec3{0, kHalfPi, 0}),
                       FromAxisAngle(Vec3::UnitY(), kHalfPi), 1e-12);
  EXPECT_EQ(FromRotationVector(Vec3::Zero()), Quat::Identity());
}

TEST(QuatTest, TinyRotationVectorsAreNotLost) {
  // Per-step rotations of a slowly spinning body at a small time step.
  for (double angle : {1e-14, 1e-12, 1e-11, 1e-10, 1e-9}) {
    const Vec3 rv{angle, 0.0, 0.0};
    const Quat q = FromRotationVector(rv);
    EXPECT_NE(q, Quat::Identity()) << angle;
    EXPECT_NEAR(ToRotationVector(q).x, angle, angle * 1e-9) << angle;
    EXPECT_NEAR(AngleBetween(q, Quat::Identity()), angle, angle * 1e-9);
  }
}

TEST(QuatTest, SameRotationToleranceIsAnAngle) {
  const Quat a = FromAxisAngle(Vec3::UnitZ(), 0.5);
  const Quat b = FromAxisAngle(Vec3::UnitZ(), 0.5 + 1e-6);
  EXPECT_TRUE(SameRotation(a, b, 2e-6));
  EXPECT_FALSE(SameRotation(a, b, 5e-7));
  EXPECT_TRUE(SameRotation(a, -a, 1e-12));
}

TEST(QuatTest, FromToMapsDirections) {
  const Vec3 a = Normalized(Vec3{1.0, 2.0, 3.0});
  const Vec3 b = Normalized(Vec3{-2.0, 0.5, 1.0});
  EXPECT_VEC3_NEAR(Rotate(FromTo(a, b), a), b, 1e-12);
  // Opposite directions take the special-case path.
  EXPECT_VEC3_NEAR(Rotate(FromTo(Vec3::UnitX(), -Vec3::UnitX()), Vec3::UnitX()),
                   -Vec3::UnitX(), 1e-12);
  EXPECT_SAME_ROTATION(FromTo(a, a), Quat::Identity(), 1e-12);
}

TEST(QuatTest, FromToNearlyOppositeDirections) {
  const Vec3 a = Vec3::UnitX();
  const Vec3 b = Normalized(Vec3{-1.0, 1e-7, 0.0});
  EXPECT_VEC3_NEAR(Rotate(FromTo(a, b), a), b, 1e-12);
  // The rotation axis must follow the tiny offset (about +Z), not be random.
  EXPECT_VEC3_NEAR(Normalized(ToRotationVector(FromTo(a, b))), Vec3::UnitZ(),
                   1e-6);
}

TEST(QuatTest, LookRotationPointsLocalForwardAlongTarget) {
  const Vec3 forward = Normalized(Vec3{1.0, 0.5, -2.0});
  const Quat q = LookRotation(forward, Vec3::UnitY());
  EXPECT_VEC3_NEAR(Rotate(q, -Vec3::UnitZ()), forward, 1e-12);
  // Local right stays horizontal when looking with a world-up hint.
  EXPECT_NEAR(Rotate(q, Vec3::UnitX()).y, 0.0, 1e-12);
  // Degenerate: looking straight up still gives a valid rotation.
  const Quat up = LookRotation(Vec3::UnitY(), Vec3::UnitY());
  EXPECT_VEC3_NEAR(Rotate(up, -Vec3::UnitZ()), Vec3::UnitY(), 1e-12);
  EXPECT_NEAR(Length(up), 1.0, 1e-12);
}

TEST(QuatTest, SlerpEndpointsMidpointAndShortestPath) {
  const Quat a = FromAxisAngle(Vec3::UnitY(), 0.0);
  const Quat b = FromAxisAngle(Vec3::UnitY(), 2.0);
  EXPECT_SAME_ROTATION(Slerp(a, b, 0.0), a, 1e-12);
  EXPECT_SAME_ROTATION(Slerp(a, b, 1.0), b, 1e-12);
  EXPECT_SAME_ROTATION(Slerp(a, b, 0.5), FromAxisAngle(Vec3::UnitY(), 1.0),
                       1e-12);
  // -b is the same rotation; slerp must not take the long way around.
  EXPECT_SAME_ROTATION(Slerp(a, -b, 0.5), FromAxisAngle(Vec3::UnitY(), 1.0),
                       1e-12);
  // Nearly identical inputs use the nlerp fallback.
  const Quat c = FromAxisAngle(Vec3::UnitY(), 1e-4);
  EXPECT_NEAR(Length(Slerp(a, c, 0.5)), 1.0, 1e-12);
}

TEST(QuatTest, AngleBetweenQuaternions) {
  const Quat a = FromAxisAngle(Vec3::UnitX(), 0.2);
  const Quat b = FromAxisAngle(Vec3::UnitX(), 0.9);
  EXPECT_NEAR(AngleBetween(a, b), 0.7, 1e-12);
  EXPECT_NEAR(AngleBetween(a, -a), 0.0, 1e-15);
  const Quat c = FromAxisAngle(Vec3::UnitX(), 0.2 + 1e-10);
  EXPECT_NEAR(AngleBetween(a, c), 1e-10, 1e-15);
}

TEST(QuatTest, DerivativeMatchesFiniteDifference) {
  const Quat q = FromAxisAngle(Vec3{0.2, 1.0, 0.1}, 0.6);
  const Vec3 omega{0.5, -1.5, 2.0};
  const double h = 1e-6;
  const Quat ahead = FromRotationVector(omega * h) * q;
  const Quat finite = (ahead + (-q)) * (1.0 / h);
  EXPECT_TRUE(NearlyEqual(Derivative(q, omega), finite, 1e-5));
}

TEST(QuatTest, NormalizedHandlesZero) {
  EXPECT_EQ(Normalized(Quat{0, 0, 0, 0}), Quat::Identity());
  EXPECT_NEAR(Length(Normalized(Quat{2, 1, 0, 0})), 1.0, 1e-15);
}

}  // namespace
}  // namespace aerolab
