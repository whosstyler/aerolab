#include "sim/force_model.h"

#include <gtest/gtest.h>

#include <cmath>

#include "math/quat.h"
#include "math/scalar.h"
#include "sim/kinematic_path.h"
#include "sim/rigid_body.h"
#include "test_util.h"

namespace aerolab::sim {
namespace {

TEST(ForceModelTest, DefaultIsGravityOnly) {
  RigidBodyState state;
  state.velocity = {5, 5, 5};
  const MassProperties mass{2.0, {1, 1, 1}};
  const ForceBreakdown f = ComputeForces(state, mass, ForceModel{}, 0.0);
  EXPECT_VEC3_NEAR(f.total_force, (Vec3{0, -19.62, 0}), 1e-12);
  EXPECT_VEC3_NEAR(f.total_torque, Vec3::Zero(), 1e-12);
  EXPECT_VEC3_NEAR(Evaluate(state, mass, ForceModel{}, 0.0).acceleration,
                   (Vec3{0, -9.81, 0}), 1e-12);
}

TEST(ForceModelTest, DragOpposesAirspeedIncludingWind) {
  ForceModel model;
  model.gravity_enabled = false;
  model.linear_drag = 0.5;
  model.quadratic_drag = 0.1;
  model.wind = {2, 0, 0};
  RigidBodyState state;
  state.velocity = {12, 0, 0};  // 10 m/s airspeed.
  const ForceBreakdown f = ComputeForces(state, MassProperties{}, model, 0.0);
  EXPECT_VEC3_NEAR(f.drag, (Vec3{-(0.5 * 10 + 0.1 * 100), 0, 0}), 1e-12);
  // Moving with the wind: no drag at all.
  state.velocity = model.wind;
  EXPECT_VEC3_NEAR(ComputeForces(state, MassProperties{}, model, 0.0).drag,
                   Vec3::Zero(), 1e-12);
}

TEST(ForceModelTest, ThrustFollowsBodyForwardAndBurnsOut) {
  ForceModel model;
  model.gravity_enabled = false;
  model.thrust = 40.0;
  model.thrust_duration = 2.0;
  RigidBodyState state;
  state.orientation = FromAxisAngle(Vec3::UnitY(), kHalfPi);  // Faces -X.
  EXPECT_VEC3_NEAR(ComputeForces(state, MassProperties{}, model, 1.0).thrust,
                   (Vec3{-40, 0, 0}), 1e-9);
  EXPECT_VEC3_NEAR(ComputeForces(state, MassProperties{}, model, 2.5).thrust,
                   Vec3::Zero(), 1e-12);
}

TEST(ForceModelTest, LiftNeedsForwardAirspeed) {
  ForceModel model;
  model.gravity_enabled = false;
  model.lift_coefficient = 0.2;
  RigidBodyState state;
  state.velocity = {0, 0, -10};  // Flying forward.
  EXPECT_VEC3_NEAR(ComputeForces(state, MassProperties{}, model, 0.0).lift,
                   (Vec3{0, 20, 0}), 1e-12);
  state.velocity = {0, 0, 10};  // Flying backward: no lift.
  EXPECT_VEC3_NEAR(ComputeForces(state, MassProperties{}, model, 0.0).lift,
                   Vec3::Zero(), 1e-12);
}

TEST(ForceModelTest, WeathervaneTurnsNoseIntoAirflow) {
  ForceModel model;
  model.gravity_enabled = false;
  model.weathervane = 0.1;
  RigidBodyState state;
  state.velocity = {10, 0, 0};  // Airflow along +X, nose along -Z.
  const ForceBreakdown f = ComputeForces(state, MassProperties{}, model, 0.0);
  // A positive torque about -Y yaws -Z toward +X.
  EXPECT_LT(f.total_torque.y, 0.0);
  EXPECT_NEAR(Length(f.total_torque), 0.1 * 10 * 10, 1e-9);
}

TEST(ForceModelTest, EnergyAccountingForConservativeTerms) {
  ForceModel model;
  model.spring_enabled = true;
  model.spring_stiffness = 8.0;
  model.spring_anchor = {0, 10, 0};
  RigidBodyState state;
  state.position = {3, 10, 4};  // 5 m from the anchor, 10 m high.
  state.velocity = {0, 2, 0};
  const MassProperties mass{2.0, {1, 1, 1}};
  EXPECT_NEAR(KineticEnergy(state, mass), 4.0, 1e-12);
  EXPECT_NEAR(PotentialEnergy(state, mass, model),
              2.0 * 9.81 * 10.0 + 0.5 * 8.0 * 25.0, 1e-9);
}

TEST(ForceModelTest, SpringForceIsMinusGradientOfPotential) {
  ForceModel model;
  model.gravity_enabled = false;
  model.central_enabled = true;
  model.central_mu = 50.0;
  RigidBodyState state;
  state.position = {3, -1, 2};
  const MassProperties mass{1.5, {1, 1, 1}};
  const double h = 1e-6;
  Vec3 gradient;
  for (int axis = 0; axis < 3; ++axis) {
    RigidBodyState plus = state;
    RigidBodyState minus = state;
    plus.position[axis] += h;
    minus.position[axis] -= h;
    gradient[axis] = (PotentialEnergy(plus, mass, model) -
                      PotentialEnergy(minus, mass, model)) /
                     (2.0 * h);
  }
  EXPECT_VEC3_NEAR(ComputeForces(state, mass, model, 0.0).central, -gradient,
                   1e-6);
}

TEST(RigidBodyTest, InertiaFormulas) {
  const MassProperties box = BoxMassProperties(12.0, {1.0, 2.0, 3.0});
  EXPECT_NEAR(box.principal_inertia.x, 4.0 + 9.0, 1e-12);
  EXPECT_NEAR(box.principal_inertia.y, 1.0 + 9.0, 1e-12);
  EXPECT_NEAR(box.principal_inertia.z, 1.0 + 4.0, 1e-12);
  const MassProperties sphere = SphereMassProperties(5.0, 2.0);
  EXPECT_NEAR(sphere.principal_inertia.x, 8.0, 1e-12);
}

TEST(RigidBodyTest, InterpolatePoseBlendsPositionAndRotation) {
  RigidBodyState a;
  RigidBodyState b;
  b.position = {10, 0, 0};
  b.orientation = FromAxisAngle(Vec3::UnitY(), 1.0);
  const Transform mid = InterpolatePose(a, b, 0.25);
  EXPECT_VEC3_NEAR(mid.position, (Vec3{2.5, 0, 0}), 1e-12);
  EXPECT_SAME_ROTATION(mid.rotation, FromAxisAngle(Vec3::UnitY(), 0.25), 1e-12);
}

class KinematicPathTest : public ::testing::TestWithParam<PathShape> {};

TEST_P(KinematicPathTest, DerivativesMatchFiniteDifferences) {
  KinematicPath path;
  path.shape = GetParam();
  path.center = {5, 20, -3};
  path.radius = 12.0;
  path.period = 9.0;
  const double t = 3.7;
  const double h = 1e-5;
  const PathSample s = SamplePath(path, t);
  const PathSample ahead = SamplePath(path, t + h);
  const PathSample behind = SamplePath(path, t - h);
  EXPECT_VEC3_NEAR(s.velocity, (ahead.position - behind.position) / (2 * h),
                   1e-5);
  EXPECT_VEC3_NEAR(s.acceleration, (ahead.velocity - behind.velocity) / (2 * h),
                   1e-5);
  // The body faces along its velocity.
  EXPECT_VEC3_NEAR(Rotate(s.orientation, -Vec3::UnitZ()),
                   Normalized(s.velocity), 1e-9);
  EXPECT_NE(PathShapeName(GetParam()), "Unknown");
}

INSTANTIATE_TEST_SUITE_P(AllShapes, KinematicPathTest,
                         ::testing::Values(PathShape::kCircle,
                                           PathShape::kFigureEight,
                                           PathShape::kHelix));

TEST(KinematicPathTest, CircleBanksIntoTheTurn) {
  KinematicPath path;
  path.radius = 20.0;
  path.period = 6.0;
  const PathSample s = SamplePath(path, 0.0);
  const Vec3 up = Rotate(s.orientation, Vec3::UnitY());
  // Centripetal acceleration points at the centre; the body tilts its up
  // vector toward it.
  EXPECT_GT(Dot(up, s.acceleration), 0.0);
  EXPECT_NEAR(Length(s.position - path.center), path.radius, 1e-9);
}

}  // namespace
}  // namespace aerolab::sim
