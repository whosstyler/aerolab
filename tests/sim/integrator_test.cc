#include "sim/integrator.h"

#include <gtest/gtest.h>

#include <cmath>

#include "math/scalar.h"
#include "sim/force_model.h"
#include "sim/rigid_body.h"
#include "test_util.h"

namespace aerolab::sim {
namespace {

// Runs `steps` fixed steps of size dt and returns the final state.
RigidBodyState Simulate(RigidBodyState state, const MassProperties& mass,
                        const ForceModel& model, double dt, int steps,
                        Integrator integrator) {
  double age = 0.0;
  for (int i = 0; i < steps; ++i) {
    state = Step(state, mass, model, age, dt, integrator);
    age += dt;
  }
  return state;
}

ForceModel GravityOnly() { return ForceModel{}; }

ForceModel Oscillator(double stiffness) {
  ForceModel model;
  model.gravity_enabled = false;
  model.spring_enabled = true;
  model.spring_stiffness = stiffness;
  return model;
}

double TotalEnergy(const RigidBodyState& s, const MassProperties& m,
                   const ForceModel& f) {
  return KineticEnergy(s, m) + PotentialEnergy(s, m, f);
}

TEST(IntegratorTest, AllNamesAreDistinct) {
  for (Integrator a : AllIntegrators()) {
    EXPECT_NE(IntegratorName(a), "Unknown");
    for (Integrator b : AllIntegrators()) {
      if (a != b) EXPECT_NE(IntegratorName(a), IntegratorName(b));
    }
  }
}

TEST(IntegratorTest, WrenchStepMatchesTheForceModelStep) {
  // The generic RK4 step under the force model's own wrench reproduces
  // the built-in RK4 step exactly.
  ForceModel model;
  model.quadratic_drag = 0.02;
  model.body_torque = {0.0, 0.3, 0.1};
  MassProperties mass = BoxMassProperties(2.0, {0.5, 0.3, 1.2});
  RigidBodyState state;
  state.velocity = {12.0, 8.0, -3.0};
  state.angular_velocity = {0.4, -0.2, 1.1};
  const WrenchFunction wrench = [&](const RigidBodyState& s, double t) {
    const ForceBreakdown f = ComputeForces(s, mass, model, t);
    return Wrench{f.total_force, f.total_torque};
  };
  RigidBodyState a = state;
  RigidBodyState b = state;
  for (int i = 0; i < 50; ++i) {
    a = Step(a, mass, model, i * 0.01, 0.01, Integrator::kRungeKutta4);
    b = StepRk4(b, mass, wrench, i * 0.01, 0.01);
  }
  EXPECT_EQ(a.position, b.position);
  EXPECT_EQ(a.velocity, b.velocity);
  EXPECT_EQ(a.angular_velocity, b.angular_velocity);
}

TEST(IntegratorTest, WrenchStepIsExactForConstantForce) {
  MassProperties mass;
  mass.mass = 4.0;
  const WrenchFunction push = [](const RigidBodyState&, double) {
    return Wrench{{8.0, -4.0, 0.0}, Vec3::Zero()};
  };
  RigidBodyState state;
  state = StepRk4(state, mass, push, 0.0, 3.0);
  // x = a t^2 / 2 with a = F / m = (2, -1, 0).
  EXPECT_VEC3_NEAR(state.position, (Vec3{9.0, -4.5, 0.0}), 1e-12);
  EXPECT_VEC3_NEAR(state.velocity, (Vec3{6.0, -3.0, 0.0}), 1e-12);
}

TEST(IntegratorTest, BallisticFlightMatchesClosedForm) {
  // Constant acceleration: second-order and higher methods are exact (up to
  // rounding); Euler variants have an O(dt) error.
  RigidBodyState start;
  start.velocity = {12.0, 20.0, -3.0};
  const MassProperties mass;
  const double dt = 0.01;
  const int steps = 300;
  const double t = dt * steps;
  const Vec3 g{0.0, -9.81, 0.0};
  const Vec3 expected = start.position + start.velocity * t + g * (0.5 * t * t);

  for (Integrator integrator :
       {Integrator::kVelocityVerlet, Integrator::kRungeKutta4}) {
    const RigidBodyState end =
        Simulate(start, mass, GravityOnly(), dt, steps, integrator);
    EXPECT_VEC3_NEAR(end.position, expected, 1e-9);
    EXPECT_VEC3_NEAR(end.velocity, start.velocity + g * t, 1e-9);
  }
  for (Integrator integrator :
       {Integrator::kExplicitEuler, Integrator::kSemiImplicitEuler}) {
    const RigidBodyState end =
        Simulate(start, mass, GravityOnly(), dt, steps, integrator);
    // Error is 0.5 g t dt = ~0.147 m, with opposite signs for the variants.
    EXPECT_NEAR(Length(end.position - expected), 0.5 * 9.81 * t * dt, 1e-6);
    EXPECT_VEC3_NEAR(end.velocity, start.velocity + g * t, 1e-9);
  }
}

TEST(IntegratorTest, ConvergenceOrderMatchesTheory) {
  // Harmonic oscillator x'' = -x with x(0) = 1: x(t) = cos(t).
  const ForceModel model = Oscillator(1.0);
  const MassProperties mass;
  RigidBodyState start;
  start.position = {1.0, 0.0, 0.0};
  const double t_end = 2.0;
  const auto error = [&](Integrator integrator, int steps) {
    const RigidBodyState end =
        Simulate(start, mass, model, t_end / steps, steps, integrator);
    return std::abs(end.position.x - std::cos(t_end));
  };
  struct Expectation {
    Integrator integrator;
    double order;
  };
  for (const Expectation& e : {Expectation{Integrator::kExplicitEuler, 1.0},
                               Expectation{Integrator::kSemiImplicitEuler, 1.0},
                               Expectation{Integrator::kVelocityVerlet, 2.0},
                               Expectation{Integrator::kRungeKutta4, 4.0}}) {
    // Halving dt should shrink the error by about 2^order.
    const double ratio = error(e.integrator, 200) / error(e.integrator, 400);
    EXPECT_NEAR(std::log2(ratio), e.order, 0.25)
        << IntegratorName(e.integrator);
  }
}

TEST(IntegratorTest, Rk4OrientationIsSecondOrder) {
  // Documented limitation: RK4 averages angular velocity and applies one
  // exponential map, so orientation converges at second order even though
  // translation and angular velocity converge at fourth order.
  const MassProperties mass = BoxMassProperties(1.0, {1.0, 1.6, 2.2});
  ForceModel model;
  model.gravity_enabled = false;
  RigidBodyState start;
  start.angular_velocity = {0.6, 2.0, 0.3};
  const RigidBodyState reference = Simulate(start, mass, model, 2.0 / 20000,
                                            20000, Integrator::kRungeKutta4);
  const auto error = [&](int steps) {
    const RigidBodyState end = Simulate(start, mass, model, 2.0 / steps, steps,
                                        Integrator::kRungeKutta4);
    return AngleBetween(end.orientation, reference.orientation);
  };
  const double order = std::log2(error(100) / error(200));
  EXPECT_GT(order, 1.7);
  EXPECT_LT(order, 3.0);
}

TEST(IntegratorTest, EnergyBehaviourOnAnOscillator) {
  const ForceModel model = Oscillator(4.0);
  const MassProperties mass;
  RigidBodyState start;
  start.position = {2.0, 0.0, 0.0};
  const double e0 = TotalEnergy(start, mass, model);
  const double dt = 0.02;
  const int steps = 5000;  // ~32 periods.

  const auto drift = [&](Integrator integrator) {
    const RigidBodyState end =
        Simulate(start, mass, model, dt, steps, integrator);
    return TotalEnergy(end, mass, model) / e0 - 1.0;
  };
  // Explicit Euler pumps energy in exponentially.
  EXPECT_GT(drift(Integrator::kExplicitEuler), 1.0);
  // Symplectic methods keep the energy error bounded and small.
  EXPECT_LT(std::abs(drift(Integrator::kSemiImplicitEuler)), 0.05);
  EXPECT_LT(std::abs(drift(Integrator::kVelocityVerlet)), 1e-3);
  // RK4 is extremely accurate per step; its slow drift is still tiny here.
  EXPECT_LT(std::abs(drift(Integrator::kRungeKutta4)), 1e-6);
}

TEST(IntegratorTest, CircularOrbitStaysCircularWithRk4) {
  ForceModel model;
  model.gravity_enabled = false;
  model.central_enabled = true;
  model.central_mu = 1000.0;
  model.central_softening = 0.0;
  const double radius = 10.0;
  RigidBodyState start;
  start.position = {radius, 0.0, 0.0};
  start.velocity = {0.0, 0.0, -std::sqrt(model.central_mu / radius)};
  const MassProperties mass;
  const double period =
      kTwoPi * std::sqrt(radius * radius * radius / model.central_mu);
  const int steps = 2000;
  const RigidBodyState end = Simulate(start, mass, model, period / steps, steps,
                                      Integrator::kRungeKutta4);
  EXPECT_NEAR(Length(end.position), radius, 1e-6);
  EXPECT_VEC3_NEAR(end.position, start.position, 1e-5);
  // Explicit Euler spirals outward over the same orbit.
  const RigidBodyState euler = Simulate(start, mass, model, period / steps,
                                        steps, Integrator::kExplicitEuler);
  EXPECT_GT(Length(euler.position), radius * 1.01);
}

TEST(IntegratorTest, StepIsDeterministic) {
  ForceModel model;
  model.quadratic_drag = 0.02;
  model.weathervane = 0.05;
  model.angular_damping = 0.01;
  model.thrust = 15.0;
  model.thrust_duration = 1.0;
  RigidBodyState start;
  start.velocity = {3.0, 10.0, -20.0};
  start.angular_velocity = {0.4, -0.2, 1.1};
  const MassProperties mass = BoxMassProperties(2.0, {0.2, 0.2, 1.5});
  for (Integrator integrator : AllIntegrators()) {
    const RigidBodyState a =
        Simulate(start, mass, model, 1.0 / 120.0, 600, integrator);
    const RigidBodyState b =
        Simulate(start, mass, model, 1.0 / 120.0, 600, integrator);
    // Bit-for-bit identical, not merely close.
    EXPECT_EQ(a.position, b.position) << IntegratorName(integrator);
    EXPECT_EQ(a.velocity, b.velocity);
    EXPECT_EQ(a.orientation, b.orientation);
    EXPECT_EQ(a.angular_velocity, b.angular_velocity);
  }
}

TEST(IntegratorTest, OrientationStaysNormalized) {
  // A sphere spins at constant rate (no gyroscopic coupling), so every
  // integrator is stable and only quaternion drift is under test.
  RigidBodyState state;
  state.angular_velocity = {3.0, -7.0, 11.0};
  const MassProperties mass = SphereMassProperties(1.0, 0.5);
  ForceModel model;
  model.gravity_enabled = false;
  for (Integrator integrator : AllIntegrators()) {
    const RigidBodyState end =
        Simulate(state, mass, model, 0.01, 2000, integrator);
    EXPECT_NEAR(Length(end.orientation), 1.0, 1e-12)
        << IntegratorName(integrator);
  }
}

TEST(IntegratorTest, FirstOrderSchemesMishandleFastTumbling) {
  // Euler's rigid-body equations are nonlinear and not separable, so the
  // "symplectic" guarantee of semi-implicit Euler does not apply. With
  // ~0.14 rad of rotation per step, explicit Euler bleeds rotational energy
  // away, while RK4 keeps it constant.
  const MassProperties mass = BoxMassProperties(1.0, {1.0, 2.0, 3.0});
  ForceModel model;
  model.gravity_enabled = false;
  RigidBodyState state;
  state.angular_velocity = {3.0, -7.0, 11.0};
  const double e0 = KineticEnergy(state, mass);
  const RigidBodyState euler =
      Simulate(state, mass, model, 0.01, 200, Integrator::kExplicitEuler);
  const RigidBodyState rk4 =
      Simulate(state, mass, model, 0.01, 200, Integrator::kRungeKutta4);
  const double euler_error = std::abs(KineticEnergy(euler, mass) - e0);
  const double rk4_error = std::abs(KineticEnergy(rk4, mass) - e0);
  EXPECT_GT(euler_error, 0.05 * e0);
  EXPECT_GT(euler_error, 50.0 * rk4_error);
}

TEST(IntegratorTest, SemiImplicitEulerCanDivergeOnFastTumbling) {
  const MassProperties mass = BoxMassProperties(1.0, {1.0, 2.0, 3.0});
  ForceModel model;
  model.gravity_enabled = false;
  RigidBodyState state;
  state.angular_velocity = {3.0, -7.0, 11.0};
  const RigidBodyState end =
      Simulate(state, mass, model, 0.01, 1500, Integrator::kSemiImplicitEuler);
  EXPECT_FALSE(IsFinite(end.angular_velocity) &&
               std::isfinite(Length(end.orientation)));
  // A finer step tames it.
  const RigidBodyState fine = Simulate(state, mass, model, 0.001, 15000,
                                       Integrator::kSemiImplicitEuler);
  EXPECT_TRUE(IsFinite(fine.angular_velocity));
}

TEST(IntegratorTest, TorqueFreeSpinConservesMomentumAndEnergy) {
  // A box spinning mostly about its intermediate axis tumbles (the
  // Dzhanibekov effect) while angular momentum and energy stay constant.
  const MassProperties mass = BoxMassProperties(2.0, {0.3, 1.0, 1.8});
  ForceModel model;
  model.gravity_enabled = false;
  RigidBodyState state;
  state.angular_velocity = {0.02, 4.0, 0.01};
  const Vec3 l0 = AngularMomentum(state, mass);
  const double e0 = KineticEnergy(state, mass);
  const RigidBodyState end =
      Simulate(state, mass, model, 1.0 / 240.0, 4800, Integrator::kRungeKutta4);
  EXPECT_VEC3_NEAR(AngularMomentum(end, mass), l0, 1e-4 * Length(l0));
  EXPECT_NEAR(KineticEnergy(end, mass), e0, 1e-4 * e0);
}

TEST(IntegratorTest, IntermediateAxisSpinFlips) {
  const MassProperties mass = BoxMassProperties(2.0, {0.3, 1.0, 1.8});
  ForceModel model;
  model.gravity_enabled = false;
  RigidBodyState state;
  state.angular_velocity = {0.02, 4.0, 0.01};
  // Track the body Y axis: it should swing to point (nearly) opposite.
  double min_alignment = 1.0;
  double age = 0.0;
  for (int i = 0; i < 2400; ++i) {
    state =
        Step(state, mass, model, age, 1.0 / 240.0, Integrator::kRungeKutta4);
    age += 1.0 / 240.0;
    min_alignment =
        std::min(min_alignment, Rotate(state.orientation, Vec3::UnitY()).y);
  }
  EXPECT_LT(min_alignment, -0.9);
}

}  // namespace
}  // namespace aerolab::sim
