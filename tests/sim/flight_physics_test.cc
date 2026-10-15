#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "math/quat.h"
#include "math/scalar.h"
#include "math/vec3.h"
#include "sim/aerodynamics.h"
#include "sim/atmosphere.h"
#include "sim/fixed_wing.h"
#include "sim/force_model.h"
#include "sim/integrator.h"
#include "sim/multirotor.h"
#include "sim/rigid_body.h"

namespace aerolab::sim {
namespace {

// A body flying level along -Z at `speed`, nose pitched up by `pitch`.
RigidBodyState LevelFlight(double speed, double pitch, double altitude = 100) {
  RigidBodyState state;
  state.position = {0.0, altitude, 0.0};
  state.velocity = {0.0, 0.0, -speed};
  state.orientation = FromEuler({.yaw = 0.0, .pitch = pitch, .roll = 0.0});
  return state;
}

// Torque in aviation axes (roll, pitch, yaw) from a world-frame torque.
Vec3 AviationTorque(const RigidBodyState& state, const Vec3& torque) {
  const Vec3 body = Rotate(Conjugate(state.orientation), torque);
  return {-body.z, body.x, -body.y};
}

TEST(AtmosphereTest, MatchesStandardTables) {
  const Atmosphere sea = StandardAtmosphere(0.0);
  EXPECT_NEAR(sea.temperature, 288.15, 1e-9);
  EXPECT_NEAR(sea.pressure, 101325.0, 1e-6);
  EXPECT_NEAR(sea.density, 1.225, 1e-3);
  EXPECT_NEAR(sea.speed_of_sound, 340.29, 0.05);
  const Atmosphere tropopause = StandardAtmosphere(11000.0);
  EXPECT_NEAR(tropopause.temperature, 216.65, 1e-9);
  EXPECT_NEAR(tropopause.pressure, 22632.0, 10.0);
  EXPECT_NEAR(tropopause.density, 0.3639, 1e-3);
  EXPECT_NEAR(StandardAtmosphere(15000.0).density, 0.1948, 2e-3);
  // Thinner as you climb; clamped outside the model.
  EXPECT_LT(StandardAtmosphere(2000.0).density, sea.density);
  EXPECT_EQ(StandardAtmosphere(-50.0).density, sea.density);
  EXPECT_EQ(StandardAtmosphere(std::nan("")).density, sea.density);
}

TEST(AerodynamicsTest, AirDataAnglesFollowTheFlow) {
  const Atmosphere air = StandardAtmosphere(0.0);
  const AirData nose_up =
      ComputeAirData(LevelFlight(14.0, Radians(3.0), 0.0), Vec3::Zero(), air);
  EXPECT_NEAR(nose_up.airspeed, 14.0, 1e-12);
  EXPECT_NEAR(nose_up.alpha, Radians(3.0), 1e-9);
  EXPECT_NEAR(nose_up.beta, 0.0, 1e-12);
  EXPECT_NEAR(nose_up.dynamic_pressure, 0.5 * air.density * 196.0, 1e-9);
  // Wind blowing toward -X means the body moves toward +X relative to the
  // air: the flow meets it from the right, so sideslip is positive.
  const AirData crosswind =
      ComputeAirData(LevelFlight(14.0, 0.0, 0.0), Vec3{-3.0, 0.0, 0.0}, air);
  EXPECT_GT(crosswind.beta, 0.0);
}

TEST(AerodynamicsTest, LiftIsLinearThenStalls) {
  const AeroCoefficients aero;
  EXPECT_NEAR(LiftCoefficient(aero, 0.0), aero.cl0, 1e-5);
  EXPECT_NEAR(LiftCoefficient(aero, Radians(5.0)),
              aero.cl0 + aero.cl_alpha * Radians(5.0), 1e-3);
  // Lift peaks around the stall angle and falls away after it.
  double best_alpha = 0.0;
  double best_lift = 0.0;
  for (double deg = 0.0; deg <= 40.0; deg += 0.25) {
    const double cl = LiftCoefficient(aero, Radians(deg));
    if (cl > best_lift) {
      best_lift = cl;
      best_alpha = deg;
    }
  }
  EXPECT_NEAR(best_alpha, 14.0, 2.5);
  EXPECT_GT(best_lift, 1.15);
  EXPECT_LT(LiftCoefficient(aero, Radians(25.0)), 0.8 * best_lift);
  // Drag grows with lift, then jumps once the flow separates.
  EXPECT_GT(DragCoefficient(aero, Radians(8.0)), DragCoefficient(aero, 0.0));
  EXPECT_GT(DragCoefficient(aero, Radians(30.0)), 0.4);
}

TEST(AerodynamicsTest, MomentsHaveTheRightSigns) {
  const AeroCoefficients aero;
  const Atmosphere air = StandardAtmosphere(100.0);
  const auto torque = [&](const RigidBodyState& state,
                          const ControlSurfaces& surfaces) {
    return AviationTorque(
        state,
        ComputeAeroWrench(aero, state, ComputeAirData(state, Vec3::Zero(), air),
                          surfaces)
            .torque);
  };
  const RigidBodyState level = LevelFlight(14.0, Radians(2.0));
  const Vec3 base = torque(level, {});
  // Control surfaces do what their names say.
  EXPECT_LT(torque(level, {.elevator = 0.1}).y, base.y);  // Nose down.
  EXPECT_GT(torque(level, {.aileron = 0.1}).x, base.x);   // Roll right.
  EXPECT_GT(torque(level, {.rudder = 0.1}).z, base.z);    // Nose right.
  // Static pitch stability: more angle of attack pitches the nose down.
  EXPECT_LT(torque(LevelFlight(14.0, Radians(8.0)), {}).y, base.y);
  // Roll damping opposes a roll rate.
  RigidBodyState rolling = level;
  rolling.angular_velocity = Rotate(level.orientation, Vec3{0.0, 0.0, -1.0});
  EXPECT_LT(torque(rolling, {}).x, base.x);
  // Weathervane: air from the right yaws the nose right, into it.
  RigidBodyState slipping = level;
  slipping.velocity += Vec3{2.0, 0.0, 0.0};
  EXPECT_GT(torque(slipping, {}).z, base.z);
}

TEST(FixedWingTest, TrimsAtASmallAngleOfAttackWithARealisticLiftToDrag) {
  const FixedWingParams plane = GenericTrainer();
  const double weight = plane.mass.mass * kStandardGravity;
  // In level flight lift is vertical and drag horizontal: find the angle
  // of attack at which lift carries the weight at 14 m/s (bisection).
  const auto vertical = [&](double alpha) {
    return FixedWingWrench(plane, FixedWingActuators{},
                           LevelFlight(14.0, alpha, 0.0), Vec3::Zero())
        .force.y;
  };
  double low = Radians(-4.0);
  double high = Radians(10.0);
  ASSERT_LT(vertical(low), 0.0);
  ASSERT_GT(vertical(high), 0.0);
  for (int i = 0; i < 60; ++i) {
    const double mid = 0.5 * (low + high);
    (vertical(mid) < 0.0 ? low : high) = mid;
  }
  const double trim = 0.5 * (low + high);
  EXPECT_NEAR(vertical(trim), 0.0, 1e-6 * weight);
  EXPECT_GT(trim, Radians(0.5));
  EXPECT_LT(trim, Radians(3.0));
  // Lift-to-drag at trim is about 10, as for a real trainer.
  // (It flies toward -Z, so drag pushes toward +Z.)
  const double drag =
      FixedWingWrench(plane, FixedWingActuators{}, LevelFlight(14.0, trim, 0.0),
                      Vec3::Zero())
          .force.z;
  EXPECT_GT(weight / drag, 8.0);
  EXPECT_LT(weight / drag, 13.0);
}

TEST(FixedWingTest, PropellerThrustFadesWithAirspeed) {
  const FixedWingParams plane = GenericTrainer();
  EXPECT_NEAR(PropellerThrust(plane, 1.0, 0.0), plane.static_thrust, 1e-12);
  EXPECT_NEAR(PropellerThrust(plane, 0.5, 0.0), 0.5 * plane.static_thrust,
              1e-12);
  EXPECT_EQ(PropellerThrust(plane, 1.0, plane.prop_pitch_speed), 0.0);
  EXPECT_EQ(PropellerThrust(plane, 1.0, 2.0 * plane.prop_pitch_speed), 0.0);
}

TEST(FixedWingTest, ActuatorsLagAndRespectLimits) {
  const FixedWingParams plane = GenericTrainer();
  FixedWingActuators actuators;
  FixedWingCommand command;
  command.throttle = 1.0;
  command.surfaces.elevator = 1.0;  // Beyond the limit.
  AdvanceActuators(plane, command, plane.throttle_time_constant, actuators);
  EXPECT_NEAR(actuators.throttle, 1.0 - std::exp(-1.0), 1e-9);
  // Servo rate: 300 deg/s for 0.15 s = 45 deg, but the limit is 25 deg.
  EXPECT_NEAR(actuators.surfaces.elevator, plane.surface_limit, 1e-12);
  FixedWingActuators slow;
  AdvanceActuators(plane, command, 0.01, slow);
  EXPECT_NEAR(slow.surfaces.elevator, plane.surface_rate * 0.01, 1e-12);
}

TEST(FixedWingTest, UnpoweredTrainerSettlesIntoARealisticGlide) {
  const FixedWingParams plane = GenericTrainer();
  const FixedWingActuators idle;  // No throttle, surfaces neutral.
  RigidBodyState state = LevelFlight(14.0, Radians(1.4), 500.0);
  const WrenchFunction wrench = [&](const RigidBodyState& s, double) {
    return FixedWingWrench(plane, idle, s, Vec3::Zero());
  };
  const double dt = 1.0 / 120.0;
  for (int i = 0; i < 60 * 120; ++i) {
    state = StepRk4(state, plane.mass, wrench, i * dt, dt);
    ASSERT_TRUE(IsFinite(state.position));
  }
  // A stable airframe with no power trades height for speed at a shallow
  // glide angle (lift-to-drag around 10) and holds its wings level.
  const double speed = Length(state.velocity);
  const double glide = std::atan2(
      -state.velocity.y, std::hypot(state.velocity.x, state.velocity.z));
  EXPECT_GT(speed, 10.0);
  EXPECT_LT(speed, 17.0);
  EXPECT_GT(glide, Radians(4.5));
  EXPECT_LT(glide, Radians(6.5));
  EXPECT_GT(Rotate(state.orientation, Vec3::UnitY()).y, 0.95);
}

TEST(MultirotorTest, HoverSpeedBalancesTheWeight) {
  const MultirotorParams quad = GenericQuadrotor();
  MultirotorState state;
  state.rotor_speed.fill(HoverRotorSpeed(quad));
  EXPECT_NEAR(HoverRotorSpeed(quad), 800.0, 25.0);
  RigidBodyState body;  // Level, at rest, at sea level.
  const Wrench w = MultirotorWrench(quad, state, body, Vec3::Zero());
  // (The standard atmosphere's sea-level density differs from the nominal
  // 1.225 kg/m^3 in the seventh digit.)
  EXPECT_NEAR(Length(w.force), 0.0, 1e-5);
  EXPECT_NEAR(Length(w.torque), 0.0, 1e-9);  // Spins cancel in the X layout.
}

TEST(MultirotorTest, MixerProducesTheRequestedThrustAndTorque) {
  const MultirotorParams quad = GenericQuadrotor();
  RigidBodyState body;
  const double weight = quad.mass.mass * kStandardGravity;
  const Vec3 request{0.15, -0.02, -0.1};  // Pitch up, yaw right, roll.
  MultirotorState state;
  state.rotor_speed = MixRotorSpeeds(quad, 1.3 * weight, request);
  const Wrench w = MultirotorWrench(quad, state, body, Vec3::Zero());
  EXPECT_NEAR(w.force.y, 0.3 * weight, 1e-5);  // Thrust minus gravity.
  EXPECT_NEAR(w.torque.x, request.x, 1e-6);
  EXPECT_NEAR(w.torque.y, request.y, 1e-6);
  EXPECT_NEAR(w.torque.z, request.z, 1e-6);
}

TEST(MultirotorTest, MixerGivesUpYawBeforeAttitude) {
  const MultirotorParams quad = GenericQuadrotor();
  RigidBodyState body;
  const double weight = quad.mass.mass * kStandardGravity;
  MultirotorState state;
  // Far more yaw than the motors can make, plus a modest roll.
  state.rotor_speed = MixRotorSpeeds(quad, weight, {0.0, 5.0, 0.2});
  const Wrench w = MultirotorWrench(quad, state, body, Vec3::Zero());
  EXPECT_NEAR(w.torque.z, 0.2, 1e-6);  // Roll kept.
  EXPECT_NEAR(w.force.y, 0.0, 1e-5);   // Thrust kept.
  // Yaw is cut back to what fits: the same direction, much less of it, with
  // a motor at the end of its range.
  EXPECT_GT(w.torque.y, 0.01);
  EXPECT_LT(w.torque.y, 0.5);
  double slowest = quad.max_rotor_speed;
  for (double speed : state.rotor_speed) {
    EXPECT_GE(speed, 0.0);
    EXPECT_LE(speed, quad.max_rotor_speed + 1e-9);
    slowest = std::min(slowest, speed);
  }
  EXPECT_NEAR(slowest, 0.0, 1e-6);
}

TEST(MultirotorTest, MotorsLagAndDragOpposesMotion) {
  const MultirotorParams quad = GenericQuadrotor();
  MultirotorState state;
  RotorArray full;
  full.fill(quad.max_rotor_speed);
  AdvanceMotors(quad, full, quad.motor_time_constant, state);
  EXPECT_NEAR(state.rotor_speed[0],
              quad.max_rotor_speed * (1.0 - std::exp(-1.0)), 1e-9);
  RigidBodyState moving;
  moving.velocity = {10.0, 0.0, 0.0};
  const Wrench w =
      MultirotorWrench(quad, MultirotorState{}, moving, Vec3::Zero());
  EXPECT_LT(w.force.x, 0.0);
  EXPECT_NEAR(w.force.x,
              -(quad.linear_drag * 10.0 + quad.quadratic_drag * 100.0), 1e-6);
}

}  // namespace
}  // namespace aerolab::sim
