#include "sim/flight_control.h"

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

constexpr double kDt = 1.0 / 120.0;

double Course(const Vec3& v) { return std::atan2(-v.x, -v.z); }
double Tilt(const Quat& q) {
  return std::acos(std::clamp(Rotate(q, Vec3::UnitY()).y, -1.0, 1.0));
}

// A quadrotor flown by its controller in closed loop.
struct QuadSim {
  MultirotorParams params = GenericQuadrotor();
  MultirotorState motors;
  RigidBodyState body;
  QuadController controller;
  ControlLimits limits;
  Vec3 wind;
  double max_tilt_seen = 0.0;

  QuadSim() {
    body.position = {0.0, 50.0, 0.0};
    motors.rotor_speed.fill(HoverRotorSpeed(params));
  }

  void Run(const FlightTarget& target, double seconds) {
    const int steps = static_cast<int>(std::lround(seconds / kDt));
    for (int i = 0; i < steps; ++i) {
      const RotorArray command =
          controller.Update(params, limits, target, body, wind, kDt);
      AdvanceMotors(params, command, kDt, motors);
      const MultirotorState frozen = motors;
      body = StepRk4(
          body, params.mass,
          [&](const RigidBodyState& s, double) {
            return MultirotorWrench(params, frozen, s, wind);
          },
          0.0, kDt);
      max_tilt_seen = std::max(max_tilt_seen, Tilt(body.orientation));
      ASSERT_TRUE(IsFinite(body.position));
    }
  }
};

// The trainer airplane flown by its autopilot in closed loop.
struct WingSim {
  FixedWingParams params = GenericTrainer();
  FixedWingActuators actuators;
  RigidBodyState body;
  WingAutopilot autopilot;
  ControlLimits limits;
  Vec3 wind;
  double max_bank_seen = 0.0;
  double max_alpha_seen = 0.0;
  double max_beta_seen = 0.0;
  double min_airspeed_seen = 1e9;

  explicit WingSim(double speed = 14.0) {
    body.position = {0.0, 120.0, 0.0};
    body.velocity = {0.0, 0.0, -speed};
    body.orientation =
        FromEuler({.yaw = 0.0, .pitch = Radians(1.5), .roll = 0.0});
    actuators.throttle = autopilot.trim_throttle;
  }

  void Run(const FlightTarget& target, double seconds) {
    const int steps = static_cast<int>(std::lround(seconds / kDt));
    for (int i = 0; i < steps; ++i) {
      const FixedWingCommand command =
          autopilot.Update(params, limits, target, body, wind, kDt);
      AdvanceActuators(params, command, kDt, actuators);
      const FixedWingActuators frozen = actuators;
      body = StepRk4(
          body, params.mass,
          [&](const RigidBodyState& s, double) {
            return FixedWingWrench(params, frozen, s, wind);
          },
          0.0, kDt);
      const AirData air =
          ComputeAirData(body, wind, StandardAtmosphere(body.position.y));
      max_bank_seen =
          std::max(max_bank_seen, std::abs(AttitudeOf(body.orientation).roll));
      max_alpha_seen = std::max(max_alpha_seen, air.alpha);
      max_beta_seen = std::max(max_beta_seen, std::abs(air.beta));
      min_airspeed_seen = std::min(min_airspeed_seen, air.airspeed);
      ASSERT_TRUE(IsFinite(body.position));
    }
  }
};

TEST(PidTest, CombinesTermsAndClampsTheIntegral) {
  Pid pid{.kp = 2.0, .ki = 1.0, .kd = 0.5, .integral_limit = 0.3};
  EXPECT_DOUBLE_EQ(pid.Update(1.0, 0.0, 0.1), 2.0 + 0.1);
  EXPECT_DOUBLE_EQ(pid.Update(0.0, 2.0, 0.1), 0.1 - 1.0);
  for (int i = 0; i < 100; ++i) pid.Update(10.0, 0.0, 0.1);
  EXPECT_DOUBLE_EQ(pid.integral, 0.3);
  pid.Reset();
  EXPECT_DOUBLE_EQ(pid.integral, 0.0);
}

TEST(AttitudeTest, ReadsRollPitchAndHeading) {
  const AttitudeAngles level = AttitudeOf(Quat::Identity());
  EXPECT_NEAR(level.roll, 0.0, 1e-12);
  EXPECT_NEAR(level.pitch, 0.0, 1e-12);
  EXPECT_NEAR(level.heading, 0.0, 1e-12);
  const AttitudeAngles a = AttitudeOf(
      FromEuler({.yaw = Radians(30.0), .pitch = Radians(10.0), .roll = 0.0}));
  EXPECT_NEAR(a.heading, Radians(30.0), 1e-9);
  EXPECT_NEAR(a.pitch, Radians(10.0), 1e-9);
  // Right wing down is positive roll.
  const Quat right_wing_down =
      FromAxisAngle(Vec3::UnitZ(), Radians(-20.0));  // About the tail axis.
  EXPECT_NEAR(AttitudeOf(right_wing_down).roll, Radians(20.0), 1e-9);
}

TEST(QuadControllerTest, HoversInPlace) {
  QuadSim sim;
  const Vec3 start = sim.body.position;
  sim.Run({}, 10.0);
  EXPECT_LT(Length(sim.body.position - start), 0.1);
  EXPECT_LT(Length(sim.body.velocity), 0.02);
  EXPECT_LT(Tilt(sim.body.orientation), Radians(0.5));
}

TEST(QuadControllerTest, FliesAtTheCommandedVelocityWithinItsTiltLimit) {
  QuadSim sim;
  sim.limits.max_tilt = Radians(25.0);
  const FlightTarget target{.velocity = {6.0, 1.0, -4.0}};
  sim.Run(target, 8.0);
  EXPECT_LT(Length(sim.body.velocity - target.velocity), 0.2);
  EXPECT_LT(sim.max_tilt_seen, Radians(27.0));
  // Nose toward the direction of travel.
  EXPECT_NEAR(AttitudeOf(sim.body.orientation).heading, Course(target.velocity),
              Radians(3.0));
}

TEST(QuadControllerTest, StopsFromCruiseWithoutOvershootingBadly) {
  QuadSim sim;
  sim.Run({.velocity = {0.0, 0.0, -10.0}}, 6.0);
  ASSERT_NEAR(sim.body.velocity.z, -10.0, 0.3);
  const double braking_from = sim.body.position.z;
  sim.Run({}, 6.0);
  EXPECT_LT(Length(sim.body.velocity), 0.1);
  // It needs room to stop, but drifts back little once stopped.
  EXPECT_LT(sim.body.position.z, braking_from);
  EXPECT_LT(braking_from - sim.body.position.z, 25.0);
}

TEST(QuadControllerTest, HoldsAHeadingWhileHovering) {
  QuadSim sim;
  sim.Run({.velocity = Vec3::Zero(),
           .heading = Radians(120.0),
           .hold_heading = true},
          6.0);
  EXPECT_NEAR(AttitudeOf(sim.body.orientation).heading, Radians(120.0),
              Radians(1.0));
  EXPECT_LT(Length(sim.body.velocity), 0.05);
}

TEST(QuadControllerTest, LeansIntoWindToHoldStill) {
  QuadSim sim;
  sim.wind = {4.0, 0.0, 0.0};
  sim.Run({}, 12.0);
  EXPECT_LT(Length(sim.body.velocity), 0.05);
  // Holding still in a wind from -X means tilting into it (toward -X).
  EXPECT_LT(Rotate(sim.body.orientation, Vec3::UnitY()).x, -0.01);
}

TEST(WingAutopilotTest, HoldsLevelCruise) {
  WingSim sim;
  const double start_altitude = sim.body.position.y;
  sim.Run({.velocity = {0.0, 0.0, -15.0}}, 30.0);
  EXPECT_NEAR(Length(sim.body.velocity), 15.0, 0.5);
  EXPECT_NEAR(sim.body.position.y, start_altitude, 3.0);
  EXPECT_NEAR(Course(sim.body.velocity), 0.0, Radians(2.0));
  EXPECT_LT(sim.max_bank_seen, Radians(5.0));
}

TEST(WingAutopilotTest, BanksToTurnOntoANewCourse) {
  WingSim sim;
  sim.limits.max_tilt = Radians(35.0);
  const double start_altitude = sim.body.position.y;
  // Turn 90 degrees left, then 90 degrees right of the original course.
  sim.Run({.velocity = {-15.0, 0.0, 0.0}}, 20.0);
  EXPECT_NEAR(Course(sim.body.velocity), Radians(90.0), Radians(3.0));
  sim.Run({.velocity = {15.0, 0.0, 0.0}}, 30.0);
  EXPECT_NEAR(Course(sim.body.velocity), Radians(-90.0), Radians(3.0));
  EXPECT_GT(sim.max_bank_seen, Radians(15.0));  // It really banked...
  EXPECT_LT(sim.max_bank_seen, Radians(38.0));  // ...within its limit.
  EXPECT_NEAR(sim.body.position.y, start_altitude, 10.0);
}

TEST(WingAutopilotTest, SteadyTurnsAreCoordinated) {
  WingSim sim;
  sim.limits.max_tilt = Radians(30.0);
  // A 150-degree course change left: long enough to sit at the bank limit.
  const FlightTarget target{.velocity = {-7.5, 0.0, 13.0}};
  sim.Run(target, 2.5);  // Roll in.
  sim.max_beta_seen = 0.0;
  double turned = 0.0;
  double predicted = 0.0;
  constexpr int kSamples = 8;
  constexpr double kInterval = 0.25;
  for (int i = 0; i < kSamples; ++i) {
    const double course = Course(sim.body.velocity);
    // A level coordinated turn: rate = g tan(bank) / speed (left turns
    // have negative bank).
    predicted +=
        kStandardGravity * std::tan(-AttitudeOf(sim.body.orientation).roll) /
        std::hypot(sim.body.velocity.x, sim.body.velocity.z) * kInterval;
    sim.Run(target, kInterval);
    turned += std::remainder(Course(sim.body.velocity) - course, kTwoPi);
  }
  EXPECT_NEAR(AttitudeOf(sim.body.orientation).roll, Radians(-30.0),
              Radians(2.0));
  EXPECT_NEAR(turned, predicted, 0.05 * predicted);
  EXPECT_LT(sim.max_beta_seen, Radians(2.0));  // No sideslip.
}

TEST(WingAutopilotTest, ClimbsAndDescendsAtTheRequestedRate) {
  WingSim sim;
  sim.Run({.velocity = {0.0, 2.0, -15.0}}, 15.0);
  EXPECT_NEAR(sim.body.velocity.y, 2.0, 0.4);
  sim.Run({.velocity = {0.0, -2.0, -15.0}}, 15.0);
  EXPECT_NEAR(sim.body.velocity.y, -2.0, 0.4);
}

TEST(WingAutopilotTest, KeepsFlyingWhenAskedToGoTooSlowly) {
  WingSim sim;
  sim.Run({.velocity = {0.0, 0.0, -2.0}}, 30.0);
  // It holds its minimum airspeed instead of stalling.
  EXPECT_GT(sim.min_airspeed_seen, 0.85 * sim.limits.min_airspeed);
  EXPECT_LT(sim.max_alpha_seen, GenericTrainer().aero.alpha_stall);
}

TEST(WingAutopilotTest, CrabsIntoACrosswind) {
  WingSim sim;
  sim.wind = {4.0, 0.0, 0.0};
  sim.Run({.velocity = {0.0, 0.0, -15.0}}, 40.0);
  // The autopilot steers the ground track it was asked for; to do it in a
  // crosswind the nose points partly into the wind.
  EXPECT_NEAR(Course(sim.body.velocity), 0.0, Radians(4.0));
  EXPECT_GT(AttitudeOf(sim.body.orientation).heading, Radians(5.0));
}

}  // namespace
}  // namespace aerolab::sim
