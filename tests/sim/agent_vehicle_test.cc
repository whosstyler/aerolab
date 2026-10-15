#include "sim/agent_vehicle.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <span>
#include <vector>

#include "math/quat.h"
#include "math/random.h"
#include "math/scalar.h"
#include "math/vec3.h"
#include "sim/aerodynamics.h"
#include "sim/atmosphere.h"
#include "sim/flight_agent.h"
#include "sim/rigid_body.h"

namespace aerolab::sim {
namespace {

constexpr double kDt = 1.0 / 120.0;

double HorizontalDistance(const Vec3& a, const Vec3& b) {
  return std::hypot(a.x - b.x, a.z - b.z);
}

// One agent flying a physical airframe, with what it went through.
struct PhysicalFlight {
  AgentParams params;
  AgentBrain brain;
  AgentVehicle vehicle;
  RigidBodyState state;
  Vec3 acceleration;
  Vec3 wind;
  int decisions = 0;
  double max_tilt = 0.0;
  double max_alpha = -1e9;
  double min_airspeed = 1e9;
  double max_area_excess = 0.0;      // m beyond the area radius.
  double max_altitude_excess = 0.0;  // m outside the altitude band.

  void Step(std::span<const AgentObstacle> obstacles, double dt = kDt) {
    if (StepPhysicalAgent(params, obstacles, wind, dt, brain, vehicle, state,
                          acceleration)) {
      ++decisions;
    }
    const AirData air =
        ComputeAirData(state, wind, StandardAtmosphere(state.position.y));
    max_tilt = std::max(
        max_tilt, std::acos(std::clamp(
                      Rotate(state.orientation, Vec3::UnitY()).y, -1.0, 1.0)));
    max_alpha = std::max(max_alpha, air.alpha);
    min_airspeed = std::min(min_airspeed, air.airspeed);
    const FlightArea& area = params.area;
    max_area_excess =
        std::max(max_area_excess,
                 HorizontalDistance(state.position, area.center) - area.radius);
    max_altitude_excess =
        std::max({max_altitude_excess, area.min_altitude - state.position.y,
                  state.position.y - area.max_altitude});
  }

  void Run(double seconds, std::span<const AgentObstacle> obstacles = {}) {
    const int steps = static_cast<int>(std::lround(seconds / kDt));
    for (int i = 0; i < steps; ++i) {
      Step(obstacles);
      ASSERT_TRUE(IsFinite(state.position) && IsFinite(state.velocity));
    }
  }
};

PhysicalFlight MakePhysicalFlight(FlightStyle style, std::uint64_t seed,
                                  const Vec3& start = {0.0, 20.0, 0.0}) {
  PhysicalFlight flight;
  flight.params = PhysicalAgentParams(style);
  flight.params.seed = seed;
  flight.params.area.center = {start.x, 0.0, start.z};
  AgentMotion launch{.position = start,
                     .velocity = Vec3::Zero(),
                     .acceleration = Vec3::Zero(),
                     .orientation = Quat::Identity()};
  FitToEnvelope(flight.params, launch);
  flight.state.position = launch.position;
  flight.state.velocity = launch.velocity;
  flight.state.orientation = launch.orientation;
  ResetBrain(flight.params, launch, flight.brain);
  ResetVehicle(style, flight.vehicle);
  return flight;
}

TEST(AgentVehicleTest, PhysicalParamsSuitTheAirframes) {
  const AgentParams wing = PhysicalAgentParams(FlightStyle::kWing);
  EXPECT_EQ(wing.model, FlightModel::kPhysical);
  const AgentVehicle vehicle;
  const double stall = StallSpeed(vehicle.wing, kSeaLevelDensity);
  EXPECT_GT(stall, 7.0);
  EXPECT_LT(stall, 8.5);
  // Cruise comfortably above the stall, never below it.
  EXPECT_GT(wing.min_speed, 1.25 * stall);
  EXPECT_GT(wing.cruise_speed_min, wing.min_speed);
  EXPECT_EQ(PhysicalAgentParams(FlightStyle::kHover).model,
            FlightModel::kPhysical);
  // The autopilot's floor rises as the air thins.
  EXPECT_GT(AutopilotLimits(wing, vehicle, 4000.0).min_airspeed,
            AutopilotLimits(wing, vehicle, 0.0).min_airspeed);
}

TEST(AgentVehicleTest, ResetTrimsTheAirframe) {
  AgentVehicle vehicle;
  vehicle.motors.rotor_speed.fill(0.0);
  vehicle.quad_autopilot.velocity_x.integral = 1.0;
  ResetVehicle(FlightStyle::kHover, vehicle);
  for (double w : vehicle.motors.rotor_speed) {
    EXPECT_DOUBLE_EQ(w, HoverRotorSpeed(vehicle.quad));
  }
  EXPECT_EQ(vehicle.quad_autopilot.velocity_x.integral, 0.0);
  ResetVehicle(FlightStyle::kWing, vehicle);
  EXPECT_GT(vehicle.actuators.throttle, 0.0);
  EXPECT_GT(vehicle.wing_stall_speed, 0.0);
  EXPECT_EQ(AirframeMass(FlightStyle::kWing, vehicle).mass,
            vehicle.wing.mass.mass);
}

TEST(AgentVehicleTest, HoverAgentFliesItsLegsInsideItsArea) {
  for (std::uint64_t seed = 1; seed <= 3; ++seed) {
    PhysicalFlight flight = MakePhysicalFlight(FlightStyle::kHover, seed);
    flight.Run(150.0);
    EXPECT_GE(flight.brain.legs_completed, 4) << "seed " << seed;
    EXPECT_LT(flight.max_area_excess, 3.0) << "seed " << seed;
    EXPECT_LT(flight.max_altitude_excess, 2.0) << "seed " << seed;
    EXPECT_LT(flight.max_tilt, flight.params.max_tilt + Radians(3.0))
        << "seed " << seed;
  }
}

TEST(AgentVehicleTest, WingAgentFliesWithoutEverStalling) {
  for (std::uint64_t seed = 1; seed <= 3; ++seed) {
    PhysicalFlight flight = MakePhysicalFlight(FlightStyle::kWing, seed);
    flight.Run(240.0);
    EXPECT_GE(flight.brain.legs_completed, 3) << "seed " << seed;
    EXPECT_LT(flight.max_alpha, flight.vehicle.wing.aero.alpha_stall)
        << "seed " << seed;
    EXPECT_GT(flight.min_airspeed,
              1.1 * StallSpeed(flight.vehicle.wing, kSeaLevelDensity))
        << "seed " << seed;
    // Real turns need room: a little overshoot of the edge is allowed.
    EXPECT_LT(flight.max_area_excess, 0.3 * flight.params.area.radius)
        << "seed " << seed;
    EXPECT_LT(flight.max_altitude_excess, 6.0) << "seed " << seed;
    EXPECT_LT(flight.max_tilt, flight.params.max_tilt + Radians(8.0))
        << "seed " << seed;
  }
}

TEST(AgentVehicleTest, SameSeedReplaysExactly) {
  for (FlightStyle style : {FlightStyle::kHover, FlightStyle::kWing}) {
    PhysicalFlight a = MakePhysicalFlight(style, 9);
    PhysicalFlight b = MakePhysicalFlight(style, 9);
    a.Run(30.0);
    b.Run(30.0);
    EXPECT_EQ(a.state.position, b.state.position);
    EXPECT_EQ(a.state.orientation, b.state.orientation);
  }
}

TEST(AgentVehicleTest, CopiedAgentForecastsTheFuture) {
  PhysicalFlight flight = MakePhysicalFlight(FlightStyle::kWing, 4);
  flight.Run(10.0);
  PhysicalFlight forecast = flight;
  forecast.Run(10.0);
  flight.Run(10.0);
  EXPECT_EQ(forecast.state.position, flight.state.position);
}

TEST(AgentVehicleTest, CoarseStepsAreSubSteppedAndStayStable) {
  for (FlightStyle style : {FlightStyle::kHover, FlightStyle::kWing}) {
    PhysicalFlight flight = MakePhysicalFlight(style, 3);
    for (int i = 0; i < 60 * 30; ++i) {
      flight.Step({}, 1.0 / 30.0);
      ASSERT_TRUE(IsFinite(flight.state.position));
    }
    EXPECT_LT(flight.max_altitude_excess, 6.0);
    EXPECT_GE(flight.brain.legs_completed, 2);
  }
}

TEST(AgentVehicleTest, HoverAgentHoldsItsAreaInAStrongWind) {
  PhysicalFlight flight = MakePhysicalFlight(FlightStyle::kHover, 6);
  flight.wind = {6.0, 0.0, 2.0};
  flight.Run(120.0);
  EXPECT_LT(flight.max_area_excess, 4.0);
  EXPECT_LT(flight.max_altitude_excess, 2.0);
  EXPECT_GE(flight.brain.legs_completed, 3);
}

TEST(AgentVehicleTest, WandersAmongObstaclesWithoutTouchingAny) {
  for (FlightStyle style : {FlightStyle::kHover, FlightStyle::kWing}) {
    for (std::uint64_t seed = 200; seed < 203; ++seed) {
      PhysicalFlight flight = MakePhysicalFlight(style, seed);
      const FlightArea& area = flight.params.area;
      Rng layout(seed * 7);
      std::vector<AgentObstacle> rocks;
      while (rocks.size() < 5) {
        const double angle = layout.Uniform(0.0, kTwoPi);
        const double r = area.radius * 0.8 * std::sqrt(layout.Uniform());
        const AgentObstacle rock{
            {r * std::cos(angle),
             layout.Uniform(area.min_altitude, area.max_altitude),
             r * std::sin(angle)},
            layout.Uniform(2.0, 6.0),
            {}};
        if (Distance(rock.center, flight.state.position) > rock.radius + 15.0) {
          rocks.push_back(rock);
        }
      }
      double closest_gap = 1e9;
      const int steps = static_cast<int>(240.0 / kDt);
      for (int i = 0; i < steps; ++i) {
        flight.Step(rocks);
        for (const AgentObstacle& rock : rocks) {
          closest_gap = std::min(closest_gap,
                                 Distance(flight.state.position, rock.center) -
                                     rock.radius - flight.params.body_radius);
        }
      }
      EXPECT_GT(closest_gap, 0.0) << FlightStyleName(style) << " seed " << seed;
      EXPECT_GE(flight.brain.legs_completed, 2)
          << FlightStyleName(style) << " seed " << seed;
    }
  }
}

TEST(AgentVehicleTest, TwoHoverAgentsSharingAnAreaNeverTouch) {
  PhysicalFlight a = MakePhysicalFlight(FlightStyle::kHover, 51);
  PhysicalFlight b =
      MakePhysicalFlight(FlightStyle::kHover, 52, {4.0, 20.0, 0.0});
  b.params.area = a.params.area;
  b.params.area.radius = a.params.area.radius = 15.0;  // Crowded.
  double closest = 1e9;
  const int steps = static_cast<int>(240.0 / kDt);
  for (int i = 0; i < steps; ++i) {
    const std::array<AgentObstacle, 1> sees_b = {AgentObstacle{
        b.state.position, b.params.body_radius, b.state.velocity}};
    const std::array<AgentObstacle, 1> sees_a = {AgentObstacle{
        a.state.position, a.params.body_radius, a.state.velocity}};
    a.Step(sees_b);
    b.Step(sees_a);
    closest = std::min(closest, Distance(a.state.position, b.state.position));
  }
  EXPECT_GT(closest, a.params.body_radius + b.params.body_radius);
}

TEST(AgentVehicleTest, NonPositiveStepDoesNothing) {
  PhysicalFlight flight = MakePhysicalFlight(FlightStyle::kHover, 1);
  const RigidBodyState before = flight.state;
  EXPECT_FALSE(StepPhysicalAgent(flight.params, {}, {}, 0.0, flight.brain,
                                 flight.vehicle, flight.state,
                                 flight.acceleration));
  EXPECT_FALSE(StepPhysicalAgent(flight.params, {}, {}, std::nan(""),
                                 flight.brain, flight.vehicle, flight.state,
                                 flight.acceleration));
  EXPECT_EQ(flight.state.position, before.position);
}

}  // namespace
}  // namespace aerolab::sim
