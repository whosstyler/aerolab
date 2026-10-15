#include "sim/agent_vehicle.h"

#include <algorithm>
#include <cmath>

#include "math/scalar.h"
#include "sim/atmosphere.h"
#include "sim/heading.h"
#include "sim/integrator.h"

namespace aerolab::sim {
namespace {

// Longest physics step: the attitude loops and motor lag need a fine step.
constexpr double kMaxPhysicsStep = 1.0 / 120.0;
constexpr int kMaxSubsteps = 64;
// Margin over the stall speed the autopilot never flies below.
constexpr double kStallMargin = 1.3;

bool IsWing(const AgentParams& p) { return p.style == FlightStyle::kWing; }

// What the RK4 wrench callbacks need, captured through a single pointer so
// the std::function never has to allocate.
struct WingFlight {
  const FixedWingParams* params;
  FixedWingActuators actuators;
  Vec3 wind;
};
struct QuadFlight {
  const MultirotorParams* params;
  MultirotorState motors;
  Vec3 wind;
};

AgentMotion MotionOf(const RigidBodyState& state, const Vec3& acceleration) {
  return {.position = state.position,
          .velocity = state.velocity,
          .acceleration = acceleration,
          .orientation = state.orientation};
}

}  // namespace

AgentParams PhysicalAgentParams(FlightStyle style) {
  AgentParams params = DefaultAgentParams(style);
  params.model = FlightModel::kPhysical;
  if (style == FlightStyle::kWing) {
    params.area.radius = 100.0;
    params.min_speed = 10.5;
    params.cruise_speed_min = 12.0;
    params.cruise_speed_max = 15.0;
    params.max_speed = 18.0;
    params.max_tilt = Radians(35.0);
    // Gentle enough that the airframe can sink at idle without speeding up.
    params.max_climb_rate = 2.0;
  }
  return params;
}

const MassProperties& AirframeMass(FlightStyle style,
                                   const AgentVehicle& vehicle) {
  return style == FlightStyle::kWing ? vehicle.wing.mass : vehicle.quad.mass;
}

ControlLimits AutopilotLimits(const AgentParams& params,
                              const AgentVehicle& vehicle, double altitude) {
  ControlLimits limits;
  limits.max_tilt = params.max_tilt;
  limits.max_climb_rate = params.max_climb_rate;
  limits.max_acceleration = params.max_acceleration;
  if (params.style == FlightStyle::kWing) {
    // Stall speed grows as the air thins: v_s ~ 1 / sqrt(density).
    const double stall =
        SeaLevelStallSpeed(vehicle) *
        std::sqrt(kSeaLevelDensity / StandardAtmosphere(altitude).density);
    limits.min_airspeed = std::max(params.min_speed, kStallMargin * stall);
    limits.max_airspeed = std::max(params.max_speed, limits.min_airspeed);
  } else {
    limits.min_airspeed = 0.0;
    limits.max_airspeed = params.max_speed;
  }
  return limits;
}

double SeaLevelStallSpeed(const AgentVehicle& vehicle) {
  return vehicle.wing_stall_speed > 0.0
             ? vehicle.wing_stall_speed
             : StallSpeed(vehicle.wing, kSeaLevelDensity);
}

void ResetVehicle(FlightStyle style, AgentVehicle& vehicle) {
  vehicle.quad_autopilot.Reset();
  vehicle.wing_autopilot.Reset();
  vehicle.motors.rotor_speed.fill(
      style == FlightStyle::kHover ? HoverRotorSpeed(vehicle.quad) : 0.0);
  vehicle.actuators = FixedWingActuators{};
  vehicle.wing_stall_speed = 0.0;
  if (style == FlightStyle::kWing) {
    vehicle.wing_stall_speed = StallSpeed(vehicle.wing, kSeaLevelDensity);
    vehicle.actuators.throttle = vehicle.wing_autopilot.trim_throttle;
  }
}

std::optional<AgentDecision> StepPhysicalAgent(
    const AgentParams& params, std::span<const AgentObstacle> obstacles,
    const Vec3& wind, double dt, AgentBrain& brain, AgentVehicle& vehicle,
    RigidBodyState& state, Vec3& acceleration) {
  if (!(dt > 0.0) || !std::isfinite(dt)) return std::nullopt;
  const AgentParams p = SanitizeAgentParams(params);
  const Vec3 air_motion = IsFinite(wind) ? wind : Vec3::Zero();
  const std::optional<AgentDecision> decision =
      PlanAgent(p, obstacles, dt, MotionOf(state, acceleration), brain);

  FlightTarget target{.velocity = brain.desired_velocity,
                      .heading = 0.0,
                      .hold_heading = false,
                      .turn_rate = brain.desired_turn_rate};
  if (!IsWing(p)) {
    // Hover craft face where the brain says (its travel direction, or a
    // slow scan while hovering), turning at the agent's yaw rate.
    target.heading = NextHoverHeading(p, state.velocity, dt, brain);
    target.hold_heading = true;
  }
  const ControlLimits limits = AutopilotLimits(p, vehicle, state.position.y);

  const Vec3 start_velocity = state.velocity;
  // Counted in double first, so an absurd dt cannot overflow the int.
  const int substeps = static_cast<int>(std::clamp(
      std::ceil(dt / kMaxPhysicsStep - 1e-9), 1.0, double{kMaxSubsteps}));
  const double h = dt / substeps;
  for (int i = 0; i < substeps; ++i) {
    if (IsWing(p)) {
      const FixedWingCommand command = vehicle.wing_autopilot.Update(
          vehicle.wing, limits, target, state, air_motion, h);
      AdvanceActuators(vehicle.wing, command, h, vehicle.actuators);
      const WingFlight flight{&vehicle.wing, vehicle.actuators, air_motion};
      state = StepRk4(
          state, vehicle.wing.mass,
          [&flight](const RigidBodyState& s, double) {
            return FixedWingWrench(*flight.params, flight.actuators, s,
                                   flight.wind);
          },
          0.0, h);
    } else {
      const RotorArray command = vehicle.quad_autopilot.Update(
          vehicle.quad, limits, target, state, air_motion, h);
      AdvanceMotors(vehicle.quad, command, h, vehicle.motors);
      const QuadFlight flight{&vehicle.quad, vehicle.motors, air_motion};
      state = StepRk4(
          state, vehicle.quad.mass,
          [&flight](const RigidBodyState& s, double) {
            return MultirotorWrench(*flight.params, flight.motors, s,
                                    flight.wind);
          },
          0.0, h);
    }
    // The ground plane is solid (the altitude band normally keeps agents
    // well above it).
    if (state.position.y < 0.0) {
      state.position.y = 0.0;
      state.velocity.y = std::max(state.velocity.y, 0.0);
    }
  }
  acceleration = (state.velocity - start_velocity) / dt;
  if (IsWing(p) && HorizontalLength(state.velocity) > 0.5) {
    brain.heading = HeadingOf(state.velocity);
  }
  return decision;
}

}  // namespace aerolab::sim
