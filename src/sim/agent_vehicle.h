#ifndef AEROLAB_SIM_AGENT_VEHICLE_H_
#define AEROLAB_SIM_AGENT_VEHICLE_H_

#include <optional>
#include <span>

#include "math/vec3.h"
#include "sim/fixed_wing.h"
#include "sim/flight_agent.h"
#include "sim/flight_control.h"
#include "sim/multirotor.h"
#include "sim/rigid_body.h"

namespace aerolab::sim {

// The physical airframe behind an autonomous agent that uses
// FlightModel::kPhysical: a generic quadrotor for hover agents, a generic
// trainer airplane for wing agents, their actuator state and autopilots.
//
// The agent's brain decides where to go exactly as with the simple model;
// the autopilot only moves actuators (rotor speeds; throttle, elevator,
// ailerons, rudder) and the airframe's forces and moments move the rigid
// body. Nothing is scripted: the flight path is whatever the physics does.
struct AgentVehicle {
  MultirotorParams quad = GenericQuadrotor();
  FixedWingParams wing = GenericTrainer();
  MultirotorState motors;
  FixedWingActuators actuators;
  QuadController quad_autopilot;
  WingAutopilot wing_autopilot;
  // The wing's stall speed at sea level, cached by ResetVehicle for wing
  // agents (0 otherwise; read it through SeaLevelStallSpeed).
  double wing_stall_speed = 0.0;
};

// DefaultAgentParams(style) with the physical model and a flight envelope
// that suits the generic airframes: the trainer stalls near 7.6 m/s and
// can reach about 20 m/s, so it cruises at 12-15 m/s (18 at most) with
// a gentle 2 m/s climb or sink and a 100 m area to turn in; the quadrotor
// manages about 11 m/s at the default 28 degree tilt.
AgentParams PhysicalAgentParams(FlightStyle style);

// Mass and inertia of the airframe that flies `style`.
const MassProperties& AirframeMass(FlightStyle style,
                                   const AgentVehicle& vehicle);

// The wing's stall speed at sea level (the cached value when there is
// one).
double SeaLevelStallSpeed(const AgentVehicle& vehicle);

// The limits the autopilot flies within: the agent's envelope, narrowed to
// what the airframe can do (a wing never slower than 1.3 times its stall
// speed at `altitude`).
ControlLimits AutopilotLimits(const AgentParams& params,
                              const AgentVehicle& vehicle, double altitude);

// Puts the actuators at trim for steady flight (rotors at hover speed;
// cruise throttle and centred surfaces) and clears the autopilots' memory.
void ResetVehicle(FlightStyle style, AgentVehicle& vehicle);

// Advances one agent flying a physical airframe by `dt` seconds through air
// moving at `wind`: the brain plans, the autopilot sets the actuators and
// the rigid body is integrated (RK4, sub-stepped to at most 1/120 s, or
// into 64 equal parts for steps longer than 64/120 s). `acceleration`
// receives the body's mean linear acceleration over the step. A failed
// integration (a non-finite state) is returned as is, for the caller to
// detect and stop the body. Returns the decision made, if any.
std::optional<AgentDecision> StepPhysicalAgent(
    const AgentParams& params, std::span<const AgentObstacle> obstacles,
    const Vec3& wind, double dt, AgentBrain& brain, AgentVehicle& vehicle,
    RigidBodyState& state, Vec3& acceleration);

}  // namespace aerolab::sim

#endif  // AEROLAB_SIM_AGENT_VEHICLE_H_
