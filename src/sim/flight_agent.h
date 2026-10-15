#ifndef AEROLAB_SIM_FLIGHT_AGENT_H_
#define AEROLAB_SIM_FLIGHT_AGENT_H_

#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

#include "math/quat.h"
#include "math/random.h"
#include "math/scalar.h"
#include "math/vec3.h"

namespace aerolab::sim {

// Game-style autonomous flight for generic aircraft.
//
// Each step an agent senses (its own state plus nearby obstacles), decides
// (a small state machine that picks its own destinations with a scored
// random search) and steers (Reynolds-style steering behaviours: arrive,
// meander, stay-in-area, altitude band, obstacle avoidance). The
// resulting acceleration is limited by a simple flight envelope and
// integrated as a point mass with a fixed step.
//
// Agents only ever head for points they chose themselves inside their own
// flight area; other objects are only ever kept away from.

enum class FlightStyle {
  kHover,  // Multirotor-like: can stop and hover; tilts to accelerate.
  kWing,   // Airplane-like: minimum airspeed; banks to turn.
};

std::string_view FlightStyleName(FlightStyle style);

enum class AgentMode {
  kTransit,  // Flying to the current destination.
  kLoiter,   // Hovering in place (kHover) or circling (kWing).
};

std::string_view AgentModeName(AgentMode mode);

// The volume an agent picks its destinations in and tries to stay inside.
struct FlightArea {
  Vec3 center;                 // Only x and z are used.
  double radius = 35.0;        // Horizontal radius, m.
  double min_altitude = 4.0;   // m above the ground plane.
  double max_altitude = 25.0;  // m
};

struct AgentParams {
  FlightStyle style = FlightStyle::kHover;
  FlightArea area;

  // Speed envelope, m/s. Each leg picks its own cruise speed uniformly in
  // [cruise_speed_min, cruise_speed_max]. Wings never fly below min_speed.
  double min_speed = 0.0;
  double cruise_speed_min = 3.0;
  double cruise_speed_max = 8.0;
  double max_speed = 11.0;

  double max_acceleration = 5.0;        // m/s^2 along and across the path.
  double max_tilt = Radians(28.0);      // Hover tilt or wing bank limit.
  double max_climb_rate = 3.0;          // m/s, up or down.
  double max_yaw_rate = Radians(90.0);  // Hover heading changes, rad/s.

  // After reaching a destination the agent loiters with this probability,
  // for a duration drawn from [loiter_min, loiter_max] seconds.
  double loiter_chance = 0.6;
  double loiter_min = 2.0;
  double loiter_max = 6.0;

  double meander = 0.35;     // 0..1: how much the heading wanders en route.
  double body_radius = 0.8;  // m, the agent's own size for avoidance.
  double clearance = 2.0;    // m kept between the agent and obstacles.
  double lookahead = 2.5;    // s of travel scanned for obstacles.

  std::uint64_t seed = 1;
};

// Sensible defaults for each flight style.
AgentParams DefaultAgentParams(FlightStyle style);

// `params` with every value made finite and clamped to a sane range (bank
// at least 1 degree, look-ahead 0.1-60 s, coordinates within 1e6 m, ...),
// inverted ranges swapped and non-finite values replaced by the style's
// defaults. StepAgent applies this itself, so any parameters are safe.
AgentParams SanitizeAgentParams(const AgentParams& params);

// Radius of the tightest level turn at `speed` (infinite when the agent
// may not bank; sanitized parameters always allow some bank).
double MinTurnRadius(const AgentParams& params, double speed);

// Something the agent must keep away from, as a bounding sphere. Moving
// obstacles report their velocity, so the agent can judge where they will
// be rather than where they are.
struct AgentObstacle {
  Vec3 center;
  double radius = 0.0;
  Vec3 velocity;  // m/s; zero for static objects.
};

// What the avoidance layer reacted to during the latest step.
struct AvoidanceStatus {
  bool altitude = false;  // Correcting back into its altitude band.
  bool boundary = false;  // Turning back toward its flight area.
  // Index into the obstacle span passed to StepAgent, or -1.
  int obstacle = -1;
};

// The agent's kinematic state.
struct AgentMotion {
  Vec3 position;
  Vec3 velocity;
  Vec3 acceleration;
  Quat orientation;  // -Z forward, +Y up, as everywhere in the engine.
};

// Decision-making state. Copying a brain (including its random generator)
// and stepping the copy replays exactly what the original will do, given
// the same obstacles.
struct AgentBrain {
  Rng rng;
  AgentMode mode = AgentMode::kTransit;
  bool has_destination = false;
  Vec3 destination;
  double leg_speed = 0.0;       // Cruise speed chosen for this leg, m/s.
  double mode_time = 0.0;       // Seconds spent in the current mode.
  double mode_duration = 0.0;   // Loiter length or transit time limit, s.
  Vec3 loiter_center;           // Circling centre (kWing loiter).
  double loiter_radius = 0.0;   // Circling radius, m (kWing loiter).
  double turn_direction = 1.0;  // +1 counter-clockwise seen from above.
  double heading = 0.0;         // Hover yaw, rad (0 faces -Z).
  double wander_angle = 0.0;    // Current meander offset, rad.
  double wander_rate = 0.0;     // Its rate of change, rad/s.
  double avoid_side = 0.0;      // Side it is passing an obstacle on: +1 left.
  int occupied_streak = 0;      // Consecutive plans whose spot was taken.
  int legs_completed = 0;

  // Diagnostics of the latest step, for the UI and debug drawing.
  Vec3 desired_velocity;
  AvoidanceStatus avoidance;
};

// A decision worth reporting (e.g. to an event log).
struct AgentDecision {
  enum class Kind {
    kNewDestination,  // Picked a new destination and leg speed.
    kLoiter,          // Started hovering or circling.
    kReplan,          // Gave up on a destination (too slow to reach, taken,
                      // or no longer inside its area).
  };
  Kind kind = Kind::kNewDestination;
  Vec3 destination;
  double distance = 0.0;  // m to the destination when chosen.
  double speed = 0.0;     // Leg cruise speed, m/s.
  double duration = 0.0;  // Loiter length, s.
};

// Puts the brain back to its initial state for `params.seed`, facing the
// way `motion.orientation` points. The first destination is chosen on the
// next StepAgent call.
void ResetBrain(const AgentParams& params, const AgentMotion& motion,
                AgentBrain& brain);

// Drops the current plan (destination, loiter, avoidance side) so the next
// StepAgent call decides afresh, keeping the random stream and the leg
// count: unlike ResetBrain it does not replay earlier choices.
void Replan(AgentBrain& brain);

// Brings a launch state inside the agent's flight envelope: speed within
// its limits, climb rate within the climb limit; wings level along their
// track (or nose, when not moving) at no less than cruise speed, hover
// craft level. Non-finite velocities become zero; acceleration is zeroed.
// Use before the first step of a launch.
void FitToEnvelope(const AgentParams& params, AgentMotion& motion);

// Advances one agent by `dt` seconds: decide, steer, move. Returns the
// decision made during this step, if any.
std::optional<AgentDecision> StepAgent(const AgentParams& params,
                                       std::span<const AgentObstacle> obstacles,
                                       double dt, AgentBrain& brain,
                                       AgentMotion& motion);

}  // namespace aerolab::sim

#endif  // AEROLAB_SIM_FLIGHT_AGENT_H_
