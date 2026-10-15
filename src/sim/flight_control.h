#ifndef AEROLAB_SIM_FLIGHT_CONTROL_H_
#define AEROLAB_SIM_FLIGHT_CONTROL_H_

#include "math/scalar.h"
#include "math/vec3.h"
#include "sim/fixed_wing.h"
#include "sim/multirotor.h"
#include "sim/rigid_body.h"

namespace aerolab::sim {

// Flight controllers that fly the physical vehicle models toward a wanted
// velocity, the way real autopilots do: they only move actuators (rotor
// speeds; throttle and control surfaces) and let the physics respond.

// Proportional-integral-derivative controller on a scalar error, with the
// integral clamped to stop wind-up and, optionally, only accumulated while
// the error is small (so large transients do not charge it). The derivative
// acts on the measurement (supplied by the caller as a rate) to avoid kicks
// on setpoint changes.
struct Pid {
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  double integral_limit = 0.0;  // |integral term| cap, output units
  double integral_zone = 0.0;   // Integrate only if |error| <= this (0: any)
  double integral = 0.0;        // Accumulated ki * error * dt

  // Returns kp e + integral - kd measured_rate and advances the integral.
  double Update(double error, double measured_rate, double dt);
  void Reset() { integral = 0.0; }
};

// Attitude angles of a body (aviation convention): roll positive right
// wing down, pitch positive nose up, heading as in the engine (0 faces -Z,
// positive turns left).
struct AttitudeAngles {
  double roll = 0.0;
  double pitch = 0.0;
  double heading = 0.0;
};

AttitudeAngles AttitudeOf(const Quat& orientation);

// What an autonomous pilot wants: a velocity in the world and, for hover
// craft, which way to face.
struct FlightTarget {
  Vec3 velocity;  // m/s, world
  double heading = 0.0;
  bool hold_heading = false;  // Hover craft: face `heading` (else travel).
  double turn_rate = 0.0;     // Wings: feed-forward turn rate, rad/s (+ left).
};

// Limits the controllers respect; filled from the agent's envelope.
struct ControlLimits {
  double max_tilt = Radians(30.0);  // Hover tilt or wing bank, rad
  double max_climb_rate = 3.0;      // m/s
  double max_acceleration = 5.0;    // m/s^2
  double min_airspeed = 9.0;        // Wings: stall margin, m/s
  double max_airspeed = 22.0;       // Wings, m/s
};

// Cascaded multirotor controller: velocity -> acceleration -> thrust
// vector -> attitude -> body rates -> torques -> rotor speeds (mixer).
struct QuadController {
  Pid velocity_x{.kp = 2.5,
                 .ki = 0.6,
                 .kd = 0.0,
                 .integral_limit = 2.0,
                 .integral_zone = 1.0};
  Pid velocity_y{.kp = 3.0,
                 .ki = 1.0,
                 .kd = 0.0,
                 .integral_limit = 3.0,
                 .integral_zone = 1.0};
  Pid velocity_z{.kp = 2.5,
                 .ki = 0.6,
                 .kd = 0.0,
                 .integral_limit = 2.0,
                 .integral_zone = 1.0};
  double attitude_gain = 9.0;  // Body-rate command per radian of error, 1/s
  double rate_gain = 28.0;     // Angular acceleration per rad/s of error, 1/s
  double max_body_rate = 6.0;  // rad/s

  void Reset();

  // Rotor speed commands for this step.
  RotorArray Update(const MultirotorParams& params, const ControlLimits& limits,
                    const FlightTarget& target, const RigidBodyState& state,
                    const Vec3& wind, double dt);
};

// Fixed-wing autopilot: course -> coordinated bank -> ailerons; climb rate
// -> pitch -> elevator; airspeed -> throttle; rudder keeps the turn
// coordinated (no sideslip).
struct WingAutopilot {
  double course_gain = 1.2;  // Turn rate per radian of course error, 1/s
  Pid airspeed{.kp = 0.12, .ki = 0.05, .kd = 0.0, .integral_limit = 0.4};
  Pid climb{.kp = 0.08, .ki = 0.03, .kd = 0.0, .integral_limit = 0.15};
  Pid roll{.kp = 1.2, .ki = 0.0, .kd = 0.15, .integral_limit = 0.0};
  Pid pitch{.kp = 1.6, .ki = 0.4, .kd = 0.25, .integral_limit = 0.15};
  double sideslip_gain = 1.5;  // Rudder per radian of sideslip
  double trim_throttle = 0.35;
  double trim_pitch = Radians(2.0);
  double climb_setpoint = 0.0;  // Climb rate being flown now, m/s

  void Reset();

  FixedWingCommand Update(const FixedWingParams& params,
                          const ControlLimits& limits,
                          const FlightTarget& target,
                          const RigidBodyState& state, const Vec3& wind,
                          double dt);
};

}  // namespace aerolab::sim

#endif  // AEROLAB_SIM_FLIGHT_CONTROL_H_
