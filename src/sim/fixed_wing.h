#ifndef AEROLAB_SIM_FIXED_WING_H_
#define AEROLAB_SIM_FIXED_WING_H_

#include "math/scalar.h"
#include "math/vec3.h"
#include "sim/aerodynamics.h"
#include "sim/force_model.h"
#include "sim/rigid_body.h"

namespace aerolab::sim {

// A small propeller-driven airplane: aerodynamics, a propeller along the
// nose, gravity, and actuators that respond with realistic lag. Defaults
// (GenericTrainer) describe a ~2 kg, 1.8 m span trainer-style UAV that
// cruises at about 14 m/s.
struct FixedWingParams {
  // Principal inertia in engine body axes: x = pitch, y = yaw, z = roll.
  MassProperties mass;
  AeroCoefficients aero;
  double static_thrust = 12.0;     // N at full throttle and zero airspeed
  double prop_pitch_speed = 30.0;  // m/s; thrust falls linearly to zero here
  double throttle_time_constant = 0.15;  // s (first-order lag)
  double surface_limit = Radians(25.0);  // Maximum deflection, rad
  double surface_rate = Radians(300.0);  // Servo slew rate, rad/s
};

FixedWingParams GenericTrainer();

// What the pilot or autopilot asks for. Throttle is in [0, 1].
struct FixedWingCommand {
  double throttle = 0.0;
  ControlSurfaces surfaces;
};

// Where the actuators actually are (they lag behind the command).
struct FixedWingActuators {
  double throttle = 0.0;
  ControlSurfaces surfaces;
};

// Moves the actuators toward `command` for `dt` seconds: throttle with a
// first-order lag, surfaces at most at the servo rate, all within limits.
void AdvanceActuators(const FixedWingParams& params,
                      const FixedWingCommand& command, double dt,
                      FixedWingActuators& actuators);

// Slowest level flight the wing can sustain in air of `density`: the speed
// at which its maximum lift coefficient just carries the weight.
double StallSpeed(const FixedWingParams& params, double density);

// Propeller thrust (N) at an airspeed and throttle setting.
double PropellerThrust(const FixedWingParams& params, double throttle,
                       double airspeed);

// Total force and moment on the airplane, world frame: aerodynamics (air
// density from the standard atmosphere at the current altitude, flow
// relative to `wind`), propeller thrust along the nose and gravity.
Wrench FixedWingWrench(const FixedWingParams& params,
                       const FixedWingActuators& actuators,
                       const RigidBodyState& state, const Vec3& wind);

}  // namespace aerolab::sim

#endif  // AEROLAB_SIM_FIXED_WING_H_
