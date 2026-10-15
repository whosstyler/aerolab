#include "sim/fixed_wing.h"

#include <algorithm>
#include <cmath>

#include "math/quat.h"
#include "sim/atmosphere.h"

namespace aerolab::sim {
namespace {

// Moves `value` toward `target` by at most `max_step`.
double SlewToward(double value, double target, double max_step) {
  return value + std::clamp(target - value, -max_step, max_step);
}

}  // namespace

FixedWingParams GenericTrainer() {
  FixedWingParams params;
  params.mass.mass = 2.0;
  // Engine body axes: x = pitch, y = yaw, z = roll.
  params.mass.principal_inertia = {0.08, 0.18, 0.12};
  return params;
}

void AdvanceActuators(const FixedWingParams& params,
                      const FixedWingCommand& command, double dt,
                      FixedWingActuators& actuators) {
  if (!(dt > 0.0)) return;
  const double throttle = std::isfinite(command.throttle)
                              ? std::clamp(command.throttle, 0.0, 1.0)
                              : 0.0;
  const double tau = std::max(params.throttle_time_constant, 1e-3);
  actuators.throttle +=
      (throttle - actuators.throttle) * (1.0 - std::exp(-dt / tau));

  const double limit = std::max(params.surface_limit, 0.0);
  const double step = std::max(params.surface_rate, 0.0) * dt;
  const auto servo = [&](double& position, double wanted) {
    const double target =
        std::isfinite(wanted) ? std::clamp(wanted, -limit, limit) : 0.0;
    position = SlewToward(position, target, step);
  };
  servo(actuators.surfaces.elevator, command.surfaces.elevator);
  servo(actuators.surfaces.aileron, command.surfaces.aileron);
  servo(actuators.surfaces.rudder, command.surfaces.rudder);
}

double StallSpeed(const FixedWingParams& params, double density) {
  // The lift curve peaks near the stall angle; search a little past it.
  double max_lift = 0.0;
  const double search_end = 2.0 * std::max(params.aero.alpha_stall, 0.01);
  for (int i = 0; i <= 400; ++i) {
    max_lift = std::max(max_lift,
                        LiftCoefficient(params.aero, search_end * (i / 400.0)));
  }
  const double lift_capacity = 0.5 * std::max(density, 1e-6) *
                               std::max(params.aero.area, 1e-6) *
                               std::max(max_lift, 1e-3);
  return std::sqrt(params.mass.mass * kStandardGravity / lift_capacity);
}

double PropellerThrust(const FixedWingParams& params, double throttle,
                       double airspeed) {
  const double pitch_speed = std::max(params.prop_pitch_speed, 1e-3);
  const double fraction = std::max(1.0 - airspeed / pitch_speed, 0.0);
  return std::max(params.static_thrust, 0.0) * std::clamp(throttle, 0.0, 1.0) *
         fraction;
}

Wrench FixedWingWrench(const FixedWingParams& params,
                       const FixedWingActuators& actuators,
                       const RigidBodyState& state, const Vec3& wind) {
  const Atmosphere atmosphere = StandardAtmosphere(state.position.y);
  const AirData air = ComputeAirData(state, wind, atmosphere);
  Wrench total = ComputeAeroWrench(params.aero, state, air, actuators.surfaces);
  // Propeller thrust along the nose, scaled by air density.
  const Vec3 nose = Rotate(state.orientation, -Vec3::UnitZ());
  const double density_ratio = atmosphere.density / kSeaLevelDensity;
  total.force +=
      nose * (PropellerThrust(params, actuators.throttle, air.airspeed) *
              density_ratio);
  total.force += Vec3{0.0, -kStandardGravity * params.mass.mass, 0.0};
  return total;
}

}  // namespace aerolab::sim
