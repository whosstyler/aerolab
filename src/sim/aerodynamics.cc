#include "sim/aerodynamics.h"

#include <algorithm>
#include <cmath>

#include "math/quat.h"

namespace aerolab::sim {
namespace {

// How sharply the attached-flow model hands over to the flat plate around
// the stall angle, 1/rad (a few degrees wide).
constexpr double kStallSharpness = 100.0;
// Below this airspeed the airframe feels no meaningful aerodynamic force.
constexpr double kMinAirspeed = 0.1;  // m/s
// Floor for the speed used to non-dimensionalize rates, so damping terms
// stay bounded when the body is nearly still in the air.
constexpr double kMinRateSpeed = 1.0;  // m/s

// Logistic step from 0 to 1; saturates cleanly when exp overflows.
double Sigmoid(double x) { return 1.0 / (1.0 + std::exp(-x)); }

// 0 well below the stall angle, 1 well beyond it (in either direction):
// one minus the product of two logistic steps, one at each stall angle.
double StallBlend(const AeroCoefficients& aero, double alpha) {
  const double stall = std::max(aero.alpha_stall, 1e-3);
  return 1.0 - Sigmoid(kStallSharpness * (stall - alpha)) *
                   Sigmoid(kStallSharpness * (alpha + stall));
}

// Lift of an attached-flow wing, linear in angle of attack.
double LinearLift(const AeroCoefficients& aero, double alpha) {
  return aero.cl0 + aero.cl_alpha * alpha;
}

// Flat plate: what is left of lift and drag once the flow has separated.
double FlatPlateLift(double alpha) {
  const double s = std::sin(alpha);
  return 2.0 * (alpha >= 0.0 ? 1.0 : -1.0) * s * s * std::cos(alpha);
}
double FlatPlateDrag(double alpha) {
  const double s = std::sin(alpha);
  return 2.0 * s * s;
}

struct LiftDrag {
  double lift = 0.0;
  double drag = 0.0;
};

// Both coefficients from one evaluation of the stall blend.
LiftDrag LiftAndDrag(const AeroCoefficients& aero, double alpha) {
  const double blend = StallBlend(aero, alpha);
  const double linear = LinearLift(aero, alpha);
  const double attached_drag = aero.cd0 + aero.induced_drag * linear * linear;
  return {(1.0 - blend) * linear + blend * FlatPlateLift(alpha),
          (1.0 - blend) * attached_drag +
              blend * (aero.cd0 + FlatPlateDrag(alpha))};
}

}  // namespace

AirData ComputeAirData(const RigidBodyState& state, const Vec3& wind,
                       const Atmosphere& atmosphere) {
  AirData air;
  air.relative_wind = state.velocity - wind;
  air.airspeed = Length(air.relative_wind);
  air.density = atmosphere.density;
  air.dynamic_pressure = 0.5 * air.density * air.airspeed * air.airspeed;
  air.mach = air.airspeed / std::max(atmosphere.speed_of_sound, 1.0);
  if (air.airspeed < 1e-9) return air;
  // Velocity through the air in aviation body axes (x fwd, y right, z down).
  const Vec3 body = Rotate(Conjugate(state.orientation), air.relative_wind);
  const double u = -body.z;
  const double v = body.x;
  const double w = -body.y;
  air.alpha = std::atan2(w, u);
  air.beta = std::asin(std::clamp(v / air.airspeed, -1.0, 1.0));
  return air;
}

double LiftCoefficient(const AeroCoefficients& aero, double alpha) {
  return LiftAndDrag(aero, alpha).lift;
}

double DragCoefficient(const AeroCoefficients& aero, double alpha) {
  return LiftAndDrag(aero, alpha).drag;
}

Wrench ComputeAeroWrench(const AeroCoefficients& aero,
                         const RigidBodyState& state, const AirData& air,
                         const ControlSurfaces& surfaces) {
  if (air.airspeed < kMinAirspeed) return Wrench{};
  const double qs = air.dynamic_pressure * aero.area;
  const double alpha = air.alpha;
  const double beta = air.beta;

  // Body rates in aviation axes and their non-dimensional forms.
  const Vec3 omega =
      Rotate(Conjugate(state.orientation), state.angular_velocity);
  const double p = -omega.z;  // Roll, right wing down.
  const double q = omega.x;   // Pitch, nose up.
  const double r = -omega.y;  // Yaw, nose right.
  const double rate_speed = 2.0 * std::max(air.airspeed, kMinRateSpeed);
  const double p_hat = p * aero.span / rate_speed;
  const double q_hat = q * aero.chord / rate_speed;
  const double r_hat = r * aero.span / rate_speed;

  // Lift and drag in the plane of symmetry, side force across it.
  const LiftDrag coefficients = LiftAndDrag(aero, alpha);
  const double lift =
      qs * (coefficients.lift + aero.cl_elevator * surfaces.elevator);
  const double drag = qs * coefficients.drag;
  const double side =
      qs * (aero.cy_beta * beta + aero.cy_rudder * surfaces.rudder);
  const double ca = std::cos(alpha);
  const double sa = std::sin(alpha);
  const double fx = -drag * ca + lift * sa;
  const double fz = -drag * sa - lift * ca;

  // Moments about the aviation axes.
  const double roll_moment =
      qs * aero.span *
      (aero.cl_beta * beta + aero.cl_p * p_hat + aero.cl_r * r_hat +
       aero.cl_aileron * surfaces.aileron);
  const double pitch_moment =
      qs * aero.chord *
      (aero.cm0 + aero.cm_alpha * alpha + aero.cm_q * q_hat +
       aero.cm_elevator * surfaces.elevator);
  const double yaw_moment =
      qs * aero.span *
      (aero.cn_beta * beta + aero.cn_r * r_hat + aero.cn_p * p_hat +
       aero.cn_rudder * surfaces.rudder);

  // Aviation (x fwd, y right, z down) -> engine body (x right, y up, z back).
  const Vec3 force_body{side, -fz, -fx};
  const Vec3 torque_body{pitch_moment, -yaw_moment, -roll_moment};
  return Wrench{Rotate(state.orientation, force_body),
                Rotate(state.orientation, torque_body)};
}

}  // namespace aerolab::sim
