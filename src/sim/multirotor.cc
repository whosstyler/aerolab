#include "sim/multirotor.h"

#include <algorithm>
#include <cmath>

#include "math/quat.h"
#include "sim/atmosphere.h"

namespace aerolab::sim {
namespace {

// Per-rotor thrusts (N) for a collective thrust and body torque, before any
// limits. Derivation: with d = arm / sqrt(2) and c = k_Q / k_T,
//   T  = f0 + f1 + f2 + f3
//   tx = d (f0 + f1 - f2 - f3)         (front rotors lift: nose up)
//   tz = d (f0 - f1 - f2 + f3)         (right rotors lift)
//   ty = c (-f0 + f1 - f2 + f3)        (reaction of the spins)
RotorArray AllocateThrusts(double d, double c, double collective,
                           const Vec3& torque) {
  const double t = collective / 4.0;
  const double x = torque.x / (4.0 * d);
  const double z = torque.z / (4.0 * d);
  const double y = torque.y / (4.0 * c);
  return {t + x + z - y, t + x - z + y, t - x - z - y, t - x + z + y};
}

double MinOf(const RotorArray& a) {
  return *std::min_element(a.begin(), a.end());
}
double MaxOf(const RotorArray& a) {
  return *std::max_element(a.begin(), a.end());
}

}  // namespace

MultirotorParams GenericQuadrotor() {
  MultirotorParams params;
  params.mass.mass = 1.2;
  // Engine body axes: x = pitch, y = yaw, z = roll.
  params.mass.principal_inertia = {0.012, 0.022, 0.012};
  return params;
}

Vec3 RotorPosition(const MultirotorParams& params, int rotor) {
  const double d = std::abs(params.arm_length) / std::sqrt(2.0);
  switch (rotor) {
    case 0:
      return {d, 0.0, -d};  // Front-right.
    case 1:
      return {-d, 0.0, -d};  // Front-left.
    case 2:
      return {-d, 0.0, d};  // Rear-left.
    default:
      return {d, 0.0, d};  // Rear-right.
  }
}

double RotorSpin(int rotor) { return rotor % 2 == 0 ? 1.0 : -1.0; }

void AdvanceMotors(const MultirotorParams& params, const RotorArray& commanded,
                   double dt, MultirotorState& state) {
  if (!(dt > 0.0)) return;
  const double tau = std::max(params.motor_time_constant, 1e-4);
  const double blend = 1.0 - std::exp(-dt / tau);
  const double top = std::max(params.max_rotor_speed, 0.0);
  for (int i = 0; i < kRotorCount; ++i) {
    const double target = std::isfinite(commanded[i])
                              ? std::clamp(commanded[i], 0.0, top)
                              : state.rotor_speed[i];
    state.rotor_speed[i] += (target - state.rotor_speed[i]) * blend;
  }
}

double HoverRotorSpeed(const MultirotorParams& params) {
  const double per_rotor = params.mass.mass * kStandardGravity / kRotorCount;
  return std::sqrt(per_rotor / std::max(params.thrust_coefficient, 1e-12));
}

Wrench MultirotorWrench(const MultirotorParams& params,
                        const MultirotorState& state,
                        const RigidBodyState& body, const Vec3& wind) {
  const Atmosphere atmosphere = StandardAtmosphere(body.position.y);
  const double density_ratio = atmosphere.density / kSeaLevelDensity;
  Vec3 force_body;
  Vec3 torque_body;
  for (int i = 0; i < kRotorCount; ++i) {
    const double w2 = state.rotor_speed[i] * state.rotor_speed[i];
    const double thrust = params.thrust_coefficient * w2 * density_ratio;
    const Vec3 r = RotorPosition(params, i);
    force_body.y += thrust;
    // r x (0, f, 0) = (-r.z f, 0, r.x f).
    torque_body += Vec3{-r.z * thrust, 0.0, r.x * thrust};
    // The motor turns the rotor one way and the body the other.
    torque_body.y -=
        RotorSpin(i) * params.torque_coefficient * w2 * density_ratio;
  }
  Wrench total{Rotate(body.orientation, force_body),
               Rotate(body.orientation, torque_body)};

  // Body drag against the flow through the air.
  const Vec3 airflow = body.velocity - wind;
  const double speed = Length(airflow);
  if (speed > 1e-9) {
    const double drag = params.linear_drag * speed +
                        params.quadratic_drag * density_ratio * speed * speed;
    total.force -= airflow * (drag / speed);
  }
  total.force += Vec3{0.0, -kStandardGravity * params.mass.mass, 0.0};
  return total;
}

RotorArray MixRotorSpeeds(const MultirotorParams& params,
                          double collective_thrust, const Vec3& body_torque) {
  const double k_t = std::max(params.thrust_coefficient, 1e-12);
  const double d = std::max(std::abs(params.arm_length) / std::sqrt(2.0), 1e-6);
  const double c = std::max(params.torque_coefficient / k_t, 1e-9);
  const double top = k_t * params.max_rotor_speed * params.max_rotor_speed;
  const double collective =
      std::isfinite(collective_thrust) ? std::max(collective_thrust, 0.0) : 0.0;
  const Vec3 torque = IsFinite(body_torque) ? body_torque : Vec3::Zero();

  RotorArray thrust = AllocateThrusts(d, c, collective, torque);
  const auto fits = [&](const RotorArray& f) {
    return MinOf(f) >= 0.0 && MaxOf(f) <= top;
  };
  if (!fits(thrust)) {
    // Give up yaw first (it is the weakest and least urgent axis): keep as
    // much of it as fits around the thrust and roll/pitch demands.
    const RotorArray base =
        AllocateThrusts(d, c, collective, {torque.x, 0.0, torque.z});
    double keep = 1.0;
    for (int i = 0; i < kRotorCount; ++i) {
      const double yaw_part = thrust[i] - base[i];
      if (yaw_part > 0.0) keep = std::min(keep, (top - base[i]) / yaw_part);
      if (yaw_part < 0.0) keep = std::min(keep, -base[i] / yaw_part);
    }
    keep = std::max(keep, 0.0);
    thrust = AllocateThrusts(d, c, collective,
                             {torque.x, torque.y * keep, torque.z});
  }
  if (!fits(thrust)) {
    // Keep attitude: shift every rotor together (changing the collective),
    // and if the spread itself is too wide, scale the torques down.
    const double spread = MaxOf(thrust) - MinOf(thrust);
    if (spread > top) {
      const double scale = top / spread;
      thrust = AllocateThrusts(d, c, collective,
                               {torque.x * scale, 0.0, torque.z * scale});
    }
    if (MinOf(thrust) < 0.0) {
      const double lift = -MinOf(thrust);
      for (double& f : thrust) f += lift;
    }
    if (MaxOf(thrust) > top) {
      const double drop = MaxOf(thrust) - top;
      for (double& f : thrust) f -= drop;
    }
  }
  RotorArray speeds{};
  for (int i = 0; i < kRotorCount; ++i) {
    speeds[i] = std::sqrt(std::clamp(thrust[i], 0.0, top) / k_t);
  }
  return speeds;
}

}  // namespace aerolab::sim
