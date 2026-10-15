#include "sim/integrator.h"

#include <string_view>

#include "math/quat.h"

namespace aerolab::sim {
namespace {

// Rotates `orientation` by the world-frame angular velocity `omega` held
// constant for `dt` seconds.
Quat AdvanceOrientation(const Quat& orientation, const Vec3& omega, double dt) {
  return Normalized(FromRotationVector(omega * dt) * orientation);
}

// state + derivative * h, used for the RK4 intermediate stages.
RigidBodyState Offset(const RigidBodyState& state, const StateDerivative& d,
                      double h) {
  RigidBodyState out;
  out.position = state.position + d.velocity * h;
  out.velocity = state.velocity + d.acceleration * h;
  out.orientation =
      AdvanceOrientation(state.orientation, d.angular_velocity, h);
  out.angular_velocity = state.angular_velocity + d.angular_acceleration * h;
  return out;
}

RigidBodyState ExplicitEuler(const RigidBodyState& s, const MassProperties& m,
                             const ForceModel& f, double age, double dt) {
  const StateDerivative d = Evaluate(s, m, f, age);
  return Offset(s, d, dt);
}

RigidBodyState SemiImplicitEuler(const RigidBodyState& s,
                                 const MassProperties& m, const ForceModel& f,
                                 double age, double dt) {
  const StateDerivative d = Evaluate(s, m, f, age);
  RigidBodyState out;
  out.velocity = s.velocity + d.acceleration * dt;
  out.position = s.position + out.velocity * dt;
  out.angular_velocity = s.angular_velocity + d.angular_acceleration * dt;
  out.orientation = AdvanceOrientation(s.orientation, out.angular_velocity, dt);
  return out;
}

RigidBodyState VelocityVerlet(const RigidBodyState& s, const MassProperties& m,
                              const ForceModel& f, double age, double dt) {
  const StateDerivative d0 = Evaluate(s, m, f, age);
  // Half kick, full drift.
  RigidBodyState half = s;
  half.velocity = s.velocity + d0.acceleration * (0.5 * dt);
  half.angular_velocity =
      s.angular_velocity + d0.angular_acceleration * (0.5 * dt);
  half.position = s.position + half.velocity * dt;
  half.orientation =
      AdvanceOrientation(s.orientation, half.angular_velocity, dt);

  // Evaluate the new acceleration at the drifted pose. Forces that depend
  // on velocity see a predicted end-of-step velocity.
  RigidBodyState predicted = half;
  predicted.velocity = s.velocity + d0.acceleration * dt;
  predicted.angular_velocity =
      s.angular_velocity + d0.angular_acceleration * dt;
  const StateDerivative d1 = Evaluate(predicted, m, f, age + dt);

  // Second half kick.
  RigidBodyState out = half;
  out.velocity = half.velocity + d1.acceleration * (0.5 * dt);
  out.angular_velocity =
      half.angular_velocity + d1.angular_acceleration * (0.5 * dt);
  return out;
}

RigidBodyState RungeKutta4(const RigidBodyState& s, const MassProperties& m,
                           const ForceModel& f, double age, double dt) {
  const double h = 0.5 * dt;
  const StateDerivative k1 = Evaluate(s, m, f, age);
  const StateDerivative k2 = Evaluate(Offset(s, k1, h), m, f, age + h);
  const StateDerivative k3 = Evaluate(Offset(s, k2, h), m, f, age + h);
  const StateDerivative k4 = Evaluate(Offset(s, k3, dt), m, f, age + dt);

  const auto blend = [](const Vec3& a, const Vec3& b, const Vec3& c,
                        const Vec3& d) {
    return (a + (b + c) * 2.0 + d) / 6.0;
  };
  StateDerivative weighted;
  weighted.velocity = blend(k1.velocity, k2.velocity, k3.velocity, k4.velocity);
  weighted.acceleration =
      blend(k1.acceleration, k2.acceleration, k3.acceleration, k4.acceleration);
  weighted.angular_velocity = blend(k1.angular_velocity, k2.angular_velocity,
                                    k3.angular_velocity, k4.angular_velocity);
  weighted.angular_acceleration =
      blend(k1.angular_acceleration, k2.angular_acceleration,
            k3.angular_acceleration, k4.angular_acceleration);
  return Offset(s, weighted, dt);
}

}  // namespace

std::string_view IntegratorName(Integrator integrator) {
  switch (integrator) {
    case Integrator::kExplicitEuler:
      return "Explicit Euler";
    case Integrator::kSemiImplicitEuler:
      return "Semi-implicit Euler";
    case Integrator::kVelocityVerlet:
      return "Velocity Verlet";
    case Integrator::kRungeKutta4:
      return "Runge-Kutta 4";
    case Integrator::kCount:
      break;
  }
  return "Unknown";
}

RigidBodyState Step(const RigidBodyState& state, const MassProperties& mass,
                    const ForceModel& model, double age, double dt,
                    Integrator integrator) {
  switch (integrator) {
    case Integrator::kExplicitEuler:
      return ExplicitEuler(state, mass, model, age, dt);
    case Integrator::kSemiImplicitEuler:
      return SemiImplicitEuler(state, mass, model, age, dt);
    case Integrator::kVelocityVerlet:
      return VelocityVerlet(state, mass, model, age, dt);
    case Integrator::kRungeKutta4:
    case Integrator::kCount:
      break;
  }
  return RungeKutta4(state, mass, model, age, dt);
}

}  // namespace aerolab::sim
