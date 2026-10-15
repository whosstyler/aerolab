#include "sim/force_model.h"

#include <algorithm>
#include <cmath>

#include "math/quat.h"

namespace aerolab::sim {
namespace {

// Guards divisions by inertia components and mass.
constexpr double kMinMass = 1e-6;

Vec3 SafeInertia(const MassProperties& mass) {
  return Max(mass.principal_inertia, Vec3{kMinMass, kMinMass, kMinMass});
}

}  // namespace

ForceBreakdown ComputeForces(const RigidBodyState& state,
                             const MassProperties& mass,
                             const ForceModel& model, double age) {
  ForceBreakdown f;
  const double m = std::max(mass.mass, kMinMass);
  const Vec3 forward = Rotate(state.orientation, -Vec3::UnitZ());
  const Vec3 up = Rotate(state.orientation, Vec3::UnitY());
  const Vec3 airspeed = state.velocity - model.wind;
  const double airspeed_length = Length(airspeed);

  if (model.gravity_enabled) f.gravity = model.gravity * m;

  f.drag = -airspeed * model.linear_drag -
           airspeed * (model.quadratic_drag * airspeed_length);

  if (model.thrust != 0.0 && age < model.thrust_duration) {
    f.thrust = forward * model.thrust;
  }

  if (model.lift_coefficient != 0.0) {
    const double forward_speed = std::max(Dot(airspeed, forward), 0.0);
    f.lift = up * (model.lift_coefficient * forward_speed * forward_speed);
  }

  if (model.spring_enabled) {
    f.spring =
        -(state.position - model.spring_anchor) * model.spring_stiffness -
        state.velocity * model.spring_damping;
  }

  if (model.central_enabled) {
    const Vec3 r = state.position - model.central_point;
    const double softened =
        LengthSquared(r) + model.central_softening * model.central_softening;
    f.central = -r * (model.central_mu * m / (softened * std::sqrt(softened)));
  }

  f.total_force = f.gravity + f.drag + f.thrust + f.lift + f.spring + f.central;

  Vec3 torque = Rotate(state.orientation, model.body_torque);
  torque -= state.angular_velocity * model.angular_damping;
  if (model.weathervane != 0.0) {
    torque += Cross(forward, airspeed) * (model.weathervane * airspeed_length);
  }
  f.total_torque = torque;
  return f;
}

StateDerivative Evaluate(const RigidBodyState& state,
                         const MassProperties& mass, const ForceModel& model,
                         double age) {
  const ForceBreakdown forces = ComputeForces(state, mass, model, age);
  StateDerivative d;
  d.velocity = state.velocity;
  d.acceleration = forces.total_force / std::max(mass.mass, kMinMass);
  d.angular_velocity = state.angular_velocity;

  // Euler's equations are simplest in body axes, where the inertia tensor is
  // diagonal: I w_b' = T_b - w_b x (I w_b). Because d/dt(R w_b) = R w_b'
  // (the R' term is w x w = 0), the result rotates straight back to world.
  const Quat to_body = Conjugate(state.orientation);
  const Vec3 inertia = SafeInertia(mass);
  const Vec3 omega_body = Rotate(to_body, state.angular_velocity);
  const Vec3 torque_body = Rotate(to_body, forces.total_torque);
  const Vec3 momentum_body = Hadamard(inertia, omega_body);
  const Vec3 rhs = torque_body - Cross(omega_body, momentum_body);
  const Vec3 alpha_body{rhs.x / inertia.x, rhs.y / inertia.y,
                        rhs.z / inertia.z};
  d.angular_acceleration = Rotate(state.orientation, alpha_body);
  return d;
}

double KineticEnergy(const RigidBodyState& state, const MassProperties& mass) {
  const Vec3 omega_body =
      Rotate(Conjugate(state.orientation), state.angular_velocity);
  return 0.5 * mass.mass * LengthSquared(state.velocity) +
         0.5 * Dot(omega_body, Hadamard(mass.principal_inertia, omega_body));
}

double PotentialEnergy(const RigidBodyState& state, const MassProperties& mass,
                       const ForceModel& model) {
  double energy = 0.0;
  if (model.gravity_enabled) {
    energy -= mass.mass * Dot(model.gravity, state.position);
  }
  if (model.spring_enabled) {
    energy += 0.5 * model.spring_stiffness *
              LengthSquared(state.position - model.spring_anchor);
  }
  if (model.central_enabled) {
    const double softened =
        LengthSquared(state.position - model.central_point) +
        model.central_softening * model.central_softening;
    energy -= model.central_mu * mass.mass / std::sqrt(softened);
  }
  return energy;
}

Vec3 AngularMomentum(const RigidBodyState& state, const MassProperties& mass) {
  const Vec3 omega_body =
      Rotate(Conjugate(state.orientation), state.angular_velocity);
  return Rotate(state.orientation,
                Hadamard(mass.principal_inertia, omega_body));
}

}  // namespace aerolab::sim
