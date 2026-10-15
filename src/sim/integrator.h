#ifndef AEROLAB_SIM_INTEGRATOR_H_
#define AEROLAB_SIM_INTEGRATOR_H_

#include <array>
#include <functional>
#include <string_view>

#include "sim/force_model.h"
#include "sim/rigid_body.h"

namespace aerolab::sim {

// Numerical integration schemes, from simplest to most accurate.
enum class Integrator {
  // x += v dt, v += a(x, v) dt. First order; gains energy on oscillators.
  kExplicitEuler,
  // v += a(x, v) dt, x += v_new dt. First order but symplectic for
  // separable translational motion: energy errors stay bounded on springs
  // and orbits. That guarantee does not extend to gyroscopic rotation,
  // where it can even diverge if a fast tumble is stepped coarsely.
  kSemiImplicitEuler,
  // Kick-drift-kick, second order and symplectic for position-only forces.
  // Velocity-dependent forces use a predicted velocity.
  kVelocityVerlet,
  // Classic fourth-order Runge-Kutta. Very accurate per step, but not
  // symplectic: energy slowly drifts over very long runs. Position,
  // velocity and angular velocity are fourth order; orientation advances
  // with the RK-averaged angular velocity through one exponential map,
  // which ignores that rotations do not commute, so it is second order.
  // (Lie-group methods such as Munthe-Kaas RK4 restore full order.)
  kRungeKutta4,
  kCount,
};

inline constexpr int kIntegratorCount = static_cast<int>(Integrator::kCount);

std::string_view IntegratorName(Integrator integrator);

constexpr std::array<Integrator, kIntegratorCount> AllIntegrators() {
  std::array<Integrator, kIntegratorCount> all{};
  for (int i = 0; i < kIntegratorCount; ++i) {
    all[i] = static_cast<Integrator>(i);
  }
  return all;
}

// Advances `state` by `dt` seconds. `age` is the body's time since launch at
// the start of the step. Time-limited forces such as thrust are sampled at
// the integrator's stage times, so a burnout inside a step is resolved only
// to the step size (and slightly differently per integrator). Orientation
// always advances through the exact quaternion exponential map and stays
// normalized. Pure and deterministic: identical inputs give bit-identical
// outputs.
RigidBodyState Step(const RigidBodyState& state, const MassProperties& mass,
                    const ForceModel& model, double age, double dt,
                    Integrator integrator);

// The wrench on a body as a function of its state and time; vehicle models
// (aerodynamics, rotors) provide one with their actuator settings held
// fixed over the step.
using WrenchFunction =
    std::function<Wrench(const RigidBodyState& state, double time)>;

// Advances `state` by `dt` seconds with classic RK4 under `wrench`,
// evaluated at the four stage times (`time` is the start of the step).
// Orientation advances through the exact quaternion exponential map.
RigidBodyState StepRk4(const RigidBodyState& state, const MassProperties& mass,
                       const WrenchFunction& wrench, double time, double dt);

}  // namespace aerolab::sim

#endif  // AEROLAB_SIM_INTEGRATOR_H_
