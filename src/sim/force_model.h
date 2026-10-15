#ifndef AEROLAB_SIM_FORCE_MODEL_H_
#define AEROLAB_SIM_FORCE_MODEL_H_

#include "math/vec3.h"
#include "sim/rigid_body.h"

namespace aerolab::sim {

// Standard gravitational acceleration at the Earth's surface, m/s^2.
inline constexpr double kStandardGravity = 9.81;

// A configurable toy force model built from independent, optional terms.
// Every term is a textbook idealization chosen for teaching; none models a
// specific real vehicle. Units are SI.
struct ForceModel {
  // Uniform gravitational acceleration.
  bool gravity_enabled = true;
  Vec3 gravity{0.0, -kStandardGravity, 0.0};

  // Aerodynamic drag on the velocity relative to the air mass (`wind`):
  // F = -c1 v_rel - c2 |v_rel| v_rel.
  Vec3 wind;
  double linear_drag = 0.0;     // N per (m/s)
  double quadratic_drag = 0.0;  // N per (m/s)^2

  // Constant thrust along body forward (-Z) while age < thrust_duration.
  double thrust = 0.0;           // N
  double thrust_duration = 0.0;  // s

  // Lift along body up, proportional to the square of forward airspeed:
  // F = c_l * max(v_fwd, 0)^2 * up. A crude stand-in for a wing.
  double lift_coefficient = 0.0;  // N per (m/s)^2

  // Linear spring-damper pulling toward `spring_anchor`.
  bool spring_enabled = false;
  Vec3 spring_anchor;
  double spring_stiffness = 0.0;  // N/m
  double spring_damping = 0.0;    // N per (m/s)

  // Inverse-square attraction toward `central_point` with gravitational
  // parameter mu (a toy "planet"). Softening avoids the singularity.
  bool central_enabled = false;
  Vec3 central_point;
  double central_mu = 0.0;          // m^3/s^2
  double central_softening = 0.05;  // m

  // Weathervane torque aligning body forward with the air-relative velocity
  // (the direction of travel through the air):
  // T = k |v_rel| (forward x v_rel). Gives arrows and darts their stability.
  double weathervane = 0.0;  // N m per (m/s)^2

  // Rotational drag: T = -c omega.
  double angular_damping = 0.0;  // N m per (rad/s)

  // Constant torque expressed in body axes.
  Vec3 body_torque;  // N m
};

// Individual force contributions at one instant, for debug vectors and the
// inspector. Forces are in newtons, world frame.
struct ForceBreakdown {
  Vec3 gravity;
  Vec3 drag;
  Vec3 thrust;
  Vec3 lift;
  Vec3 spring;
  Vec3 central;
  Vec3 total_force;
  Vec3 total_torque;  // N m, world frame
};

// Total force and torque on a body about its centre of mass, world frame.
struct Wrench {
  Vec3 force;   // N
  Vec3 torque;  // N m
};

// Time derivative of a rigid-body state.
struct StateDerivative {
  Vec3 velocity;              // d(position)/dt
  Vec3 acceleration;          // d(velocity)/dt
  Vec3 angular_velocity;      // drives d(orientation)/dt
  Vec3 angular_acceleration;  // d(angular_velocity)/dt, world frame
};

// Evaluates every force and torque term. `age` is the body's time since
// launch, used by time-limited terms such as thrust.
ForceBreakdown ComputeForces(const RigidBodyState& state,
                             const MassProperties& mass,
                             const ForceModel& model, double age);

// Newton-Euler equations for an arbitrary wrench: linear acceleration F / m
// and angular acceleration from Euler's rigid-body equations,
// I w' + w x (I w) = T, in body axes.
StateDerivative DerivativeFromWrench(const RigidBodyState& state,
                                     const MassProperties& mass,
                                     const Wrench& wrench);

// Newton-Euler equations: linear acceleration F / m and angular acceleration
// from Euler's rigid-body equations, I w' + w x (I w) = T, in body axes.
StateDerivative Evaluate(const RigidBodyState& state,
                         const MassProperties& mass, const ForceModel& model,
                         double age);

// Translational plus rotational kinetic energy (J).
double KineticEnergy(const RigidBodyState& state, const MassProperties& mass);

// Potential energy of the conservative terms (gravity, spring, central).
double PotentialEnergy(const RigidBodyState& state, const MassProperties& mass,
                       const ForceModel& model);

// Angular momentum about the centre of mass, world frame (kg m^2/s).
Vec3 AngularMomentum(const RigidBodyState& state, const MassProperties& mass);

}  // namespace aerolab::sim

#endif  // AEROLAB_SIM_FORCE_MODEL_H_
