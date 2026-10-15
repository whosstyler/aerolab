#ifndef AEROLAB_SIM_MULTIROTOR_H_
#define AEROLAB_SIM_MULTIROTOR_H_

#include <array>

#include "math/vec3.h"
#include "sim/force_model.h"
#include "sim/rigid_body.h"

namespace aerolab::sim {

inline constexpr int kRotorCount = 4;
using RotorArray = std::array<double, kRotorCount>;

// A quadrotor in the "X" layout: four rotors on arms at 45 degrees to the
// nose, each producing thrust k_T w^2 along body up and a reaction torque
// k_Q w^2 about body up opposite to its spin, driven by motors that lag
// their command. Defaults (GenericQuadrotor) describe a ~1.2 kg drone that
// hovers at about 800 rad/s per rotor with a thrust-to-weight near 3.
struct MultirotorParams {
  // Principal inertia in engine body axes: x = pitch, y = yaw, z = roll.
  MassProperties mass;
  double arm_length = 0.18;            // m, centre to each rotor hub
  double thrust_coefficient = 4.6e-6;  // k_T, N per (rad/s)^2
  double torque_coefficient = 7.4e-8;  // k_Q, N m per (rad/s)^2
  double max_rotor_speed = 1400.0;     // rad/s
  double motor_time_constant = 0.03;   // s (first-order lag)
  double linear_drag = 0.25;           // N per m/s of airspeed
  double quadratic_drag = 0.03;        // N per (m/s)^2, at sea level
};

MultirotorParams GenericQuadrotor();

// Rotor layout. Index 0 front-right, 1 front-left, 2 rear-left,
// 3 rear-right; positions in engine body axes (front is -Z, right is +X).
// Front-right and rear-left spin counter-clockwise seen from above (+1),
// the other two clockwise (-1), so their reaction torques cancel in hover.
Vec3 RotorPosition(const MultirotorParams& params, int rotor);
double RotorSpin(int rotor);

struct MultirotorState {
  RotorArray rotor_speed{};  // rad/s, each in [0, max_rotor_speed]
};

// Moves each rotor toward its commanded speed with the motor lag.
void AdvanceMotors(const MultirotorParams& params, const RotorArray& commanded,
                   double dt, MultirotorState& state);

// Rotor speed at which the four rotors together carry the weight.
double HoverRotorSpeed(const MultirotorParams& params);

// Total force and moment, world frame: rotor thrusts and reaction torques,
// body drag against the airflow (relative to `wind`, scaled by the standard
// atmosphere's density at the current altitude) and gravity.
Wrench MultirotorWrench(const MultirotorParams& params,
                        const MultirotorState& state,
                        const RigidBodyState& body, const Vec3& wind);

// The mixer: rotor speeds that produce `collective_thrust` (N, along body
// up) and `body_torque` (N m, engine body axes), clamped to the motors'
// range. When the request cannot be met, yaw authority is given up first,
// then the collective is trimmed so attitude control is kept.
RotorArray MixRotorSpeeds(const MultirotorParams& params,
                          double collective_thrust, const Vec3& body_torque);

}  // namespace aerolab::sim

#endif  // AEROLAB_SIM_MULTIROTOR_H_
