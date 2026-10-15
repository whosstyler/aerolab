#include "sim/flight_control.h"

#include <algorithm>
#include <cmath>

#include "math/mat3.h"
#include "math/quat.h"
#include "sim/aerodynamics.h"
#include "sim/atmosphere.h"
#include "sim/force_model.h"
#include "sim/heading.h"

namespace aerolab::sim {
namespace {

// Hover craft never ask for more than this downward acceleration: the
// rotors can only push, so falling faster than gravity is impossible.
constexpr double kMaxDescentAcceleration = 0.7 * kStandardGravity;
constexpr double kMaxWingPitch = Radians(20.0);
// Pitch-down per m/s below the minimum airspeed (stall protection).
constexpr double kStallRecoveryGain = Radians(4.0);
// Overspeed (m/s) at which a wing stops descending altogether.
constexpr double kOverspeedBand = 3.0;
// How far below the stall angle the autopilot keeps the angle of attack.
constexpr double kAlphaMargin = Radians(4.0);
// Turns keep the airspeed this many times above the stall speed in the turn.
constexpr double kTurnStallMargin = 1.15;
// Fastest change of a wing's climb-rate setpoint, m/s^2 (about 0.2 g).
constexpr double kMaxClimbChange = 2.0;
// Extra throttle per m/s of climb asked for (climbing costs power).
constexpr double kClimbThrottle = 0.05;
// The hover thrust vector never drops below this fraction of the weight,
// so the craft stays upright and controllable while descending.
constexpr double kMinLiftFraction = 0.1;
// Pitch beyond which Euler roll rates are not trusted (tan blows up).
constexpr double kMaxRatePitch = 1.4;  // rad, about 80 degrees

// Orientation whose up axis is `up` and whose nose points as close to
// `heading` as that allows.
Quat AttitudeFor(const Vec3& up, double heading) {
  const Vec3 right =
      NormalizedOr(Cross(HeadingVector(heading), up), Vec3::UnitX());
  const Vec3 forward = Cross(up, right);
  return FromMat3(Mat3::FromColumns(right, up, -forward));
}

}  // namespace

double Pid::Update(double error, double measured_rate, double dt) {
  const bool in_zone = integral_zone <= 0.0 || std::abs(error) <= integral_zone;
  if (dt > 0.0 && ki != 0.0 && in_zone && std::isfinite(error)) {
    const double limit = std::abs(integral_limit);
    integral = std::clamp(integral + ki * error * dt, -limit, limit);
  }
  return kp * error + integral - kd * measured_rate;
}

double StallSafeBank(double stall_speed, double airspeed, double bank_limit) {
  if (!(stall_speed > 0.0)) return bank_limit;
  // A bank of phi raises the stall speed by 1 / sqrt(cos phi).
  const double ratio = kTurnStallMargin * stall_speed / std::max(airspeed, 1.0);
  return std::min(bank_limit, std::max(std::acos(std::min(ratio * ratio, 1.0)),
                                       Radians(1.0)));
}

AttitudeAngles AttitudeOf(const Quat& orientation) {
  const Vec3 forward = Rotate(orientation, -Vec3::UnitZ());
  const Vec3 right = Rotate(orientation, Vec3::UnitX());
  const Vec3 up = Rotate(orientation, Vec3::UnitY());
  AttitudeAngles angles;
  angles.pitch = std::asin(std::clamp(forward.y, -1.0, 1.0));
  angles.roll = std::atan2(-right.y, up.y);
  angles.heading = HorizontalLength(forward) > 1e-9 ? HeadingOf(forward) : 0.0;
  return angles;
}

void QuadController::Reset() {
  velocity_x.Reset();
  velocity_y.Reset();
  velocity_z.Reset();
}

RotorArray QuadController::Update(const MultirotorParams& params,
                                  const ControlLimits& limits,
                                  const FlightTarget& target,
                                  const RigidBodyState& state, const Vec3& wind,
                                  double dt) {
  const double mass = params.mass.mass;
  const double tilt =
      std::clamp(limits.max_tilt, Radians(1.0), kMaxAutopilotTilt);
  // Rotors make less thrust and torque in thin air (the mixer works in
  // sea-level terms), so demands are scaled up to compensate.
  const double density_ratio = std::max(
      StandardAtmosphere(state.position.y).density / kSeaLevelDensity, 0.05);

  // 1. Velocity error -> wanted acceleration (world axes), within limits.
  const Vec3 error = target.velocity - state.velocity;
  Vec3 accel{velocity_x.Update(error.x, 0.0, dt),
             velocity_y.Update(error.y, 0.0, dt),
             velocity_z.Update(error.z, 0.0, dt)};
  accel.y = std::clamp(accel.y, -kMaxDescentAcceleration,
                       std::max(limits.max_acceleration, 0.0));
  const double horizontal_limit =
      std::min(std::max(limits.max_acceleration, 0.0),
               kStandardGravity * std::tan(tilt));
  const double horizontal = std::hypot(accel.x, accel.z);
  if (horizontal > horizontal_limit) {
    accel.x *= horizontal_limit / horizontal;
    accel.z *= horizontal_limit / horizontal;
  }

  // 2. Thrust vector: carry the weight, produce the acceleration and cancel
  //    the body drag the craft is feeling.
  Vec3 thrust = (accel + Vec3{0.0, kStandardGravity, 0.0}) * mass;
  const Vec3 airflow = state.velocity - wind;
  const double airspeed = Length(airflow);
  if (airspeed > 1e-6) {
    const double drag = params.linear_drag * airspeed + params.quadratic_drag *
                                                            density_ratio *
                                                            airspeed * airspeed;
    thrust += airflow * (drag / airspeed);
  }
  // Never tilt beyond the limit: keep the vertical part, trim the rest.
  thrust.y = std::max(thrust.y, kMinLiftFraction * mass * kStandardGravity);
  const double side = std::hypot(thrust.x, thrust.z);
  const double side_limit = thrust.y * std::tan(tilt);
  if (side > side_limit) {
    thrust.x *= side_limit / side;
    thrust.z *= side_limit / side;
  }

  // 3. Wanted attitude: thrust axis along the thrust vector, nose toward the
  //    wanted heading (or the direction of travel).
  double heading = AttitudeOf(state.orientation).heading;
  if (target.hold_heading) {
    heading = target.heading;
  } else if (HorizontalLength(target.velocity) > 1.0) {
    heading = HeadingOf(target.velocity);
  }
  const Quat wanted = AttitudeFor(Normalized(thrust), heading);

  // Collective along the current thrust axis: what actually helps now.
  const Vec3 up = Rotate(state.orientation, Vec3::UnitY());
  const double collective = std::max(Dot(thrust, up), 0.0);

  // 4. Attitude error -> wanted body rates (world axes), limited.
  Vec3 rates =
      ToRotationVector(wanted * Conjugate(state.orientation)) * attitude_gain;
  const double rate = Length(rates);
  if (rate > max_body_rate) rates *= max_body_rate / rate;

  // 5. Rate error -> torque, with the gyroscopic term compensated.
  const Quat to_body = Conjugate(state.orientation);
  const Vec3 omega = Rotate(to_body, state.angular_velocity);
  const Vec3 angular_accel =
      Rotate(to_body, (rates - state.angular_velocity) * rate_gain);
  const Vec3 inertia = params.mass.principal_inertia;
  const Vec3 torque =
      Hadamard(inertia, angular_accel) + Cross(omega, Hadamard(inertia, omega));

  // 6. Mixer: rotor speeds for that thrust and torque in this air.
  return MixRotorSpeeds(params, collective / density_ratio,
                        torque / density_ratio);
}

void WingAutopilot::Reset() {
  climb_setpoint = 0.0;
  airspeed.Reset();
  climb.Reset();
  roll.Reset();
  pitch.Reset();
}

FixedWingCommand WingAutopilot::Update(const FixedWingParams& params,
                                       const ControlLimits& limits,
                                       const FlightTarget& target,
                                       const RigidBodyState& state,
                                       const Vec3& wind, double dt) {
  const AirData air =
      ComputeAirData(state, wind, StandardAtmosphere(state.position.y));
  const AttitudeAngles attitude = AttitudeOf(state.orientation);
  const Vec3 omega =
      Rotate(Conjugate(state.orientation), state.angular_velocity);
  // Body rates (aviation axes) turned into the rates of the roll and pitch
  // angles: a steady banked turn pitches the body continuously, yet its
  // pitch angle stays put, so damping must act on the angle's rate.
  const double p = -omega.z;  // Roll, right wing down.
  const double q = omega.x;   // Pitch, nose up.
  const double r = -omega.y;  // Yaw, nose right.
  const double sin_roll = std::sin(attitude.roll);
  const double cos_roll = std::cos(attitude.roll);
  const double roll_rate =
      p +
      (q * sin_roll + r * cos_roll) *
          std::tan(std::clamp(attitude.pitch, -kMaxRatePitch, kMaxRatePitch));
  const double pitch_rate = q * cos_roll - r * sin_roll;
  const double speed = std::max(air.airspeed, 1.0);
  const double bank_limit = StallSafeBank(
      limits.stall_speed, speed,
      std::clamp(limits.max_tilt, Radians(1.0), kMaxAutopilotBank));

  // Course -> turn rate (positive left, like headings) -> coordinated bank.
  // Turning left means the left wing goes down: negative roll.
  const double wanted_course = HorizontalLength(target.velocity) > 0.5
                                   ? HeadingOf(target.velocity)
                                   : attitude.heading;
  const double course = HorizontalLength(state.velocity) > 0.5
                            ? HeadingOf(state.velocity)
                            : attitude.heading;
  const double max_turn = kStandardGravity * std::tan(bank_limit) / speed;
  const double feed_forward =
      std::isfinite(target.turn_rate) ? target.turn_rate : 0.0;
  const double turn_rate =
      std::clamp(feed_forward + course_gain * WrapAngle(wanted_course - course),
                 -max_turn, max_turn);
  const double bank = -std::atan(speed * turn_rate / kStandardGravity);

  // Airspeed -> throttle. The wanted airspeed is what gives the requested
  // ground velocity in this wind (within limits); climbing costs power, so
  // the climb is fed forward.
  const double wanted_speed =
      std::clamp(Length(target.velocity - wind), limits.min_airspeed,
                 std::max(limits.max_airspeed, limits.min_airspeed));
  double wanted_climb = std::clamp(target.velocity.y, -limits.max_climb_rate,
                                   limits.max_climb_rate);
  // Energy management: a descent steeper than the airframe can glide at
  // idle only builds up speed, so overspeed gives priority to airspeed and
  // the descent is eased off.
  const double overspeed = air.airspeed - wanted_speed;
  if (wanted_climb < 0.0 && overspeed > 0.0) {
    wanted_climb *= std::max(1.0 - overspeed / kOverspeedBand, 0.0);
  }
  // Climb-rate changes are eased in at a gentle vertical acceleration
  // instead of a push-over (or pull-up) the moment the request changes.
  if (dt > 0.0) {
    const double step = kMaxClimbChange * dt;
    climb_setpoint = std::isfinite(climb_setpoint)
                         ? std::clamp(wanted_climb, climb_setpoint - step,
                                      climb_setpoint + step)
                         : wanted_climb;
  }
  wanted_climb = climb_setpoint;
  FixedWingCommand command;
  command.throttle = std::clamp(
      trim_throttle + airspeed.Update(wanted_speed - air.airspeed, 0.0, dt) +
          kClimbThrottle * wanted_climb,
      0.0, 1.0);

  // Climb rate -> pitch, with the flight-path angle fed forward, and the
  // nose lowered when too slow (stall protection).
  double wanted_pitch = trim_pitch + wanted_climb / speed +
                        climb.Update(wanted_climb - state.velocity.y, 0.0, dt);
  if (air.airspeed < limits.min_airspeed) {
    wanted_pitch -= kStallRecoveryGain * (limits.min_airspeed - air.airspeed);
  }
  // Angle-of-attack limit: the nose never rises above the flight path (through
  // the air) by more than the wing can take, so a stalled wing is flown out
  // of the stall instead of being held in it.
  if (air.airspeed > 1e-3) {
    const double path =
        std::asin(std::clamp(air.relative_wind.y / air.airspeed, -1.0, 1.0));
    const double alpha_limit = params.aero.alpha_stall - kAlphaMargin;
    wanted_pitch =
        std::min(wanted_pitch, path + alpha_limit * std::cos(attitude.roll));
  }
  wanted_pitch = std::clamp(wanted_pitch, -kMaxWingPitch, kMaxWingPitch);

  // Inner loops, with rate damping. Elevator > 0 lowers the nose. Rudder
  // flies out any sideslip (air from the right: nose right).
  const double limit = std::max(params.surface_limit, 0.0);
  command.surfaces.aileron = std::clamp(
      roll.Update(bank - attitude.roll, roll_rate, dt), -limit, limit);
  command.surfaces.elevator =
      std::clamp(-pitch.Update(wanted_pitch - attitude.pitch, pitch_rate, dt),
                 -limit, limit);
  command.surfaces.rudder = std::clamp(sideslip_gain * air.beta, -limit, limit);
  return command;
}

}  // namespace aerolab::sim
