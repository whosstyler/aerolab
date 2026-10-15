#include "camera/camera_controller.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <span>

#include "math/quat.h"
#include "math/scalar.h"

namespace aerolab {
namespace {

// Keep the view just short of straight up/down so yaw stays well defined.
constexpr double kMaxPitch = Radians(89.0);

// Focus animations finish once the remaining error drops below these.
constexpr double kFocusPositionDone = 1e-3;
constexpr double kFocusDistanceDone = 1e-3;
constexpr double kLookDone = 1e-4;  // rad

// First-person walking.
constexpr double kWheelSpeedStep = 1.2;  // Speed factor per wheel notch.
constexpr double kMinWalkSpeed = 0.2;    // m/s
constexpr double kMaxWalkSpeed = 40.0;   // m/s
constexpr double kMinWalkSubstep = 0.1;  // m
constexpr int kMaxWalkSubsteps = 128;
// Spheres larger than this are not walk blockers (absurd data).
constexpr double kMaxBlockerRadius = 1e6;  // m

Quat OrientationFromYawPitch(double yaw, double pitch) {
  return FromEuler({.yaw = yaw, .pitch = pitch});
}

// Moves `position` (on the ground) out of every blocker it overlaps and
// removes the part of `velocity` heading into them. Two passes settle a
// walker squeezed between neighbours; a third overlapping blocker may still
// be grazed. `fallback` is the push direction from a blocker's exact centre.
void PushOutOfBlockers(std::span<const WalkBlocker> blockers, double clearance,
                       const Vec3& fallback, Vec3& position, Vec3& velocity) {
  for (int pass = 0; pass < 2; ++pass) {
    for (const WalkBlocker& blocker : blockers) {
      if (!IsFinite(blocker.center) || !std::isfinite(blocker.radius)) continue;
      const Vec3 center{blocker.center.x, 0.0, blocker.center.z};
      const Vec3 offset{position.x - center.x, 0.0, position.z - center.z};
      const double reach = std::max(blocker.radius, 0.0) + clearance;
      const double distance = Length(offset);
      if (distance >= reach) continue;
      const Vec3 normal = distance > 1e-9 ? offset / distance : fallback;
      position = center + normal * reach;
      const double into = Dot(velocity, normal);
      if (into < 0.0) velocity -= normal * into;
    }
  }
}

}  // namespace

CameraRig::CameraRig() { LookAt({14.0, 7.0, 18.0}, {0.0, 1.5, 0.0}); }

void CameraRig::SetMode(CameraMode mode) {
  if (mode == mode_) return;
  if (mode_ == CameraMode::kFreeFly || mode_ == CameraMode::kFirstPerson) {
    // Leaving a mode that moves the eye directly: put the pivot where the
    // camera is looking, at the last orbit distance, so the view does not
    // jump.
    pivot_ = camera_.position + CameraForward(camera_) * distance_;
  }
  if (mode == CameraMode::kFirstPerson) {
    // Drop straight down onto the ground, keeping where it was looking.
    EnterFirstPerson(camera_.position, yaw_, pitch_);
    return;
  }
  look_goal_.reset();
  velocity_ = Vec3::Zero();
  mode_ = mode;
}

void CameraRig::EnterFirstPerson(const Vec3& feet, double yaw, double pitch) {
  mode_ = CameraMode::kFirstPerson;
  yaw_ = WrapAngle(yaw);
  pitch_ =
      std::isfinite(pitch) ? std::clamp(pitch, -kMaxPitch, kMaxPitch) : 0.0;
  velocity_ = Vec3::Zero();
  pivot_goal_.reset();
  distance_goal_.reset();
  look_goal_.reset();
  camera_.orientation = OrientationFromYawPitch(yaw_, pitch_);
  camera_.position = {feet.x, settings_.eye_height, feet.z};
}

Vec3 CameraRig::feet() const {
  return {camera_.position.x, 0.0, camera_.position.z};
}

void CameraRig::LookAt(const Vec3& eye, const Vec3& target) {
  // A walker stays on the ground under `eye`, so aim from there.
  const Vec3 from = mode_ == CameraMode::kFirstPerson
                        ? Vec3{eye.x, settings_.eye_height, eye.z}
                        : eye;
  const Vec3 offset = target - from;
  distance_ = std::clamp(Length(offset), settings_.min_distance,
                         settings_.max_distance);
  const Vec3 d = NormalizedOr(offset, -Vec3::UnitZ());
  // Invert forward = Ry(yaw) * Rx(pitch) * (0, 0, -1)
  //                = (-sin(yaw)cos(pitch), sin(pitch), -cos(yaw)cos(pitch)).
  pitch_ =
      std::clamp(std::asin(std::clamp(d.y, -1.0, 1.0)), -kMaxPitch, kMaxPitch);
  yaw_ = std::atan2(-d.x, -d.z);
  pivot_ = target;
  pivot_goal_.reset();
  distance_goal_.reset();
  look_goal_.reset();
  velocity_ = Vec3::Zero();
  if (mode_ == CameraMode::kFirstPerson) {
    camera_.orientation = OrientationFromYawPitch(yaw_, pitch_);
    camera_.position = from;
    return;
  }
  SyncOrbitPose();
}

void CameraRig::Focus(const Vec3& center, double radius) {
  if (!IsFinite(center)) return;
  if (mode_ == CameraMode::kFirstPerson) {
    look_goal_ = center;  // Turn to face it; the walker stays put.
    return;
  }
  if (mode_ == CameraMode::kFreeFly) {
    pivot_ = camera_.position + CameraForward(camera_) * distance_;
  }
  // Distance at which a sphere of `radius` fills most of the vertical FOV.
  const double half_fov = 0.5 * camera_.vertical_fov_radians;
  const double fit = std::max(radius, 0.1) / std::sin(half_fov) * 1.3;
  pivot_goal_ = center;
  distance_goal_ =
      std::clamp(fit, settings_.min_distance, settings_.max_distance);
  velocity_ = Vec3::Zero();
}

void CameraRig::Update(const CameraInput& input, double dt,
                       const std::optional<Vec3>& follow_target,
                       std::span<const WalkBlocker> blockers) {
  if (!(dt > 0.0) || !std::isfinite(dt)) dt = 0.0;  // NaN, inf, negative.
  if (mode_ == CameraMode::kFirstPerson) {
    UpdateFirstPerson(input, dt, blockers);
  } else if (mode_ == CameraMode::kFreeFly) {
    UpdateFreeFly(input, dt);
  } else {
    UpdateOrbit(input, dt,
                mode_ == CameraMode::kFollow ? follow_target : std::nullopt);
  }
}

void CameraRig::ApplyRotation(const Vec2& rotate_delta) {
  // Moving the mouse right turns right; moving it down looks down.
  yaw_ = WrapAngle(yaw_ - rotate_delta.x * settings_.rotate_sensitivity);
  pitch_ = std::clamp(pitch_ - rotate_delta.y * settings_.rotate_sensitivity,
                      -kMaxPitch, kMaxPitch);
}

void CameraRig::SyncOrbitPose() {
  camera_.orientation = OrientationFromYawPitch(yaw_, pitch_);
  camera_.position = pivot_ - CameraForward(camera_) * distance_;
}

void CameraRig::UpdateFreeFly(const CameraInput& input, double dt) {
  ApplyRotation(input.rotate_delta);
  camera_.orientation = OrientationFromYawPitch(yaw_, pitch_);

  // A pending focus animation slides the camera along its view direction.
  if (pivot_goal_.has_value() || distance_goal_.has_value()) {
    UpdateOrbit(CameraInput{}, dt, std::nullopt);
    return;
  }

  if (input.wheel != 0.0 && std::isfinite(input.wheel)) {
    settings_.move_speed = std::clamp(
        settings_.move_speed * std::pow(1.2, input.wheel), 0.1, 2000.0);
  }

  double speed = settings_.move_speed;
  if (input.boost) speed *= settings_.boost_multiplier;
  if (input.slow) speed *= settings_.slow_multiplier;

  // Horizontal movement follows the view; vertical movement uses world up so
  // Q/E always rise and sink regardless of where the camera looks.
  Vec3 direction = CameraRight(camera_) * input.move.x +
                   Vec3::UnitY() * input.move.y +
                   CameraForward(camera_) * input.move.z;
  if (LengthSquared(direction) > 1.0) direction = Normalized(direction);
  const Vec3 target_velocity = direction * speed;

  const double k = settings_.move_sharpness;
  if (k <= 0.0) {
    velocity_ = target_velocity;
    camera_.position += target_velocity * dt;
  } else {
    // Exact solution of dv/dt = k (target - v) over the step, integrated
    // for position too. Splitting a frame into smaller steps gives the same
    // answer, which makes the motion frame-rate independent.
    const double decay = std::exp(-k * dt);
    const Vec3 initial_gap = velocity_ - target_velocity;
    camera_.position +=
        target_velocity * dt + initial_gap * ((1.0 - decay) / k);
    velocity_ = target_velocity + initial_gap * decay;
  }

  // Panning slides the camera in its view plane at a reference distance.
  if (input.pan_delta.x != 0.0 || input.pan_delta.y != 0.0) {
    const double units =
        WorldUnitsPerPixel(camera_, distance_, input.viewport_height);
    camera_.position += (CameraUp(camera_) * input.pan_delta.y -
                         CameraRight(camera_) * input.pan_delta.x) *
                        units;
  }
}

void CameraRig::UpdateOrbit(const CameraInput& input, double dt,
                            const std::optional<Vec3>& follow_target) {
  ApplyRotation(input.rotate_delta);
  camera_.orientation = OrientationFromYawPitch(yaw_, pitch_);

  if (input.wheel != 0.0 && std::isfinite(input.wheel)) {
    distance_goal_.reset();
    distance_ =
        std::clamp(distance_ * std::pow(1.0 - settings_.zoom_step, input.wheel),
                   settings_.min_distance, settings_.max_distance);
  }

  if (input.pan_delta.x != 0.0 || input.pan_delta.y != 0.0) {
    pivot_goal_.reset();
    const double units =
        WorldUnitsPerPixel(camera_, distance_, input.viewport_height);
    // Dragging right moves the scene right, i.e. the pivot left.
    pivot_ += (CameraUp(camera_) * input.pan_delta.y -
               CameraRight(camera_) * input.pan_delta.x) *
              units;
  }

  // Keyboard movement slides the pivot across the ground plane at a speed
  // proportional to the orbit distance, like a strategy-game camera.
  if (LengthSquared(input.move) > 0.0 && !follow_target.has_value()) {
    pivot_goal_.reset();
    const Vec3 forward_flat = NormalizedOr(
        Vec3{CameraForward(camera_).x, 0.0, CameraForward(camera_).z},
        -Vec3::UnitZ());
    const Vec3 right_flat = Cross(forward_flat, Vec3::UnitY());
    double speed = distance_ * 0.8;
    if (input.boost) speed *= settings_.boost_multiplier;
    if (input.slow) speed *= settings_.slow_multiplier;
    pivot_ += (right_flat * input.move.x + Vec3::UnitY() * input.move.y +
               forward_flat * input.move.z) *
              (speed * dt);
  }

  const double blend = SmoothingFactor(settings_.focus_sharpness, dt);
  if (follow_target.has_value()) {
    pivot_goal_.reset();
    pivot_ = Lerp(pivot_, *follow_target, blend);
  } else if (pivot_goal_.has_value()) {
    pivot_ = Lerp(pivot_, *pivot_goal_, blend);
    if (Distance(pivot_, *pivot_goal_) < kFocusPositionDone) {
      pivot_ = *pivot_goal_;
      pivot_goal_.reset();
    }
  }
  if (distance_goal_.has_value()) {
    distance_ = aerolab::Lerp(distance_, *distance_goal_, blend);
    if (std::abs(distance_ - *distance_goal_) < kFocusDistanceDone) {
      distance_ = *distance_goal_;
      distance_goal_.reset();
    }
  }
  SyncOrbitPose();
}

void CameraRig::UpdateFirstPerson(const CameraInput& input, double dt,
                                  std::span<const WalkBlocker> blockers) {
  // Looking around cancels a pending "turn to face".
  if (input.rotate_delta.x != 0.0 || input.rotate_delta.y != 0.0) {
    look_goal_.reset();
  }
  ApplyRotation(input.rotate_delta);
  if (look_goal_.has_value()) TurnTowardLookGoal(dt);
  camera_.orientation = OrientationFromYawPitch(yaw_, pitch_);

  if (input.wheel != 0.0 && std::isfinite(input.wheel)) {
    settings_.walk_speed = std::clamp(
        settings_.walk_speed * std::pow(kWheelSpeedStep, input.wheel),
        kMinWalkSpeed, kMaxWalkSpeed);
  }
  double speed = settings_.walk_speed;
  if (input.boost) speed *= settings_.run_multiplier;
  if (input.slow) speed *= settings_.slow_multiplier;

  // Walk along the ground in the direction the body faces, whatever the
  // head pitch; Q/E do nothing for a walker.
  const Vec3 forward{-std::sin(yaw_), 0.0, -std::cos(yaw_)};
  const Vec3 right = Cross(forward, Vec3::UnitY());
  Vec3 direction = right * input.move.x + forward * input.move.z;
  if (LengthSquared(direction) > 1.0) direction = Normalized(direction);
  const Vec3 target_velocity = direction * speed;

  const Vec3 start = feet();
  Vec3 end = start;
  const double k = settings_.move_sharpness;
  if (k <= 0.0) {
    velocity_ = target_velocity;
    end += target_velocity * dt;
  } else {
    // Same exact exponential approach as free-fly: frame-rate independent.
    const double decay = std::exp(-k * dt);
    const Vec3 initial_gap = velocity_ - target_velocity;
    end += target_velocity * dt + initial_gap * ((1.0 - decay) / k);
    velocity_ = target_velocity + initial_gap * decay;
  }
  end.y = 0.0;

  // Travel in steps no longer than the walker's radius, settling out of
  // blockers after each one, so a long frame or a high speed cannot carry
  // the walker through a thin object.
  const double clearance = std::max(settings_.walker_radius, 0.0);
  const Vec3 travel = end - start;
  const double step_length = std::max(clearance, kMinWalkSubstep);
  const int steps =
      static_cast<int>(std::clamp(std::ceil(Length(travel) / step_length), 1.0,
                                  static_cast<double>(kMaxWalkSubsteps)));
  Vec3 position = start;
  for (int i = 0; i < steps; ++i) {
    position += travel / static_cast<double>(steps);
    PushOutOfBlockers(blockers, clearance, -forward, position, velocity_);
  }
  if (!IsFinite(position) || !IsFinite(velocity_)) {
    position = start;  // Never let bad data strand the walker.
    velocity_ = Vec3::Zero();
  }
  camera_.position = position + Vec3{0.0, settings_.eye_height, 0.0};
}

void CameraRig::TurnTowardLookGoal(double dt) {
  const Vec3 to_goal = *look_goal_ - camera_.position;
  if (LengthSquared(to_goal) < 1e-12) {
    look_goal_.reset();
    return;
  }
  const Vec3 d = Normalized(to_goal);
  // Straight up or down the heading is undefined: keep it, only pitch.
  const double goal_yaw =
      std::hypot(d.x, d.z) > 1e-6 ? std::atan2(-d.x, -d.z) : yaw_;
  const double goal_pitch =
      std::clamp(std::asin(std::clamp(d.y, -1.0, 1.0)), -kMaxPitch, kMaxPitch);
  const double blend = SmoothingFactor(settings_.focus_sharpness, dt);
  yaw_ = WrapAngle(yaw_ + WrapAngle(goal_yaw - yaw_) * blend);
  pitch_ += (goal_pitch - pitch_) * blend;
  if (std::abs(WrapAngle(goal_yaw - yaw_)) < kLookDone &&
      std::abs(goal_pitch - pitch_) < kLookDone) {
    yaw_ = goal_yaw;
    pitch_ = goal_pitch;
    look_goal_.reset();
  }
}

std::optional<WalkBlocker> WalkBlockerFromSphere(const Vec3& center,
                                                 double radius, double height) {
  if (!(radius > 0.0) || radius > kMaxBlockerRadius || !IsFinite(center) ||
      !std::isfinite(height)) {
    return std::nullopt;
  }
  // The sphere's widest section within the walker's height [0, height].
  const double nearest = std::clamp(center.y, 0.0, std::max(height, 0.0));
  const double dy = center.y - nearest;
  if (std::abs(dy) >= radius) return std::nullopt;
  const double gap = std::abs(dy);
  return WalkBlocker{{center.x, 0.0, center.z},
                     std::sqrt((radius - gap) * (radius + gap))};
}

CameraInput CameraInputFromState(const InputState& input, bool use_mouse,
                                 bool use_keyboard, double viewport_height,
                                 double pixels_per_point) {
  CameraInput result;
  result.viewport_height = viewport_height;
  if (use_mouse) {
    if (input.IsDown(MouseButton::kRight)) {
      result.rotate_delta =
          input.mouse_delta() / std::max(pixels_per_point, 1e-3);
    }
    if (input.IsDown(MouseButton::kMiddle)) {
      result.pan_delta = input.mouse_delta();
    }
    result.wheel = input.wheel();
  }
  if (use_keyboard) {
    const auto axis = [&input](Key negative, Key positive) {
      return (input.IsDown(positive) ? 1.0 : 0.0) -
             (input.IsDown(negative) ? 1.0 : 0.0);
    };
    result.move = {axis(Key::kA, Key::kD), axis(Key::kQ, Key::kE),
                   axis(Key::kS, Key::kW)};
    result.boost = input.IsDown(Key::kShift);
    result.slow = input.IsDown(Key::kCtrl);
  }
  return result;
}

const char* CameraModeName(CameraMode mode) {
  switch (mode) {
    case CameraMode::kFreeFly:
      return "Free-fly";
    case CameraMode::kOrbit:
      return "Orbit";
    case CameraMode::kFollow:
      return "Follow";
    case CameraMode::kFirstPerson:
      return "First person";
  }
  return "Unknown";
}

}  // namespace aerolab
