#include "camera/camera_controller.h"

#include <algorithm>
#include <cmath>
#include <optional>

#include "math/quat.h"
#include "math/scalar.h"

namespace aerolab {
namespace {

// Keep the view just short of straight up/down so yaw stays well defined.
constexpr double kMaxPitch = Radians(89.0);

// Focus animations finish once the remaining error drops below these.
constexpr double kFocusPositionDone = 1e-3;
constexpr double kFocusDistanceDone = 1e-3;

Quat OrientationFromYawPitch(double yaw, double pitch) {
  return FromEuler({.yaw = yaw, .pitch = pitch});
}

}  // namespace

CameraRig::CameraRig() { LookAt({14.0, 7.0, 18.0}, {0.0, 1.5, 0.0}); }

void CameraRig::SetMode(CameraMode mode) {
  if (mode == mode_) return;
  if (mode_ == CameraMode::kFreeFly) {
    // Entering an orbiting mode: put the pivot where the camera is looking,
    // at the last orbit distance, so the view does not jump.
    pivot_ = camera_.position + CameraForward(camera_) * distance_;
  }
  velocity_ = Vec3::Zero();
  mode_ = mode;
}

void CameraRig::LookAt(const Vec3& eye, const Vec3& target) {
  const Vec3 offset = target - eye;
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
  velocity_ = Vec3::Zero();
  SyncOrbitPose();
}

void CameraRig::Focus(const Vec3& center, double radius) {
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
                       const std::optional<Vec3>& follow_target) {
  dt = std::max(dt, 0.0);
  if (mode_ == CameraMode::kFreeFly) {
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

  if (input.wheel != 0.0) {
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

  if (input.wheel != 0.0) {
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
  }
  return "Unknown";
}

}  // namespace aerolab
