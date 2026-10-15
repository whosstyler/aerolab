#ifndef AEROLAB_CAMERA_CAMERA_CONTROLLER_H_
#define AEROLAB_CAMERA_CAMERA_CONTROLLER_H_

#include <optional>
#include <span>

#include "camera/camera.h"
#include "core/input.h"
#include "math/vec2.h"
#include "math/vec3.h"

namespace aerolab {

// Device-independent camera intent for one frame. The application maps raw
// keyboard/mouse state onto this, so controllers are testable in isolation.
struct CameraInput {
  // Movement along camera axes in [-1, 1]: x = right, y = up, z = forward.
  Vec3 move;
  // Mouse motion in pixels (x right, y down) while rotating the view.
  Vec2 rotate_delta;
  // Mouse motion in pixels (x right, y down) while panning.
  Vec2 pan_delta;
  // Mouse wheel notches; positive zooms in (orbit) or speeds up (free-fly,
  // first person).
  double wheel = 0.0;
  bool boost = false;
  bool slow = false;
  // Height of the viewport in pixels, for pixel-accurate panning.
  double viewport_height = 720.0;
};

enum class CameraMode { kFreeFly, kOrbit, kFollow, kFirstPerson };

struct CameraSettings {
  double move_speed = 12.0;           // Free-fly cruise speed, m/s.
  double boost_multiplier = 4.0;      // Speed factor while boosting.
  double slow_multiplier = 0.2;       // Speed factor while moving precisely.
  double rotate_sensitivity = 0.004;  // Radians per pixel.
  double zoom_step = 0.15;            // Fractional distance change per notch.
  // Velocity smoothing rate in 1/s; zero moves instantly.
  double move_sharpness = 10.0;
  // Rate at which focus and follow targets are approached, in 1/s.
  double focus_sharpness = 9.0;
  double min_distance = 0.25;
  double max_distance = 4000.0;

  // First person: walking on the ground plane.
  double eye_height = 1.65;     // m above the feet.
  double walk_speed = 4.0;      // m/s
  double run_multiplier = 2.5;  // Speed factor while running (boost).
  double walker_radius = 0.3;   // m, kept clear of walk blockers.
};

// Something a first-person walker cannot pass through, as a vertical
// cylinder: a circle on the ground plane (`center.y` is ignored).
struct WalkBlocker {
  Vec3 center;
  double radius = 0.0;
};

// The blocker a sphere makes for a walker of `height`: the sphere's widest
// horizontal section within [0, height], or nothing when the sphere is
// entirely above or below the walker.
std::optional<WalkBlocker> WalkBlockerFromSphere(const Vec3& center,
                                                 double radius, double height);

// Owns the camera and switches between navigation modes without jumps:
//
//  * Free-fly: mouse-look with WASD/QE translation (velocity smoothed with
//    an exact exponential solution, so motion is frame-rate independent).
//  * Orbit: rotate around a pivot, pan the pivot, exponential zoom.
//  * Follow: orbit whose pivot smoothly tracks a moving target.
//  * First person: stands on the ground plane at eye height, walks with
//    WASD (Shift runs, Ctrl creeps), looks with the mouse, and cannot walk
//    through walk blockers.
class CameraRig {
 public:
  CameraRig();

  const Camera& camera() const { return camera_; }
  // Lens parameters (FOV, clip planes) may be edited directly; pose changes
  // should go through the rig so its internal state stays consistent.
  Camera& mutable_lens() { return camera_; }

  CameraMode mode() const { return mode_; }
  // Switches mode keeping the view direction. Entering first person this
  // way drops the eye straight down onto the ground (camera only: scene
  // observers are the application's business).
  void SetMode(CameraMode mode);

  CameraSettings& settings() { return settings_; }
  const CameraSettings& settings() const { return settings_; }

  // Places the camera at `eye` looking at `target`, and uses `target` as the
  // orbit pivot.
  void LookAt(const Vec3& eye, const Vec3& target);

  // Smoothly frames a sphere of `radius` around `center`. In first person
  // the walker stays put and turns to face it instead.
  void Focus(const Vec3& center, double radius);

  // Switches to first person standing at `feet` (projected onto the ground
  // plane), facing `yaw` and looking `pitch` up (level by default).
  void EnterFirstPerson(const Vec3& feet, double yaw, double pitch = 0.0);

  // Advances the camera by `dt` seconds of real (unscaled) time.
  // `follow_target` is used in follow mode; without one, follow behaves like
  // orbit. `blockers` are respected in first person.
  void Update(const CameraInput& input, double dt,
              const std::optional<Vec3>& follow_target = std::nullopt,
              std::span<const WalkBlocker> blockers = {});

  const Vec3& pivot() const { return pivot_; }
  double distance() const { return distance_; }
  double yaw() const { return yaw_; }
  double pitch() const { return pitch_; }
  const Vec3& velocity() const { return velocity_; }
  // Ground position under the first-person eye.
  Vec3 feet() const;

 private:
  void UpdateFreeFly(const CameraInput& input, double dt);
  void UpdateOrbit(const CameraInput& input, double dt,
                   const std::optional<Vec3>& follow_target);
  void UpdateFirstPerson(const CameraInput& input, double dt,
                         std::span<const WalkBlocker> blockers);
  void TurnTowardLookGoal(double dt);
  void ApplyRotation(const Vec2& rotate_delta);
  void SyncOrbitPose();

  Camera camera_;
  CameraSettings settings_;
  CameraMode mode_ = CameraMode::kOrbit;

  double yaw_ = 0.0;
  double pitch_ = 0.0;
  Vec3 pivot_;
  double distance_ = 10.0;
  Vec3 velocity_;

  // Pending focus animation goals.
  std::optional<Vec3> pivot_goal_;
  std::optional<double> distance_goal_;
  // First person: a point to turn and face smoothly.
  std::optional<Vec3> look_goal_;
};

const char* CameraModeName(CameraMode mode);

// Default bindings from raw input to camera intent:
//   right mouse drag  rotate (look in free-fly and first person, orbit
//                     otherwise)
//   middle mouse drag pan
//   wheel             zoom (orbit/follow) or speed (free-fly, first person)
//   W/A/S/D, Q/E      move forward/left/back/right, down/up
//   Shift / Ctrl      faster / slower
// `use_mouse` and `use_keyboard` let the UI withhold input it is consuming.
// `pixels_per_point` (the display's pixel density) keeps mouse-look speed
// the same on high-DPI screens; panning stays in pixels so the grabbed
// point tracks the cursor exactly.
CameraInput CameraInputFromState(const InputState& input, bool use_mouse,
                                 bool use_keyboard, double viewport_height,
                                 double pixels_per_point = 1.0);

}  // namespace aerolab

#endif  // AEROLAB_CAMERA_CAMERA_CONTROLLER_H_
