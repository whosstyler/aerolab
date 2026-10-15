#include "camera/camera.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <optional>

#include "camera/camera_controller.h"
#include "core/input.h"
#include "math/projection.h"
#include "math/quat.h"
#include "test_util.h"

namespace aerolab {
namespace {

CameraInput Forward() {
  CameraInput input;
  input.move = {0.0, 0.0, 1.0};
  return input;
}

TEST(CameraTest, ViewMatrixMapsCameraToOrigin) {
  Camera camera;
  camera.position = {3, 4, 5};
  camera.orientation = FromEuler({.yaw = 0.3, .pitch = -0.2, .roll = 0.1});
  const Mat4 view = ViewMatrix(camera);
  EXPECT_VEC3_NEAR(TransformPoint(view, camera.position), Vec3::Zero(), 1e-12);
  // A point straight ahead ends up on the camera's -Z axis.
  const Vec3 ahead = camera.position + CameraForward(camera) * 7.0;
  EXPECT_VEC3_NEAR(TransformPoint(view, ahead), (Vec3{0, 0, -7}), 1e-12);
  EXPECT_VEC3_NEAR(WorldToCameraSpace(camera, ahead), (Vec3{0, 0, -7}), 1e-12);
  EXPECT_VEC3_NEAR(CameraToWorldSpace(camera, {0, 0, -7}), ahead, 1e-12);
}

TEST(CameraTest, ViewMatrixAgreesWithLookAt) {
  Camera camera;
  camera.position = {5, 2, 9};
  camera.orientation = LookRotation(-camera.position, Vec3::UnitY());
  EXPECT_TRUE(NearlyEqual(ViewMatrix(camera),
                          LookAt(camera.position, Vec3::Zero(), Vec3::UnitY()),
                          1e-12));
}

TEST(CameraTest, WorldUnitsPerPixelScalesWithDistance) {
  Camera camera;
  camera.vertical_fov_radians = Radians(90.0);
  // A 90 degree FOV shows 2 * distance vertically.
  EXPECT_NEAR(WorldUnitsPerPixel(camera, 10.0, 200.0), 0.1, 1e-12);
}

TEST(CameraRigTest, LookAtFramesTarget) {
  CameraRig rig;
  rig.LookAt({10, 5, 10}, {0, 1, 0});
  EXPECT_VEC3_NEAR(rig.camera().position, (Vec3{10, 5, 10}), 1e-9);
  EXPECT_VEC3_NEAR(CameraForward(rig.camera()), Normalized(Vec3{-10, -4, -10}),
                   1e-9);
  EXPECT_VEC3_NEAR(rig.pivot(), (Vec3{0, 1, 0}), 1e-12);
}

TEST(CameraRigTest, FreeFlyIsFrameRateIndependent) {
  CameraRig coarse;
  CameraRig fine;
  coarse.SetMode(CameraMode::kFreeFly);
  fine.SetMode(CameraMode::kFreeFly);
  // One 0.5 s frame versus fifty 10 ms frames of the same input, starting
  // from rest with velocity smoothing enabled.
  coarse.Update(Forward(), 0.5);
  for (int i = 0; i < 50; ++i) fine.Update(Forward(), 0.01);
  EXPECT_VEC3_NEAR(coarse.camera().position, fine.camera().position, 1e-9);
  EXPECT_VEC3_NEAR(coarse.velocity(), fine.velocity(), 1e-9);
}

TEST(CameraRigTest, FreeFlyReachesCruiseSpeed) {
  CameraRig rig;
  rig.SetMode(CameraMode::kFreeFly);
  rig.settings().move_sharpness = 0.0;  // Instant response.
  const Vec3 start = rig.camera().position;
  const Vec3 forward = CameraForward(rig.camera());
  rig.Update(Forward(), 2.0);
  EXPECT_VEC3_NEAR(rig.camera().position,
                   start + forward * (rig.settings().move_speed * 2.0), 1e-9);
}

TEST(CameraRigTest, OrbitRotationKeepsDistanceToPivot) {
  CameraRig rig;
  rig.LookAt({0, 0, 10}, Vec3::Zero());
  CameraInput input;
  input.rotate_delta = {120.0, -40.0};
  rig.Update(input, 0.016);
  EXPECT_NEAR(Distance(rig.camera().position, rig.pivot()), 10.0, 1e-9);
  // The camera still looks straight at the pivot.
  EXPECT_VEC3_NEAR(CameraForward(rig.camera()),
                   Normalized(rig.pivot() - rig.camera().position), 1e-9);
}

TEST(CameraRigTest, MouseLookSignConventions) {
  CameraRig rig;
  rig.SetMode(CameraMode::kFreeFly);
  rig.LookAt({0, 2, 10}, {0, 2, 0});  // Looking along -Z, level.
  CameraInput right;
  right.rotate_delta = {50.0, 0.0};
  rig.Update(right, 0.016);
  // Moving the mouse right turns the view right (toward +X).
  EXPECT_GT(CameraForward(rig.camera()).x, 0.0);
  CameraInput down;
  down.rotate_delta = {0.0, 50.0};
  rig.Update(down, 0.016);
  // Moving the mouse down looks down.
  EXPECT_LT(CameraForward(rig.camera()).y, 0.0);
}

TEST(CameraRigTest, FreeFlyPanSlidesInViewPlane) {
  CameraRig rig;
  rig.LookAt({0, 2, 10}, {0, 2, 0});
  rig.SetMode(CameraMode::kFreeFly);
  const Vec3 start = rig.camera().position;
  CameraInput input;
  input.pan_delta = {0.0, 40.0};  // Drag down: the scene follows the cursor.
  rig.Update(input, 0.016);
  EXPECT_GT(rig.camera().position.y, start.y);
  EXPECT_NEAR(rig.camera().position.z, start.z, 1e-9);
}

TEST(CameraRigTest, FocusFromFreeFlyKeepsOrientation) {
  CameraRig rig;
  rig.LookAt({0, 2, 10}, {0, 2, 0});
  rig.SetMode(CameraMode::kFreeFly);
  const Quat orientation = rig.camera().orientation;
  rig.Focus({0, 2, -30}, 1.0);
  for (int i = 0; i < 600; ++i) rig.Update(CameraInput{}, 1.0 / 60.0);
  EXPECT_SAME_ROTATION(rig.camera().orientation, orientation, 1e-9);
  // The target ends up straight ahead.
  const Vec3 to_target = Normalized(Vec3{0, 2, -30} - rig.camera().position);
  EXPECT_VEC3_NEAR(to_target, CameraForward(rig.camera()), 1e-6);
}

TEST(CameraRigTest, PitchIsClampedShortOfVertical) {
  CameraRig rig;
  CameraInput input;
  input.rotate_delta = {0.0, -1e6};  // Huge upward drag.
  rig.Update(input, 0.016);
  EXPECT_LT(rig.pitch(), kHalfPi);
  EXPECT_GT(rig.pitch(), Radians(88.0));
}

TEST(CameraRigTest, ZoomStepsComposeMultiplicatively) {
  CameraRig a;
  CameraRig b;
  a.LookAt({0, 0, 20}, Vec3::Zero());
  b.LookAt({0, 0, 20}, Vec3::Zero());
  CameraInput two_notches;
  two_notches.wheel = 2.0;
  CameraInput one_notch;
  one_notch.wheel = 1.0;
  a.Update(two_notches, 0.016);
  b.Update(one_notch, 0.016);
  b.Update(one_notch, 0.016);
  EXPECT_NEAR(a.distance(), b.distance(), 1e-9);
  EXPECT_LT(a.distance(), 20.0);
}

TEST(CameraRigTest, PanMovesPivotInViewPlane) {
  CameraRig rig;
  rig.LookAt({0, 0, 10}, Vec3::Zero());
  CameraInput input;
  input.pan_delta = {100.0, 0.0};
  input.viewport_height = 500.0;
  rig.Update(input, 0.016);
  // Dragging right moves the pivot left (-X) without changing depth.
  EXPECT_LT(rig.pivot().x, 0.0);
  EXPECT_NEAR(rig.pivot().z, 0.0, 1e-9);
  EXPECT_NEAR(Distance(rig.camera().position, rig.pivot()), 10.0, 1e-9);
}

TEST(CameraRigTest, SwitchingModesKeepsTheView) {
  CameraRig rig;
  rig.LookAt({4, 3, 8}, {0, 1, 0});
  const Camera before = rig.camera();
  rig.SetMode(CameraMode::kFreeFly);
  rig.Update(CameraInput{}, 0.016);
  rig.SetMode(CameraMode::kOrbit);
  rig.Update(CameraInput{}, 0.016);
  EXPECT_VEC3_NEAR(rig.camera().position, before.position, 1e-9);
  EXPECT_SAME_ROTATION(rig.camera().orientation, before.orientation, 1e-9);
}

TEST(CameraRigTest, FocusConvergesOnTarget) {
  CameraRig rig;
  rig.Focus({20, 3, -5}, 1.0);
  for (int i = 0; i < 400; ++i) rig.Update(CameraInput{}, 1.0 / 60.0);
  EXPECT_VEC3_NEAR(rig.pivot(), (Vec3{20, 3, -5}), 1e-9);
  EXPECT_VEC3_NEAR(CameraForward(rig.camera()),
                   Normalized(rig.pivot() - rig.camera().position), 1e-9);
}

TEST(CameraRigTest, FollowTracksMovingTarget) {
  CameraRig rig;
  rig.SetMode(CameraMode::kFollow);
  Vec3 target{0, 2, 0};
  for (int i = 0; i < 600; ++i) {
    target += Vec3{0.1, 0, 0};
    rig.Update(CameraInput{}, 1.0 / 60.0, target);
  }
  // Exponential tracking lags a constant-velocity target by a fixed amount.
  EXPECT_LT(Distance(rig.pivot(), target), 1.0);
}

TEST(FirstPersonTest, EntersStandingOnTheGroundAtEyeHeight) {
  CameraRig rig;
  rig.EnterFirstPerson({4.0, 9.0, -2.0}, 0.5);
  EXPECT_EQ(rig.mode(), CameraMode::kFirstPerson);
  EXPECT_VEC3_NEAR(rig.feet(), (Vec3{4.0, 0.0, -2.0}), 1e-12);
  EXPECT_VEC3_NEAR(rig.camera().position,
                   (Vec3{4.0, rig.settings().eye_height, -2.0}), 1e-12);
  EXPECT_NEAR(rig.yaw(), 0.5, 1e-12);
  EXPECT_NEAR(rig.pitch(), 0.0, 1e-12);
}

TEST(FirstPersonTest, WalksAlongTheGroundAtWalkingSpeed) {
  CameraRig rig;
  rig.EnterFirstPerson({0.0, 0.0, 0.0}, 0.0);  // Facing -Z.
  CameraInput input = Forward();
  input.rotate_delta = {0.0, 150.0};  // Looking down does not dig in.
  rig.Update(input, 0.0);
  ASSERT_LT(rig.pitch(), -0.3);
  input.rotate_delta = {};
  for (int i = 0; i < 200; ++i) rig.Update(input, 0.01);
  EXPECT_NEAR(Length(rig.velocity()), rig.settings().walk_speed, 1e-3);
  EXPECT_NEAR(rig.feet().y, 0.0, 1e-12);
  EXPECT_LT(rig.feet().z, -5.0);
  EXPECT_NEAR(rig.feet().x, 0.0, 1e-9);
  EXPECT_NEAR(rig.camera().position.y, rig.settings().eye_height, 1e-12);
  // Q/E do not lift a walker off the ground.
  input.move = {0.0, 1.0, 0.0};
  for (int i = 0; i < 50; ++i) rig.Update(input, 0.01);
  EXPECT_NEAR(rig.feet().y, 0.0, 1e-12);
}

TEST(FirstPersonTest, RunningAndCreepingScaleTheSpeed) {
  const auto settled_speed = [](bool boost, bool slow) {
    CameraRig rig;
    rig.EnterFirstPerson({}, 0.0);
    CameraInput input = Forward();
    input.boost = boost;
    input.slow = slow;
    for (int i = 0; i < 300; ++i) rig.Update(input, 0.01);
    return Length(rig.velocity());
  };
  const double walk = settled_speed(false, false);
  EXPECT_NEAR(settled_speed(true, false), walk * 2.5, 1e-3);
  EXPECT_LT(settled_speed(false, true), walk);
}

TEST(FirstPersonTest, WalkingIsFrameRateIndependent) {
  CameraRig a;
  CameraRig b;
  a.EnterFirstPerson({}, 0.3);
  b.EnterFirstPerson({}, 0.3);
  a.Update(Forward(), 0.5);
  for (int i = 0; i < 50; ++i) b.Update(Forward(), 0.01);
  EXPECT_VEC3_NEAR(a.feet(), b.feet(), 1e-9);
}

TEST(FirstPersonTest, CannotWalkThroughBlockers) {
  CameraRig rig;
  rig.EnterFirstPerson({0.0, 0.0, 0.0}, 0.0);  // Facing -Z.
  const std::array<WalkBlocker, 1> pillar = {
      WalkBlocker{{0.0, 0.0, -5.0}, 1.0}};
  for (int i = 0; i < 500; ++i) {
    rig.Update(Forward(), 0.01, std::nullopt, pillar);
    const Vec3 feet = rig.feet();
    ASSERT_GE(std::hypot(feet.x - 0.0, feet.z + 5.0),
              1.0 + rig.settings().walker_radius - 1e-9);
  }
  // Pressed against it, the walker stops at its surface.
  EXPECT_NEAR(rig.feet().z, -5.0 + 1.0 + rig.settings().walker_radius, 1e-6);
}

TEST(FirstPersonTest, FastWalkersAndLongFramesDoNotTunnel) {
  CameraRig rig;
  rig.settings().walk_speed = 40.0;  // The fastest the wheel allows.
  rig.EnterFirstPerson({0.0, 0.0, 0.0}, 0.0);
  CameraInput input = Forward();
  input.boost = true;  // 100 m/s.
  const std::array<WalkBlocker, 1> post = {WalkBlocker{{0.0, 0.0, -20.0}, 0.2}};
  for (int i = 0; i < 60; ++i) {
    rig.Update(input, 1.0 / 15.0, std::nullopt, post);  // 6+ m per frame.
    ASSERT_GT(rig.feet().z, -20.0);
  }
  EXPECT_NEAR(rig.feet().z, -20.0 + 0.2 + rig.settings().walker_radius, 1e-6);
}

TEST(FirstPersonTest, SlidesAroundAnOffCentreBlocker) {
  CameraRig rig;
  rig.EnterFirstPerson({0.6, 0.0, 0.0}, 0.0);
  const std::array<WalkBlocker, 1> pillar = {
      WalkBlocker{{0.0, 0.0, -5.0}, 1.0}};
  for (int i = 0; i < 400; ++i) {
    rig.Update(Forward(), 0.01, std::nullopt, pillar);
    const Vec3 feet = rig.feet();
    ASSERT_GE(std::hypot(feet.x, feet.z + 5.0), 1.3 - 1e-9);
  }
  EXPECT_LT(rig.feet().z, -7.0);  // Got past it.
}

TEST(FirstPersonTest, StartingInsideABlockerStepsOut) {
  CameraRig rig;
  rig.EnterFirstPerson({0.2, 0.0, 0.0}, 0.0);
  const std::array<WalkBlocker, 1> crate = {WalkBlocker{{0.0, 0.0, 0.0}, 1.0}};
  rig.Update(CameraInput{}, 0.01, std::nullopt, crate);
  EXPECT_NEAR(Length(rig.feet()), 1.3, 1e-9);
}

TEST(FirstPersonTest, WalksThroughAGapBetweenBlockers) {
  CameraRig rig;
  rig.EnterFirstPerson({0.0, 0.0, 0.0}, 0.0);
  const std::array<WalkBlocker, 2> gate = {WalkBlocker{{-1.5, 0.0, -5.0}, 1.0},
                                           WalkBlocker{{1.5, 0.0, -5.0}, 1.0}};
  for (int i = 0; i < 400; ++i) {
    rig.Update(Forward(), 0.01, std::nullopt, gate);
  }
  EXPECT_LT(rig.feet().z, -8.0);
  EXPECT_NEAR(rig.feet().x, 0.0, 1e-9);
}

TEST(FirstPersonTest, InstantResponseWithoutSmoothing) {
  CameraRig rig;
  rig.settings().move_sharpness = 0.0;
  rig.EnterFirstPerson({}, 0.0);
  rig.Update(Forward(), 0.5);
  EXPECT_VEC3_NEAR(rig.feet(), (Vec3{0.0, 0.0, -2.0}), 1e-12);
}

TEST(FirstPersonTest, BadInputIsIgnored) {
  CameraRig rig;
  rig.EnterFirstPerson({1.0, 0.0, 2.0}, 0.0);
  CameraInput input = Forward();
  input.wheel = std::nan("");
  rig.Update(input, std::nan(""));
  EXPECT_VEC3_NEAR(rig.feet(), (Vec3{1.0, 0.0, 2.0}), 1e-12);
  EXPECT_EQ(rig.settings().walk_speed, CameraSettings{}.walk_speed);
  EXPECT_FALSE(WalkBlockerFromSphere({0.0, 1.0, 0.0}, 1e9, 1.8).has_value());
  EXPECT_FALSE(
      WalkBlockerFromSphere({std::nan(""), 1.0, 0.0}, 1.0, 1.8).has_value());
}

TEST(FirstPersonTest, SpheresBlockOnlyWithinBodyHeight) {
  // A unit sphere resting on the ground blocks with its full radius.
  const std::optional<WalkBlocker> ball =
      WalkBlockerFromSphere({3.0, 1.0, 4.0}, 1.0, 1.8);
  ASSERT_TRUE(ball.has_value());
  EXPECT_NEAR(ball->radius, 1.0, 1e-12);
  EXPECT_NEAR(ball->center.x, 3.0, 1e-12);
  // Hanging overhead or buried below: no blocker.
  EXPECT_FALSE(WalkBlockerFromSphere({0.0, 5.0, 0.0}, 1.0, 1.8).has_value());
  EXPECT_FALSE(WalkBlockerFromSphere({0.0, -3.0, 0.0}, 1.0, 1.8).has_value());
  // Partly overlapping the body: its section at the nearest height.
  const std::optional<WalkBlocker> low =
      WalkBlockerFromSphere({0.0, 2.3, 0.0}, 1.0, 1.8);
  ASSERT_TRUE(low.has_value());
  EXPECT_NEAR(low->radius, std::sqrt(1.0 - 0.25), 1e-12);
}

TEST(FirstPersonTest, FocusTurnsToFaceWithoutMoving) {
  CameraRig rig;
  rig.EnterFirstPerson({0.0, 0.0, 0.0}, 0.0);
  rig.Focus({10.0, 1.65, 0.0}, 1.0);  // Off to the right (+X).
  for (int i = 0; i < 200; ++i) rig.Update(CameraInput{}, 0.01);
  EXPECT_VEC3_NEAR(rig.feet(), Vec3::Zero(), 1e-12);
  EXPECT_VEC3_NEAR(CameraForward(rig.camera()), Vec3::UnitX(), 1e-3);
}

TEST(FirstPersonTest, LookingAroundCancelsTheTurn) {
  CameraRig rig;
  rig.EnterFirstPerson({0.0, 0.0, 0.0}, 0.0);
  rig.Focus({10.0, 1.65, 0.0}, 1.0);
  CameraInput look;
  look.rotate_delta = {-1.0, 0.0};
  rig.Update(look, 0.01);
  const double yaw = rig.yaw();
  for (int i = 0; i < 100; ++i) rig.Update(CameraInput{}, 0.01);
  EXPECT_NEAR(rig.yaw(), yaw, 1e-12);  // No longer turning.
}

TEST(FirstPersonTest, FacingStraightDownKeepsTheHeading) {
  CameraRig rig;
  rig.EnterFirstPerson({0.0, 0.0, 0.0}, 0.7);
  rig.Focus({0.0, 0.0, 0.0}, 0.5);  // Its own feet.
  for (int i = 0; i < 200; ++i) rig.Update(CameraInput{}, 0.01);
  EXPECT_NEAR(rig.yaw(), 0.7, 1e-12);
  EXPECT_LT(rig.pitch(), -1.5);
}

TEST(FirstPersonTest, EnteringDropsToTheGroundKeepingTheViewDirection) {
  CameraRig rig;
  rig.LookAt({6.0, 8.0, 6.0}, {0.0, 0.0, 0.0});
  const Vec3 looking = CameraForward(rig.camera());
  rig.SetMode(CameraMode::kFirstPerson);
  EXPECT_VEC3_NEAR(rig.feet(), (Vec3{6.0, 0.0, 6.0}), 1e-12);
  EXPECT_VEC3_NEAR(CameraForward(rig.camera()), looking, 1e-12);
}

TEST(FirstPersonTest, LeavingKeepsTheView) {
  CameraRig rig;
  rig.LookAt({6.0, 8.0, 6.0}, {0.0, 0.0, 0.0});
  rig.SetMode(CameraMode::kFirstPerson);
  const Vec3 eye = rig.camera().position;
  const Vec3 forward = CameraForward(rig.camera());
  rig.SetMode(CameraMode::kOrbit);
  rig.Update(CameraInput{}, 0.0);
  EXPECT_VEC3_NEAR(rig.camera().position, eye, 1e-9);
  EXPECT_VEC3_NEAR(CameraForward(rig.camera()), forward, 1e-9);
}

TEST(CameraInputMappingTest, KeysAndButtonsMapToIntent) {
  InputState input;
  input.BeginFrame();
  input.SetKey(Key::kW, true);
  input.SetKey(Key::kD, true);
  input.SetKey(Key::kShift, true);
  input.SetMouseButton(MouseButton::kRight, true);
  input.AddMouseDelta({5, -3});
  input.AddWheel(2.0);
  const CameraInput mapped = CameraInputFromState(input, true, true, 480.0);
  EXPECT_VEC3_NEAR(mapped.move, (Vec3{1, 0, 1}), 1e-12);
  EXPECT_TRUE(mapped.boost);
  EXPECT_DOUBLE_EQ(mapped.rotate_delta.x, 5.0);
  EXPECT_DOUBLE_EQ(mapped.pan_delta.x, 0.0);
  EXPECT_DOUBLE_EQ(mapped.wheel, 2.0);
  EXPECT_DOUBLE_EQ(mapped.viewport_height, 480.0);

  // On a 2x display the same physical motion produces twice the pixels;
  // the rotation delta is normalized back to points.
  const CameraInput hidpi = CameraInputFromState(input, true, true, 480.0, 2.0);
  EXPECT_DOUBLE_EQ(hidpi.rotate_delta.x, 2.5);

  const CameraInput blocked = CameraInputFromState(input, false, false, 480.0);
  EXPECT_VEC3_NEAR(blocked.move, Vec3::Zero(), 1e-12);
  EXPECT_DOUBLE_EQ(blocked.wheel, 0.0);
}

}  // namespace
}  // namespace aerolab
