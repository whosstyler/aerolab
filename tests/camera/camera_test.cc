#include "camera/camera.h"

#include <gtest/gtest.h>

#include <cmath>

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
