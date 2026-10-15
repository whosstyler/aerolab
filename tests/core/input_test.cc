#include "core/input.h"

#include <gtest/gtest.h>

namespace aerolab {
namespace {

TEST(InputStateTest, PressAndReleaseEdgesLastOneFrame) {
  InputState input;
  input.BeginFrame();
  input.SetKey(Key::kW, true);
  EXPECT_TRUE(input.IsDown(Key::kW));
  EXPECT_TRUE(input.WasPressed(Key::kW));
  EXPECT_FALSE(input.WasReleased(Key::kW));

  input.BeginFrame();
  EXPECT_TRUE(input.IsDown(Key::kW));
  EXPECT_FALSE(input.WasPressed(Key::kW));

  input.SetKey(Key::kW, false);
  EXPECT_FALSE(input.IsDown(Key::kW));
  EXPECT_TRUE(input.WasReleased(Key::kW));
}

TEST(InputStateTest, TapWithinOneFrameIsStillSeen) {
  InputState input;
  input.BeginFrame();
  input.SetMouseButton(MouseButton::kLeft, true);
  input.SetMouseButton(MouseButton::kLeft, false);
  EXPECT_FALSE(input.IsDown(MouseButton::kLeft));
  EXPECT_TRUE(input.WasPressed(MouseButton::kLeft));
  EXPECT_TRUE(input.WasReleased(MouseButton::kLeft));
}

TEST(InputStateTest, RepeatedDownDoesNotRetrigger) {
  InputState input;
  input.BeginFrame();
  input.SetKey(Key::kE, true);
  input.BeginFrame();
  input.SetKey(Key::kE, true);
  EXPECT_FALSE(input.WasPressed(Key::kE));
}

TEST(InputStateTest, MouseDeltaAndWheelAccumulatePerFrame) {
  InputState input;
  input.BeginFrame();
  input.AddMouseDelta({3, 4});
  input.AddMouseDelta({1, -2});
  input.AddWheel(1.0);
  input.AddWheel(0.5);
  input.SetMousePosition({100, 50});
  EXPECT_DOUBLE_EQ(input.mouse_delta().x, 4.0);
  EXPECT_DOUBLE_EQ(input.mouse_delta().y, 2.0);
  EXPECT_DOUBLE_EQ(input.wheel(), 1.5);
  input.BeginFrame();
  EXPECT_DOUBLE_EQ(input.mouse_delta().x, 0.0);
  EXPECT_DOUBLE_EQ(input.wheel(), 0.0);
  EXPECT_DOUBLE_EQ(input.mouse_position().x, 100.0);  // Position persists.
}

TEST(InputStateTest, ReleaseAllClearsHeldState) {
  InputState input;
  input.SetKey(Key::kShift, true);
  input.SetMouseButton(MouseButton::kRight, true);
  input.BeginFrame();
  input.ReleaseAll();
  EXPECT_FALSE(input.IsDown(Key::kShift));
  EXPECT_FALSE(input.IsDown(MouseButton::kRight));
  EXPECT_TRUE(input.WasReleased(MouseButton::kRight));
}

TEST(InputStateTest, ClickRequiresSmallMovementBetweenPressAndRelease) {
  InputState input;
  input.BeginFrame();
  input.SetMousePosition({100, 100});
  input.SetMouseButton(MouseButton::kLeft, true);
  EXPECT_DOUBLE_EQ(input.press_position(MouseButton::kLeft).x, 100.0);
  input.BeginFrame();
  input.SetMousePosition({102, 101});
  input.SetMouseButton(MouseButton::kLeft, false);
  EXPECT_TRUE(input.WasClicked(MouseButton::kLeft, 4.0));

  // A drag is not a click.
  input.BeginFrame();
  input.SetMouseButton(MouseButton::kLeft, true);
  input.SetMousePosition({160, 101});
  input.SetMouseButton(MouseButton::kLeft, false);
  EXPECT_FALSE(input.WasClicked(MouseButton::kLeft, 4.0));
  // Nothing was released this frame: no click.
  input.BeginFrame();
  EXPECT_FALSE(input.WasClicked(MouseButton::kLeft, 4.0));
}

TEST(InputStateTest, ForcedReleaseIsNotAClick) {
  InputState input;
  input.BeginFrame();
  input.SetMouseButton(MouseButton::kLeft, true);
  input.BeginFrame();
  input.ReleaseAll();  // e.g. the window lost focus mid-press.
  EXPECT_TRUE(input.WasReleased(MouseButton::kLeft));
  EXPECT_FALSE(input.WasClicked(MouseButton::kLeft, 4.0));
}

TEST(InputStateTest, MouseInsideFlag) {
  InputState input;
  EXPECT_TRUE(input.mouse_inside());
  input.SetMouseInside(false);
  EXPECT_FALSE(input.mouse_inside());
}

TEST(InputStateTest, SentinelValuesAreIgnored) {
  InputState input;
  input.SetKey(Key::kCount, true);
  input.SetMouseButton(MouseButton::kCount, true);
  SUCCEED();
}

}  // namespace
}  // namespace aerolab
