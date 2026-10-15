#include "app/sdl_input.h"

#include <SDL3/SDL_events.h>
#include <SDL3/SDL_mouse.h>
#include <SDL3/SDL_scancode.h>
#include <gtest/gtest.h>

#include "core/input.h"

namespace aerolab {
namespace {

SDL_Event KeyEvent(SDL_Scancode scancode, bool down, bool repeat = false) {
  SDL_Event event{};
  event.type = down ? SDL_EVENT_KEY_DOWN : SDL_EVENT_KEY_UP;
  event.key.scancode = scancode;
  event.key.repeat = repeat;
  return event;
}

TEST(SdlInputTest, KeysMapByScancode) {
  InputState input;
  input.BeginFrame();
  ApplySdlEvent(KeyEvent(SDL_SCANCODE_W, true), 1.0f, input);
  ApplySdlEvent(KeyEvent(SDL_SCANCODE_RSHIFT, true), 1.0f, input);
  EXPECT_TRUE(input.IsDown(Key::kW));
  EXPECT_TRUE(input.IsDown(Key::kShift));
  ApplySdlEvent(KeyEvent(SDL_SCANCODE_W, false), 1.0f, input);
  EXPECT_FALSE(input.IsDown(Key::kW));
}

TEST(SdlInputTest, KeyRepeatDoesNotRetrigger) {
  InputState input;
  input.BeginFrame();
  ApplySdlEvent(KeyEvent(SDL_SCANCODE_SPACE, true), 1.0f, input);
  input.BeginFrame();
  ApplySdlEvent(KeyEvent(SDL_SCANCODE_SPACE, true, /*repeat=*/true), 1.0f,
                input);
  EXPECT_FALSE(input.WasPressed(Key::kSpace));
  EXPECT_TRUE(input.IsDown(Key::kSpace));
}

TEST(SdlInputTest, MouseMotionScalesToPixels) {
  InputState input;
  input.BeginFrame();
  SDL_Event motion{};
  motion.type = SDL_EVENT_MOUSE_MOTION;
  motion.motion.x = 10.0f;
  motion.motion.y = 20.0f;
  motion.motion.xrel = 3.0f;
  motion.motion.yrel = -1.0f;
  ApplySdlEvent(motion, 2.0f, input);
  EXPECT_DOUBLE_EQ(input.mouse_position().x, 20.0);
  EXPECT_DOUBLE_EQ(input.mouse_position().y, 40.0);
  EXPECT_DOUBLE_EQ(input.mouse_delta().x, 6.0);
  EXPECT_DOUBLE_EQ(input.mouse_delta().y, -2.0);
}

TEST(SdlInputTest, ButtonsWheelAndWindowEvents) {
  InputState input;
  input.BeginFrame();
  SDL_Event press{};
  press.type = SDL_EVENT_MOUSE_BUTTON_DOWN;
  press.button.button = SDL_BUTTON_RIGHT;
  press.button.x = 5.0f;
  press.button.y = 6.0f;
  ApplySdlEvent(press, 1.0f, input);
  EXPECT_TRUE(input.IsDown(MouseButton::kRight));
  EXPECT_DOUBLE_EQ(input.press_position(MouseButton::kRight).x, 5.0);

  SDL_Event wheel{};
  wheel.type = SDL_EVENT_MOUSE_WHEEL;
  wheel.wheel.y = -1.5f;
  ApplySdlEvent(wheel, 1.0f, input);
  EXPECT_DOUBLE_EQ(input.wheel(), -1.5);

  SDL_Event leave{};
  leave.type = SDL_EVENT_WINDOW_MOUSE_LEAVE;
  ApplySdlEvent(leave, 1.0f, input);
  EXPECT_FALSE(input.mouse_inside());

  SDL_Event focus_lost{};
  focus_lost.type = SDL_EVENT_WINDOW_FOCUS_LOST;
  ApplySdlEvent(focus_lost, 1.0f, input);
  EXPECT_FALSE(input.IsDown(MouseButton::kRight));
}

}  // namespace
}  // namespace aerolab
