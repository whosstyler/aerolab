#include "app/sdl_input.h"

#include <SDL3/SDL_events.h>
#include <SDL3/SDL_mouse.h>
#include <SDL3/SDL_scancode.h>

#include "math/vec2.h"

namespace aerolab {
namespace {

// Scancodes are layout independent, so WASD stays in place on AZERTY etc.
Key KeyFromScancode(SDL_Scancode scancode) {
  switch (scancode) {
    case SDL_SCANCODE_W:
      return Key::kW;
    case SDL_SCANCODE_A:
      return Key::kA;
    case SDL_SCANCODE_S:
      return Key::kS;
    case SDL_SCANCODE_D:
      return Key::kD;
    case SDL_SCANCODE_Q:
      return Key::kQ;
    case SDL_SCANCODE_E:
      return Key::kE;
    case SDL_SCANCODE_F:
      return Key::kF;
    case SDL_SCANCODE_R:
      return Key::kR;
    case SDL_SCANCODE_SPACE:
      return Key::kSpace;
    case SDL_SCANCODE_PERIOD:
      return Key::kPeriod;
    case SDL_SCANCODE_DELETE:
      return Key::kDelete;
    case SDL_SCANCODE_ESCAPE:
      return Key::kEscape;
    case SDL_SCANCODE_LSHIFT:
    case SDL_SCANCODE_RSHIFT:
      return Key::kShift;
    case SDL_SCANCODE_LCTRL:
    case SDL_SCANCODE_RCTRL:
      return Key::kCtrl;
    case SDL_SCANCODE_LALT:
    case SDL_SCANCODE_RALT:
      return Key::kAlt;
    case SDL_SCANCODE_1:
      return Key::k1;
    case SDL_SCANCODE_2:
      return Key::k2;
    case SDL_SCANCODE_3:
      return Key::k3;
    default:
      return Key::kCount;
  }
}

MouseButton ButtonFromSdl(Uint8 button) {
  switch (button) {
    case SDL_BUTTON_LEFT:
      return MouseButton::kLeft;
    case SDL_BUTTON_MIDDLE:
      return MouseButton::kMiddle;
    case SDL_BUTTON_RIGHT:
      return MouseButton::kRight;
    default:
      return MouseButton::kCount;
  }
}

}  // namespace

void ApplySdlEvent(const SDL_Event& event, float pixel_density,
                   InputState& input) {
  const double scale = pixel_density > 0.0f ? pixel_density : 1.0f;
  switch (event.type) {
    case SDL_EVENT_KEY_DOWN:
    case SDL_EVENT_KEY_UP:
      if (!event.key.repeat) {
        input.SetKey(KeyFromScancode(event.key.scancode),
                     event.type == SDL_EVENT_KEY_DOWN);
      }
      break;
    case SDL_EVENT_MOUSE_BUTTON_DOWN:
    case SDL_EVENT_MOUSE_BUTTON_UP:
      // Button events carry their own position; use it so press and
      // release locations are exact even without motion events.
      input.SetMousePosition(
          Vec2{event.button.x * scale, event.button.y * scale});
      input.SetMouseButton(ButtonFromSdl(event.button.button),
                           event.type == SDL_EVENT_MOUSE_BUTTON_DOWN);
      break;
    case SDL_EVENT_MOUSE_MOTION:
      input.SetMousePosition(
          Vec2{event.motion.x * scale, event.motion.y * scale});
      input.AddMouseDelta(
          Vec2{event.motion.xrel * scale, event.motion.yrel * scale});
      break;
    case SDL_EVENT_MOUSE_WHEEL:
      input.AddWheel(event.wheel.y);
      break;
    case SDL_EVENT_WINDOW_FOCUS_LOST:
      input.ReleaseAll();
      break;
    case SDL_EVENT_WINDOW_MOUSE_ENTER:
      input.SetMouseInside(true);
      break;
    case SDL_EVENT_WINDOW_MOUSE_LEAVE:
      input.SetMouseInside(false);
      break;
    default:
      break;
  }
}

}  // namespace aerolab
