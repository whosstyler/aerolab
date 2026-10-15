#include "core/input.h"

namespace aerolab {

void InputState::Apply(ButtonState& state, bool down) {
  if (down && !state.down) state.pressed = true;
  if (!down && state.down) state.released = true;
  state.down = down;
}

void InputState::BeginFrame() {
  for (ButtonState& key : keys_) {
    key.pressed = key.released = key.cancelled = false;
  }
  for (ButtonState& button : buttons_) {
    button.pressed = button.released = button.cancelled = false;
  }
  mouse_delta_ = Vec2{};
  wheel_ = 0.0;
}

void InputState::SetKey(Key key, bool down) {
  if (key == Key::kCount) return;
  Apply(keys_[Index(key)], down);
}

void InputState::SetMouseButton(MouseButton button, bool down) {
  if (button == MouseButton::kCount) return;
  ButtonState& state = buttons_[Index(button)];
  if (down && !state.down) {
    press_positions_[Index(button)] = mouse_position_;
    state.cancelled = false;
  }
  Apply(state, down);
}

void InputState::ReleaseAll() {
  for (ButtonState& key : keys_) {
    if (key.down) key.cancelled = true;
    Apply(key, false);
  }
  for (ButtonState& button : buttons_) {
    if (button.down) button.cancelled = true;
    Apply(button, false);
  }
}

bool InputState::WasClicked(MouseButton button, double slop) const {
  if (button == MouseButton::kCount) return false;
  const ButtonState& state = buttons_[Index(button)];
  if (!state.released || state.cancelled) return false;
  return Length(mouse_position_ - press_positions_[Index(button)]) < slop;
}

}  // namespace aerolab
