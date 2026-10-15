#ifndef AEROLAB_CORE_INPUT_H_
#define AEROLAB_CORE_INPUT_H_

#include <array>
#include <cstddef>

#include "math/vec2.h"

namespace aerolab {

// Keys the engine itself reacts to (camera navigation). The platform layer
// translates its own key codes into these, so nothing outside the platform
// layer depends on SDL. Editor shortcuts go through the UI toolkit instead.
enum class Key {
  kW,
  kA,
  kS,
  kD,
  kQ,
  kE,
  kShift,
  kCtrl,
  kCount,
};

enum class MouseButton { kLeft, kMiddle, kRight, kCount };

// Snapshot of keyboard and mouse state with per-frame edge detection.
//
// The platform layer calls BeginFrame() once per frame before feeding that
// frame's events. Queries then report the current level (IsDown) and edges
// that happened during the frame (WasPressed/WasReleased), even if a key went
// down and up again within a single frame.
class InputState {
 public:
  InputState() = default;

  // --- Producer side (platform layer) ---
  void BeginFrame();
  void SetKey(Key key, bool down);
  void SetMouseButton(MouseButton button, bool down);
  void SetMousePosition(const Vec2& position) { mouse_position_ = position; }
  void AddMouseDelta(const Vec2& delta) { mouse_delta_ += delta; }
  void AddWheel(double notches) { wheel_ += notches; }
  void SetMouseInside(bool inside) { mouse_inside_ = inside; }
  // Releases everything, e.g. when the window loses focus. Releases caused
  // this way never count as clicks.
  void ReleaseAll();

  // --- Consumer side ---
  bool IsDown(Key key) const { return keys_[Index(key)].down; }
  bool WasPressed(Key key) const { return keys_[Index(key)].pressed; }
  bool WasReleased(Key key) const { return keys_[Index(key)].released; }

  bool IsDown(MouseButton button) const { return buttons_[Index(button)].down; }
  bool WasPressed(MouseButton button) const {
    return buttons_[Index(button)].pressed;
  }
  bool WasReleased(MouseButton button) const {
    return buttons_[Index(button)].released;
  }

  // True when the button was released this frame close to where it was
  // pressed (within `slop` pixels): a click rather than a drag. Releases
  // forced by ReleaseAll() are never clicks.
  bool WasClicked(MouseButton button, double slop) const;

  // Where the button last went down, in window pixels.
  const Vec2& press_position(MouseButton button) const {
    return press_positions_[Index(button)];
  }

  // False once the pointer has left the window, until it returns.
  bool mouse_inside() const { return mouse_inside_; }
  // Pointer position in window pixels (top-left origin).
  const Vec2& mouse_position() const { return mouse_position_; }
  // Accumulated pointer motion this frame, in pixels.
  const Vec2& mouse_delta() const { return mouse_delta_; }
  // Accumulated wheel notches this frame; positive is away from the user.
  double wheel() const { return wheel_; }

 private:
  struct ButtonState {
    bool down = false;
    bool pressed = false;
    bool released = false;
    bool cancelled = false;  // Released by ReleaseAll(), not by the user.
  };

  static void Apply(ButtonState& state, bool down);
  static std::size_t Index(Key key) { return static_cast<std::size_t>(key); }
  static std::size_t Index(MouseButton button) {
    return static_cast<std::size_t>(button);
  }

  std::array<ButtonState, static_cast<std::size_t>(Key::kCount)> keys_{};
  std::array<ButtonState, static_cast<std::size_t>(MouseButton::kCount)>
      buttons_{};
  std::array<Vec2, static_cast<std::size_t>(MouseButton::kCount)>
      press_positions_{};
  Vec2 mouse_position_;
  Vec2 mouse_delta_;
  double wheel_ = 0.0;
  bool mouse_inside_ = true;
};

}  // namespace aerolab

#endif  // AEROLAB_CORE_INPUT_H_
