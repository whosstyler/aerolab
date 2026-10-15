#include "core/clock.h"

#include <algorithm>
#include <chrono>

namespace aerolab {

Stopwatch::Stopwatch() : start_(std::chrono::steady_clock::now()) {}

void Stopwatch::Reset() { start_ = std::chrono::steady_clock::now(); }

double Stopwatch::ElapsedSeconds() const {
  const auto elapsed = std::chrono::steady_clock::now() - start_;
  return std::chrono::duration<double>(elapsed).count();
}

double Stopwatch::ElapsedMilliseconds() const {
  return ElapsedSeconds() * 1000.0;
}

FrameClock::FrameClock(double max_delta_seconds)
    : max_delta_seconds_(max_delta_seconds) {}

double FrameClock::Tick(double now_seconds) {
  ++frame_count_;
  if (!has_last_) {
    has_last_ = true;
    last_seconds_ = now_seconds;
    raw_delta_seconds_ = 0.0;
    return 0.0;
  }
  raw_delta_seconds_ = std::max(0.0, now_seconds - last_seconds_);
  last_seconds_ = now_seconds;
  return std::min(raw_delta_seconds_, max_delta_seconds_);
}

}  // namespace aerolab
