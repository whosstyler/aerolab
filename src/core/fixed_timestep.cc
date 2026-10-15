#include "core/fixed_timestep.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace aerolab {
namespace {

constexpr double kMinStep = 1e-5;
constexpr double kMaxStep = 0.1;

}  // namespace

FixedTimestep::FixedTimestep(double step, int max_steps_per_frame)
    : step_(std::clamp(step, kMinStep, kMaxStep)),
      max_steps_per_frame_(std::max(max_steps_per_frame, 1)) {}

int FixedTimestep::Accumulate(double elapsed) {
  // Rejects zero, negatives, NaN and infinity.
  if (!(elapsed > 0.0) || !std::isfinite(elapsed)) return 0;
  accumulator_ += elapsed;
  const double due = std::floor(accumulator_ / step_);
  if (due > static_cast<double>(max_steps_per_frame_)) {
    // The simulation cannot keep up: run the budget, drop the rest, and
    // keep only the fractional remainder so the backlog cannot snowball.
    const double dropped = due - static_cast<double>(max_steps_per_frame_);
    dropped_steps_ += static_cast<std::int64_t>(std::min(dropped, 1e15));
    accumulator_ = std::fmod(accumulator_, step_);
    return max_steps_per_frame_;
  }
  int steps = static_cast<int>(due);
  accumulator_ -= static_cast<double>(steps) * step_;
  // Guard against rounding leaving a remainder of a full step.
  if (accumulator_ >= step_) {
    accumulator_ -= step_;
    ++steps;
  }
  accumulator_ = std::max(accumulator_, 0.0);
  return steps;
}

void FixedTimestep::set_step(double step) {
  step_ = std::clamp(step, kMinStep, kMaxStep);
  accumulator_ = 0.0;
}

void FixedTimestep::set_max_steps_per_frame(int steps) {
  max_steps_per_frame_ = std::max(steps, 1);
}

void FixedTimestep::Reset() {
  accumulator_ = 0.0;
  dropped_steps_ = 0;
}

}  // namespace aerolab
