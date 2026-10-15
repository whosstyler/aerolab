#ifndef AEROLAB_CORE_FIXED_TIMESTEP_H_
#define AEROLAB_CORE_FIXED_TIMESTEP_H_

#include <cstdint>

namespace aerolab {

// Accumulator that converts variable frame times into a whole number of
// fixed simulation steps ("Fix Your Timestep!", G. Fiedler).
//
// The simulation always advances in identical `step` increments, so its
// result never depends on the rendering frame rate. The leftover fraction
// alpha() lets the renderer interpolate between the last two states.
class FixedTimestep {
 public:
  static constexpr double kDefaultStep = 1.0 / 120.0;
  static constexpr int kDefaultMaxStepsPerFrame = 32;

  explicit FixedTimestep(double step = kDefaultStep,
                         int max_steps_per_frame = kDefaultMaxStepsPerFrame);

  // Adds `elapsed` seconds of simulation time and returns how many fixed
  // steps are due. If more than the per-frame budget is due (the simulation
  // cannot keep up), the excess is discarded rather than snowballing.
  int Accumulate(double elapsed);

  // Fraction of a step left in the accumulator, in [0, 1).
  double alpha() const { return accumulator_ / step_; }

  double step() const { return step_; }
  // Changes the step size; clamped to a sane range. Resets the remainder.
  void set_step(double step);

  int max_steps_per_frame() const { return max_steps_per_frame_; }
  void set_max_steps_per_frame(int steps);

  // Total steps discarded because the budget was exceeded.
  std::int64_t dropped_steps() const { return dropped_steps_; }

  void Reset();

 private:
  double step_;
  int max_steps_per_frame_;
  double accumulator_ = 0.0;
  std::int64_t dropped_steps_ = 0;
};

}  // namespace aerolab

#endif  // AEROLAB_CORE_FIXED_TIMESTEP_H_
