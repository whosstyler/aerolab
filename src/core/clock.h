#ifndef AEROLAB_CORE_CLOCK_H_
#define AEROLAB_CORE_CLOCK_H_

#include <chrono>

namespace aerolab {

// Measures monotonic wall-clock time since construction or the last Reset().
class Stopwatch {
 public:
  Stopwatch();

  void Reset();
  double ElapsedSeconds() const;
  double ElapsedMilliseconds() const;

 private:
  std::chrono::steady_clock::time_point start_;
};

// Converts a sequence of absolute timestamps into per-frame deltas.
//
// The delta is clamped to `max_delta_seconds` so that a long stall (debugger
// break, window drag, device sleep) does not hand the simulation a huge time
// step. Timestamps are supplied by the caller, which keeps the class
// deterministic and testable.
class FrameClock {
 public:
  static constexpr double kDefaultMaxDeltaSeconds = 0.25;

  explicit FrameClock(double max_delta_seconds = kDefaultMaxDeltaSeconds);

  // Advances to `now_seconds` and returns the clamped delta. The first call
  // returns zero. Timestamps that go backwards also yield zero.
  double Tick(double now_seconds);

  // Unclamped delta from the most recent Tick().
  double raw_delta_seconds() const { return raw_delta_seconds_; }
  long long frame_count() const { return frame_count_; }

 private:
  double max_delta_seconds_;
  double last_seconds_ = 0.0;
  double raw_delta_seconds_ = 0.0;
  long long frame_count_ = 0;
  bool has_last_ = false;
};

}  // namespace aerolab

#endif  // AEROLAB_CORE_CLOCK_H_
