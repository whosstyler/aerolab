#ifndef AEROLAB_RENDER_GPU_TIMER_H_
#define AEROLAB_RENDER_GPU_TIMER_H_

#include <array>
#include <optional>

#include "render/gl.h"

namespace aerolab {

// Measures GPU time of a span of GL commands with GL_TIME_ELAPSED queries.
//
// Results arrive asynchronously. A small ring of queries is used and only
// results the driver reports as available are read, so measuring never
// stalls the pipeline; the reported time is from a frame or two earlier.
class GpuTimer {
 public:
  GpuTimer() = default;
  ~GpuTimer();
  GpuTimer(const GpuTimer&) = delete;
  GpuTimer& operator=(const GpuTimer&) = delete;

  void Begin();
  void End();

  // Returns the newest measurement that completed since the last call, in
  // milliseconds, or nullopt if none finished (so callers never record the
  // same sample twice).
  std::optional<double> PollMilliseconds();

 private:
  static constexpr int kQueryCount = 4;

  void EnsureCreated();

  std::array<GLuint, kQueryCount> queries_{};
  std::array<bool, kQueryCount> pending_{};
  int next_ = 0;
  bool created_ = false;
  bool active_ = false;
};

}  // namespace aerolab

#endif  // AEROLAB_RENDER_GPU_TIMER_H_
