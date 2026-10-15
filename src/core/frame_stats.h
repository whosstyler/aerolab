#ifndef AEROLAB_CORE_FRAME_STATS_H_
#define AEROLAB_CORE_FRAME_STATS_H_

#include <array>
#include <cstddef>
#include <string_view>

#include "core/ring_buffer.h"

namespace aerolab {

// Per-frame timing channels shown by the performance panel.
enum class FrameTiming {
  kFrame,       // Wall time between frames.
  kSimulation,  // CPU time spent in fixed simulation steps.
  kSceneCpu,    // CPU time building and submitting the 3D frame.
  kSceneGpu,    // GPU time of the 3D frame (timer queries, ~1-2 frames late).
  kUi,          // CPU time building the editor UI.
  kCount,
};

inline constexpr int kFrameTimingCount = static_cast<int>(FrameTiming::kCount);

std::string_view FrameTimingName(FrameTiming timing);

// Rolling window of recent timings in milliseconds.
class FrameStats {
 public:
  static constexpr std::size_t kDefaultWindow = 240;

  explicit FrameStats(std::size_t window = kDefaultWindow);

  void Record(FrameTiming timing, double milliseconds);

  struct Summary {
    double average = 0.0;
    double minimum = 0.0;
    double maximum = 0.0;
    double latest = 0.0;
  };
  Summary Summarize(FrameTiming timing) const;

  // Frames per second from the average frame time (0 if unknown).
  double Fps() const;

  const RingBuffer<float>& history(FrameTiming timing) const {
    return histories_[Index(timing)];
  }

 private:
  static std::size_t Index(FrameTiming timing) {
    return static_cast<std::size_t>(timing);
  }

  std::array<RingBuffer<float>, kFrameTimingCount> histories_;
};

}  // namespace aerolab

#endif  // AEROLAB_CORE_FRAME_STATS_H_
