#include "core/frame_stats.h"

#include <algorithm>
#include <cstddef>
#include <string_view>

namespace aerolab {

std::string_view FrameTimingName(FrameTiming timing) {
  switch (timing) {
    case FrameTiming::kFrame:
      return "Frame";
    case FrameTiming::kSimulation:
      return "Simulation";
    case FrameTiming::kSceneCpu:
      return "Scene (CPU)";
    case FrameTiming::kSceneGpu:
      return "Scene (GPU)";
    case FrameTiming::kUi:
      return "Editor UI";
    case FrameTiming::kCount:
      break;
  }
  return "?";
}

FrameStats::FrameStats(std::size_t window) {
  for (RingBuffer<float>& history : histories_) history.SetCapacity(window);
}

void FrameStats::Record(FrameTiming timing, double milliseconds) {
  if (timing == FrameTiming::kCount) return;
  histories_[Index(timing)].Push(static_cast<float>(milliseconds));
}

FrameStats::Summary FrameStats::Summarize(FrameTiming timing) const {
  Summary summary;
  if (timing == FrameTiming::kCount) return summary;
  const RingBuffer<float>& history = histories_[Index(timing)];
  if (history.empty()) return summary;
  double sum = 0.0;
  summary.minimum = history[0];
  summary.maximum = history[0];
  for (std::size_t i = 0; i < history.size(); ++i) {
    const double value = history[i];
    sum += value;
    summary.minimum = std::min(summary.minimum, value);
    summary.maximum = std::max(summary.maximum, value);
  }
  summary.average = sum / static_cast<double>(history.size());
  summary.latest = history.back();
  return summary;
}

double FrameStats::Fps() const {
  const double average = Summarize(FrameTiming::kFrame).average;
  return average > 0.0 ? 1000.0 / average : 0.0;
}

}  // namespace aerolab
