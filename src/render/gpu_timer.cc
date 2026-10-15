#include "render/gpu_timer.h"

#include <optional>

namespace aerolab {

GpuTimer::~GpuTimer() {
  if (created_) glDeleteQueries(kQueryCount, queries_.data());
}

void GpuTimer::EnsureCreated() {
  if (created_) return;
  glGenQueries(kQueryCount, queries_.data());
  created_ = true;
}

void GpuTimer::Begin() {
  EnsureCreated();
  // Skip this frame if the slot is still waiting for an old result; the
  // measurement rate simply drops instead of blocking.
  if (pending_[next_] || active_) return;
  glBeginQuery(GL_TIME_ELAPSED, queries_[next_]);
  active_ = true;
}

void GpuTimer::End() {
  if (!active_) return;
  glEndQuery(GL_TIME_ELAPSED);
  pending_[next_] = true;
  active_ = false;
  next_ = (next_ + 1) % kQueryCount;
}

std::optional<double> GpuTimer::PollMilliseconds() {
  if (!created_) return std::nullopt;
  std::optional<double> newest;
  // Visit slots oldest first (next_ is the oldest outstanding query), so
  // the newest completed result wins.
  for (int offset = 0; offset < kQueryCount; ++offset) {
    const int i = (next_ + offset) % kQueryCount;
    if (!pending_[i]) continue;
    GLint available = 0;
    glGetQueryObjectiv(queries_[i], GL_QUERY_RESULT_AVAILABLE, &available);
    if (available == 0) continue;
    GLuint64 nanoseconds = 0;
    glGetQueryObjectui64v(queries_[i], GL_QUERY_RESULT, &nanoseconds);
    pending_[i] = false;
    newest = static_cast<double>(nanoseconds) * 1e-6;
  }
  return newest;
}

}  // namespace aerolab
