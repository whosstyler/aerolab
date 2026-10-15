#include "core/log.h"

#include <cstdio>
#include <mutex>
#include <string_view>
#include <utility>

namespace aerolab {
namespace {

struct SinkState {
  std::mutex mutex;
  LogSink sink;
};

SinkState& State() {
  // Intentionally leaked so logging stays valid during static destruction.
  static SinkState* state = new SinkState();
  return *state;
}

// Set while a sink runs on this thread, so a sink that logs (violating the
// contract) is ignored instead of recursing forever.
thread_local bool in_sink = false;

}  // namespace

std::string_view LogLevelName(LogLevel level) {
  switch (level) {
    case LogLevel::kDebug:
      return "debug";
    case LogLevel::kInfo:
      return "info";
    case LogLevel::kWarning:
      return "warning";
    case LogLevel::kError:
      return "error";
  }
  return "unknown";
}

void SetLogSink(LogSink sink) {
  SinkState& state = State();
  std::lock_guard<std::mutex> lock(state.mutex);
  state.sink = std::move(sink);
}

void LogMessage(LogLevel level, std::string_view message) {
  const std::string_view name = LogLevelName(level);
  std::fprintf(stderr, "[%.*s] %.*s\n", static_cast<int>(name.size()),
               name.data(), static_cast<int>(message.size()), message.data());
  if (in_sink) return;

  // Copy under the lock and call outside it, so a slow sink never blocks
  // SetLogSink and a sink may safely replace itself.
  LogSink sink;
  {
    SinkState& state = State();
    std::lock_guard<std::mutex> lock(state.mutex);
    sink = state.sink;
  }
  if (!sink) return;
  in_sink = true;
  sink(level, message);
  in_sink = false;
}

}  // namespace aerolab
