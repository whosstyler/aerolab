#include "core/event_log.h"

#include <string>
#include <string_view>
#include <utility>

namespace aerolab {

std::string_view EventSourceName(EventSource source) {
  switch (source) {
    case EventSource::kApp:
      return "app";
    case EventSource::kSimulation:
      return "sim";
  }
  return "?";
}

void EventLog::Add(LogLevel level, EventSource source, std::string message,
                   double wall_seconds, double sim_seconds) {
  EventEntry entry;
  entry.sequence = next_sequence_++;
  entry.run = current_run_;
  entry.wall_seconds = wall_seconds;
  entry.sim_seconds = sim_seconds;
  entry.level = level;
  entry.source = source;
  entry.message = std::move(message);
  entries_.Push(std::move(entry));
}

}  // namespace aerolab
