#ifndef AEROLAB_CORE_EVENT_LOG_H_
#define AEROLAB_CORE_EVENT_LOG_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

#include "core/log.h"
#include "core/ring_buffer.h"

namespace aerolab {

// Where an event came from, for filtering in the UI.
enum class EventSource { kApp, kSimulation };

std::string_view EventSourceName(EventSource source);

struct EventEntry {
  std::uint64_t sequence = 0;  // Monotonic id, unique per log.
  std::uint32_t run = 0;       // Simulation run (see EventLog::BeginRun).
  double wall_seconds = 0.0;   // Real time since startup.
  double sim_seconds = -1.0;   // Simulation time, or < 0 if not applicable.
  LogLevel level = LogLevel::kInfo;
  EventSource source = EventSource::kApp;
  std::string message;
};

// Bounded, in-memory history of notable events for the editor's log panel.
// Not thread-safe: it is fed by the log sink, so all logging must happen on
// the main thread (the engine is single-threaded today).
class EventLog {
 public:
  explicit EventLog(std::size_t capacity = 2000) : entries_(capacity) {}

  void Add(LogLevel level, EventSource source, std::string message,
           double wall_seconds, double sim_seconds = -1.0);

  void Clear() { entries_.Clear(); }

  // Starts a new simulation run (after a reset). Later entries carry the
  // new run number so timelines can ignore events from earlier runs.
  void BeginRun() { ++current_run_; }
  std::uint32_t current_run() const { return current_run_; }

  std::size_t size() const { return entries_.size(); }
  // Oldest first.
  const EventEntry& operator[](std::size_t i) const { return entries_[i]; }

  // Total entries ever added (including evicted ones).
  std::uint64_t total_added() const { return next_sequence_; }

 private:
  RingBuffer<EventEntry> entries_;
  std::uint64_t next_sequence_ = 0;
  std::uint32_t current_run_ = 0;
};

}  // namespace aerolab

#endif  // AEROLAB_CORE_EVENT_LOG_H_
