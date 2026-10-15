#include "core/event_log.h"

#include <gtest/gtest.h>

#include <string>

#include "core/frame_stats.h"

namespace aerolab {
namespace {

TEST(EventLogTest, KeepsNewestEntriesWithSequenceNumbers) {
  EventLog log(3);
  for (int i = 0; i < 5; ++i) {
    log.Add(LogLevel::kInfo, EventSource::kApp, "event " + std::to_string(i),
            static_cast<double>(i));
  }
  ASSERT_EQ(log.size(), 3u);
  EXPECT_EQ(log[0].message, "event 2");
  EXPECT_EQ(log[2].message, "event 4");
  EXPECT_EQ(log[2].sequence, 4u);
  EXPECT_EQ(log.total_added(), 5u);
  EXPECT_LT(log[0].sim_seconds, 0.0);  // Not a simulation event.
}

TEST(EventLogTest, RunsSeparateSimulationResets) {
  EventLog log;
  log.Add(LogLevel::kInfo, EventSource::kSimulation, "first run", 1.0, 1.0);
  log.BeginRun();
  log.Add(LogLevel::kInfo, EventSource::kSimulation, "second run", 2.0, 0.5);
  EXPECT_EQ(log[0].run, 0u);
  EXPECT_EQ(log[1].run, 1u);
  EXPECT_EQ(log.current_run(), 1u);
}

TEST(EventLogTest, SimulationEventsCarrySimTime) {
  EventLog log;
  log.Add(LogLevel::kWarning, EventSource::kSimulation, "impact", 10.0, 2.5);
  EXPECT_EQ(log[0].sim_seconds, 2.5);
  EXPECT_EQ(log[0].source, EventSource::kSimulation);
  EXPECT_EQ(EventSourceName(log[0].source), "sim");
  log.Clear();
  EXPECT_EQ(log.size(), 0u);
}

TEST(FrameStatsTest, PartialWindowAndUnknownFps) {
  FrameStats stats(8);
  EXPECT_EQ(stats.Fps(), 0.0);
  stats.Record(FrameTiming::kFrame, 20.0);
  stats.Record(FrameTiming::kFrame, 10.0);
  const FrameStats::Summary s = stats.Summarize(FrameTiming::kFrame);
  EXPECT_DOUBLE_EQ(s.average, 15.0);
  EXPECT_DOUBLE_EQ(s.minimum, 10.0);
  EXPECT_DOUBLE_EQ(s.latest, 10.0);
  EXPECT_NEAR(stats.Fps(), 1000.0 / 15.0, 1e-9);
}

TEST(FrameStatsTest, SummarizesWindow) {
  FrameStats stats(4);
  for (double ms : {10.0, 20.0, 30.0, 40.0, 50.0}) {
    stats.Record(FrameTiming::kFrame, ms);
  }
  const FrameStats::Summary s = stats.Summarize(FrameTiming::kFrame);
  // Only the last four samples are kept.
  EXPECT_DOUBLE_EQ(s.average, 35.0);
  EXPECT_DOUBLE_EQ(s.minimum, 20.0);
  EXPECT_DOUBLE_EQ(s.maximum, 50.0);
  EXPECT_DOUBLE_EQ(s.latest, 50.0);
  EXPECT_NEAR(stats.Fps(), 1000.0 / 35.0, 1e-9);
  EXPECT_EQ(stats.Summarize(FrameTiming::kUi).average, 0.0);
  EXPECT_EQ(stats.history(FrameTiming::kFrame).size(), 4u);
  EXPECT_NE(FrameTimingName(FrameTiming::kSceneGpu), "?");
}

}  // namespace
}  // namespace aerolab
