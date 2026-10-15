#include "core/clock.h"

#include <gtest/gtest.h>

namespace aerolab {
namespace {

TEST(FrameClockTest, FirstTickIsZero) {
  FrameClock clock;
  EXPECT_EQ(clock.Tick(12.5), 0.0);
  EXPECT_EQ(clock.frame_count(), 1);
}

TEST(FrameClockTest, ReturnsDeltaBetweenTicks) {
  FrameClock clock;
  clock.Tick(1.0);
  EXPECT_DOUBLE_EQ(clock.Tick(1.016), 0.016);
  EXPECT_DOUBLE_EQ(clock.Tick(1.050), 0.034);
}

TEST(FrameClockTest, ClampsLongStallsButReportsRawDelta) {
  FrameClock clock(0.1);
  clock.Tick(0.0);
  EXPECT_DOUBLE_EQ(clock.Tick(3.0), 0.1);
  EXPECT_DOUBLE_EQ(clock.raw_delta_seconds(), 3.0);
}

TEST(FrameClockTest, BackwardsTimeYieldsZero) {
  FrameClock clock;
  clock.Tick(5.0);
  EXPECT_EQ(clock.Tick(4.0), 0.0);
}

TEST(StopwatchTest, ElapsedIsMonotonic) {
  Stopwatch watch;
  const double first = watch.ElapsedSeconds();
  const double second = watch.ElapsedSeconds();
  EXPECT_GE(first, 0.0);
  EXPECT_GE(second, first);
  watch.Reset();
  EXPECT_GE(watch.ElapsedMilliseconds(), 0.0);
}

}  // namespace
}  // namespace aerolab
