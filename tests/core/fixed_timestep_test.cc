#include "core/fixed_timestep.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "core/ring_buffer.h"

namespace aerolab {
namespace {

TEST(FixedTimestepTest, ConvertsFrameTimeIntoWholeSteps) {
  FixedTimestep timestep(0.025);
  EXPECT_EQ(timestep.Accumulate(0.01), 0);
  EXPECT_NEAR(timestep.alpha(), 0.4, 1e-12);
  EXPECT_EQ(timestep.Accumulate(0.02), 1);
  EXPECT_NEAR(timestep.alpha(), 0.2, 1e-12);
  EXPECT_EQ(timestep.Accumulate(0.075), 3);
}

TEST(FixedTimestepTest, StepCountIndependentOfFrameChunking) {
  // Binary-exact sizes, so any split of the same total gives the same count.
  FixedTimestep coarse(1.0 / 128.0);
  FixedTimestep fine(1.0 / 128.0);
  int coarse_steps = 0;
  int fine_steps = 0;
  for (int i = 0; i < 10; ++i) coarse_steps += coarse.Accumulate(1.0 / 16.0);
  for (int i = 0; i < 160; ++i) fine_steps += fine.Accumulate(1.0 / 256.0);
  EXPECT_EQ(coarse_steps, fine_steps);
  EXPECT_EQ(coarse_steps, 80);
}

TEST(FixedTimestepTest, LimitsStepsPerFrameAndCountsDropped) {
  FixedTimestep timestep(0.01, 5);
  EXPECT_EQ(timestep.Accumulate(1.0), 5);
  EXPECT_EQ(timestep.dropped_steps(), 95);
  EXPECT_LT(timestep.alpha(), 1.0);
  timestep.Reset();
  EXPECT_EQ(timestep.dropped_steps(), 0);
}

TEST(FixedTimestepTest, IgnoresNonPositiveAndNanInput) {
  FixedTimestep timestep(0.01);
  EXPECT_EQ(timestep.Accumulate(-1.0), 0);
  EXPECT_EQ(timestep.Accumulate(0.0), 0);
  EXPECT_EQ(timestep.Accumulate(std::nan("")), 0);
  EXPECT_EQ(timestep.alpha(), 0.0);
}

TEST(FixedTimestepTest, InfiniteAndHugeFramesStayBounded) {
  FixedTimestep timestep(0.01, 8);
  EXPECT_EQ(timestep.Accumulate(std::numeric_limits<double>::infinity()), 0);
  EXPECT_EQ(timestep.Accumulate(1e12), 8);
  EXPECT_LT(timestep.alpha(), 1.0);
  EXPECT_GE(timestep.alpha(), 0.0);
  // The next ordinary frame behaves normally again.
  EXPECT_LE(timestep.Accumulate(0.016), 2);
}

TEST(FixedTimestepTest, StepSizeIsClamped) {
  FixedTimestep timestep(10.0);
  EXPECT_LE(timestep.step(), 0.1);
  timestep.set_step(0.0);
  EXPECT_GT(timestep.step(), 0.0);
}

TEST(RingBufferTest, OverwritesOldestWhenFull) {
  RingBuffer<int> ring(3);
  EXPECT_TRUE(ring.empty());
  for (int i = 1; i <= 5; ++i) ring.Push(i);
  EXPECT_TRUE(ring.full());
  EXPECT_EQ(ring.size(), 3u);
  EXPECT_EQ(ring.front(), 3);
  EXPECT_EQ(ring.back(), 5);
  EXPECT_EQ(ring.ToVector(), (std::vector<int>{3, 4, 5}));
  ring.Clear();
  EXPECT_TRUE(ring.empty());
}

TEST(RingBufferTest, ResizeKeepsNewestElements) {
  RingBuffer<int> ring(5);
  for (int i = 1; i <= 7; ++i) ring.Push(i);  // Holds 3..7.
  ring.SetCapacity(2);
  EXPECT_EQ(ring.ToVector(), (std::vector<int>{6, 7}));
  ring.SetCapacity(4);
  ring.Push(8);
  EXPECT_EQ(ring.ToVector(), (std::vector<int>{6, 7, 8}));
}

}  // namespace
}  // namespace aerolab
