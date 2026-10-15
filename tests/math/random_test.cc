#include "math/random.h"

#include <gtest/gtest.h>

#include <cstdint>

namespace aerolab {
namespace {

TEST(RngTest, SameSeedSameSequence) {
  Rng a(42);
  Rng b(42);
  for (int i = 0; i < 100; ++i) EXPECT_EQ(a.NextU64(), b.NextU64());
}

TEST(RngTest, CopyForksAnIdenticalStream) {
  Rng original(7);
  original.NextU64();
  Rng copy = original;
  for (int i = 0; i < 10; ++i) EXPECT_EQ(original.Uniform(), copy.Uniform());
}

TEST(RngTest, DifferentSeedsDiffer) {
  Rng a(1);
  Rng b(2);
  int same = 0;
  for (int i = 0; i < 100; ++i) same += a.NextU64() == b.NextU64() ? 1 : 0;
  EXPECT_EQ(same, 0);
}

TEST(RngTest, UniformStaysInRangeWithPlausibleMean) {
  Rng rng(123);
  double sum = 0.0;
  constexpr int kSamples = 20000;
  for (int i = 0; i < kSamples; ++i) {
    const double u = rng.Uniform(-2.0, 3.0);
    ASSERT_GE(u, -2.0);
    ASSERT_LT(u, 3.0);
    sum += u;
  }
  EXPECT_NEAR(sum / kSamples, 0.5, 0.05);
}

TEST(RngTest, ChanceMatchesProbability) {
  Rng rng(99);
  int hits = 0;
  for (int i = 0; i < 10000; ++i) hits += rng.Chance(0.25) ? 1 : 0;
  EXPECT_NEAR(hits / 10000.0, 0.25, 0.02);
}

TEST(RngTest, MixSeedSeparatesNearbyInputs) {
  EXPECT_NE(MixSeed(1, 0), MixSeed(1, 1));
  EXPECT_NE(MixSeed(1, 0), MixSeed(2, 0));
  const std::uint64_t seed = MixSeed(5, 9);
  EXPECT_EQ(seed, MixSeed(5, 9));
  static_assert(MixSeed(3, 4) != MixSeed(4, 3));  // Usable at compile time.
}

}  // namespace
}  // namespace aerolab
