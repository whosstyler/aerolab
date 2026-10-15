#ifndef AEROLAB_MATH_RANDOM_H_
#define AEROLAB_MATH_RANDOM_H_

#include <cstdint>

namespace aerolab {

// Small, fast, deterministic pseudo-random generator (SplitMix64). The whole
// state is one 64-bit word, so copying an Rng forks an identical stream:
// that is what lets a forecast replay exactly the choices the simulation
// will make. Not suitable for cryptography.
class Rng {
 public:
  constexpr Rng() = default;
  explicit constexpr Rng(std::uint64_t seed) : state_(seed) {}

  constexpr std::uint64_t NextU64() {
    state_ += 0x9E3779B97F4A7C15ULL;
    std::uint64_t z = state_;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
  }

  // Uniform in [0, 1), using the top 53 bits.
  constexpr double Uniform() {
    return static_cast<double>(NextU64() >> 11) * 0x1.0p-53;
  }

  // Uniform in [lo, hi) (rounding can, very rarely, return hi itself).
  constexpr double Uniform(double lo, double hi) {
    return lo + (hi - lo) * Uniform();
  }

  // True with probability `p`.
  constexpr bool Chance(double p) { return Uniform() < p; }

  constexpr std::uint64_t state() const { return state_; }

 private:
  std::uint64_t state_ = 0;
};

// Mixes several values into one well-distributed seed.
constexpr std::uint64_t MixSeed(std::uint64_t a, std::uint64_t b) {
  Rng rng(a ^ (b * 0xD1B54A32D192ED03ULL));
  return rng.NextU64();
}

}  // namespace aerolab

#endif  // AEROLAB_MATH_RANDOM_H_
