#ifndef AEROLAB_MATH_SCALAR_H_
#define AEROLAB_MATH_SCALAR_H_

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace aerolab {

inline constexpr double kPi = std::numbers::pi;
inline constexpr double kTwoPi = 2.0 * std::numbers::pi;
inline constexpr double kHalfPi = 0.5 * std::numbers::pi;

// Default tolerance for geometric comparisons in metres / radians.
inline constexpr double kEpsilon = 1e-9;

// Relative tolerance below which a matrix is treated as singular. The
// determinant is compared against the product of the column lengths
// (Hadamard's bound), which makes the test independent of the matrix scale.
inline constexpr double kSingularTolerance = 1e-12;

// Squared length below which a direction is too short to be meaningful.
inline constexpr double kDegenerateLengthSquared = 1e-24;

// Largest finite double; used as the "empty" sentinel for bounding boxes.
inline constexpr double kMaxDouble = std::numeric_limits<double>::max();

constexpr double Radians(double degrees) { return degrees * (kPi / 180.0); }
constexpr double Degrees(double radians) { return radians * (180.0 / kPi); }

constexpr double Lerp(double a, double b, double t) { return a + (b - a) * t; }

constexpr double Square(double value) { return value * value; }

// True when |a - b| is within `tolerance`, scaled by the operands' magnitude
// once they exceed 1 (a combined absolute/relative test).
inline bool NearlyEqual(double a, double b, double tolerance = kEpsilon) {
  const double scale = std::max({1.0, std::abs(a), std::abs(b)});
  return std::abs(a - b) <= tolerance * scale;
}

// Wraps an angle to (-pi, pi].
inline double WrapAngle(double radians) {
  double wrapped = std::remainder(radians, kTwoPi);
  if (wrapped <= -kPi) wrapped += kTwoPi;
  return wrapped;
}

// Frame-rate independent exponential smoothing factor: the fraction of the
// remaining distance covered in `dt` seconds when `sharpness` is the decay
// rate in 1/s. Applying it every frame gives the same result at any FPS.
inline double SmoothingFactor(double sharpness, double dt) {
  return 1.0 - std::exp(-sharpness * dt);
}

}  // namespace aerolab

#endif  // AEROLAB_MATH_SCALAR_H_
