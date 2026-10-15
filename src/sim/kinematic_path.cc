#include "sim/kinematic_path.h"

#include <algorithm>
#include <cmath>
#include <string_view>

#include "math/scalar.h"
#include "math/vec3.h"
#include "sim/force_model.h"

namespace aerolab::sim {
namespace {

// Corners are never sharper than this, so accelerations stay finite.
constexpr double kMinCornerRadius = 0.05;  // m
// Chords used to measure the lap of shapes without a closed form.
constexpr int kLapSamples = 4096;
// Half the time window over which a square's bank is averaged.
constexpr double kBankHalfWindow = 0.6;  // s

struct SquareGeometry {
  double half;      // Half the side length.
  double corner;    // Corner radius.
  double straight;  // Straight part of each side.
  double arc;       // Length of each rounded corner.
  double lap;
};

SquareGeometry MakeSquareGeometry(const KinematicPath& path) {
  SquareGeometry g;
  g.half = std::max(std::abs(path.radius), kMinCornerRadius);
  g.corner = std::clamp(path.corner_radius, kMinCornerRadius, g.half);
  g.straight = 2.0 * (g.half - g.corner);
  g.arc = kHalfPi * g.corner;
  g.lap = 4.0 * (g.straight + g.arc);
  return g;
}

// A rounded square flown at constant speed, counter-clockwise seen from
// above, starting in the middle of the +X side heading -Z (like the
// circle). Fills position (relative to the centre), velocity and
// acceleration of `out`.
void SampleSquare(const KinematicPath& path, double time, PathSample& out) {
  const SquareGeometry g = MakeSquareGeometry(path);
  const double speed = g.lap / std::max(path.period, 1e-3);
  double s = std::fmod(speed * time, g.lap);  // Distance into the lap.
  if (s < 0.0) s += g.lap;
  Vec3 p{g.half, 0.0, 0.0};  // Start of the current segment.
  Vec3 d = -Vec3::UnitZ();   // Heading along it.
  const auto straight = [&](double length) {
    if (s <= length) {
      out.position = p + d * s;
      out.velocity = d * speed;
      out.acceleration = Vec3::Zero();
      return true;
    }
    p += d * length;
    s -= length;
    return false;
  };
  const auto corner = [&]() {
    // A left turn about a centre one corner radius to the left.
    const Vec3 left = Cross(Vec3::UnitY(), d);
    const Vec3 center = p + left * g.corner;
    if (s <= g.arc) {
      const double angle = s / g.corner;
      const double c = std::cos(angle);
      const double sn = std::sin(angle);
      out.position = center - left * (g.corner * c) + d * (g.corner * sn);
      out.velocity = (left * sn + d * c) * speed;
      out.acceleration = (left * c - d * sn) * (speed * speed / g.corner);
      return true;
    }
    p = center + d * g.corner;
    d = left;
    s -= g.arc;
    return false;
  };
  if (straight(0.5 * g.straight)) return;
  for (int side = 0; side < 3; ++side) {
    if (corner() || straight(g.straight)) return;
  }
  if (corner()) return;
  if (!straight(0.5 * g.straight)) {
    // Rounding at the very end of the lap: back at the start.
    out.position = {g.half, 0.0, 0.0};
    out.velocity = -Vec3::UnitZ() * speed;
    out.acceleration = Vec3::Zero();
  }
}

// The square in the path's turning direction: clockwise mirrors it across
// the X axis (the same square, turning right).
void SampleSquareInPlane(const KinematicPath& path, double time,
                         PathSample& out) {
  SampleSquare(path, time, out);
  if (path.clockwise) {
    out.position.z = -out.position.z;
    out.velocity.z = -out.velocity.z;
    out.acceleration.z = -out.acceleration.z;
  }
}

}  // namespace

std::string_view PathShapeName(PathShape shape) {
  switch (shape) {
    case PathShape::kCircle:
      return "Circle";
    case PathShape::kFigureEight:
      return "Figure eight";
    case PathShape::kHelix:
      return "Helix";
    case PathShape::kSquare:
      return "Square";
    case PathShape::kCount:
      break;
  }
  return "Unknown";
}

PathSample SamplePath(const KinematicPath& path, double time) {
  const double omega =
      (path.clockwise ? -1.0 : 1.0) * kTwoPi / std::max(path.period, 1e-3);
  const double theta = omega * time;
  const double r = path.radius;
  const double s = std::sin(theta);
  const double c = std::cos(theta);

  PathSample out;
  switch (path.shape) {
    case PathShape::kSquare:
      SampleSquareInPlane(path, time, out);
      break;
    case PathShape::kFigureEight: {
      // x = r sin(t), z = (r/2) sin(2t); derivatives by the chain rule.
      const double s2 = std::sin(2.0 * theta);
      const double c2 = std::cos(2.0 * theta);
      out.position = {r * s, 0.0, 0.5 * r * s2};
      out.velocity = {r * omega * c, 0.0, r * omega * c2};
      out.acceleration = {-r * omega * omega * s, 0.0,
                          -2.0 * r * omega * omega * s2};
      break;
    }
    case PathShape::kHelix:
    case PathShape::kCircle:
    case PathShape::kCount: {
      // Counter-clockwise from above: +X toward -Z.
      out.position = {r * c, 0.0, -r * s};
      out.velocity = {-r * omega * s, 0.0, -r * omega * c};
      out.acceleration = {-r * omega * omega * c, 0.0, r * omega * omega * s};
      if (path.shape == PathShape::kHelix) {
        out.position.y = path.climb_rate * time;
        out.velocity.y = path.climb_rate;
      }
      break;
    }
  }
  out.position += path.center;

  // Coordinated turn: the felt "up" is the specific force a - g. A square's
  // corners switch the turn on and off abruptly, so its bank follows the
  // turn averaged over a short window (exactly the velocity change across
  // the window over its length): it rolls in just before each corner and
  // out just after, as a pilot would, instead of snapping.
  Vec3 turn = out.acceleration;
  if (path.shape == PathShape::kSquare) {
    PathSample before;
    PathSample after;
    SampleSquareInPlane(path, time - kBankHalfWindow, before);
    SampleSquareInPlane(path, time + kBankHalfWindow, after);
    turn = (after.velocity - before.velocity) / (2.0 * kBankHalfWindow);
  }
  const Vec3 apparent_up = turn + Vec3{0.0, kStandardGravity, 0.0};
  out.orientation = LookRotation(out.velocity, apparent_up);
  return out;
}

double PathLapLength(const KinematicPath& path) {
  const double r = std::abs(path.radius);
  switch (path.shape) {
    case PathShape::kCircle:
      return kTwoPi * r;
    case PathShape::kHelix:
      return std::hypot(kTwoPi * r,
                        path.climb_rate * std::max(path.period, 1e-3));
    case PathShape::kSquare:
      return MakeSquareGeometry(path).lap;
    case PathShape::kFigureEight:
    case PathShape::kCount:
      break;
  }
  // No closed form: add up short chords over one lap.
  const double period = std::max(path.period, 1e-3);
  double length = 0.0;
  Vec3 previous = SamplePath(path, 0.0).position;
  for (int i = 1; i <= kLapSamples; ++i) {
    const Vec3 next = SamplePath(path, period * i / kLapSamples).position;
    length += Distance(previous, next);
    previous = next;
  }
  return length;
}

double PathPeriodForSpeed(const KinematicPath& path, double speed) {
  speed = std::max(std::abs(speed), 1e-6);
  if (path.shape == PathShape::kHelix) {
    // The climb is part of the speed: solve for the horizontal part.
    const double horizontal =
        std::sqrt(std::max(Square(speed) - Square(path.climb_rate), 1e-12));
    return kTwoPi * std::abs(path.radius) / horizontal;
  }
  return PathLapLength(path) / speed;
}

Vec3 PathCenterForStart(const KinematicPath& path, const Vec3& start) {
  KinematicPath at_origin = path;
  at_origin.center = Vec3::Zero();
  return start - SamplePath(at_origin, 0.0).position;
}

}  // namespace aerolab::sim
