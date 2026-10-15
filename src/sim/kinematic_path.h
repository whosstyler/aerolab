#ifndef AEROLAB_SIM_KINEMATIC_PATH_H_
#define AEROLAB_SIM_KINEMATIC_PATH_H_

#include <array>
#include <string_view>

#include "math/quat.h"
#include "math/vec3.h"

namespace aerolab::sim {

enum class PathShape {
  kCircle,       // Horizontal circle.
  kFigureEight,  // Lemniscate of Gerono in the horizontal plane.
  kHelix,        // Circle with a constant climb rate.
  kSquare,       // Square with rounded corners, at constant speed.
  kCount,
};

inline constexpr int kPathShapeCount = static_cast<int>(PathShape::kCount);

std::string_view PathShapeName(PathShape shape);

constexpr std::array<PathShape, kPathShapeCount> AllPathShapes() {
  std::array<PathShape, kPathShapeCount> all{};
  for (int i = 0; i < kPathShapeCount; ++i) all[i] = static_cast<PathShape>(i);
  return all;
}

// A closed-form trajectory: position, velocity and acceleration are known
// analytically at any time, so no integration is involved. Useful as a
// reference motion and for scripted (non-dynamic) objects.
struct KinematicPath {
  PathShape shape = PathShape::kCircle;
  Vec3 center;
  double radius = 15.0;        // m (half the side for a square)
  double corner_radius = 4.0;  // m, square only; at most `radius`.
  double period = 12.0;        // s per lap
  double climb_rate = 0.5;     // m/s (helix only)
  bool clockwise = false;      // Seen from above.
};

struct PathSample {
  Vec3 position;
  Vec3 velocity;
  Vec3 acceleration;
  // Faces along the velocity and banks into the turn as a coordinated
  // aircraft would: "up" follows the apparent gravity (a - g).
  Quat orientation;
};

PathSample SamplePath(const KinematicPath& path, double time);

// Distance flown in one lap (one period), m. The speed of a constant-speed
// path is PathLapLength / period.
double PathLapLength(const KinematicPath& path);

// The period that makes `path` average `speed` (m/s) around a lap. For a
// helix the climb rate counts toward the speed, so speeds below it are
// not reachable (the result tends to an infinitely slow circle).
double PathPeriodForSpeed(const KinematicPath& path, double speed);

// Centre that makes `path` pass through `start` at time zero (the path's
// shape, size and timing are kept). Used to place a path on an object.
Vec3 PathCenterForStart(const KinematicPath& path, const Vec3& start);

}  // namespace aerolab::sim

#endif  // AEROLAB_SIM_KINEMATIC_PATH_H_
