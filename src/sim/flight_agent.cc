#include "sim/flight_agent.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <limits>
#include <optional>
#include <span>
#include <string_view>
#include <utility>

#include "math/mat3.h"
#include "math/quat.h"
#include "math/random.h"
#include "math/scalar.h"
#include "math/vec3.h"
#include "sim/force_model.h"
#include "sim/heading.h"

namespace aerolab::sim {
namespace {

// --- Decision making -------------------------------------------------------

// Random destinations scored per decision; the best one wins.
constexpr int kDestinationCandidates = 16;
// A leg may take this many times its ideal duration (plus slack) before
// the agent gives up on it.
constexpr double kTransitTimeFactor = 2.5;
constexpr double kTransitTimeSlack = 10.0;  // s
// Minimum time on a leg before an occupied destination is given up.
constexpr double kOccupiedCooldown = 1.0;  // s
// The cooldown doubles each time the replacement spot is occupied too, up to
// 2^kMaxBackoffSteps times.
constexpr int kMaxBackoffSteps = 5;
// Objects slower than this count as parked when judging whether a spot is
// taken.
constexpr double kParkedSpeed = 1.0;      // m/s
constexpr double kHoverMinLeg = 8.0;      // m
constexpr double kWingMinLegTurns = 3.0;  // Shortest leg, in turn radii.
// Wings start turning back about this many turn radii inside the area edge.
constexpr double kWingEdgeTurns = 1.2;
constexpr double kHoverArriveRadius = 1.0;   // m
constexpr double kHoverArriveSpeed = 0.8;    // m/s
constexpr double kWingArriveHeight = 4.0;    // m
constexpr double kLoiterRadiusFactor = 1.3;  // Circle radius / tightest turn.
constexpr double kHoverScanRate = Radians(25.0);  // Yaw while hovering.

// --- Steering --------------------------------------------------------------

constexpr double kAltitudeGain = 0.8;    // Climb rate per metre of error.
constexpr double kHoldGain = 0.6;        // Final-approach speed per metre, 1/s.
constexpr double kAltitudeMargin = 1.0;  // m kept inside the altitude band.
constexpr double kOrbitGain = 0.5;       // Radial speed per metre of error.
constexpr double kWanderNoise = 1.2;     // Turn-rate noise, rad/s per sqrt(s).
constexpr double kWanderRateDecay = 1.0;  // 1/s: how quickly the rate forgets.
constexpr double kWanderDecay = 0.3;      // 1/s pull back toward straight.
constexpr double kMaxWander = Radians(50.0);
constexpr double kHoverEdgeMargin = 4.0;  // m
constexpr double kEscapeGain = 2.0;       // Escape speed per leg speed.
// Directions tried on each side when the way ahead is blocked (over 180
// degrees, so 12 gives 15-degree steps).
constexpr int kAvoidSearchSteps = 12;
// How many search steps smaller the other side's deviation must be before
// an agent switches the side it is passing an obstacle on (45 degrees).
constexpr int kSideSwitchSteps = 3;
// Speed of a hover agent stepping sideways out of a blocked column, m/s.
constexpr double kSidestepSpeed = 1.5;

// --- Vehicle response ------------------------------------------------------

constexpr double kHoverResponse = 0.8;  // s to close a velocity error.
// Shortest time to swing from zero to full acceleration (a jerk limit, so
// the tilt changes smoothly).
constexpr double kHoverJerkTime = 0.35;  // s
// Thrust never points down: the craft can at most partly free-fall.
constexpr double kMaxDescentAccel = 0.7 * kStandardGravity;
constexpr double kMaxBank = Radians(80.0);
constexpr double kMinBank = Radians(1.0);

// Parameter limits applied by SanitizeAgentParams.
constexpr double kMaxCoordinate = 1e6;      // m, as the simulation bounds.
constexpr double kMaxParamSpeed = 1e4;      // m/s
constexpr double kMaxParamAccel = 1e4;      // m/s^2
constexpr double kMaxParamTime = 1e6;       // s
constexpr double kMaxParamSize = 1e3;       // m
constexpr double kWingSpeedResponse = 2.0;  // s
// Shortest time to roll from level into the steepest allowed turn.
constexpr double kWingRollTime = 0.5;       // s
constexpr double kWingTurnResponse = 1.0;   // s
constexpr double kWingClimbResponse = 0.8;  // s
constexpr double kWingMaxPathAngle = Radians(25.0);

Vec3 Horizontal(const Vec3& v) { return {v.x, 0.0, v.z}; }

// Rotates `v` about +Y; positive angles turn left like headings do.
Vec3 RotateAboutUp(const Vec3& v, double angle) {
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  return {v.x * c + v.z * s, v.y, -v.x * s + v.z * c};
}

bool IsWing(const AgentParams& p) { return p.style == FlightStyle::kWing; }

// Largest horizontal acceleration a level bank (or tilt) can provide.
double TurnAccelLimit(const AgentParams& p) {
  return kStandardGravity * std::tan(std::clamp(p.max_tilt, 0.0, kMaxBank));
}

double MinFlightSpeed(const AgentParams& p) {
  return IsWing(p) ? std::max(p.min_speed, 1.0) : std::max(p.min_speed, 0.0);
}

double MaxFlightSpeed(const AgentParams& p) {
  return std::max(p.max_speed, MinFlightSpeed(p) + 0.1);
}

// Hover climb limit, kept below the top speed so some speed is left over
// for horizontal flight.
double HoverClimbLimit(const AgentParams& p) {
  return std::clamp(p.max_climb_rate, 0.0, 0.9 * MaxFlightSpeed(p));
}

// Horizontal speed a hover agent may request: together with the climb
// limit it keeps the total speed within max_speed.
double HoverHorizontalCap(const AgentParams& p) {
  return std::sqrt(Square(MaxFlightSpeed(p)) - Square(HoverClimbLimit(p)));
}

struct AltitudeBand {
  double low;
  double high;
};

AltitudeBand Band(const FlightArea& area) {
  return {std::min(area.min_altitude, area.max_altitude),
          std::max(area.min_altitude, area.max_altitude)};
}

double AreaRadius(const FlightArea& area) { return std::max(area.radius, 1.0); }

// Width of the band inside the area edge where the agent starts turning
// back: a fixed margin when hovering, about a turn radius for wings.
double EdgeMargin(const AgentParams& p, double speed) {
  const double radius = AreaRadius(p.area);
  if (IsWing(p)) {
    return std::min(kWingEdgeTurns * MinTurnRadius(p, speed), 0.4 * radius);
  }
  return std::min(kHoverEdgeMargin, 0.3 * radius);
}

double InflatedRadius(const AgentParams& p, const AgentObstacle& o) {
  return std::max(o.radius, 0.0) + std::max(p.body_radius, 0.0) +
         std::max(p.clearance, 0.0);
}

// Whether the agent should take an obstacle into account. Non-finite data
// (e.g. from an absurd edit) is ignored rather than poisoning every agent
// that senses it, and so is an obstacle whose sphere engulfs the agent's
// entire flight area (say a huge floor slab): it cannot be avoided, and
// trying would only push the agent out of its area.
bool IsRelevant(const AgentParams& p, const AgentObstacle& o) {
  if (!IsFinite(o.center) || !std::isfinite(o.radius) || o.radius < 0.0) {
    return false;
  }
  const AltitudeBand band = Band(p.area);
  const double across =
      HorizontalLength(o.center - p.area.center) + AreaRadius(p.area);
  const double vertical = std::max(std::abs(band.high - o.center.y),
                                   std::abs(band.low - o.center.y));
  // The bare size (plus the agent's own) decides: an obstacle that leaves
  // some of the area free is still avoided.
  return std::hypot(across, vertical) > o.radius + std::max(p.body_radius, 0.0);
}

// Closest point to `c` on the segment from `a` to `a + d`.
Vec3 ClosestOnSegment(const Vec3& a, const Vec3& d, const Vec3& c) {
  const double length_squared = LengthSquared(d);
  if (length_squared < 1e-12) return a;
  return a + d * std::clamp(Dot(c - a, d) / length_squared, 0.0, 1.0);
}

// Uniform random point in the flight area's disc (at most
// `radius_limit` from its centre) and inside its altitude band.
Vec3 RandomPointInArea(const FlightArea& area, double radius_limit, Rng& rng) {
  const AltitudeBand band = Band(area);
  const double margin =
      std::min(kAltitudeMargin, 0.25 * (band.high - band.low));
  const double angle = rng.Uniform(0.0, kTwoPi);
  const double r = radius_limit * std::sqrt(rng.Uniform());
  return {area.center.x + r * std::cos(angle),
          rng.Uniform(band.low + margin, band.high - margin),
          area.center.z + r * std::sin(angle)};
}

// True when `point` lies within the flight area (with a metre of slack).
bool InsideArea(const FlightArea& area, const Vec3& point) {
  const AltitudeBand band = Band(area);
  return HorizontalLength(point - area.center) <= AreaRadius(area) + 1.0 &&
         point.y >= band.low - 1.0 && point.y <= band.high + 1.0;
}

// True when `point` lies inside the clearance zone of something parked
// there (e.g. another agent hovering). Objects just passing through do
// not count: they will be gone by the time the agent arrives.
bool IsOccupied(const AgentParams& p, std::span<const AgentObstacle> obstacles,
                const Vec3& point) {
  for (const AgentObstacle& o : obstacles) {
    if (IsRelevant(p, o) && Length(o.velocity) < kParkedSpeed &&
        Distance(point, o.center) < InflatedRadius(p, o)) {
      return true;
    }
  }
  return false;
}

// Scored random search ("utility AI"): sample candidate destinations and
// keep the one that best balances leg length, how far the agent has to
// turn, clear lines of flight, novelty and a little randomness.
AgentDecision ChooseDestination(const AgentParams& p,
                                std::span<const AgentObstacle> obstacles,
                                const AgentMotion& m, AgentBrain& b,
                                AgentDecision::Kind kind) {
  const bool wing = IsWing(p);
  const double radius = AreaRadius(p.area);
  double speed =
      b.rng.Uniform(std::min(p.cruise_speed_min, p.cruise_speed_max),
                    std::max(p.cruise_speed_min, p.cruise_speed_max));
  // Never plan a leg faster than the craft can fly.
  speed = wing ? std::clamp(speed, MinFlightSpeed(p), MaxFlightSpeed(p))
               : std::min(speed, HoverHorizontalCap(p));
  const double turn_radius = wing ? MinTurnRadius(p, speed) : 0.0;
  // Destinations stay clear of the edge band where StayInArea steers.
  const double sample_radius = radius - EdgeMargin(p, speed);
  const double min_leg = wing ? kWingMinLegTurns * turn_radius : kHoverMinLeg;
  const Vec3 track = Horizontal(m.velocity);

  Vec3 best = m.position;
  double best_score = -std::numeric_limits<double>::infinity();
  for (int i = 0; i < kDestinationCandidates; ++i) {
    const Vec3 candidate = RandomPointInArea(p.area, sample_radius, b.rng);
    const Vec3 to = candidate - m.position;
    const double distance = Length(to);
    double score = b.rng.Uniform(0.0, 0.3);
    if (distance < min_leg) {
      score -= 2.0 * (min_leg - distance) / std::max(min_leg, 1.0);
    }
    if (HorizontalLength(track) > 1.0 && HorizontalLength(to) > 1e-6) {
      const double turn = AngleBetween(track, Horizontal(to)) / kPi;
      score -= (wing ? 1.5 : 0.3) * turn;
    }
    for (const AgentObstacle& o : obstacles) {
      if (!IsRelevant(p, o)) continue;
      const double reach = InflatedRadius(p, o);
      if (Distance(candidate, o.center) < reach) score -= 3.0;
      if (Distance(ClosestOnSegment(m.position, to, o.center), o.center) <
          reach) {
        score -= 1.0;
      }
    }
    if (b.has_destination) {
      score += 0.3 * std::min(Distance(candidate, b.destination) / radius, 1.0);
    }
    if (score > best_score) {
      best_score = score;
      best = candidate;
    }
  }

  const double distance = Distance(best, m.position);
  // When even the best spot is taken (the area is crowded or nearly filled
  // by something), wait longer before giving up on it again.
  b.occupied_streak =
      IsOccupied(p, obstacles, best) ? b.occupied_streak + 1 : 0;
  b.has_destination = true;
  b.destination = best;
  b.leg_speed = speed;
  b.mode = AgentMode::kTransit;
  b.mode_time = 0.0;
  b.mode_duration =
      distance / std::max(speed, 0.5) * kTransitTimeFactor + kTransitTimeSlack;
  return {.kind = kind,
          .destination = best,
          .distance = distance,
          .speed = speed,
          .duration = 0.0};
}

// True when a horizontal circle (at the agent's altitude) keeps every
// obstacle outside its inflated sphere.
bool CircleIsClear(const AgentParams& p,
                   std::span<const AgentObstacle> obstacles, const Vec3& center,
                   double radius) {
  for (const AgentObstacle& o : obstacles) {
    if (!IsRelevant(p, o)) continue;
    const double inflated = InflatedRadius(p, o);
    const double height = std::abs(o.center.y - center.y);
    if (height >= inflated) continue;
    // Closest approach of the ring to the obstacle centre.
    const double across = HorizontalLength(o.center - center) - radius;
    if (std::hypot(across, height) < inflated) return false;
  }
  return true;
}

// Starts loitering at the destination, or returns nothing when a wing has
// no clear circle to fly there.
std::optional<AgentDecision> StartLoiter(
    const AgentParams& p, std::span<const AgentObstacle> obstacles,
    const AgentMotion& m, AgentBrain& b) {
  const double duration =
      std::max(b.rng.Uniform(std::min(p.loiter_min, p.loiter_max),
                             std::max(p.loiter_min, p.loiter_max)),
               0.0);
  if (IsWing(p)) {
    // Circle on a clear side, preferring the one nearer the area centre.
    const double radius = kLoiterRadiusFactor * MinTurnRadius(p, b.leg_speed);
    const Vec3 track =
        NormalizedOr(Horizontal(m.velocity), HeadingVector(b.heading));
    const Vec3 left = Cross(Vec3::UnitY(), track);
    const Vec3 left_center = m.position + left * radius;
    const Vec3 right_center = m.position - left * radius;
    const Vec3 area_center = Horizontal(p.area.center);
    // A usable circle stays inside the flight area and clear of obstacles
    // (staying in the area is not steered while circling).
    const auto usable = [&](const Vec3& center) {
      return HorizontalLength(center - area_center) + radius <=
                 AreaRadius(p.area) &&
             CircleIsClear(p, obstacles, center, radius);
    };
    const bool left_clear = usable(left_center);
    const bool right_clear = usable(right_center);
    if (!left_clear && !right_clear) return std::nullopt;
    const bool prefer_left = HorizontalLength(left_center - area_center) <=
                             HorizontalLength(right_center - area_center);
    const bool turn_left = left_clear && (prefer_left || !right_clear);
    b.turn_direction = turn_left ? 1.0 : -1.0;
    b.loiter_center = turn_left ? left_center : right_center;
    b.loiter_radius = radius;
  } else {
    b.turn_direction = b.rng.Chance(0.5) ? 1.0 : -1.0;  // Scan direction.
    b.loiter_center = b.destination;
    b.loiter_radius = 0.0;
  }
  b.mode = AgentMode::kLoiter;
  b.mode_time = 0.0;
  b.mode_duration = duration;
  return AgentDecision{.kind = AgentDecision::Kind::kLoiter,
                       .destination = b.destination,
                       .distance = 0.0,
                       .speed = b.leg_speed,
                       .duration = duration};
}

bool ReachedDestination(const AgentParams& p, const AgentMotion& m,
                        const AgentBrain& b) {
  if (IsWing(p)) {
    const double capture = std::max(6.0, 0.5 * MinTurnRadius(p, b.leg_speed));
    return HorizontalLength(m.position - b.destination) < capture &&
           std::abs(m.position.y - b.destination.y) < kWingArriveHeight;
  }
  return Distance(m.position, b.destination) < kHoverArriveRadius &&
         Length(m.velocity) < kHoverArriveSpeed;
}

// The state machine: transit -> (arrive) -> loiter or a new leg; a leg
// that takes far too long, or whose destination is occupied, is abandoned
// for a new one.
std::optional<AgentDecision> Decide(const AgentParams& p,
                                    std::span<const AgentObstacle> obstacles,
                                    double dt, const AgentMotion& m,
                                    AgentBrain& b) {
  if (!b.has_destination) {
    return ChooseDestination(p, obstacles, m, b,
                             AgentDecision::Kind::kNewDestination);
  }
  b.mode_time += dt;
  // Another object has taken the spot, or the area was edited so the spot
  // is no longer inside it.
  const double cooldown =
      kOccupiedCooldown *
      static_cast<double>(1 << std::min(b.occupied_streak, kMaxBackoffSteps));
  const bool spot_unusable =
      b.mode_time > cooldown && (IsOccupied(p, obstacles, b.destination) ||
                                 !InsideArea(p.area, b.destination));
  if (b.mode == AgentMode::kTransit) {
    if (ReachedDestination(p, m, b)) {
      ++b.legs_completed;
      if (std::max(p.loiter_min, p.loiter_max) > 0.0 &&
          b.rng.Chance(std::clamp(p.loiter_chance, 0.0, 1.0))) {
        if (std::optional<AgentDecision> loiter =
                StartLoiter(p, obstacles, m, b)) {
          return loiter;
        }
      }
      return ChooseDestination(p, obstacles, m, b,
                               AgentDecision::Kind::kNewDestination);
    }
    // Give up on a leg that takes far too long, or whose destination has
    // become unusable (after a short cooldown, so it cannot churn).
    if (b.mode_time > b.mode_duration || spot_unusable) {
      return ChooseDestination(p, obstacles, m, b,
                               AgentDecision::Kind::kReplan);
    }
    return std::nullopt;
  }
  if (spot_unusable) {
    return ChooseDestination(p, obstacles, m, b, AgentDecision::Kind::kReplan);
  }
  if (b.mode_time >= b.mode_duration) {
    return ChooseDestination(p, obstacles, m, b,
                             AgentDecision::Kind::kNewDestination);
  }
  return std::nullopt;
}

struct Steering {
  Vec3 desired;            // Requested velocity, m/s.
  double turn_rate = 0.0;  // Feed-forward turn rate for circling, rad/s.
};

// Goal-seeking part of the request: head for the destination (slowing to
// stop there when hovering), meander a little, or circle while loitering.
Steering SeekGoal(const AgentParams& p, double dt, const AgentMotion& m,
                  AgentBrain& b) {
  const bool wing = IsWing(p);
  const AltitudeBand band = Band(p.area);
  const double climb = wing ? p.max_climb_rate : HoverClimbLimit(p);
  const double target_altitude =
      std::clamp(b.destination.y, band.low, band.high);
  Steering out;
  out.desired.y = std::clamp(kAltitudeGain * (target_altitude - m.position.y),
                             -climb, climb);

  Vec3 horizontal;
  if (wing && b.mode == AgentMode::kLoiter) {
    const Vec3 from_center = Horizontal(m.position - b.loiter_center);
    const double r = Length(from_center);
    const Vec3 outward = NormalizedOr(from_center, Vec3::UnitX());
    const Vec3 tangent = Cross(Vec3::UnitY(), outward) * b.turn_direction;
    const double v = b.leg_speed;
    horizontal =
        tangent * v + outward * std::clamp(kOrbitGain * (b.loiter_radius - r),
                                           -0.5 * v, 0.5 * v);
    out.turn_rate = b.turn_direction * v / std::max(b.loiter_radius, 1.0);
  } else {
    const Vec3 to = Horizontal(b.destination - m.position);
    const double distance = Length(to);
    if (wing) {
      const Vec3 track =
          NormalizedOr(Horizontal(m.velocity), HeadingVector(b.heading));
      horizontal = NormalizedOr(to, track) * b.leg_speed;
    } else if (distance > 1e-9) {
      // Arrive: never faster than lets it brake to a stop on the spot, and
      // proportional to the distance at the very end (a pure braking curve
      // has unbounded gain there and would chatter around the goal).
      const double brake =
          0.5 * std::min(p.max_acceleration, TurnAccelLimit(p));
      const double speed =
          std::min({b.leg_speed, std::sqrt(2.0 * brake * distance),
                    kHoldGain * distance});
      horizontal = to * (speed / distance);
    }
    if (b.mode == AgentMode::kTransit) {
      // Meander: random turning *rate* (filtered noise), integrated into a
      // heading offset that is pulled back toward straight. Driving the
      // rate rather than the angle keeps the path smooth.
      b.wander_rate = b.wander_rate * (1.0 - kWanderRateDecay * dt) +
                      b.rng.Uniform(-1.0, 1.0) * kWanderNoise * std::sqrt(dt);
      b.wander_angle += (b.wander_rate - kWanderDecay * b.wander_angle) * dt;
      if (std::abs(b.wander_angle) > kMaxWander) {
        b.wander_angle = std::clamp(b.wander_angle, -kMaxWander, kMaxWander);
        b.wander_rate = 0.0;
      }
      const double fade = wing ? 1.0 : std::min(1.0, distance / 10.0);
      horizontal = RotateAboutUp(
          horizontal, b.wander_angle * std::clamp(p.meander, 0.0, 1.0) * fade);
    }
  }
  out.desired.x = horizontal.x;
  out.desired.z = horizontal.z;
  return out;
}

// Turns back toward the area once the agent nears its edge.
void StayInArea(const AgentParams& p, const AgentMotion& m, AgentBrain& b,
                Steering& s) {
  if (IsWing(p) && b.mode == AgentMode::kLoiter) return;
  const double radius = AreaRadius(p.area);
  const double margin = EdgeMargin(p, Length(m.velocity));
  const Vec3 offset = Horizontal(m.position - p.area.center);
  const double inner = radius - margin;
  const double distance = Length(offset);
  if (distance <= inner || margin <= 0.0) return;
  const double weight = std::clamp((distance - inner) / margin, 0.0, 1.0);
  const Vec3 inward = -offset / distance;
  const Vec3 horizontal = Horizontal(s.desired);
  // Wings turn back at cruise speed; a hover agent only insists on moving
  // once it is actually outside (it may hover at a goal near the edge).
  double speed = Length(horizontal);
  if (IsWing(p)) {
    speed = std::max(speed, b.leg_speed);
  } else if (distance > radius) {
    speed = std::max(speed, 1.5);
  }
  const Vec3 blended =
      NormalizedOr(Lerp(NormalizedOr(horizontal, inward), inward, weight),
                   inward) *
      speed;
  s.desired.x = blended.x;
  s.desired.z = blended.z;
  b.avoidance.boundary = true;
}

// Keeps the agent inside its altitude band, looking a little ahead so a
// descent is arrested before the floor rather than at it.
void HoldAltitudeBand(const AgentParams& p, const AgentMotion& m, AgentBrain& b,
                      Steering& s) {
  const AltitudeBand band = Band(p.area);
  const double climb = IsWing(p) ? p.max_climb_rate : HoverClimbLimit(p);
  const double ahead = m.position.y + m.velocity.y * 0.5 * p.lookahead;
  if (m.position.y < band.low || ahead < band.low) {
    const double up =
        std::clamp(kAltitudeGain * (band.low + kAltitudeMargin - m.position.y),
                   0.5, std::max(climb, 0.5));
    s.desired.y = std::max(s.desired.y, up);
    b.avoidance.altitude = true;
  }
  if (m.position.y > band.high || ahead > band.high) {
    const double down = std::clamp(
        kAltitudeGain * (m.position.y - (band.high - kAltitudeMargin)), 0.5,
        std::max(climb, 0.5));
    s.desired.y = std::min(s.desired.y, -down);
    b.avoidance.altitude = true;
  }
}

// Index of the obstacle the agent would first enter (its inflated sphere)
// if it flew from `from` at `velocity` for `horizon` seconds, or -1. The
// test runs in each obstacle's own moving frame (relative velocity), so a
// moving obstacle is judged by where it is going, not just where it is.
// `start_time` shifts obstacles forward, for feelers that begin in the
// future. Obstacles the agent is already inside are left to the
// separation push.
int FirstBlocker(const AgentParams& p, std::span<const AgentObstacle> obstacles,
                 const Vec3& from, const Vec3& velocity, double horizon,
                 double start_time = 0.0) {
  int blocker = -1;
  double soonest = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < obstacles.size(); ++i) {
    const AgentObstacle& o = obstacles[i];
    if (!IsRelevant(p, o)) continue;
    const double inflated = InflatedRadius(p, o);
    const Vec3 to_center = o.center + o.velocity * start_time - from;
    if (LengthSquared(to_center) < Square(inflated)) continue;
    const Vec3 relative = velocity - o.velocity;
    const double relative_speed = Length(relative);
    if (relative_speed < 1e-9) continue;
    const Vec3 direction = relative / relative_speed;
    const double length = relative_speed * horizon;
    const double along = Dot(to_center, direction);
    if (along <= 0.0 || along - inflated > length) continue;
    const Vec3 closest = direction * std::min(along, length);
    const double time = along / relative_speed;
    if (Distance(closest, to_center) < inflated && time < soonest) {
      soonest = time;
      blocker = static_cast<int>(i);
    }
  }
  return blocker;
}

// Index of an obstacle whose inflated sphere the wing's path enters if it
// keeps turning at its current rate for `seconds`, or -1. Straight-line
// feelers cannot see a turn carrying the craft into something.
int ArcBlocker(const AgentParams& p, std::span<const AgentObstacle> obstacles,
               const AgentMotion& m, double heading, double seconds) {
  const double horizontal = HorizontalLength(m.velocity);
  if (horizontal < 0.5 || seconds <= 0.0) return -1;
  // Current turn rate, read back from the sideways acceleration.
  const double rate =
      Dot(m.acceleration, Cross(Vec3::UnitY(), HeadingVector(heading))) /
      horizontal;
  constexpr int kArcSegments = 12;
  const double dt = seconds / kArcSegments;
  Vec3 point = m.position;
  double h = heading;
  for (int i = 0; i < kArcSegments; ++i) {
    const Vec3 velocity = HeadingVector(h + 0.5 * rate * dt) * horizontal +
                          Vec3{0.0, m.velocity.y, 0.0};
    const int blocker = FirstBlocker(p, obstacles, point, velocity, dt, i * dt);
    if (blocker >= 0) return blocker;
    point += velocity * dt;
    h += rate * dt;
  }
  return -1;
}

// Obstacle avoidance by ray casting ("context steering"): if the requested
// direction would enter an obstacle's inflated sphere within the look-ahead
// distance, fly the smallest left or right deviation that is clear instead,
// sticking to one side until the way ahead is clear.
void PickClearDirection(const AgentParams& p,
                        std::span<const AgentObstacle> obstacles,
                        double goal_range, const AgentMotion& m, AgentBrain& b,
                        Steering& s) {
  const double speed = Length(m.velocity);
  const double request = Length(s.desired);
  const Vec3 direction = NormalizedOr(
      s.desired, speed > 0.5 ? m.velocity / speed : HeadingVector(b.heading));
  // Probe at the speed it is (or wants to be) flying, for the look-ahead
  // time but not far beyond the destination, and at least a couple of
  // metres.
  const double probe_speed = std::max({speed, request, 0.5});
  const double horizon =
      std::max(std::min(p.lookahead, std::max(goal_range, 2.0) / probe_speed),
               2.0 / probe_speed);

  int blocker =
      FirstBlocker(p, obstacles, m.position, direction * probe_speed, horizon);
  // A wing cannot swing onto the requested direction at once, so the track
  // it is actually flying is checked too and takes priority when blocked:
  // deviations are then measured from where it is really going.
  Vec3 base = direction;
  int first_step = 1;
  if (IsWing(p) && speed > 0.5) {
    const Vec3 track = m.velocity / speed;
    const int track_blocker =
        FirstBlocker(p, obstacles, m.position, m.velocity, p.lookahead);
    const int arc_blocker = ArcBlocker(p, obstacles, m, b.heading, p.lookahead);
    if (track_blocker >= 0 || arc_blocker >= 0) {
      blocker = track_blocker >= 0 ? track_blocker : arc_blocker;
      base = track;
      // Only the turn is the problem: flying straight on is an option.
      if (track_blocker < 0) first_step = 0;
    }
  }
  if (blocker < 0) {
    b.avoid_side = 0.0;
    return;
  }
  b.avoidance.obstacle = blocker;
  if (b.avoid_side == 0.0) {
    // Turn away from the side the obstacle is on; head-on, keep right.
    const Vec3 to_obstacle =
        obstacles[static_cast<std::size_t>(blocker)].center - m.position;
    b.avoid_side = Cross(base, to_obstacle).y < -1e-9 ? 1.0 : -1.0;
  }

  // Smallest clear deviation on each side. Stay on the current side unless
  // the other needs a clearly smaller turn: switching on any small
  // difference would let a moving obstacle flip the choice step after step.
  const auto search = [&](const Vec3& from, int first) -> std::optional<Vec3> {
    const auto smallest_clear_step = [&](double side) {
      for (int step = first; step <= kAvoidSearchSteps; ++step) {
        const Vec3 candidate =
            RotateAboutUp(from, side * kPi * step / kAvoidSearchSteps);
        if (FirstBlocker(p, obstacles, m.position, candidate * probe_speed,
                         horizon) < 0) {
          return step;
        }
      }
      return kAvoidSearchSteps + 1;  // Nothing clear on this side.
    };
    int step = smallest_clear_step(b.avoid_side);
    const int other = smallest_clear_step(-b.avoid_side);
    if (other < step &&
        (step > kAvoidSearchSteps || other + kSideSwitchSteps <= step)) {
      b.avoid_side = -b.avoid_side;
      step = other;
    }
    if (step > kAvoidSearchSteps) return std::nullopt;
    return RotateAboutUp(from, b.avoid_side * kPi * step / kAvoidSearchSteps);
  };
  std::optional<Vec3> chosen = search(base, first_step);
  // Turning about the vertical cannot steer a mostly vertical request
  // around something below or above: hold height and sidestep out of its
  // column instead (away from it, or to the right if dead centre).
  bool sidestepped = false;
  if (!chosen.has_value() || HorizontalLength(base) < 0.5) {
    const Vec3 out_of_column = Horizontal(
        m.position - obstacles[static_cast<std::size_t>(blocker)].center);
    const Vec3 level = NormalizedOr(
        out_of_column,
        NormalizedOr(Cross(HeadingVector(b.heading), Vec3::UnitY()),
                     Vec3::UnitX()));
    if (std::optional<Vec3> sidestep = search(level, 0)) {
      chosen = sidestep;
      sidestepped = true;
    }
  }
  double magnitude = IsWing(p) ? std::max(request, b.leg_speed) : request;
  if (sidestepped && !IsWing(p))
    magnitude = std::max(magnitude, kSidestepSpeed);
  if (chosen.has_value()) {
    s.desired = *chosen * magnitude;
  } else {
    // Boxed in: a hover agent stops; a wing turns as hard as it can.
    s.desired = IsWing(p)
                    ? RotateAboutUp(base, b.avoid_side * kHalfPi) * magnitude
                    : Vec3::Zero();
  }
  s.turn_rate = 0.0;  // Avoidance overrides any circling.
}

// Separation: anything the agent is already within the clearance of pushes
// it straight away, harder the deeper it is.
void ApplySeparation(const AgentParams& p,
                     std::span<const AgentObstacle> obstacles,
                     const AgentMotion& m, AgentBrain& b, Steering& s) {
  Vec3 push;
  double deepest = 0.0;
  for (std::size_t i = 0; i < obstacles.size(); ++i) {
    const AgentObstacle& o = obstacles[i];
    if (!IsRelevant(p, o)) continue;
    const double inflated = InflatedRadius(p, o);
    const Vec3 away = m.position - o.center;
    const double distance = Length(away);
    if (inflated <= 0.0 || distance >= inflated) continue;
    const double depth = (inflated - distance) / inflated;
    push += NormalizedOr(away, Vec3::UnitX()) * (1.0 + depth);
    if (depth > deepest) {
      deepest = depth;
      b.avoidance.obstacle = static_cast<int>(i);
    }
  }
  if (deepest > 0.0) {
    s.desired += push * (kEscapeGain * std::max(b.leg_speed, 2.0));
    s.turn_rate = 0.0;
  }
}

// Clamps the request to what the vehicle may ask for.
void LimitRequest(const AgentParams& p, Steering& s) {
  if (IsWing(p)) {
    s.desired.y = std::clamp(s.desired.y, -p.max_climb_rate, p.max_climb_rate);
    return;
  }
  const double climb = HoverClimbLimit(p);
  s.desired.y = std::clamp(s.desired.y, -climb, climb);
  const double cap = HoverHorizontalCap(p);
  const double horizontal = HorizontalLength(s.desired);
  if (horizontal > cap) {
    s.desired.x *= cap / horizontal;
    s.desired.z *= cap / horizontal;
  }
}

// Never through the ground plane (the altitude band normally keeps agents
// well clear of it).
void KeepAboveGround(AgentMotion& m) {
  if (m.position.y < 0.0) {
    m.position.y = 0.0;
    m.velocity.y = std::max(m.velocity.y, 0.0);
  }
}

// Multirotor-like point mass. Acceleration is limited so the thrust axis
// (along a + g) never tilts beyond max_tilt and never points down; the
// body yaws to face its direction of travel, or scans while hovering.
void MoveHover(const AgentParams& p, const Steering& s, double dt,
               AgentBrain& b, AgentMotion& m) {
  const Vec3 v = m.velocity;
  Vec3 a = (s.desired - v) * std::min(1.0 / kHoverResponse, 1.0 / dt);
  // Jerk limit: the acceleration (and with it the tilt) changes smoothly.
  const double max_change =
      std::max(p.max_acceleration, 0.0) / kHoverJerkTime * dt;
  const Vec3 change = a - m.acceleration;
  const double change_length = Length(change);
  if (change_length > max_change) {
    a = m.acceleration + change * (max_change / change_length);
  }

  // Vertical first, so the tilt limit can account for it.
  const double climb = HoverClimbLimit(p);
  const double accel = std::max(p.max_acceleration, 0.0);
  double low = std::max(-accel, -kMaxDescentAccel);
  double high = accel;
  const double keep_low = (-climb - v.y) / dt;
  const double keep_high = (climb - v.y) / dt;
  if (std::max(low, keep_low) <= std::min(high, keep_high)) {
    low = std::max(low, keep_low);
    high = std::min(high, keep_high);
  }
  a.y = std::clamp(a.y, low, high);

  const double tilt = std::clamp(p.max_tilt, 0.0, kMaxBank);
  const double horizontal_limit =
      std::min(accel, (kStandardGravity + a.y) * std::tan(tilt));
  const double horizontal = std::hypot(a.x, a.z);
  if (horizontal > horizontal_limit) {
    const double scale = horizontal_limit / horizontal;
    a.x *= scale;
    a.z *= scale;
  }

  m.velocity = v + a * dt;
  m.position += m.velocity * dt;
  KeepAboveGround(m);
  m.acceleration = a;

  NextHoverHeading(p, m.velocity, dt, b);

  // Attitude: thrust axis along a + g, nose as close to the heading as the
  // tilt allows.
  const Vec3 up = Normalized(a + Vec3{0.0, kStandardGravity, 0.0});
  const Vec3 right =
      NormalizedOr(Cross(HeadingVector(b.heading), up), Vec3::UnitX());
  const Vec3 forward = Cross(up, right);
  m.orientation = FromMat3(Mat3::FromColumns(right, up, -forward));
}

// Airplane-like point mass flown by speed, track and climb rate: speed
// stays within [min, max], turns are limited by the bank angle, the climb
// rate by the climb limit and a path angle, and the body banks into turns
// (its "up" follows a + g, as in a coordinated turn).
void MoveWing(const AgentParams& p, const Steering& s, double dt, AgentBrain& b,
              AgentMotion& m) {
  const Vec3 v = m.velocity;
  const double min_speed = MinFlightSpeed(p);
  const double max_speed = MaxFlightSpeed(p);
  const double speed = std::clamp(Length(v), min_speed, max_speed);
  const double heading = HorizontalLength(v) > 1e-6 ? HeadingOf(v) : b.heading;
  const double horizontal_speed =
      std::sqrt(std::max(Square(speed) - Square(v.y), 0.0));
  const double accel = std::max(p.max_acceleration, 0.0);

  // Speed toward the requested speed.
  const double target_speed =
      std::clamp(Length(s.desired), min_speed, max_speed);
  const double along =
      std::clamp((target_speed - speed) / kWingSpeedResponse, -accel, accel);
  const double new_speed = std::clamp(speed + along * dt, min_speed, max_speed);

  // Track toward the requested direction, at most a full-bank turn.
  const Vec3 request = Horizontal(s.desired);
  const double error = HorizontalLength(request) > 1e-6
                           ? WrapAngle(HeadingOf(request) - heading)
                           : 0.0;
  const double max_rate = TurnAccelLimit(p) / std::max(horizontal_speed, 1e-3);
  // Roll-rate limit: the turn rate (bank) builds up rather than jumping.
  // The current rate is read back from the sideways acceleration.
  const double current_rate =
      Dot(m.acceleration, Cross(Vec3::UnitY(), HeadingVector(heading))) /
      std::max(horizontal_speed, 1e-3);
  const double roll_step = max_rate / kWingRollTime * dt;
  const double turn_rate =
      std::clamp(std::clamp(s.turn_rate + error / kWingTurnResponse,
                            current_rate - roll_step, current_rate + roll_step),
                 -max_rate, max_rate);
  const double new_heading = heading + turn_rate * dt;

  // Climb rate toward the request, within the climb and path-angle limits.
  const double climb_limit =
      std::min(p.max_climb_rate, new_speed * std::sin(kWingMaxPathAngle));
  const double vertical = std::clamp((s.desired.y - v.y) / kWingClimbResponse,
                                     -0.5 * accel, 0.5 * accel);
  const double new_vy =
      std::clamp(v.y + vertical * dt, -climb_limit, climb_limit);

  const double new_horizontal =
      std::sqrt(std::max(Square(new_speed) - Square(new_vy), 0.0));
  const Vec3 track = HeadingVector(new_heading);
  m.velocity = {track.x * new_horizontal, new_vy, track.z * new_horizontal};
  m.acceleration = (m.velocity - v) / dt;
  m.position += m.velocity * dt;
  KeepAboveGround(m);
  b.heading = WrapAngle(new_heading);
  m.orientation = LookRotation(
      m.velocity, m.acceleration + Vec3{0.0, kStandardGravity, 0.0});
}

}  // namespace

std::string_view FlightStyleName(FlightStyle style) {
  switch (style) {
    case FlightStyle::kHover:
      return "Hover";
    case FlightStyle::kWing:
      return "Wing";
  }
  return "Unknown";
}

std::string_view FlightModelName(FlightModel model) {
  switch (model) {
    case FlightModel::kSimple:
      return "Simple";
    case FlightModel::kPhysical:
      return "Physical";
  }
  return "Unknown";
}

std::string_view AgentModeName(AgentMode mode) {
  switch (mode) {
    case AgentMode::kTransit:
      return "Transit";
    case AgentMode::kLoiter:
      return "Loiter";
  }
  return "Unknown";
}

AgentParams DefaultAgentParams(FlightStyle style) {
  AgentParams p;  // The member defaults describe a small hover agent.
  p.style = style;
  if (style == FlightStyle::kWing) {
    p.area.radius = 80.0;
    p.area.min_altitude = 12.0;
    p.area.max_altitude = 40.0;
    p.min_speed = 7.0;
    p.cruise_speed_min = 10.0;
    p.cruise_speed_max = 15.0;
    p.max_speed = 18.0;
    p.max_acceleration = 3.0;
    p.max_tilt = Radians(40.0);
    p.max_climb_rate = 3.0;
    p.loiter_chance = 0.5;
    p.loiter_min = 6.0;
    p.loiter_max = 12.0;
    p.meander = 0.25;
    p.body_radius = 1.2;
    p.clearance = 3.0;
    p.lookahead = 3.0;
  }
  return p;
}

AgentParams SanitizeAgentParams(const AgentParams& params) {
  const AgentParams d = DefaultAgentParams(params.style);
  // Finite and within [low, high], or the default when not a number.
  const auto fix = [](double value, double fallback, double low, double high) {
    return std::isfinite(value) ? std::clamp(value, low, high)
                                : std::clamp(fallback, low, high);
  };
  AgentParams p = params;
  FlightArea& a = p.area;
  a.center = {fix(a.center.x, 0.0, -kMaxCoordinate, kMaxCoordinate),
              fix(a.center.y, 0.0, -kMaxCoordinate, kMaxCoordinate),
              fix(a.center.z, 0.0, -kMaxCoordinate, kMaxCoordinate)};
  a.radius = fix(a.radius, d.area.radius, 1.0, kMaxCoordinate);
  a.min_altitude =
      fix(a.min_altitude, d.area.min_altitude, 0.0, kMaxCoordinate);
  a.max_altitude =
      fix(a.max_altitude, d.area.max_altitude, 0.0, kMaxCoordinate);
  if (a.min_altitude > a.max_altitude) {
    std::swap(a.min_altitude, a.max_altitude);
  }
  p.min_speed = fix(p.min_speed, d.min_speed, 0.0, kMaxParamSpeed);
  p.cruise_speed_min =
      fix(p.cruise_speed_min, d.cruise_speed_min, 0.0, kMaxParamSpeed);
  p.cruise_speed_max =
      fix(p.cruise_speed_max, d.cruise_speed_max, 0.0, kMaxParamSpeed);
  if (p.cruise_speed_min > p.cruise_speed_max) {
    std::swap(p.cruise_speed_min, p.cruise_speed_max);
  }
  p.max_speed = fix(p.max_speed, d.max_speed, 0.1, kMaxParamSpeed);
  p.max_acceleration =
      fix(p.max_acceleration, d.max_acceleration, 0.01, kMaxParamAccel);
  p.max_tilt = fix(p.max_tilt, d.max_tilt, kMinBank, kMaxBank);
  p.max_climb_rate =
      fix(p.max_climb_rate, d.max_climb_rate, 0.0, kMaxParamSpeed);
  p.max_yaw_rate = fix(p.max_yaw_rate, d.max_yaw_rate, 0.0, 100.0);
  p.loiter_chance = fix(p.loiter_chance, d.loiter_chance, 0.0, 1.0);
  p.loiter_min = fix(p.loiter_min, d.loiter_min, 0.0, kMaxParamTime);
  p.loiter_max = fix(p.loiter_max, d.loiter_max, 0.0, kMaxParamTime);
  if (p.loiter_min > p.loiter_max) std::swap(p.loiter_min, p.loiter_max);
  p.meander = fix(p.meander, d.meander, 0.0, 1.0);
  p.body_radius = fix(p.body_radius, d.body_radius, 0.0, kMaxParamSize);
  p.clearance = fix(p.clearance, d.clearance, 0.0, kMaxParamSize);
  p.lookahead = fix(p.lookahead, d.lookahead, 0.1, 60.0);
  if (p.model != FlightModel::kPhysical) p.model = FlightModel::kSimple;
  return p;
}

double MinTurnRadius(const AgentParams& params, double speed) {
  const double accel = TurnAccelLimit(params);
  if (accel <= 1e-9) return std::numeric_limits<double>::infinity();
  return Square(speed) / accel;
}

void ResetBrain(const AgentParams& params, const AgentMotion& motion,
                AgentBrain& brain) {
  brain = AgentBrain();
  brain.rng = Rng(params.seed);
  brain.destination = motion.position;
  brain.loiter_center = motion.position;
  const Vec3 forward = Rotate(motion.orientation, -Vec3::UnitZ());
  brain.heading = HorizontalLength(forward) > 1e-6 ? HeadingOf(forward) : 0.0;
}

void Replan(AgentBrain& brain) {
  brain.has_destination = false;
  brain.mode = AgentMode::kTransit;
  brain.mode_time = 0.0;
  brain.mode_duration = 0.0;
  brain.avoid_side = 0.0;
  brain.wander_angle = 0.0;
  brain.wander_rate = 0.0;
}

void FitToEnvelope(const AgentParams& params, AgentMotion& motion) {
  const AgentParams p = SanitizeAgentParams(params);
  if (!IsFinite(motion.velocity)) motion.velocity = Vec3::Zero();
  // Heading: along the track if moving, else along the nose; a nose that
  // points straight up or down takes it from the body's up axis (which
  // then points backward or forward).
  const Vec3 forward = Rotate(motion.orientation, -Vec3::UnitZ());
  const Vec3 up = Rotate(motion.orientation, Vec3::UnitY());
  double heading = 0.0;
  if (HorizontalLength(motion.velocity) > 1e-6) {
    heading = HeadingOf(motion.velocity);
  } else if (HorizontalLength(forward) > 1e-6) {
    heading = HeadingOf(forward);
  } else if (HorizontalLength(up) > 1e-6) {
    heading = HeadingOf(forward.y > 0.0 ? -up : up);
  }
  motion.acceleration = Vec3::Zero();
  if (IsWing(p)) {
    // Level flight along the nose (or track), within the speed range.
    const double speed =
        std::clamp(std::max(Length(motion.velocity), p.cruise_speed_min),
                   MinFlightSpeed(p), MaxFlightSpeed(p));
    motion.velocity = HeadingVector(heading) * speed;
    motion.orientation = LookRotation(motion.velocity, Vec3::UnitY());
    return;
  }
  // Hover: within the climb limit and the horizontal speed cap, level.
  const double climb = HoverClimbLimit(p);
  Vec3 v = motion.velocity;
  v.y = std::clamp(v.y, -climb, climb);
  const double horizontal = HorizontalLength(v);
  const double cap = HoverHorizontalCap(p);
  if (horizontal > cap) {
    v.x *= cap / horizontal;
    v.z *= cap / horizontal;
  }
  motion.velocity = v;
  motion.orientation = LookRotation(HeadingVector(heading), Vec3::UnitY());
}

std::optional<AgentDecision> PlanAgent(const AgentParams& params,
                                       std::span<const AgentObstacle> obstacles,
                                       double dt, const AgentMotion& motion,
                                       AgentBrain& brain) {
  if (!(dt > 0.0) || !std::isfinite(dt)) return std::nullopt;
  const AgentParams p = SanitizeAgentParams(params);
  brain.avoidance = AvoidanceStatus();
  const std::optional<AgentDecision> decision =
      Decide(p, obstacles, dt, motion, brain);

  Steering steering = SeekGoal(p, dt, motion, brain);
  StayInArea(p, motion, brain, steering);
  HoldAltitudeBand(p, motion, brain, steering);
  // Obstacles beyond the destination do not matter (except when circling).
  const double goal_range = IsWing(p) && brain.mode == AgentMode::kLoiter
                                ? std::numeric_limits<double>::infinity()
                                : Distance(motion.position, brain.destination);
  PickClearDirection(p, obstacles, goal_range, motion, brain, steering);
  ApplySeparation(p, obstacles, motion, brain, steering);
  // Again, so no avoidance push can drive it out of its altitude band.
  HoldAltitudeBand(p, motion, brain, steering);
  LimitRequest(p, steering);
  brain.desired_velocity = steering.desired;
  brain.desired_turn_rate = steering.turn_rate;
  return decision;
}

double NextHoverHeading(const AgentParams& params, const Vec3& velocity,
                        double dt, AgentBrain& brain) {
  const double yaw_step = std::isfinite(params.max_yaw_rate) && dt > 0.0
                              ? std::max(params.max_yaw_rate, 0.0) * dt
                              : 0.0;
  if (brain.mode == AgentMode::kLoiter) {
    brain.heading +=
        brain.turn_direction * std::min(kHoverScanRate * dt, yaw_step);
  } else if (HorizontalLength(velocity) > 1.0) {
    const double error = WrapAngle(HeadingOf(velocity) - brain.heading);
    brain.heading += std::clamp(error, -yaw_step, yaw_step);
  }
  brain.heading = WrapAngle(brain.heading);
  return brain.heading;
}

std::optional<AgentDecision> StepAgent(const AgentParams& params,
                                       std::span<const AgentObstacle> obstacles,
                                       double dt, AgentBrain& brain,
                                       AgentMotion& motion) {
  if (!(dt > 0.0) || !std::isfinite(dt)) return std::nullopt;
  const std::optional<AgentDecision> decision =
      PlanAgent(params, obstacles, dt, motion, brain);
  const AgentParams p = SanitizeAgentParams(params);
  const Steering steering{.desired = brain.desired_velocity,
                          .turn_rate = brain.desired_turn_rate};
  if (IsWing(p)) {
    MoveWing(p, steering, dt, brain, motion);
  } else {
    MoveHover(p, steering, dt, brain, motion);
  }
  return decision;
}

}  // namespace aerolab::sim
