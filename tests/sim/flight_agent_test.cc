#include "sim/flight_agent.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include "math/quat.h"
#include "math/random.h"
#include "math/scalar.h"
#include "math/vec3.h"
#include "sim/force_model.h"

namespace aerolab::sim {
namespace {

constexpr double kDt = 1.0 / 120.0;

double HorizontalDistance(const Vec3& a, const Vec3& b) {
  return std::hypot(a.x - b.x, a.z - b.z);
}

// One agent plus everything it decided.
struct Flight {
  AgentParams params;
  AgentBrain brain;
  AgentMotion motion;
  std::vector<AgentDecision> decisions;

  // Steps for `seconds`, calling `check(*this)` after every step.
  template <typename Check>
  void Run(double seconds, std::span<const AgentObstacle> obstacles,
           Check&& check) {
    const int steps = static_cast<int>(std::lround(seconds / kDt));
    for (int i = 0; i < steps; ++i) {
      if (const std::optional<AgentDecision> decision =
              StepAgent(params, obstacles, kDt, brain, motion)) {
        decisions.push_back(*decision);
      }
      check(*this);
    }
  }
  void Run(double seconds, std::span<const AgentObstacle> obstacles = {}) {
    Run(seconds, obstacles, [](const Flight&) {});
  }

  int CountDecisions(AgentDecision::Kind kind) const {
    return static_cast<int>(std::count_if(
        decisions.begin(), decisions.end(),
        [kind](const AgentDecision& d) { return d.kind == kind; }));
  }
};

Flight MakeFlight(FlightStyle style, std::uint64_t seed,
                  const Vec3& start = {0.0, 12.0, 0.0}) {
  Flight flight;
  flight.params = DefaultAgentParams(style);
  flight.params.seed = seed;
  flight.params.area.center = {start.x, 0.0, start.z};
  flight.motion.position = start;
  flight.motion.orientation = Quat::Identity();  // Facing -Z.
  if (style == FlightStyle::kWing) {
    flight.motion.velocity = {0.0, 0.0, -flight.params.cruise_speed_min};
  }
  ResetBrain(flight.params, flight.motion, flight.brain);
  return flight;
}

Vec3 Forward(const Quat& q) { return Rotate(q, -Vec3::UnitZ()); }
Vec3 Up(const Quat& q) { return Rotate(q, Vec3::UnitY()); }

TEST(FlightAgentTest, NamesAreReadable) {
  EXPECT_EQ(FlightStyleName(FlightStyle::kHover), "Hover");
  EXPECT_EQ(FlightStyleName(FlightStyle::kWing), "Wing");
  EXPECT_EQ(AgentModeName(AgentMode::kTransit), "Transit");
  EXPECT_EQ(AgentModeName(AgentMode::kLoiter), "Loiter");
}

TEST(FlightAgentTest, FirstStepChoosesADestination) {
  Flight flight = MakeFlight(FlightStyle::kHover, 3);
  EXPECT_FALSE(flight.brain.has_destination);
  flight.Run(kDt);
  ASSERT_EQ(flight.decisions.size(), 1u);
  EXPECT_EQ(flight.decisions[0].kind, AgentDecision::Kind::kNewDestination);
  EXPECT_TRUE(flight.brain.has_destination);
  EXPECT_EQ(flight.brain.mode, AgentMode::kTransit);
}

TEST(FlightAgentTest, SameSeedReplaysExactly) {
  for (FlightStyle style : {FlightStyle::kHover, FlightStyle::kWing}) {
    Flight a = MakeFlight(style, 11);
    Flight b = MakeFlight(style, 11);
    a.Run(90.0);
    b.Run(90.0);
    EXPECT_EQ(a.motion.position, b.motion.position);
    EXPECT_EQ(a.motion.velocity, b.motion.velocity);
    EXPECT_EQ(a.motion.orientation.w, b.motion.orientation.w);
    EXPECT_EQ(a.motion.orientation.x, b.motion.orientation.x);
    EXPECT_EQ(a.motion.orientation.y, b.motion.orientation.y);
    EXPECT_EQ(a.motion.orientation.z, b.motion.orientation.z);
    EXPECT_EQ(a.brain.rng.state(), b.brain.rng.state());
    EXPECT_EQ(a.decisions.size(), b.decisions.size());
  }
}

TEST(FlightAgentTest, DifferentSeedsFlyDifferently) {
  for (FlightStyle style : {FlightStyle::kHover, FlightStyle::kWing}) {
    Flight a = MakeFlight(style, 1);
    Flight b = MakeFlight(style, 2);
    a.Run(30.0);
    b.Run(30.0);
    EXPECT_GT(Distance(a.motion.position, b.motion.position), 1.0);
  }
}

TEST(FlightAgentTest, CopiedBrainForecastsTheFuture) {
  Flight flight = MakeFlight(FlightStyle::kWing, 5);
  flight.Run(10.0);
  Flight forecast = flight;
  forecast.Run(8.0);
  flight.Run(8.0);
  EXPECT_EQ(forecast.motion.position, flight.motion.position);
}

TEST(FlightAgentTest, HoverStaysInItsAreaAndAltitudeBand) {
  Flight flight = MakeFlight(FlightStyle::kHover, 21);
  const FlightArea area = flight.params.area;
  double worst_outside = 0.0;
  double lowest = 1e9;
  double highest = -1e9;
  flight.Run(600.0, {}, [&](const Flight& f) {
    worst_outside = std::max(
        worst_outside,
        HorizontalDistance(f.motion.position, area.center) - area.radius);
    lowest = std::min(lowest, f.motion.position.y);
    highest = std::max(highest, f.motion.position.y);
  });
  EXPECT_LT(worst_outside, 2.0);
  EXPECT_GT(lowest, area.min_altitude - 1.0);
  EXPECT_LT(highest, area.max_altitude + 1.0);
}

TEST(FlightAgentTest, WingStaysInItsAreaAndAltitudeBand) {
  Flight flight = MakeFlight(FlightStyle::kWing, 22, {0.0, 25.0, 0.0});
  const FlightArea area = flight.params.area;
  double worst_outside = 0.0;
  double lowest = 1e9;
  double highest = -1e9;
  flight.Run(600.0, {}, [&](const Flight& f) {
    worst_outside = std::max(
        worst_outside,
        HorizontalDistance(f.motion.position, area.center) - area.radius);
    lowest = std::min(lowest, f.motion.position.y);
    highest = std::max(highest, f.motion.position.y);
  });
  // A wing cannot stop, so it may overshoot the edge by part of a turn.
  EXPECT_LT(worst_outside, 0.25 * area.radius);
  EXPECT_GT(lowest, area.min_altitude - 2.0);
  EXPECT_LT(highest, area.max_altitude + 2.0);
}

TEST(FlightAgentTest, ClimbsOffTheGroundIntoItsBand) {
  Flight flight = MakeFlight(FlightStyle::kHover, 4, {0.0, 0.3, 0.0});
  double lowest = 1e9;
  flight.Run(10.0, {}, [&](const Flight& f) {
    lowest = std::min(lowest, f.motion.position.y);
  });
  EXPECT_GE(lowest, 0.0);
  EXPECT_GT(flight.motion.position.y, flight.params.area.min_altitude - 0.5);
}

TEST(FlightAgentTest, HoverRespectsItsFlightEnvelope) {
  Flight flight = MakeFlight(FlightStyle::kHover, 8);
  const AgentParams& p = flight.params;
  flight.Run(300.0, {}, [&](const Flight& f) {
    const AgentMotion& m = f.motion;
    ASSERT_LE(Length(m.velocity), p.max_speed + 1e-6);
    ASSERT_LE(std::abs(m.velocity.y), p.max_climb_rate + 1e-6);
    const Vec3& a = m.acceleration;
    ASSERT_LE(std::hypot(a.x, a.z), p.max_acceleration + 1e-6);
    ASSERT_LE(std::abs(a.y), p.max_acceleration + 1e-6);
    // The body tilts so its thrust axis carries gravity plus acceleration.
    const Vec3 thrust_axis = Normalized(a + Vec3{0.0, kStandardGravity, 0.0});
    ASSERT_LT(AngleBetween(Up(m.orientation), thrust_axis), 1e-6);
    ASSERT_LE(AngleBetween(Up(m.orientation), Vec3::UnitY()),
              p.max_tilt + 1e-6);
  });
}

TEST(FlightAgentTest, WingRespectsItsFlightEnvelope) {
  Flight flight = MakeFlight(FlightStyle::kWing, 9, {0.0, 25.0, 0.0});
  const AgentParams& p = flight.params;
  const double max_turn_accel = kStandardGravity * std::tan(p.max_tilt);
  flight.Run(300.0, {}, [&](const Flight& f) {
    const AgentMotion& m = f.motion;
    const double speed = Length(m.velocity);
    ASSERT_GE(speed, p.min_speed - 1e-6);
    ASSERT_LE(speed, p.max_speed + 1e-6);
    ASSERT_LE(std::abs(m.velocity.y), p.max_climb_rate + 1e-6);
    // Horizontal acceleration across the track is what banking provides.
    const Vec3 track = Normalized(Vec3{m.velocity.x, 0.0, m.velocity.z});
    const Vec3 across = Cross(Vec3::UnitY(), track);
    ASSERT_LE(std::abs(Dot(m.acceleration, across)),
              max_turn_accel * 1.02 + 1e-6);
    // The nose points along the flight path.
    ASSERT_LT(AngleBetween(Forward(m.orientation), m.velocity), 1e-6);
  });
}

TEST(FlightAgentTest, HoverReachesDestinationsAndHovers) {
  Flight flight = MakeFlight(FlightStyle::kHover, 13);
  double slowest_settled_hover = 0.0;
  flight.Run(240.0, {}, [&](const Flight& f) {
    if (f.brain.mode == AgentMode::kLoiter && f.brain.mode_time > 1.5) {
      slowest_settled_hover =
          std::max(slowest_settled_hover, Length(f.motion.velocity));
    }
  });
  EXPECT_GE(flight.brain.legs_completed, 6);
  EXPECT_GE(flight.CountDecisions(AgentDecision::Kind::kLoiter), 1);
  EXPECT_LT(slowest_settled_hover, 0.6);
}

TEST(FlightAgentTest, WingCirclesWhileLoitering) {
  Flight flight = MakeFlight(FlightStyle::kWing, 14, {0.0, 25.0, 0.0});
  double worst_radius_error = 0.0;
  int loiter_samples = 0;
  flight.Run(400.0, {}, [&](const Flight& f) {
    if (f.brain.mode != AgentMode::kLoiter || f.brain.mode_time < 6.0) return;
    ++loiter_samples;
    const double radius =
        HorizontalDistance(f.motion.position, f.brain.loiter_center);
    const double expected = MinTurnRadius(f.params, f.brain.leg_speed) * 1.3;
    worst_radius_error =
        std::max(worst_radius_error, std::abs(radius - expected) / expected);
  });
  EXPECT_GE(flight.CountDecisions(AgentDecision::Kind::kLoiter), 1);
  ASSERT_GT(loiter_samples, 0);
  EXPECT_LT(worst_radius_error, 0.25);
  EXPECT_GE(flight.brain.legs_completed, 3);
}

TEST(FlightAgentTest, DestinationsLieInsideTheAreaAndSpeedsVary) {
  for (FlightStyle style : {FlightStyle::kHover, FlightStyle::kWing}) {
    Flight flight = MakeFlight(style, 31, {0.0, 20.0, 0.0});
    flight.Run(600.0);
    const AgentParams& p = flight.params;
    double slowest = 1e9;
    double fastest = 0.0;
    int count = 0;
    for (const AgentDecision& d : flight.decisions) {
      if (d.kind == AgentDecision::Kind::kLoiter) continue;
      ++count;
      EXPECT_LE(HorizontalDistance(d.destination, p.area.center),
                p.area.radius + 1e-9);
      EXPECT_GE(d.destination.y, p.area.min_altitude);
      EXPECT_LE(d.destination.y, p.area.max_altitude);
      EXPECT_GE(d.speed, p.cruise_speed_min);
      EXPECT_LE(d.speed, p.cruise_speed_max);
      slowest = std::min(slowest, d.speed);
      fastest = std::max(fastest, d.speed);
    }
    EXPECT_GE(count, 8);
    EXPECT_GT(fastest - slowest,
              0.3 * (p.cruise_speed_max - p.cruise_speed_min));
  }
}

TEST(FlightAgentTest, AvoidsAnObstacleOnItsPath) {
  for (FlightStyle style : {FlightStyle::kHover, FlightStyle::kWing}) {
    for (std::uint64_t seed : {41u, 42u, 43u}) {
      for (double size : {3.0, 6.0, 12.0}) {
        Flight flight = MakeFlight(style, seed, {0.0, 20.0, 0.0});
        flight.params.area.radius = 200.0;
        flight.params.meander = 0.0;  // Keep the obstacle squarely ahead.
        // Fly straight at a goal with a large sphere squarely in the way.
        flight.brain.has_destination = true;
        flight.brain.destination = {0.0, 20.0, -120.0};
        flight.brain.leg_speed = flight.params.cruise_speed_max;
        flight.brain.mode_duration = 1000.0;
        flight.motion.velocity =
            style == FlightStyle::kWing
                ? Vec3{0.0, 0.0, -flight.params.cruise_speed_max}
                : Vec3::Zero();
        const std::array<AgentObstacle, 1> rock = {
            AgentObstacle{{0.0, 20.0, -60.0}, size, {}}};
        double closest_gap = 1e9;
        bool reacted = false;
        flight.Run(25.0, rock, [&](const Flight& f) {
          closest_gap = std::min(closest_gap,
                                 Distance(f.motion.position, rock[0].center) -
                                     rock[0].radius - f.params.body_radius);
          reacted = reacted || f.brain.avoidance.obstacle == 0;
        });
        EXPECT_TRUE(reacted)
            << FlightStyleName(style) << " seed " << seed << " size " << size;
        EXPECT_GT(closest_gap, 0.0)
            << FlightStyleName(style) << " seed " << seed << " size " << size;
        EXPECT_GE(flight.brain.legs_completed, 1)
            << FlightStyleName(style) << " seed " << seed << " size " << size;
      }
    }
  }
}

TEST(FlightAgentTest, WandersAmongObstaclesWithoutTouchingAny) {
  for (FlightStyle style : {FlightStyle::kHover, FlightStyle::kWing}) {
    for (std::uint64_t seed = 200; seed < 206; ++seed) {
      Flight flight = MakeFlight(style, seed, {0.0, 20.0, 0.0});
      const FlightArea& area = flight.params.area;
      // A handful of random spheres inside the flight volume.
      Rng layout(seed * 7);
      std::vector<AgentObstacle> rocks;
      while (rocks.size() < 5) {
        const double angle = layout.Uniform(0.0, kTwoPi);
        const double r = area.radius * 0.8 * std::sqrt(layout.Uniform());
        const AgentObstacle rock{
            {r * std::cos(angle),
             layout.Uniform(area.min_altitude, area.max_altitude),
             r * std::sin(angle)},
            layout.Uniform(2.0, 6.0),
            {}};
        // Keep the start point clear: the agent must not begin inside one.
        if (Distance(rock.center, flight.motion.position) >
            rock.radius + 10.0) {
          rocks.push_back(rock);
        }
      }
      double closest_gap = 1e9;
      flight.Run(300.0, rocks, [&](const Flight& f) {
        for (const AgentObstacle& rock : rocks) {
          closest_gap =
              std::min(closest_gap, Distance(f.motion.position, rock.center) -
                                        rock.radius - f.params.body_radius);
        }
      });
      EXPECT_GT(closest_gap, 0.0) << FlightStyleName(style) << " seed " << seed;
      EXPECT_GE(flight.brain.legs_completed, 3)
          << FlightStyleName(style) << " seed " << seed;
    }
  }
}

TEST(FlightAgentTest, TwoAgentsSharingAnAreaNeverTouch) {
  Flight a = MakeFlight(FlightStyle::kHover, 51, {0.0, 12.0, 0.0});
  Flight b = MakeFlight(FlightStyle::kHover, 52, {4.0, 12.0, 0.0});
  b.params.area = a.params.area;
  b.params.area.radius = a.params.area.radius = 15.0;  // Crowded.
  double closest = 1e9;
  const int steps = static_cast<int>(300.0 / kDt);
  for (int i = 0; i < steps; ++i) {
    const std::array<AgentObstacle, 1> sees_b = {AgentObstacle{
        b.motion.position, b.params.body_radius, b.motion.velocity}};
    const std::array<AgentObstacle, 1> sees_a = {AgentObstacle{
        a.motion.position, a.params.body_radius, a.motion.velocity}};
    StepAgent(a.params, sees_b, kDt, a.brain, a.motion);
    StepAgent(b.params, sees_a, kDt, b.brain, b.motion);
    closest = std::min(closest, Distance(a.motion.position, b.motion.position));
  }
  // They keep apart, preserving most of the clearance band, not merely
  // avoiding contact.
  EXPECT_GT(closest, a.params.body_radius + b.params.body_radius +
                         0.5 * a.params.clearance);
}

TEST(FlightAgentTest, GivesUpOnADestinationItCannotReach) {
  Flight flight = MakeFlight(FlightStyle::kHover, 61);
  flight.Run(kDt);
  // Above the ceiling: the altitude band makes it unreachable.
  flight.brain.destination = {5.0, flight.params.area.max_altitude + 30.0, 5.0};
  flight.brain.mode = AgentMode::kTransit;
  flight.brain.mode_time = 0.0;
  flight.brain.mode_duration = 10.0;
  flight.decisions.clear();
  flight.Run(12.0);
  ASSERT_GE(flight.CountDecisions(AgentDecision::Kind::kReplan), 1);
  EXPECT_LE(flight.brain.destination.y, flight.params.area.max_altitude);
}

TEST(FlightAgentTest, InvariantsHoldAcrossManySeeds) {
  for (FlightStyle style : {FlightStyle::kHover, FlightStyle::kWing}) {
    for (std::uint64_t seed = 100; seed < 116; ++seed) {
      Flight flight = MakeFlight(style, seed, {0.0, 20.0, 0.0});
      const AgentParams& p = flight.params;
      const double edge_allowance =
          style == FlightStyle::kWing ? 0.25 * p.area.radius : 2.0;
      flight.Run(180.0, {}, [&](const Flight& f) {
        const AgentMotion& m = f.motion;
        ASSERT_TRUE(IsFinite(m.position));
        ASSERT_LT(HorizontalDistance(m.position, p.area.center),
                  p.area.radius + edge_allowance)
            << FlightStyleName(style) << " seed " << seed;
        ASSERT_GT(m.position.y, p.area.min_altitude - 2.0);
        ASSERT_LT(m.position.y, p.area.max_altitude + 2.0);
        ASSERT_LE(Length(m.velocity), p.max_speed + 1e-6);
      });
      EXPECT_GE(flight.brain.legs_completed, 2)
          << FlightStyleName(style) << " seed " << seed;
    }
  }
}

TEST(FlightAgentTest, MinTurnRadiusFollowsBankLimit) {
  const AgentParams p = DefaultAgentParams(FlightStyle::kWing);
  const double v = 15.0;
  EXPECT_NEAR(MinTurnRadius(p, v),
              v * v / (kStandardGravity * std::tan(p.max_tilt)), 1e-9);
  EXPECT_GT(MinTurnRadius(p, 2.0 * v), MinTurnRadius(p, v));
}

TEST(FlightAgentTest, SanitizingRepairsNonsense) {
  AgentParams p = DefaultAgentParams(FlightStyle::kWing);
  p.lookahead = std::nan("");
  p.max_tilt = 0.0;
  p.cruise_speed_min = 20.0;
  p.cruise_speed_max = 5.0;
  p.area.min_altitude = 50.0;
  p.area.max_altitude = 10.0;
  p.max_climb_rate = -3.0;
  const AgentParams s = SanitizeAgentParams(p);
  EXPECT_EQ(s.lookahead, DefaultAgentParams(FlightStyle::kWing).lookahead);
  EXPECT_GE(s.max_tilt, Radians(1.0) - 1e-12);
  EXPECT_TRUE(std::isfinite(MinTurnRadius(s, 15.0)));
  EXPECT_EQ(s.cruise_speed_min, 5.0);
  EXPECT_EQ(s.cruise_speed_max, 20.0);
  EXPECT_EQ(s.area.min_altitude, 10.0);
  EXPECT_EQ(s.area.max_altitude, 50.0);
  EXPECT_EQ(s.max_climb_rate, 0.0);
}

TEST(FlightAgentTest, DegenerateParametersNeverBreakTheFlight) {
  using Mutation = void (*)(AgentParams&);
  const std::array<Mutation, 10> mutations = {
      [](AgentParams& p) { p.max_tilt = 0.0; },
      [](AgentParams& p) {
        std::swap(p.area.min_altitude, p.area.max_altitude);
      },
      [](AgentParams& p) { std::swap(p.cruise_speed_min, p.cruise_speed_max); },
      [](AgentParams& p) { p.area.radius = -5.0; },
      [](AgentParams& p) {
        p.clearance = 0.0;
        p.body_radius = 0.0;
      },
      [](AgentParams& p) { p.lookahead = std::nan(""); },
      [](AgentParams& p) { p.max_climb_rate = -1.0; },
      [](AgentParams& p) { p.cruise_speed_min = -4.0; },
      [](AgentParams& p) { p.area.center.x = std::nan(""); },
      [](AgentParams& p) {
        p.max_speed = 0.0;
        p.max_acceleration = 0.0;
      },
  };
  const std::array<AgentObstacle, 1> rock = {
      AgentObstacle{{12.0, 15.0, -12.0}, 3.0, {}}};
  for (FlightStyle style : {FlightStyle::kHover, FlightStyle::kWing}) {
    for (std::size_t i = 0; i < mutations.size(); ++i) {
      Flight flight = MakeFlight(style, 70 + i, {0.0, 15.0, 0.0});
      mutations[i](flight.params);
      flight.Run(60.0, rock, [&](const Flight& f) {
        ASSERT_TRUE(IsFinite(f.motion.position))
            << FlightStyleName(style) << " mutation " << i;
        ASSERT_TRUE(IsFinite(f.motion.velocity));
        ASSERT_TRUE(IsFinite(f.motion.acceleration));
        ASSERT_GE(f.motion.position.y, 0.0);
      });
      // No decision churn (a decision every step would flood the log).
      EXPECT_LT(flight.decisions.size(), 120u)
          << FlightStyleName(style) << " mutation " << i;
    }
  }
}

TEST(FlightAgentTest, NonPositiveStepDoesNothing) {
  Flight flight = MakeFlight(FlightStyle::kHover, 81);
  const AgentMotion before = flight.motion;
  for (double dt : {0.0, -0.01, std::nan("")}) {
    EXPECT_FALSE(StepAgent(flight.params, {}, dt, flight.brain, flight.motion)
                     .has_value());
  }
  EXPECT_EQ(flight.motion.position, before.position);
  EXPECT_FALSE(flight.brain.has_destination);
}

TEST(FlightAgentTest, BacksOutOfAnObstacleItStartsInside) {
  for (FlightStyle style : {FlightStyle::kHover, FlightStyle::kWing}) {
    Flight flight = MakeFlight(style, 91, {0.0, 20.0, 0.0});
    // Inside the clearance band, though not touching.
    const std::array<AgentObstacle, 1> rock = {
        AgentObstacle{{2.5, 20.0, 0.0}, 0.5, {}}};
    const double start = Distance(flight.motion.position, rock[0].center);
    double closest = start;
    flight.Run(5.0, rock, [&](const Flight& f) {
      closest = std::min(closest, Distance(f.motion.position, rock[0].center));
    });
    EXPECT_GT(closest, rock[0].radius + flight.params.body_radius)
        << FlightStyleName(style);
    EXPECT_GT(Distance(flight.motion.position, rock[0].center), start)
        << FlightStyleName(style);
  }
}

TEST(FlightAgentTest, WingLoiterCirclesFitInsideTheArea) {
  for (std::uint64_t seed = 300; seed < 304; ++seed) {
    Flight flight = MakeFlight(FlightStyle::kWing, seed, {0.0, 25.0, 0.0});
    flight.params.area.radius = 60.0;  // Tight for a wing.
    const FlightArea& area = flight.params.area;
    int loiters = 0;
    flight.Run(400.0, {}, [&](const Flight& f) {
      if (f.brain.mode != AgentMode::kLoiter || f.brain.mode_time > 0.0) {
        return;
      }
      ++loiters;
      ASSERT_LE(HorizontalDistance(f.brain.loiter_center, area.center) +
                    f.brain.loiter_radius,
                area.radius + 1e-9)
          << "seed " << seed;
    });
    EXPECT_GE(flight.brain.legs_completed, 3) << "seed " << seed;
  }
}

TEST(FlightAgentTest, ReplanningMidFlightPicksANewLeg) {
  Flight flight = MakeFlight(FlightStyle::kHover, 95);
  flight.Run(12.0);
  ResetBrain(flight.params, flight.motion, flight.brain);
  flight.decisions.clear();
  flight.Run(kDt);
  ASSERT_EQ(flight.decisions.size(), 1u);
  EXPECT_EQ(flight.decisions[0].kind, AgentDecision::Kind::kNewDestination);
  flight.Run(30.0);
  EXPECT_TRUE(IsFinite(flight.motion.position));
  EXPECT_GE(flight.brain.legs_completed, 1);
}

TEST(FlightAgentTest, HoverSidestepsSomethingDirectlyBelow) {
  Flight flight = MakeFlight(FlightStyle::kHover, 401, {0.0, 20.0, 0.0});
  flight.brain.has_destination = true;
  flight.brain.destination = {0.0, 6.0, 0.0};  // Straight down.
  flight.brain.leg_speed = 4.0;
  flight.brain.mode_duration = 1000.0;
  const std::array<AgentObstacle, 1> post = {
      AgentObstacle{{0.3, 13.0, 0.2}, 1.0, {}}};
  double closest_gap = 1e9;
  flight.Run(30.0, post, [&](const Flight& f) {
    closest_gap =
        std::min(closest_gap, Distance(f.motion.position, post[0].center) -
                                  post[0].radius - f.params.body_radius);
  });
  EXPECT_GT(closest_gap, 0.0);
  EXPECT_GE(flight.brain.legs_completed, 1);  // Got there, no freeze.
  EXPECT_EQ(flight.CountDecisions(AgentDecision::Kind::kReplan), 0);
}

TEST(FlightAgentTest, IgnoresAnObstacleThatEngulfsItsWholeArea) {
  Flight flight = MakeFlight(FlightStyle::kHover, 402, {0.0, 12.0, 0.0});
  // A huge floor slab seen as a sphere: it contains the whole area.
  const std::array<AgentObstacle, 1> slab = {
      AgentObstacle{{0.0, 0.5, 0.0}, 141.0, {}}};
  double worst_outside = 0.0;
  flight.Run(120.0, slab, [&](const Flight& f) {
    worst_outside =
        std::max(worst_outside,
                 HorizontalDistance(f.motion.position, f.params.area.center) -
                     f.params.area.radius);
  });
  EXPECT_LT(worst_outside, 2.0);
  EXPECT_GE(flight.brain.legs_completed, 5);
  EXPECT_LE(flight.CountDecisions(AgentDecision::Kind::kReplan), 1);
}

TEST(FlightAgentTest, FitToEnvelopeTamesAWildLaunch) {
  // A wing launched steeply nose-up, far too fast.
  Flight wing = MakeFlight(FlightStyle::kWing, 403, {0.0, 20.0, 0.0});
  wing.motion.velocity = {0.0, 32.0 * std::sin(Radians(40.0)),
                          -32.0 * std::cos(Radians(40.0))};
  FitToEnvelope(wing.params, wing.motion);
  EXPECT_LE(Length(wing.motion.velocity), wing.params.max_speed + 1e-9);
  EXPECT_EQ(wing.motion.velocity.y, 0.0);
  EXPECT_LT(
      AngleBetween(Forward(wing.motion.orientation), wing.motion.velocity),
      1e-9);
  ResetBrain(wing.params, wing.motion, wing.brain);
  double worst_accel = 0.0;
  double lowest_up = 1.0;
  wing.Run(3.0, {}, [&](const Flight& f) {
    worst_accel = std::max(worst_accel, Length(f.motion.acceleration));
    lowest_up = std::min(lowest_up, Up(f.motion.orientation).y);
  });
  EXPECT_LT(worst_accel, 2.0 * kStandardGravity);  // No jolt.
  EXPECT_GT(lowest_up, 0.5);                       // Never flips over.

  // A hover craft thrown upward and sideways.
  Flight hover = MakeFlight(FlightStyle::kHover, 404);
  hover.motion.velocity = {20.0, 30.0, 0.0};
  FitToEnvelope(hover.params, hover.motion);
  EXPECT_LE(std::abs(hover.motion.velocity.y), hover.params.max_climb_rate);
  EXPECT_LE(Length(hover.motion.velocity), hover.params.max_speed + 1e-9);
  EXPECT_NEAR(Up(hover.motion.orientation).y, 1.0, 1e-12);
}

TEST(FlightAgentTest, ReplanKeepsTheRandomStreamAndLegCount) {
  Flight flight = MakeFlight(FlightStyle::kHover, 405);
  flight.Run(40.0);
  const int legs = flight.brain.legs_completed;
  const std::uint64_t stream = flight.brain.rng.state();
  ASSERT_GE(legs, 1);
  Replan(flight.brain);
  EXPECT_EQ(flight.brain.rng.state(), stream);
  EXPECT_EQ(flight.brain.legs_completed, legs);
  flight.decisions.clear();
  flight.Run(kDt);
  ASSERT_EQ(flight.decisions.size(), 1u);
  EXPECT_EQ(flight.decisions[0].kind, AgentDecision::Kind::kNewDestination);
}

TEST(FlightAgentTest, BigButAvoidableObstaclesAreStillAvoided) {
  // A large sphere in the middle of a hover area, leaving a ring free.
  Flight flight = MakeFlight(FlightStyle::kHover, 406, {30.0, 14.5, 0.0});
  flight.params.area.center = {0.0, 0.0, 0.0};
  const std::array<AgentObstacle, 1> dome = {
      AgentObstacle{{0.0, 14.5, 0.0}, 25.0, {}}};
  double closest_gap = 1e9;
  bool avoided = false;
  flight.Run(120.0, dome, [&](const Flight& f) {
    closest_gap =
        std::min(closest_gap, Distance(f.motion.position, dome[0].center) -
                                  dome[0].radius - f.params.body_radius);
    avoided = avoided || f.brain.avoidance.obstacle == 0;
  });
  EXPECT_TRUE(avoided);
  EXPECT_GT(closest_gap, 0.0);
  // Few free spots, but no once-a-second churn.
  EXPECT_LE(flight.CountDecisions(AgentDecision::Kind::kReplan), 30);
}

TEST(FlightAgentTest, NearlyFillingObstaclesBackOffInsteadOfChurning) {
  // Every candidate spot lies within this sphere's clearance zone.
  Flight flight = MakeFlight(FlightStyle::kHover, 407, {33.0, 14.5, 0.0});
  flight.params.area.center = {0.0, 0.0, 0.0};
  const std::array<AgentObstacle, 1> dome = {
      AgentObstacle{{0.0, 14.5, 0.0}, 30.0, {}}};
  flight.Run(120.0, dome);
  EXPECT_LE(flight.CountDecisions(AgentDecision::Kind::kReplan), 12);
  EXPECT_TRUE(IsFinite(flight.motion.position));
}

TEST(FlightAgentTest, TrafficPassingThroughADestinationIsNotAnOccupant) {
  Flight flight = MakeFlight(FlightStyle::kHover, 408, {0.0, 12.0, 0.0});
  flight.Run(kDt);
  flight.brain.mode_duration = 1000.0;
  flight.decisions.clear();
  // Something sits on the destination, but moving fast.
  const std::array<AgentObstacle, 1> passer = {
      AgentObstacle{flight.brain.destination, 1.0, Vec3{8.0, 0.0, 0.0}}};
  flight.Run(3.0, passer);
  EXPECT_EQ(flight.CountDecisions(AgentDecision::Kind::kReplan), 0);
  // The same thing parked there does take the spot.
  const std::array<AgentObstacle, 1> parked = {
      AgentObstacle{flight.brain.destination, 1.0, {}}};
  flight.Run(3.0, parked);
  EXPECT_GE(flight.CountDecisions(AgentDecision::Kind::kReplan), 1);
}

TEST(FlightAgentTest, FitToEnvelopeRepairsNonFiniteVelocity) {
  Flight wing = MakeFlight(FlightStyle::kWing, 409);
  wing.motion.velocity = {std::nan(""), 1.0, 2.0};
  FitToEnvelope(wing.params, wing.motion);
  EXPECT_TRUE(IsFinite(wing.motion.velocity));
  EXPECT_GE(Length(wing.motion.velocity), wing.params.min_speed);
  Flight hover = MakeFlight(FlightStyle::kHover, 410);
  hover.motion.velocity = {std::numeric_limits<double>::infinity(), 0.0, 0.0};
  FitToEnvelope(hover.params, hover.motion);
  EXPECT_TRUE(IsFinite(hover.motion.velocity));
}

}  // namespace
}  // namespace aerolab::sim
