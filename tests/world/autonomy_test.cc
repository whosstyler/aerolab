#include "world/autonomy.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "math/quat.h"
#include "math/scalar.h"
#include "math/vec3.h"
#include "scene/entity.h"
#include "scene/scene.h"
#include "sim/agent_vehicle.h"
#include "sim/atmosphere.h"
#include "sim/fixed_wing.h"
#include "sim/flight_agent.h"
#include "sim/force_model.h"
#include "world/debug_visualizer.h"
#include "world/presets.h"
#include "world/simulation.h"
#include "world/spawner.h"

namespace aerolab {
namespace {

constexpr double kStep = 1.0 / 120.0;

EntityId SpawnFlyer(Scene& scene, Archetype archetype, MotionPreset preset,
                    const Vec3& position) {
  const EntityId id = Spawn(scene, archetype, position);
  ApplyMotionPreset(scene, id, preset);
  return id;
}

void RunSteps(SimulationSystem& simulation, Scene& scene, int steps) {
  for (int i = 0; i < steps; ++i) simulation.StepOnce(scene);
}

TEST(AutonomyTest, AiPresetsConfigureAutonomousBodies) {
  Scene scene;
  const EntityId quad = SpawnFlyer(scene, Archetype::kQuadrotor,
                                   MotionPreset::kAutoHover, {5, 8, -3});
  const EntityId plane = SpawnFlyer(scene, Archetype::kFixedWing,
                                    MotionPreset::kAutoWing, {-20, 25, 10});
  const BodyComponent& q = *scene.Get(quad)->body;
  const BodyComponent& p = *scene.Get(plane)->body;
  EXPECT_EQ(q.motion, MotionType::kAutonomous);
  EXPECT_EQ(q.agent.style, sim::FlightStyle::kHover);
  EXPECT_EQ(p.agent.style, sim::FlightStyle::kWing);
  // Flight areas are centred under the objects; each gets its own seed.
  EXPECT_EQ(q.agent.area.center, (Vec3{5, 0, -3}));
  EXPECT_EQ(p.agent.area.center, (Vec3{-20, 0, 10}));
  EXPECT_NE(q.agent.seed, p.agent.seed);
  // Wings start at cruise speed along the nose.
  EXPECT_NEAR(Length(p.state.velocity), p.agent.cruise_speed_min, 1e-9);
  EXPECT_FALSE(q.ground_contact);
}

TEST(AutonomyTest, ScenePresetsFlyPhysicalAirframes) {
  Scene scene;
  const EntityId quad = SpawnFlyer(scene, Archetype::kQuadrotor,
                                   MotionPreset::kAutoHover, {5, 15, -3});
  const EntityId plane = SpawnFlyer(scene, Archetype::kFixedWing,
                                    MotionPreset::kAutoWing, {-20, 25, 10});
  SimulationSystem simulation(kStep);
  simulation.Reset(scene);
  const BodyComponent& q = *scene.Get(quad)->body;
  const BodyComponent& p = *scene.Get(plane)->body;
  EXPECT_EQ(q.agent.model, sim::FlightModel::kPhysical);
  EXPECT_EQ(p.agent.model, sim::FlightModel::kPhysical);
  // Telemetry uses the airframe's mass; the body keeps its own for when
  // it stops flying the physical model.
  EXPECT_EQ(SimulatedMass(q).mass, q.vehicle.quad.mass.mass);
  EXPECT_EQ(SimulatedMass(p).mass, p.vehicle.wing.mass.mass);
  EXPECT_NE(p.mass.mass, p.vehicle.wing.mass.mass);

  RunSteps(simulation, scene, 20 * 120);
  EXPECT_EQ(q.halt, BodyHalt::kNone);
  EXPECT_EQ(p.halt, BodyHalt::kNone);
  // Both are really flying: rotors spinning, the plane above its stall.
  for (double w : q.vehicle.motors.rotor_speed) EXPECT_GT(w, 300.0);
  EXPECT_GT(Length(p.state.velocity - p.forces.wind),
            sim::StallSpeed(p.vehicle.wing, sim::kSeaLevelDensity));
  EXPECT_GT(q.state.position.y, q.agent.area.min_altitude - 2.0);
  EXPECT_GT(p.state.position.y, p.agent.area.min_altitude - 6.0);
}

TEST(AutonomyTest, PhysicalForecastMatchesWhatTheAirframeThenDoes) {
  for (const auto [archetype, preset] :
       {std::pair{Archetype::kQuadrotor, MotionPreset::kAutoHover},
        std::pair{Archetype::kFixedWing, MotionPreset::kAutoWing}}) {
    Scene scene;
    const EntityId flyer = SpawnFlyer(scene, archetype, preset, {0, 20, 0});
    Spawn(scene, Archetype::kCube, {3, 0.5, -4});  // A static landmark.
    SimulationSystem simulation(kStep);
    simulation.Reset(scene);
    ASSERT_EQ(scene.Get(flyer)->body->agent.model, sim::FlightModel::kPhysical);
    RunSteps(simulation, scene, 5 * 120);

    std::vector<SceneObstacle> all;
    std::vector<sim::AgentObstacle> shapes;
    std::vector<EntityId> ids;
    CollectObstacles(scene, all);
    ObstaclesFor(all, flyer, shapes, ids);
    // At the simulation's own step the forecast flies the same airframe,
    // autopilot and brain, so it is exact.
    const std::vector<Vec3> forecast =
        PredictTrajectory(*scene.Get(flyer)->body, 4.0, kStep, shapes);
    ASSERT_EQ(forecast.size(), 4u * 120u + 1u);
    RunSteps(simulation, scene, 4 * 120);
    EXPECT_EQ(forecast.back(), scene.Get(flyer)->body->state.position);
  }
}

TEST(AutonomyTest, PhysicalWingLaunchesAtItsAirspeedInATailwind) {
  Scene scene;
  const EntityId plane = SpawnFlyer(scene, Archetype::kFixedWing,
                                    MotionPreset::kAutoWing, {0, 20, 0});
  BodyComponent& body = *scene.Get(plane)->body;
  const Vec3 nose = Rotate(body.initial.orientation, -Vec3::UnitZ());
  body.forces.wind = Vec3{nose.x, 0.0, nose.z} * 6.0;  // Straight behind.
  SimulationSystem simulation(kStep);
  simulation.Reset(scene);
  // Without the wind it would launch at 12 m/s over the ground: only 6 m/s
  // through the air, below its 7.6 m/s stall speed.
  EXPECT_NEAR(Length(body.state.velocity - body.forces.wind),
              body.agent.cruise_speed_min, 1e-9);
  RunSteps(simulation, scene, 20 * 120);
  EXPECT_EQ(body.halt, BodyHalt::kNone);
  EXPECT_GT(body.state.position.y, body.agent.area.min_altitude - 6.0);
}

TEST(AutonomyTest, SimpleFlightModelStillFliesAndForecasts) {
  Scene scene;
  const EntityId quad = SpawnFlyer(scene, Archetype::kQuadrotor,
                                   MotionPreset::kAutoHover, {0, 8, 0});
  scene.Get(quad)->body->agent.model = sim::FlightModel::kSimple;
  SimulationSystem simulation(kStep);
  simulation.Reset(scene);
  RunSteps(simulation, scene, 3 * 120);
  const std::vector<Vec3> forecast =
      PredictTrajectory(*scene.Get(quad)->body, 2.0, kStep, {});
  RunSteps(simulation, scene, 2 * 120);
  EXPECT_EQ(forecast.back(), scene.Get(quad)->body->state.position);
  EXPECT_EQ(scene.Get(quad)->body->halt, BodyHalt::kNone);
}

TEST(AutonomyTest, NaturalStyleFollowsTheShape) {
  Scene scene;
  const EntityId wing = Spawn(scene, Archetype::kFixedWing, {});
  const EntityId dart = Spawn(scene, Archetype::kDart, {});
  const EntityId quad = Spawn(scene, Archetype::kQuadrotor, {});
  const EntityId ball = Spawn(scene, Archetype::kSphere, {});
  EXPECT_EQ(NaturalFlightStyle(*scene.Get(wing)), sim::FlightStyle::kWing);
  EXPECT_EQ(NaturalFlightStyle(*scene.Get(dart)), sim::FlightStyle::kWing);
  EXPECT_EQ(NaturalFlightStyle(*scene.Get(quad)), sim::FlightStyle::kHover);
  EXPECT_EQ(NaturalFlightStyle(*scene.Get(ball)), sim::FlightStyle::kHover);
}

TEST(AutonomyTest, ObstaclesIncludeBodiesAndPropsButNotAttachedParts) {
  Scene scene;
  const EntityId quad = SpawnFlyer(scene, Archetype::kQuadrotor,
                                   MotionPreset::kAutoHover, {0, 8, 0});
  ASSERT_FALSE(scene.Get(quad)->children.empty());  // Rotors and so on.
  const EntityId cube = Spawn(scene, Archetype::kCube, {10, 0.5, 0});

  std::vector<SceneObstacle> all;
  CollectObstacles(scene, all);
  ASSERT_EQ(all.size(), 2u);  // The quadrotor (once) and the cube.

  std::vector<sim::AgentObstacle> shapes;
  std::vector<EntityId> ids;
  ObstaclesFor(all, quad, shapes, ids);
  ASSERT_EQ(ids.size(), 1u);
  EXPECT_EQ(ids[0], cube);
  EXPECT_NEAR(shapes[0].center.x, 10.0, 1e-9);
  EXPECT_GT(shapes[0].radius, 0.4);
}

TEST(AutonomyTest, AgentsFlyAndReportTheirDecisions) {
  Scene scene;
  const EntityId quad = SpawnFlyer(scene, Archetype::kQuadrotor,
                                   MotionPreset::kAutoHover, {0, 8, 0});
  const Vec3 start = scene.Get(quad)->body->state.position;
  SimulationSystem simulation(kStep);
  int decisions = 0;
  for (int i = 0; i < 20 * 120; ++i) {
    simulation.StepOnce(scene);
    for (const SimulationEvent& event : simulation.TakeEvents()) {
      if (event.kind == SimulationEvent::Kind::kDecision) {
        ++decisions;
        EXPECT_EQ(event.entity, quad);
        EXPECT_NE(event.message.find("Quadrotor"), std::string::npos);
      }
    }
  }
  const BodyComponent& body = *scene.Get(quad)->body;
  EXPECT_GE(decisions, 2);
  EXPECT_GT(Distance(body.state.position, start), 3.0);
  // The scene graph follows the simulation.
  EXPECT_EQ(scene.WorldTransform(quad).position, body.state.position);
  ASSERT_FALSE(body.telemetry.empty());
  EXPECT_TRUE(std::isfinite(body.telemetry[body.telemetry.size() - 1].energy));
}

TEST(AutonomyTest, ResetReplaysTheSameFlight) {
  Scene scene;
  const EntityId plane = SpawnFlyer(scene, Archetype::kFixedWing,
                                    MotionPreset::kAutoWing, {0, 25, 0});
  SimulationSystem simulation(kStep);
  RunSteps(simulation, scene, 30 * 120);
  const Vec3 first = scene.Get(plane)->body->state.position;
  simulation.Reset(scene);
  RunSteps(simulation, scene, 30 * 120);
  EXPECT_EQ(scene.Get(plane)->body->state.position, first);
}

TEST(AutonomyTest, ForecastMatchesWhatTheAgentThenDoes) {
  Scene scene;
  const EntityId quad = SpawnFlyer(scene, Archetype::kQuadrotor,
                                   MotionPreset::kAutoHover, {0, 8, 0});
  Spawn(scene, Archetype::kCube, {3, 0.5, -4});  // A static landmark.
  SimulationSystem simulation(kStep);
  RunSteps(simulation, scene, 5 * 120);

  std::vector<SceneObstacle> all;
  std::vector<sim::AgentObstacle> shapes;
  std::vector<EntityId> ids;
  CollectObstacles(scene, all);
  ObstaclesFor(all, quad, shapes, ids);
  const std::vector<Vec3> forecast =
      PredictTrajectory(*scene.Get(quad)->body, 4.0, kStep, shapes);
  ASSERT_EQ(forecast.size(), 4u * 120u + 1u);

  RunSteps(simulation, scene, 4 * 120);
  EXPECT_EQ(forecast.back(), scene.Get(quad)->body->state.position);
}

TEST(AutonomyTest, AgentSteersAroundAStaticObject) {
  Scene scene;
  const EntityId quad = SpawnFlyer(scene, Archetype::kQuadrotor,
                                   MotionPreset::kAutoHover, {0, 10, 0});
  const EntityId block = Spawn(scene, Archetype::kCube, {0, 10, -25});
  scene.Get(block)->local.scale = {6, 6, 6};
  const sim::AgentObstacle shape = *ObstacleShape(scene, block);

  BodyComponent& body = *scene.Get(quad)->body;
  body.agent.area.radius = 100.0;
  body.brain.has_destination = true;
  body.brain.destination = {0, 10, -50};
  body.brain.leg_speed = body.agent.cruise_speed_max;
  body.brain.mode_duration = 1000.0;

  SimulationSystem simulation(kStep);
  double closest = 1e9;
  bool flagged = false;
  for (int i = 0; i < 20 * 120; ++i) {
    simulation.StepOnce(scene);
    const BodyComponent& b = *scene.Get(quad)->body;
    closest = std::min(closest, Distance(b.state.position, shape.center));
    flagged = flagged || b.avoiding == block;
  }
  EXPECT_TRUE(flagged);
  EXPECT_GT(closest, shape.radius + scene.Get(quad)->body->agent.body_radius);
  EXPECT_GE(scene.Get(quad)->body->brain.legs_completed, 1);
}

TEST(AutonomyTest, CorruptStateHaltsTheAgentCleanly) {
  Scene scene;
  const EntityId quad = SpawnFlyer(scene, Archetype::kQuadrotor,
                                   MotionPreset::kAutoHover, {0, 8, 0});
  SimulationSystem simulation(kStep);
  RunSteps(simulation, scene, 120);
  simulation.TakeEvents();
  scene.Get(quad)->body->state.velocity.x = std::nan("");
  simulation.StepOnce(scene);
  const BodyComponent& body = *scene.Get(quad)->body;
  EXPECT_EQ(body.halt, BodyHalt::kDiverged);
  EXPECT_TRUE(IsFinite(body.state.position));
  EXPECT_TRUE(IsFinite(body.state.velocity));
  EXPECT_EQ(body.acceleration, Vec3::Zero());
  bool reported = false;
  for (const SimulationEvent& event : simulation.TakeEvents()) {
    reported |= event.kind == SimulationEvent::Kind::kDivergence;
  }
  EXPECT_TRUE(reported);
  // It stays put until reset, then flies again.
  const Vec3 frozen = body.state.position;
  RunSteps(simulation, scene, 60);
  EXPECT_EQ(scene.Get(quad)->body->state.position, frozen);
  simulation.Reset(scene);
  RunSteps(simulation, scene, 600);
  EXPECT_EQ(scene.Get(quad)->body->halt, BodyHalt::kNone);
}

TEST(AutonomyTest, SwitchingAFastDartToAutonomousLaunchesSmoothly) {
  Scene scene;
  const EntityId dart =
      Spawn(scene, Archetype::kDart, {0, 2, 0},
            FromEuler({.yaw = 0.0, .pitch = Radians(40.0), .roll = 0.0}));
  ApplyMotionPreset(scene, dart, MotionPreset::kBallistic);  // 32 m/s.
  BodyComponent& body = *scene.Get(dart)->body;
  body.motion = MotionType::kAutonomous;
  body.agent = AgentParamsFor(scene, dart, sim::FlightStyle::kWing);
  RelaunchFromInitialPose(scene, dart);
  SimulationSystem simulation(kStep);
  double worst_accel = 0.0;
  double lowest_up = 1.0;
  for (int i = 0; i < 3 * 120; ++i) {
    simulation.StepOnce(scene);
    const BodyComponent& b = *scene.Get(dart)->body;
    worst_accel = std::max(worst_accel, Length(b.acceleration));
    lowest_up =
        std::min(lowest_up, Rotate(b.state.orientation, Vec3::UnitY()).y);
  }
  EXPECT_LT(worst_accel, 2.0 * sim::kStandardGravity);
  EXPECT_GT(lowest_up, 0.5);
  EXPECT_LE(Length(scene.Get(dart)->body->state.velocity),
            body.agent.max_speed + 1e-9);
}

TEST(AutonomyTest, DemoSceneFlyersAreAutonomous) {
  Scene scene;
  BuildDemoScene(scene);
  int hover = 0;
  int wing = 0;
  scene.ForEachEntity([&](EntityId, const Entity& entity) {
    if (!entity.body.has_value() ||
        entity.body->motion != MotionType::kAutonomous) {
      return;
    }
    if (entity.name == "Quadrotor") {
      EXPECT_EQ(entity.body->agent.style, sim::FlightStyle::kHover);
      ++hover;
    }
    if (entity.name == "Patrol plane") {
      EXPECT_EQ(entity.body->agent.style, sim::FlightStyle::kWing);
      ++wing;
    }
  });
  EXPECT_EQ(hover, 1);
  EXPECT_EQ(wing, 1);
}

TEST(AutonomyTest, DecisionsReadNaturally) {
  sim::AgentDecision decision;
  decision.kind = sim::AgentDecision::Kind::kLoiter;
  decision.duration = 3.25;
  EXPECT_EQ(DescribeDecision("Quad", sim::FlightStyle::kHover, decision),
            "Quad: hovering for 3.2 s");
  EXPECT_EQ(DescribeDecision("Plane", sim::FlightStyle::kWing, decision),
            "Plane: circling for 3.2 s");
  decision.kind = sim::AgentDecision::Kind::kNewDestination;
  decision.distance = 41.6;
  decision.speed = 6.04;
  EXPECT_EQ(DescribeDecision("Quad", sim::FlightStyle::kHover, decision),
            "Quad: heading to a point 42 m away at 6.0 m/s");
}

}  // namespace
}  // namespace aerolab
