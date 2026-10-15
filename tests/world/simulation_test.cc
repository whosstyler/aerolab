#include "world/simulation.h"

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "math/quat.h"
#include "scene/scene.h"
#include "test_util.h"
#include "world/debug_visualizer.h"
#include "world/presets.h"
#include "world/spawner.h"

namespace aerolab {
namespace {

EntityId SpawnBallistic(Scene& scene) {
  const EntityId id = Spawn(scene, Archetype::kDart, {0, 1, 0},
                            FromEuler({.yaw = 0.0, .pitch = 0.6, .roll = 0}));
  ApplyMotionPreset(scene, id, MotionPreset::kBallistic);
  return id;
}

TEST(SimulationTest, FrameRateDoesNotChangeTheResult) {
  // Same simulated time, chopped into very different frame lengths.
  Scene slow_scene;
  Scene fast_scene;
  const EntityId slow_id = SpawnBallistic(slow_scene);
  const EntityId fast_id = SpawnBallistic(fast_scene);
  SimulationSystem slow(1.0 / 128.0);
  SimulationSystem fast(1.0 / 128.0);
  for (int i = 0; i < 40; ++i) slow.Update(slow_scene, 1.0 / 16.0);
  for (int i = 0; i < 640; ++i) fast.Update(fast_scene, 1.0 / 256.0);
  ASSERT_EQ(slow.step_count(), fast.step_count());
  const BodyComponent& a = *slow_scene.Get(slow_id)->body;
  const BodyComponent& b = *fast_scene.Get(fast_id)->body;
  // Bit-identical, because every fixed step is the same computation.
  EXPECT_EQ(a.state.position, b.state.position);
  EXPECT_EQ(a.state.orientation, b.state.orientation);
}

TEST(SimulationTest, IrregularFrameRatesAgreeStepForStep) {
  // 60 Hz versus 144 Hz frames over the same wall time: step counts may
  // differ by one at the end, but equal step counts give identical states.
  Scene scene_a;
  Scene scene_b;
  const EntityId a = SpawnBallistic(scene_a);
  const EntityId b = SpawnBallistic(scene_b);
  SimulationSystem sim_a;
  SimulationSystem sim_b;
  for (int i = 0; i < 120; ++i) sim_a.Update(scene_a, 1.0 / 60.0);
  for (int i = 0; i < 288; ++i) sim_b.Update(scene_b, 1.0 / 144.0);
  EXPECT_LE(std::abs(sim_a.step_count() - sim_b.step_count()), 1);
  while (sim_a.step_count() < sim_b.step_count()) sim_a.StepOnce(scene_a);
  while (sim_b.step_count() < sim_a.step_count()) sim_b.StepOnce(scene_b);
  EXPECT_EQ(scene_a.Get(a)->body->state.position,
            scene_b.Get(b)->body->state.position);
}

TEST(SimulationTest, PausingFreezesTheRenderedBlend) {
  Scene scene;
  SpawnBallistic(scene);
  SimulationSystem simulation(0.01);
  simulation.Update(scene, 0.025);  // Two steps plus half a step.
  const double running_alpha = simulation.alpha();
  EXPECT_NEAR(running_alpha, 0.5, 1e-9);
  simulation.set_paused(true);
  EXPECT_EQ(simulation.alpha(), running_alpha);  // No jump on pause.
  simulation.StepOnce(scene);
  EXPECT_EQ(simulation.alpha(), 1.0);  // A manual step shows its result.
}

TEST(SimulationTest, TrailSamplesAtItsInterval) {
  Scene scene;
  const EntityId id = SpawnBallistic(scene);
  scene.Get(id)->trail->sample_interval = 0.1;
  SimulationSystem simulation(1.0 / 120.0);
  for (int i = 0; i < 120; ++i) simulation.StepOnce(scene);  // 1 s.
  // One point immediately, then one every 0.1 s.
  EXPECT_NEAR(static_cast<double>(scene.Get(id)->trail->points.size()), 11.0,
              1.0);
}

TEST(SimulationTest, KinematicPrepareRecentersThePath) {
  Scene scene;
  const EntityId id = Spawn(scene, Archetype::kFixedWing, {10, 20, 0});
  ApplyMotionPreset(scene, id, MotionPreset::kCirclePath);
  SetLaunchPose(scene, id, {-40, 15, 30}, Quat::Identity());
  EXPECT_VEC3_NEAR(scene.Get(id)->body->state.position, (Vec3{-40, 15, 30}),
                   1e-9);
  EXPECT_VEC3_NEAR(scene.WorldTransform(id).position, (Vec3{-40, 15, 30}),
                   1e-9);
}

TEST(SimulationTest, SlidingFrictionDoesNotDependOnStepSize) {
  const auto slide_distance = [](double step) {
    Scene scene;
    const EntityId id = Spawn(scene, Archetype::kCube, {0, 0.5, 0});
    ApplyMotionPreset(scene, id, MotionPreset::kBallistic);
    BodyComponent& body = *scene.Get(id)->body;
    body.forces = sim::ForceModel{};  // Gravity only.
    body.ground_friction = 0.5;
    body.initial.velocity = {8, 0, 0};
    body.initial.angular_velocity = Vec3::Zero();
    PrepareBody(scene, id);
    SimulationSystem simulation(step);
    for (double t = 0.0; t < 4.0; t += step) simulation.StepOnce(scene);
    return scene.Get(id)->body->state.position.x;
  };
  // Coulomb sliding: v^2 / (2 mu g) = 64 / 9.81 = 6.52 m.
  const double coarse = slide_distance(1.0 / 60.0);
  const double fine = slide_distance(1.0 / 240.0);
  EXPECT_NEAR(fine, 64.0 / (2.0 * 0.5 * 9.81), 0.1);
  EXPECT_NEAR(coarse, fine, 0.15);
}

TEST(SimulationTest, PendingEventsAreCapped) {
  Scene scene;
  for (int i = 0; i < 40; ++i) {
    const EntityId id = Spawn(scene, Archetype::kSphere,
                              {static_cast<double>(i) * 3.0, 30.0, 0.0});
    ApplyMotionPreset(scene, id, MotionPreset::kBallistic);
    scene.Get(id)->body->restitution = 0.9;  // Bounce a lot.
  }
  SimulationSystem simulation;
  for (int i = 0; i < 3000; ++i) simulation.Update(scene, 1.0 / 60.0);
  EXPECT_LE(simulation.TakeEvents().size(),
            SimulationSystem::kMaxPendingEvents);
}

TEST(SimulationTest, ChildBodiesFollowParentsWithinTheSameStep) {
  Scene scene;
  const EntityId parent = Spawn(scene, Archetype::kSphere, {0, 20, 0});
  ApplyMotionPreset(scene, parent, MotionPreset::kOscillator);
  const EntityId child = Spawn(scene, Archetype::kSphere, {5, 20, 0});
  ApplyMotionPreset(scene, child, MotionPreset::kOscillator);
  ASSERT_TRUE(scene.SetParent(child, parent));
  SimulationSystem simulation(0.01);
  for (int i = 0; i < 37; ++i) simulation.Update(scene, 0.01);
  // The hierarchy and the simulation agree on where both bodies are.
  EXPECT_VEC3_NEAR(scene.WorldTransform(child).position,
                   scene.Get(child)->body->state.position, 1e-9);
  EXPECT_VEC3_NEAR(scene.WorldTransform(parent).position,
                   scene.Get(parent)->body->state.position, 1e-9);
}

TEST(SimulationTest, PauseStepAndTimeScale) {
  Scene scene;
  SpawnBallistic(scene);
  SimulationSystem simulation(0.01);
  simulation.set_paused(true);
  EXPECT_EQ(simulation.Update(scene, 1.0), 0);
  EXPECT_EQ(simulation.time(), 0.0);
  simulation.StepOnce(scene);
  EXPECT_EQ(simulation.step_count(), 1);
  EXPECT_NEAR(simulation.time(), 0.01, 1e-15);
  EXPECT_EQ(simulation.alpha(), 1.0);  // Paused: show the latest state.

  simulation.set_paused(false);
  simulation.set_time_scale(0.5);
  simulation.Update(scene, 0.1);  // 0.05 s of simulated time.
  EXPECT_NEAR(simulation.time(), 0.06, 1e-12);
  simulation.set_time_scale(100.0);
  EXPECT_EQ(simulation.time_scale(), 16.0);
}

TEST(SimulationTest, ResetRestoresLaunchConditions) {
  Scene scene;
  const EntityId id = SpawnBallistic(scene);
  const sim::RigidBodyState launch = scene.Get(id)->body->state;
  SimulationSystem simulation(0.01);
  for (int i = 0; i < 100; ++i) simulation.Update(scene, 0.01);
  EXPECT_GT(Distance(scene.Get(id)->body->state.position, launch.position),
            1.0);
  EXPECT_FALSE(scene.Get(id)->trail->points.empty());
  EXPECT_FALSE(scene.Get(id)->body->telemetry.empty());

  simulation.Reset(scene);
  const Entity* entity = scene.Get(id);
  EXPECT_EQ(entity->body->state.position, launch.position);
  EXPECT_EQ(entity->body->state.velocity, launch.velocity);
  EXPECT_EQ(entity->body->age, 0.0);
  EXPECT_TRUE(entity->trail->points.empty());
  EXPECT_TRUE(entity->body->telemetry.empty());
  EXPECT_EQ(simulation.time(), 0.0);
  EXPECT_VEC3_NEAR(entity->local.position, launch.position, 1e-12);
}

TEST(SimulationTest, BodiesBounceAndSettleOnTheGround) {
  Scene scene;
  const EntityId id = Spawn(scene, Archetype::kSphere, {0, 10, 0});
  ApplyMotionPreset(scene, id, MotionPreset::kBallistic);
  BodyComponent& body = *scene.Get(id)->body;
  body.initial.velocity = Vec3::Zero();
  PrepareBody(scene, id);
  SimulationSystem simulation(1.0 / 120.0);
  bool impact_reported = false;
  for (int i = 0; i < 1200; ++i) {
    simulation.Update(scene, 1.0 / 120.0);
    for (const SimulationEvent& event : simulation.TakeEvents()) {
      if (event.kind == SimulationEvent::Kind::kImpact) impact_reported = true;
    }
  }
  const BodyComponent& settled = *scene.Get(id)->body;
  EXPECT_TRUE(impact_reported);
  EXPECT_NEAR(settled.state.position.y, settled.collision_radius, 1e-6);
  EXPECT_LT(std::abs(settled.state.velocity.y), 0.25);
}

TEST(SimulationTest, PoweredFlightReportsBurnout) {
  Scene scene;
  const EntityId id = Spawn(scene, Archetype::kDart, {0, 1, 0},
                            FromEuler({.yaw = 0, .pitch = 1.2, .roll = 0}));
  ApplyMotionPreset(scene, id, MotionPreset::kPowered);
  SimulationSystem simulation(0.01);
  int burnouts = 0;
  for (int i = 0; i < 400; ++i) {
    simulation.Update(scene, 0.01);
    for (const SimulationEvent& event : simulation.TakeEvents()) {
      if (event.kind == SimulationEvent::Kind::kBurnout) ++burnouts;
    }
  }
  EXPECT_EQ(burnouts, 1);
  EXPECT_GT(scene.Get(id)->body->state.position.y, 20.0);
}

TEST(SimulationTest, DivergingBodyIsStoppedNotPoisoned) {
  Scene scene;
  const EntityId id = Spawn(scene, Archetype::kCube, {0, 20, 0});
  ApplyMotionPreset(scene, id, MotionPreset::kTumble);
  BodyComponent& body = *scene.Get(id)->body;
  body.mass = sim::BoxMassProperties(1.0, {1.0, 2.0, 3.0});
  body.integrator = sim::Integrator::kSemiImplicitEuler;
  body.initial.angular_velocity = {3.0, -7.0, 11.0};
  PrepareBody(scene, id);
  SimulationSystem simulation(0.01);
  bool diverged = false;
  for (int i = 0; i < 5000 && !diverged; ++i) {
    simulation.Update(scene, 0.01);
    for (const SimulationEvent& event : simulation.TakeEvents()) {
      diverged =
          diverged || event.message.find("divergence") != std::string::npos;
    }
  }
  EXPECT_TRUE(diverged);
  const sim::RigidBodyState& state = scene.Get(id)->body->state;
  EXPECT_TRUE(IsFinite(state.position));
  EXPECT_TRUE(IsFinite(state.angular_velocity));
  EXPECT_TRUE(std::isfinite(Length(state.orientation)));
}

TEST(SimulationTest, KinematicBodiesFollowTheirPath) {
  Scene scene;
  const EntityId id = Spawn(scene, Archetype::kFixedWing, {10, 20, 0});
  ApplyMotionPreset(scene, id, MotionPreset::kCirclePath);
  const BodyComponent& body = *scene.Get(id)->body;
  EXPECT_VEC3_NEAR(body.state.position, (Vec3{10, 20, 0}), 1e-9);
  SimulationSystem simulation(0.01);
  for (int i = 0; i < 350; ++i) simulation.Update(scene, 0.01);
  const sim::PathSample expected =
      sim::SamplePath(scene.Get(id)->body->path, 3.5);
  EXPECT_VEC3_NEAR(scene.Get(id)->body->state.position, expected.position,
                   1e-9);
}

TEST(SimulationTest, RenderMatrixInterpolatesBetweenSteps) {
  Scene scene;
  const EntityId id = SpawnBallistic(scene);
  SimulationSystem simulation(0.1);
  simulation.Update(scene, 0.1);
  const BodyComponent& body = *scene.Get(id)->body;
  const Vec3 mid = RenderWorldMatrix(scene, id, 0.5).TranslationPart();
  EXPECT_VEC3_NEAR(mid, Lerp(body.previous.position, body.state.position, 0.5),
                   1e-12);
  // Children of a body follow its interpolated pose.
  const EntityId child = scene.Create("Marker", id);
  scene.Get(child)->local.position = {0, 1, 0};
  EXPECT_VEC3_NEAR(RenderWorldMatrix(scene, child, 0.5).TranslationPart(),
                   mid + Rotate(Slerp(body.previous.orientation,
                                      body.state.orientation, 0.5),
                                Vec3::UnitY()),
                   1e-9);
}

TEST(SimulationTest, BodyUnderAMovedParentStaysInWorldSpace) {
  Scene scene;
  const EntityId group = scene.Create("Group");
  scene.Get(group)->local.position = {100, 0, 0};
  const EntityId id = Spawn(scene, Archetype::kSphere, {100, 5, 0});
  ASSERT_TRUE(scene.SetParent(id, group));
  ApplyMotionPreset(scene, id, MotionPreset::kOscillator);
  SimulationSystem simulation(0.01);
  for (int i = 0; i < 50; ++i) simulation.Update(scene, 0.01);
  EXPECT_VEC3_NEAR(scene.WorldTransform(id).position,
                   scene.Get(id)->body->state.position, 1e-9);
}

TEST(PresetsTest, EveryPresetProducesAPlausibleBody) {
  for (MotionPreset preset : AllMotionPresets()) {
    Scene scene;
    const EntityId id = Spawn(scene, Archetype::kSphere, {0, 30, 0});
    ApplyMotionPreset(scene, id, preset);
    const Entity* entity = scene.Get(id);
    ASSERT_TRUE(entity->body.has_value()) << MotionPresetName(preset);
    EXPECT_TRUE(entity->trail.has_value());
    EXPECT_FALSE(MotionPresetDescription(preset).empty());
    SimulationSystem simulation;
    for (int i = 0; i < 240; ++i) simulation.Update(scene, 1.0 / 120.0);
    const sim::RigidBodyState& state = scene.Get(id)->body->state;
    EXPECT_TRUE(IsFinite(state.position)) << MotionPresetName(preset);
    EXPECT_NEAR(Length(state.orientation), 1.0, 1e-9);
  }
}

TEST(PresetsTest, OrbitAndOscillatorConserveEnergy) {
  for (MotionPreset preset :
       {MotionPreset::kOrbit, MotionPreset::kOscillator}) {
    Scene scene;
    const EntityId id = Spawn(scene, Archetype::kSphere, {0, 20, 0});
    ApplyMotionPreset(scene, id, preset);
    const double e0 = MechanicalEnergy(*scene.Get(id)->body);
    SimulationSystem simulation;
    for (int i = 0; i < 1200; ++i) simulation.Update(scene, 1.0 / 120.0);
    EXPECT_NEAR(MechanicalEnergy(*scene.Get(id)->body), e0, 1e-6 * std::abs(e0))
        << MotionPresetName(preset);
  }
}

TEST(PresetsTest, IntegratorComparisonDivergesFromIdenticalStart) {
  Scene scene;
  const EntityId group = SpawnIntegratorComparison(scene, {0, 20, 0});
  const std::vector<EntityId>& bodies = scene.Get(group)->children;
  ASSERT_EQ(bodies.size(), static_cast<std::size_t>(sim::kIntegratorCount));
  for (EntityId id : bodies) {
    EXPECT_EQ(scene.Get(id)->body->state.position,
              scene.Get(bodies[0])->body->state.position);
  }
  SimulationSystem simulation(1.0 / 30.0);  // Coarse steps exaggerate error.
  for (int i = 0; i < 600; ++i) simulation.Update(scene, 1.0 / 30.0);
  const auto radius = [&](sim::Integrator integrator) {
    for (EntityId id : bodies) {
      if (scene.Get(id)->body->integrator == integrator) {
        return Distance(scene.Get(id)->body->state.position, {0, 20, 0});
      }
    }
    return 0.0;
  };
  EXPECT_NEAR(radius(sim::Integrator::kRungeKutta4), 10.0, 0.01);
  EXPECT_GT(radius(sim::Integrator::kExplicitEuler), 11.0);
}

TEST(DemoSceneTest, BuildsAndRunsWithoutTrouble) {
  Scene scene;
  BuildDemoScene(scene);
  EXPECT_GE(scene.roots().size(), 8u);
  SimulationSystem simulation;
  for (int i = 0; i < 600; ++i) simulation.Update(scene, 1.0 / 60.0);
  for (EntityId id : scene.AllEntities()) {
    const Entity* entity = scene.Get(id);
    if (entity->body.has_value()) {
      EXPECT_TRUE(IsFinite(entity->body->state.position)) << entity->name;
    }
  }
}

TEST(DebugVisualizerTest, PredictionMatchesTheSimulation) {
  Scene scene;
  const EntityId id = SpawnBallistic(scene);
  for (sim::Integrator integrator :
       {sim::Integrator::kExplicitEuler, sim::Integrator::kRungeKutta4}) {
    Scene copy;
    const EntityId body_id = SpawnBallistic(copy);
    copy.Get(body_id)->body->integrator = integrator;
    const std::vector<Vec3> prediction =
        PredictTrajectory(*copy.Get(body_id)->body, 1.0, 1.0 / 60.0);
    ASSERT_EQ(prediction.size(), 61u);
    SimulationSystem simulation(1.0 / 60.0);
    for (int i = 0; i < 60; ++i) simulation.StepOnce(copy);
    // Same integrator and step as the simulation, so the forecast is exact.
    EXPECT_VEC3_NEAR(prediction.back(), copy.Get(body_id)->body->state.position,
                     1e-9)
        << sim::IntegratorName(integrator);
  }
  EXPECT_TRUE(scene.IsAlive(id));
}

TEST(DebugVisualizerTest, LayersCanBeToggled) {
  Scene scene;
  const EntityId id = SpawnBallistic(scene);
  SimulationSystem simulation;
  for (int i = 0; i < 30; ++i) simulation.Update(scene, 1.0 / 60.0);

  DebugViewOptions none;
  none.world_axes = none.trails = none.velocity_vectors = false;
  none.drop_lines = none.attractors = none.predicted_path = false;
  DebugDrawList empty;
  AddDebugVisualization(scene, none, 1.0, id, empty);
  EXPECT_TRUE(empty.empty());

  DebugViewOptions all;
  all.acceleration_vectors = all.force_vectors = all.local_frames = true;
  all.bounding_boxes = true;
  DebugDrawList full;
  AddDebugVisualization(scene, all, 1.0, id, full);
  EXPECT_GT(full.segment_count(), 40u);
}

}  // namespace
}  // namespace aerolab
