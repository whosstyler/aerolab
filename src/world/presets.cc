#include "world/presets.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <string_view>

#include "geometry/color.h"
#include "geometry/mesh_library.h"
#include "math/aabb.h"
#include "math/quat.h"
#include "math/scalar.h"
#include "math/transform.h"
#include "sim/kinematic_path.h"
#include "sim/rigid_body.h"
#include "world/simulation.h"
#include "world/spawner.h"

namespace aerolab {
namespace {

// Approximate distance from the body origin to the ground when resting:
// the smallest half-extent of the (scaled) mesh bounds.
double CollisionRadius(const Entity& entity) {
  if (!entity.render.has_value()) return 0.25;
  const Aabb& bounds = MeshLocalBounds(entity.render->mesh);
  const Vec3 half = Hadamard(HalfExtents(bounds), Abs(entity.local.scale));
  return std::max(std::min({half.x, half.y, half.z}), 0.05);
}

Color PresetTrailColor(MotionPreset preset) {
  switch (preset) {
    case MotionPreset::kBallistic:
      return FromHex(0x7DD3FC);
    case MotionPreset::kPowered:
      return FromHex(0xFDBA74);
    case MotionPreset::kGlider:
      return FromHex(0xA7F3D0);
    case MotionPreset::kOrbit:
      return FromHex(0xC4B5FD);
    case MotionPreset::kOscillator:
      return FromHex(0xF9A8D4);
    case MotionPreset::kTumble:
      return FromHex(0xFDE68A);
    case MotionPreset::kCirclePath:
    case MotionPreset::kFigureEight:
    case MotionPreset::kHelix:
    case MotionPreset::kCount:
      break;
  }
  return FromHex(0xE2E8F0);
}

}  // namespace

std::string_view MotionPresetName(MotionPreset preset) {
  switch (preset) {
    case MotionPreset::kBallistic:
      return "Ballistic";
    case MotionPreset::kPowered:
      return "Powered ascent";
    case MotionPreset::kGlider:
      return "Glider";
    case MotionPreset::kOrbit:
      return "Orbit";
    case MotionPreset::kOscillator:
      return "Spring oscillator";
    case MotionPreset::kTumble:
      return "Free tumble";
    case MotionPreset::kCirclePath:
      return "Circle path";
    case MotionPreset::kFigureEight:
      return "Figure-eight path";
    case MotionPreset::kHelix:
      return "Helix path";
    case MotionPreset::kCount:
      break;
  }
  return "Unknown";
}

std::string_view MotionPresetDescription(MotionPreset preset) {
  switch (preset) {
    case MotionPreset::kBallistic:
      return "Launched at speed; gravity and quadratic drag bend the arc "
             "while weathervane torque keeps the nose along the flow.";
    case MotionPreset::kPowered:
      return "Constant thrust along the body axis for a few seconds, then "
             "an unpowered ballistic coast.";
    case MotionPreset::kGlider:
      return "Lift proportional to forward airspeed squared balances "
             "weight at trim speed; diving trades height for speed and "
             "the extra lift pulls it back up.";
    case MotionPreset::kOrbit:
      return "Inverse-square attraction with no gravity: a circular orbit "
             "that exposes integrator energy drift.";
    case MotionPreset::kOscillator:
      return "Linear spring toward an anchor: an elliptical 3D loop with "
             "constant total energy.";
    case MotionPreset::kTumble:
      return "Torque-free rotation about the intermediate principal axis "
             "is unstable: the body periodically flips (Dzhanibekov).";
    case MotionPreset::kCirclePath:
      return "Kinematic: a closed-form banked circle, no forces involved.";
    case MotionPreset::kFigureEight:
      return "Kinematic: a lemniscate path with analytic velocity and "
             "acceleration.";
    case MotionPreset::kHelix:
      return "Kinematic: a circle with constant climb rate.";
    case MotionPreset::kCount:
      break;
  }
  return "";
}

Color IntegratorColor(sim::Integrator integrator) {
  switch (integrator) {
    case sim::Integrator::kExplicitEuler:
      return FromHex(0xF87171);
    case sim::Integrator::kSemiImplicitEuler:
      return FromHex(0xFBBF24);
    case sim::Integrator::kVelocityVerlet:
      return FromHex(0x4ADE80);
    case sim::Integrator::kRungeKutta4:
    case sim::Integrator::kCount:
      break;
  }
  return FromHex(0x60A5FA);
}

void ApplyMotionPreset(Scene& scene, EntityId id, MotionPreset preset) {
  Entity* entity = scene.Get(id);
  if (entity == nullptr) return;

  const Transform world = scene.WorldTransform(id);
  const Vec3 forward = Rotate(world.rotation, -Vec3::UnitZ());
  const Vec3 right = Rotate(world.rotation, Vec3::UnitX());
  const Vec3 up = Rotate(world.rotation, Vec3::UnitY());

  BodyComponent body;
  body.collision_radius = CollisionRadius(*entity);
  sim::ForceModel& f = body.forces;
  sim::RigidBodyState& launch = body.initial;

  switch (preset) {
    case MotionPreset::kBallistic:
      body.mass = sim::BoxMassProperties(2.0, {0.25, 0.25, 1.6});
      f.quadratic_drag = 0.004;
      f.weathervane = 0.02;
      f.angular_damping = 1.5;  // Damping ratio ~0.3: the nose settles.
      launch.velocity = forward * 32.0;
      break;
    case MotionPreset::kPowered:
      body.mass = sim::BoxMassProperties(2.0, {0.25, 0.25, 1.6});
      f.thrust = 55.0;
      f.thrust_duration = 2.5;
      f.quadratic_drag = 0.005;
      f.weathervane = 0.03;
      f.angular_damping = 1.5;
      launch.velocity = forward * 2.0;
      break;
    case MotionPreset::kGlider: {
      body.mass = sim::BoxMassProperties(1.5, {2.4, 0.3, 1.8});
      constexpr double kTrimSpeed = 16.0;
      // Lift equals weight at trim speed.
      f.lift_coefficient =
          body.mass.mass * sim::kStandardGravity / (kTrimSpeed * kTrimSpeed);
      f.quadratic_drag = 0.0045;
      f.weathervane = 0.03;
      f.angular_damping = 0.25;
      body.restitution = 0.1;
      body.ground_friction = 0.8;
      launch.velocity = forward * kTrimSpeed;
      break;
    }
    case MotionPreset::kOrbit: {
      constexpr double kRadius = 8.0;
      body.mass = sim::SphereMassProperties(1.0, 0.5);
      f.gravity_enabled = false;
      f.central_enabled = true;
      f.central_mu = 400.0;
      f.central_point = world.position - right * kRadius;
      body.ground_contact = false;
      // Circular orbit speed v = sqrt(mu / r), perpendicular to the radius.
      launch.velocity = forward * std::sqrt(f.central_mu / kRadius);
      break;
    }
    case MotionPreset::kOscillator:
      body.mass = sim::SphereMassProperties(1.5, 0.5);
      f.gravity_enabled = false;
      f.spring_enabled = true;
      f.spring_stiffness = 6.0;
      f.spring_anchor = world.position - right * 4.0;
      body.ground_contact = false;
      launch.velocity = forward * 5.0 + up * 2.0;
      break;
    case MotionPreset::kTumble:
      body.mass = sim::BoxMassProperties(2.0, Abs(entity->local.scale));
      f.gravity_enabled = false;
      body.ground_contact = false;
      // Mostly about body Y with a tiny disturbance on the other axes.
      launch.angular_velocity = Rotate(world.rotation, Vec3{0.05, 3.5, 0.02});
      break;
    case MotionPreset::kCirclePath:
    case MotionPreset::kFigureEight:
    case MotionPreset::kHelix: {
      body.motion = MotionType::kKinematic;
      body.ground_contact = false;
      sim::KinematicPath& path = body.path;
      if (preset == MotionPreset::kFigureEight) {
        path.shape = sim::PathShape::kFigureEight;
        path.radius = 14.0;
        path.period = 18.0;
      } else {
        path.shape = preset == MotionPreset::kHelix ? sim::PathShape::kHelix
                                                    : sim::PathShape::kCircle;
        path.radius = preset == MotionPreset::kHelix ? 12.0 : 25.0;
        path.period = preset == MotionPreset::kHelix ? 10.0 : 14.0;
        path.climb_rate = 1.0;
      }
      // Place the path so it starts where the object is.
      path.center = sim::PathCenterForStart(path, world.position);
      break;
    }
    case MotionPreset::kCount:
      return;
  }

  entity->body = body;
  TrailComponent trail;
  trail.color = PresetTrailColor(preset);
  entity->trail = trail;
  PrepareBody(scene, id);
}

void BuildDemoScene(Scene& scene) {
  const auto spawn = [&scene](Archetype archetype, const char* name,
                              const Vec3& position, const EulerAngles& angles,
                              MotionPreset preset) {
    const EntityId id = Spawn(scene, archetype, position, FromEuler(angles));
    // Clear the default name first so it does not collide with itself.
    scene.Get(id)->name.clear();
    scene.Get(id)->name = scene.MakeUniqueName(name);
    ApplyMotionPreset(scene, id, preset);
    return id;
  };

  // Facing +X (yaw -90 degrees) keeps most launches across the default view.
  spawn(Archetype::kDart, "Ballistic dart", {-45.0, 1.0, 18.0},
        {.yaw = -kHalfPi, .pitch = Radians(40.0), .roll = 0.0},
        MotionPreset::kBallistic);
  spawn(Archetype::kDart, "Powered dart", {-10.0, 1.0, 30.0},
        {.yaw = -kHalfPi, .pitch = Radians(70.0), .roll = 0.0},
        MotionPreset::kPowered);
  spawn(Archetype::kFixedWing, "Glider", {-60.0, 45.0, -25.0},
        {.yaw = -kHalfPi, .pitch = 0.0, .roll = 0.0}, MotionPreset::kGlider);
  spawn(Archetype::kFixedWing, "Patrol circle", {35.0, 20.0, -15.0},
        {.yaw = 0.0, .pitch = 0.0, .roll = 0.0}, MotionPreset::kCirclePath);
  spawn(Archetype::kQuadrotor, "Quadrotor", {0.0, 7.0, -20.0},
        {.yaw = 0.0, .pitch = 0.0, .roll = 0.0}, MotionPreset::kFigureEight);
  spawn(Archetype::kSphere, "Orbiter", {48.0, 14.0, 20.0},
        {.yaw = 0.0, .pitch = 0.0, .roll = 0.0}, MotionPreset::kOrbit);
  spawn(Archetype::kSphere, "Oscillator", {-30.0, 10.0, -5.0},
        {.yaw = 0.0, .pitch = 0.0, .roll = 0.0}, MotionPreset::kOscillator);

  const EntityId tumbler = Spawn(scene, Archetype::kCube, {15.0, 12.0, 25.0});
  Entity* box = scene.Get(tumbler);
  box->name.clear();
  box->name = scene.MakeUniqueName("Tumbling box");
  box->local.scale = {0.3, 1.0, 1.8};
  if (box->render.has_value()) {
    box->render->material.base_color = FromHex(0xE8C547);
  }
  ApplyMotionPreset(scene, tumbler, MotionPreset::kTumble);

  // Static landmarks give the ground scale and depth cues.
  Spawn(scene, Archetype::kCube, {0.0, 0.5, 0.0});
  Spawn(scene, Archetype::kSphere, {6.0, 0.5, 6.0});
}

EntityId SpawnIntegratorComparison(Scene& scene, const Vec3& center) {
  const EntityId group =
      scene.Create(scene.MakeUniqueName("Integrator comparison"));
  scene.Get(group)->local.position = center;
  for (sim::Integrator integrator : sim::AllIntegrators()) {
    // Every body starts from exactly the same state; only the integrator
    // differs, so diverging trails show each method's error.
    const EntityId id =
        Spawn(scene, Archetype::kSphere, center + Vec3{10.0, 0.0, 0.0});
    scene.SetParent(id, group);
    Entity* entity = scene.Get(id);
    entity->name = std::string(sim::IntegratorName(integrator));
    entity->local.scale = {0.5, 0.5, 0.5};
    if (entity->render.has_value()) {
      entity->render->material.base_color = IntegratorColor(integrator);
    }
    ApplyMotionPreset(scene, id, MotionPreset::kOrbit);
    if (!entity->body.has_value() || !entity->trail.has_value()) continue;
    // Orbit the group centre rather than a point beside the sphere.
    BodyComponent& body = *entity->body;
    body.forces.central_point = center;
    body.initial.velocity =
        Vec3{0.0, 0.0, -std::sqrt(body.forces.central_mu / 10.0)};
    body.integrator = integrator;
    entity->trail->color = IntegratorColor(integrator);
    PrepareBody(scene, id);
  }
  return group;
}

}  // namespace aerolab
