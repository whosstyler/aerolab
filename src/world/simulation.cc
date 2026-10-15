#include "world/simulation.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <format>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "core/clock.h"
#include "geometry/mesh_library.h"
#include "math/intersection.h"
#include "math/quat.h"
#include "math/transform.h"
#include "sim/agent_vehicle.h"
#include "sim/flight_agent.h"
#include "sim/force_model.h"
#include "sim/integrator.h"
#include "sim/kinematic_path.h"
#include "world/animation.h"
#include "world/autonomy.h"

namespace aerolab {
namespace {

// Impacts slower than this are treated as settling, not as events.
constexpr double kImpactReportSpeed = 1.0;
// Normal speeds below this after a bounce come to rest.
constexpr double kRestSpeed = 0.2;
// Real impacts also scrub spin; resting contact does not.
constexpr double kImpactSpinRetention = 0.6;

// True when every component of the state is a finite number. Orientation
// is checked too: a huge but finite spin rate can overflow inside the
// quaternion exponential map before any vector component becomes infinite.
bool IsFiniteState(const sim::RigidBodyState& s) {
  const Quat& q = s.orientation;
  return IsFinite(s.position) && IsFinite(s.velocity) &&
         IsFinite(s.angular_velocity) && std::isfinite(q.w) &&
         std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z);
}

// Beyond these magnitudes a toy body has certainly blown up, even while
// still finite; catching it here keeps single-precision rendering sane.
constexpr double kRunawayPosition = 1e6;  // m
constexpr double kRunawaySpeed = 1e5;     // m/s and rad/s

bool IsRunaway(const sim::RigidBodyState& s) {
  return MaxComponent(Abs(s.position)) > kRunawayPosition ||
         MaxComponent(Abs(s.velocity)) > kRunawaySpeed ||
         MaxComponent(Abs(s.angular_velocity)) > kRunawaySpeed;
}

// State of a kinematic body at `age`, with angular velocity estimated from
// the orientation change since `previous`.
sim::RigidBodyState KinematicState(const BodyComponent& body, double age,
                                   const sim::RigidBodyState& previous,
                                   double dt) {
  const sim::PathSample sample = sim::SamplePath(body.path, age);
  sim::RigidBodyState state;
  state.position = sample.position;
  state.velocity = sample.velocity;
  state.orientation = sample.orientation;
  if (dt > 0.0) {
    const Quat delta = sample.orientation * Conjugate(previous.orientation);
    state.angular_velocity = ToRotationVector(delta) / dt;
  }
  return state;
}

Vec3 CurrentAcceleration(const BodyComponent& body) {
  if (body.motion == MotionType::kAutonomous) return body.acceleration;
  if (body.motion == MotionType::kKinematic) {
    return sim::SamplePath(body.path, body.age).acceleration;
  }
  return sim::Evaluate(body.state, body.mass, body.forces, body.age)
      .acceleration;
}

// Puts a body back at its launch conditions.
void ResetBody(BodyComponent& body) {
  body.age = 0.0;
  body.state = body.motion == MotionType::kKinematic
                   ? KinematicState(body, 0.0, body.initial, 0.0)
                   : body.initial;
  body.halt = BodyHalt::kNone;
  body.telemetry.Clear();
  if (body.motion == MotionType::kAutonomous) {
    body.acceleration = Vec3::Zero();
    body.state.angular_velocity = Vec3::Zero();
    // Launch inside the flight envelope (a wing cannot start from rest; a
    // dart switched to autonomous may have been launched far too fast).
    sim::AgentMotion launch{.position = body.state.position,
                            .velocity = body.state.velocity,
                            .acceleration = Vec3::Zero(),
                            .orientation = body.state.orientation};
    sim::FitToEnvelope(body.agent, launch);
    body.state.velocity = launch.velocity;
    body.state.orientation = launch.orientation;
    sim::ResetBrain(body.agent, launch, body.brain);
    sim::ResetVehicle(body.agent.style, body.vehicle);
    body.avoiding = kNoEntity;
  } else {
    body.acceleration = CurrentAcceleration(body);
  }
  body.previous = body.state;
}

// Writes body poses into the hierarchy parents-first, so a body attached to
// another body is placed relative to its parent's new pose.
void SyncBodyPoses(Scene& scene, EntityId id) {
  Entity* entity = scene.Get(id);
  if (entity == nullptr) return;
  if (entity->body.has_value()) {
    scene.SetWorldPose(id, entity->body->state.position,
                       entity->body->state.orientation);
  }
  for (EntityId child : entity->children) SyncBodyPoses(scene, child);
}

}  // namespace

SimulationSystem::SimulationSystem(double fixed_step) : timestep_(fixed_step) {}

void SimulationSystem::set_time_scale(double scale) {
  time_scale_ = std::clamp(scale, 0.0, 16.0);
}

int SimulationSystem::Update(Scene& scene, double real_dt) {
  stats_.steps_last_frame = 0;
  stats_.step_milliseconds = 0.0;
  if (paused_) return 0;
  const int steps = timestep_.Accumulate(real_dt * time_scale_);
  stats_.dropped_steps = timestep_.dropped_steps();
  if (steps == 0) return 0;
  Stopwatch watch;
  for (int i = 0; i < steps; ++i) Step(scene, timestep_.step());
  show_latest_ = false;
  stats_.steps_last_frame = steps;
  stats_.step_milliseconds = watch.ElapsedMilliseconds();
  return steps;
}

void SimulationSystem::StepOnce(Scene& scene) {
  Stopwatch watch;
  Step(scene, timestep_.step());
  show_latest_ = true;
  stats_.steps_last_frame = 1;
  stats_.step_milliseconds = watch.ElapsedMilliseconds();
  stats_.dropped_steps = timestep_.dropped_steps();
}

void SimulationSystem::Reset(Scene& scene) {
  scene.ForEachEntity([](EntityId, Entity& entity) {
    if (entity.body.has_value()) ResetBody(*entity.body);
    if (entity.trail.has_value()) {
      entity.trail->points.Clear();
      entity.trail->since_last_sample = 0.0;
    }
  });
  for (EntityId root : scene.roots()) SyncBodyPoses(scene, root);
  ResetSpinners(scene);
  time_ = 0.0;
  step_count_ = 0;
  telemetry_clock_ = 0.0;
  timestep_.Reset();
  stats_ = SimulationStats();
  show_latest_ = true;
  events_.clear();
}

std::vector<SimulationEvent> SimulationSystem::TakeEvents() {
  return std::exchange(events_, {});
}

void SimulationSystem::PushEvent(SimulationEvent::Kind kind, EntityId id,
                                 std::string message) {
  if (events_.size() >= kMaxPendingEvents) events_.erase(events_.begin());
  events_.push_back({kind, time_, id, std::move(message)});
}

void SimulationSystem::Step(Scene& scene, double dt) {
  // Events raised during this step are stamped with its end time.
  time_ += dt;
  ++step_count_;

  // Agents sense the world as it was at the start of the step, so the
  // order in which bodies are stepped does not matter.
  bool has_agents = false;
  scene.ForEachEntity([&has_agents](EntityId, const Entity& entity) {
    has_agents = has_agents || (entity.body.has_value() &&
                                entity.body->motion == MotionType::kAutonomous);
  });
  if (has_agents) {
    CollectObstacles(scene, obstacles_);
  } else {
    obstacles_.clear();
  }

  scene.ForEachEntity([this, dt](EntityId id, Entity& entity) {
    if (entity.body.has_value()) StepBody(id, entity.name, *entity.body, dt);
  });
  for (EntityId root : scene.roots()) SyncBodyPoses(scene, root);
  AdvanceSpinners(scene, dt);

  telemetry_clock_ += dt;
  const bool sample_telemetry = telemetry_clock_ >= kTelemetryInterval;
  if (sample_telemetry) telemetry_clock_ -= kTelemetryInterval;

  scene.ForEachEntity([&](EntityId id, Entity& entity) {
    if (entity.trail.has_value() && entity.trail->enabled) {
      TrailComponent& trail = *entity.trail;
      trail.since_last_sample += dt;
      if (trail.points.empty() ||
          trail.since_last_sample >= trail.sample_interval) {
        // Keep the overshoot so the cadence does not drift, but never let
        // it build up (e.g. after the interval is shortened).
        trail.since_last_sample = std::min(
            std::max(trail.since_last_sample - trail.sample_interval, 0.0),
            trail.sample_interval);
        trail.points.Push(entity.body.has_value()
                              ? entity.body->state.position
                              : scene.WorldMatrix(id).TranslationPart());
      }
    }
    if (sample_telemetry && entity.body.has_value()) {
      RecordHistory(*entity.body);
    }
  });
}

void SimulationSystem::StepBody(EntityId id, const std::string& name,
                                BodyComponent& body, double dt) {
  body.previous = body.state;

  if (body.motion == MotionType::kKinematic) {
    body.state = KinematicState(body, body.age + dt, body.previous, dt);
    body.age += dt;
    body.acceleration = CurrentAcceleration(body);
    return;
  }

  // A halted body stays where it stopped until the next reset.
  if (body.halt != BodyHalt::kNone) {
    body.age += dt;
    return;
  }
  if (body.motion == MotionType::kAutonomous) {
    StepAgentBody(id, name, body, dt);
    return;
  }
  const double burn_end = body.forces.thrust_duration;
  body.state = sim::Step(body.state, body.mass, body.forces, body.age, dt,
                         body.integrator);
  const bool finite = IsFiniteState(body.state);
  if (!finite || IsRunaway(body.state)) {
    // Freeze the body at its last good state rather than spreading NaNs
    // or float overflow through rendering and the UI.
    body.state = body.previous;
    body.state.velocity = Vec3::Zero();
    body.state.angular_velocity = Vec3::Zero();
    if (finite) {
      body.halt = BodyHalt::kOutOfBounds;
      PushEvent(SimulationEvent::Kind::kOutOfBounds, id,
                std::format("{}: left the simulation bounds (|x| > {:g} m or "
                            "speed > {:g}); body stopped",
                            name, kRunawayPosition, kRunawaySpeed));
    } else {
      body.halt = BodyHalt::kDiverged;
      PushEvent(SimulationEvent::Kind::kDivergence, id,
                std::format("{}: numerical divergence with {}; body stopped",
                            name, sim::IntegratorName(body.integrator)));
    }
  }
  if (body.forces.thrust != 0.0 && body.age < burn_end &&
      body.age + dt >= burn_end) {
    PushEvent(SimulationEvent::Kind::kBurnout, id,
              std::format("{}: thrust burnout at {:.1f} m altitude", name,
                          body.state.position.y));
  }
  ApplyGroundContact(id, name, body);
  body.age += dt;
  body.acceleration = CurrentAcceleration(body);
}

void SimulationSystem::StepAgentBody(EntityId id, const std::string& name,
                                     BodyComponent& body, double dt) {
  if (!(dt > 0.0)) return;
  ObstaclesFor(obstacles_, id, agent_shapes_, agent_ids_);
  std::optional<sim::AgentDecision> decision;
  if (body.agent.model == sim::FlightModel::kPhysical) {
    // A real airframe: forces, moments and an autopilot move the body.
    decision = sim::StepPhysicalAgent(
        body.agent, agent_shapes_, body.forces.wind, dt, body.brain,
        body.vehicle, body.state, body.acceleration);
  } else {
    sim::AgentMotion motion{.position = body.state.position,
                            .velocity = body.state.velocity,
                            .acceleration = body.acceleration,
                            .orientation = body.state.orientation};
    decision =
        sim::StepAgent(body.agent, agent_shapes_, dt, body.brain, motion);
    body.state.position = motion.position;
    body.state.velocity = motion.velocity;
    body.state.orientation = motion.orientation;
    body.state.angular_velocity =
        ToRotationVector(motion.orientation *
                         Conjugate(body.previous.orientation)) /
        dt;
    body.acceleration = motion.acceleration;
  }
  const int avoided = body.brain.avoidance.obstacle;
  body.avoiding =
      avoided >= 0 && static_cast<std::size_t>(avoided) < agent_ids_.size()
          ? agent_ids_[static_cast<std::size_t>(avoided)]
          : kNoEntity;
  body.age += dt;

  const bool finite = IsFiniteState(body.state);
  if (!finite || IsRunaway(body.state)) {
    // Defensive (the agent sanitizes its parameters): never let a bad
    // state spread through rendering and the UI.
    body.state = body.previous;
    body.state.velocity = Vec3::Zero();
    body.state.angular_velocity = Vec3::Zero();
    body.acceleration = Vec3::Zero();
    body.halt = finite ? BodyHalt::kOutOfBounds : BodyHalt::kDiverged;
    PushEvent(
        finite ? SimulationEvent::Kind::kOutOfBounds
               : SimulationEvent::Kind::kDivergence,
        id,
        std::format("{}: autopilot left the valid range; body stopped", name));
    return;
  }
  if (decision.has_value()) {
    PushEvent(SimulationEvent::Kind::kDecision, id,
              DescribeDecision(name, body.agent.style, *decision));
  }
}

void SimulationSystem::ApplyGroundContact(EntityId id, const std::string& name,
                                          BodyComponent& body) {
  const double floor = body.collision_radius;
  if (!body.ground_contact || body.state.position.y >= floor) return;
  body.state.position.y = floor;
  const double normal_speed = -body.state.velocity.y;
  if (normal_speed <= 0.0) return;

  if (normal_speed > kImpactReportSpeed) {
    PushEvent(
        SimulationEvent::Kind::kImpact, id,
        std::format("{}: ground impact at {:.1f} m/s", name, normal_speed));
    body.state.angular_velocity *= kImpactSpinRetention;
  }
  const double bounce = normal_speed * body.restitution;
  body.state.velocity.y = bounce > kRestSpeed ? bounce : 0.0;

  // Coulomb friction: the tangential impulse is at most mu times the normal
  // impulse. A resting body re-contacts every step with normal speed g*dt,
  // so it decelerates at mu*g independent of the step size.
  const double normal_change = normal_speed + body.state.velocity.y;
  const Vec3 tangential{body.state.velocity.x, 0.0, body.state.velocity.z};
  const double slide = Length(tangential);
  if (slide > 0.0) {
    const double reduced = std::max(
        slide - std::max(body.ground_friction, 0.0) * normal_change, 0.0);
    body.state.velocity.x *= reduced / slide;
    body.state.velocity.z *= reduced / slide;
  }
}

void SimulationSystem::RecordHistory(BodyComponent& body) {
  TelemetrySample sample;
  sample.time = time_;
  sample.speed = Length(body.state.velocity);
  sample.altitude = body.state.position.y;
  sample.acceleration = Length(body.acceleration);
  sample.energy = body.motion == MotionType::kKinematic
                      ? std::numeric_limits<double>::quiet_NaN()
                      : MechanicalEnergy(body);
  body.telemetry.Push(sample);
}

void PrepareBody(Scene& scene, EntityId id) {
  Entity* entity = scene.Get(id);
  if (entity == nullptr || !entity->body.has_value()) return;
  BodyComponent& body = *entity->body;
  const Transform world = scene.WorldTransform(id);
  body.initial.position = world.position;
  body.initial.orientation = world.rotation;
  if (body.motion == MotionType::kKinematic) {
    body.path.center = sim::PathCenterForStart(body.path, world.position);
  }
  ResetBody(body);
  scene.SetWorldPose(id, body.state.position, body.state.orientation);
}

void SetLaunchPose(Scene& scene, EntityId id, const Vec3& position,
                   const Quat& orientation) {
  if (!scene.IsAlive(id)) return;
  scene.SetWorldPose(id, position, orientation);
  PrepareBody(scene, id);
}

void RelaunchFromInitialPose(Scene& scene, EntityId id) {
  const Entity* entity = scene.Get(id);
  if (entity == nullptr || !entity->body.has_value()) return;
  // PrepareBody reads the entity's current pose, which the simulation has
  // moved; put it back at the launch pose first.
  SetLaunchPose(scene, id, entity->body->initial.position,
                entity->body->initial.orientation);
}

Mat4 RenderWorldMatrix(const Scene& scene, EntityId id, double alpha) {
  const Entity* entity = scene.Get(id);
  if (entity == nullptr) return Mat4::Identity();
  const bool has_parent = scene.IsAlive(entity->parent);
  if (entity->body.has_value()) {
    // Bodies are simulated in world space; only the parent's scale carries
    // over (exact for uniformly scaled parents).
    const Transform pose = sim::InterpolatePose(entity->body->previous,
                                                entity->body->state, alpha);
    const Vec3 parent_scale =
        has_parent ? FromMatrix(scene.ParentWorldMatrix(id)).scale
                   : Vec3::One();
    return Mat4::Trs(pose.position, pose.rotation,
                     Hadamard(parent_scale, entity->local.scale));
  }
  Transform local = entity->local;
  if (entity->spin.has_value()) {
    local.rotation = InterpolatedSpinRotation(*entity->spin, alpha);
  }
  const Mat4 parent = has_parent
                          ? RenderWorldMatrix(scene, entity->parent, alpha)
                          : Mat4::Identity();
  return parent * ToMatrix(local);
}

std::optional<Aabb> RenderWorldBounds(const Scene& scene, EntityId id,
                                      double alpha) {
  const Entity* entity = scene.Get(id);
  if (entity == nullptr || !entity->render.has_value()) return std::nullopt;
  return TransformAabb(MeshLocalBounds(entity->render->mesh),
                       RenderWorldMatrix(scene, id, alpha));
}

const sim::MassProperties& SimulatedMass(const BodyComponent& body) {
  if (body.motion == MotionType::kAutonomous &&
      body.agent.model == sim::FlightModel::kPhysical) {
    return sim::AirframeMass(body.agent.style, body.vehicle);
  }
  return body.mass;
}

double PotentialEnergy(const BodyComponent& body) {
  // Agents ignore the force model, so only plain gravity applies to them.
  if (body.motion == MotionType::kAutonomous) {
    return SimulatedMass(body).mass * sim::kStandardGravity *
           body.state.position.y;
  }
  return sim::PotentialEnergy(body.state, body.mass, body.forces);
}

double MechanicalEnergy(const BodyComponent& body) {
  return sim::KineticEnergy(body.state, SimulatedMass(body)) +
         PotentialEnergy(body);
}

}  // namespace aerolab
