#include "world/debug_visualizer.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <span>
#include <vector>

#include "core/ring_buffer.h"
#include "geometry/color.h"
#include "geometry/mesh_library.h"
#include "math/mat4.h"
#include "math/scalar.h"
#include "sim/flight_agent.h"
#include "sim/force_model.h"
#include "sim/integrator.h"
#include "sim/kinematic_path.h"
#include "sim/rigid_body.h"
#include "world/autonomy.h"
#include "world/simulation.h"

namespace aerolab {
namespace {

constexpr Color kDropLineColor = FromHex(0xCBD5E1, 0.35f);
constexpr Color kBoundsColor = FromHex(0x94A3B8, 0.55f);
constexpr Color kPredictionColor = FromHex(0xFDE68A, 0.9f);
constexpr Color kAttractorColor = FromHex(0xC4B5FD, 0.7f);
constexpr Color kAgentGoalColor = FromHex(0x5EEAD4, 0.9f);
constexpr Color kAgentAreaColor = FromHex(0x5EEAD4, 0.22f);
constexpr Color kAgentRequestColor = FromHex(0xF0ABFC, 0.9f);
constexpr Color kAgentAvoidColor = FromHex(0xF87171, 0.85f);
// At a 120 Hz fixed step this is 20 s of forecast.
constexpr int kMaxAgentForecastSteps = 2400;

struct ForceStyle {
  const Vec3 sim::ForceBreakdown::* term;
  Color color;
};

void AddForceVectors(const BodyComponent& body, const Vec3& origin,
                     double scale, DebugDrawList& lines) {
  const sim::ForceBreakdown forces =
      sim::ComputeForces(body.state, body.mass, body.forces, body.age);
  // Forces are drawn as the acceleration each term would cause alone.
  const double per_newton = scale / std::max(body.mass.mass, 1e-6);
  const std::array<ForceStyle, 6> styles = {{
      {&sim::ForceBreakdown::gravity, FromHex(0xE2E8F0)},
      {&sim::ForceBreakdown::drag, FromHex(0xF87171)},
      {&sim::ForceBreakdown::thrust, FromHex(0xFB923C)},
      {&sim::ForceBreakdown::lift, FromHex(0x4ADE80)},
      {&sim::ForceBreakdown::spring, FromHex(0xF472B6)},
      {&sim::ForceBreakdown::central, FromHex(0xA78BFA)},
  }};
  for (const ForceStyle& style : styles) {
    const Vec3& force = forces.*style.term;
    if (LengthSquared(force) > 1e-12) {
      lines.AddArrow(origin, force * per_newton, style.color);
    }
  }
}

// Draws a trail oldest-to-newest with alpha rising toward the body, reading
// the ring buffer in place (no copy) and ending at the drawn position.
void AddTrail(const RingBuffer<Vec3>& points, const Vec3& head,
              const Color& color, DebugDrawList& lines) {
  const std::size_t n = points.size();
  const auto alpha_at = [&](std::size_t i) {
    return color.a * static_cast<float>(i) / static_cast<float>(n);
  };
  for (std::size_t i = 0; i + 1 < n; ++i) {
    lines.AddLine(points[i], points[i + 1], WithAlpha(color, alpha_at(i)),
                  WithAlpha(color, alpha_at(i + 1)));
  }
  lines.AddLine(points[n - 1], head, WithAlpha(color, alpha_at(n - 1)), color);
}

// Draws the forecast from the drawn position outward, fading with time.
void AddPrediction(const std::vector<Vec3>& prediction, const Vec3& start,
                   DebugDrawList& lines) {
  if (prediction.size() < 2) return;
  const auto alpha_at = [&](std::size_t i) {
    return kPredictionColor.a *
           (1.0f - 0.95f * static_cast<float>(i) /
                       static_cast<float>(prediction.size() - 1));
  };
  // The body is drawn interpolated, slightly behind its latest state.
  lines.AddLine(start, prediction[0], kPredictionColor, kPredictionColor);
  for (std::size_t i = 0; i + 1 < prediction.size(); ++i) {
    lines.AddLine(prediction[i], prediction[i + 1],
                  WithAlpha(kPredictionColor, alpha_at(i)),
                  WithAlpha(kPredictionColor, alpha_at(i + 1)));
  }
}

void AddAttractors(const BodyComponent& body, const Vec3& position,
                   DebugDrawList& lines) {
  if (body.forces.spring_enabled) {
    lines.AddCross(body.forces.spring_anchor, 0.4, kAttractorColor);
    lines.AddLine(position, body.forces.spring_anchor,
                  WithAlpha(kAttractorColor, 0.3f));
  }
  if (body.forces.central_enabled) {
    lines.AddWireSphere(body.forces.central_point, 0.5, kAttractorColor, 24);
  }
}

// An autonomous agent's plan: where it is heading (for every agent) and,
// when selected, its flight area, loiter circle, steering request and the
// object it is currently keeping clear of.
void AddAgentPlan(const Scene& scene, const BodyComponent& body,
                  const Vec3& position, bool selected, double velocity_scale,
                  DebugDrawList& lines) {
  const sim::AgentBrain& brain = body.brain;
  const sim::AgentParams& agent = body.agent;
  const Color goal = WithAlpha(kAgentGoalColor, selected ? 0.9f : 0.4f);
  if (brain.has_destination) {
    if (brain.mode == sim::AgentMode::kTransit) {
      lines.AddLine(position, brain.destination, WithAlpha(goal, 0.05f), goal);
    }
    lines.AddCross(brain.destination, 0.6, goal);
    lines.AddCircle(brain.destination, Vec3::UnitY(), 1.2, goal, 24);
  }
  if (!selected) return;

  // Flight area: rings at both altitude limits joined by four uprights.
  const sim::FlightArea& area = agent.area;
  const double low = std::min(area.min_altitude, area.max_altitude);
  const double high = std::max(area.min_altitude, area.max_altitude);
  for (double altitude : {low, high}) {
    lines.AddCircle({area.center.x, altitude, area.center.z}, Vec3::UnitY(),
                    area.radius, kAgentAreaColor, 96);
  }
  for (int i = 0; i < 4; ++i) {
    const double angle = kHalfPi * i;
    const double x = area.center.x + area.radius * std::cos(angle);
    const double z = area.center.z + area.radius * std::sin(angle);
    lines.AddLine({x, low, z}, {x, high, z}, kAgentAreaColor);
  }
  if (agent.style == sim::FlightStyle::kWing &&
      brain.mode == sim::AgentMode::kLoiter) {
    lines.AddCircle(brain.loiter_center, Vec3::UnitY(), brain.loiter_radius,
                    goal, 64);
  }
  lines.AddArrow(position, brain.desired_velocity * velocity_scale,
                 kAgentRequestColor);
  if (IsSet(body.avoiding)) {
    if (const std::optional<sim::AgentObstacle> shape =
            ObstacleShape(scene, body.avoiding)) {
      lines.AddWireSphere(shape->center,
                          shape->radius + agent.body_radius + agent.clearance,
                          kAgentAvoidColor, 32);
    }
  }
}

}  // namespace

std::vector<Vec3> PredictTrajectory(
    const BodyComponent& body, double seconds, double dt,
    std::span<const sim::AgentObstacle> obstacles) {
  std::vector<Vec3> points;
  if (seconds <= 0.0 || dt <= 0.0) return points;
  if (body.halt != BodyHalt::kNone) return {body.state.position};
  int steps = static_cast<int>(std::lround(std::min(seconds / dt, 20000.0)));
  points.reserve(static_cast<std::size_t>(steps) + 1);
  points.push_back(body.state.position);
  if (body.motion == MotionType::kKinematic) {
    for (int i = 1; i <= steps; ++i) {
      points.push_back(sim::SamplePath(body.path, body.age + i * dt).position);
    }
    return points;
  }
  if (body.motion == MotionType::kAutonomous) {
    // Replay the agent's own decisions on a copy of its brain. Each step
    // runs the full AI, so the forecast is capped to keep frames cheap.
    steps = std::min(steps, kMaxAgentForecastSteps);
    sim::AgentBrain brain = body.brain;
    sim::AgentMotion motion{.position = body.state.position,
                            .velocity = body.state.velocity,
                            .acceleration = body.acceleration,
                            .orientation = body.state.orientation};
    // Obstacles carry on at their current velocities.
    std::vector<sim::AgentObstacle> moving(obstacles.begin(), obstacles.end());
    for (int i = 0; i < steps; ++i) {
      sim::StepAgent(body.agent, moving, dt, brain, motion);
      points.push_back(motion.position);
      for (sim::AgentObstacle& o : moving) o.center += o.velocity * dt;
    }
    return points;
  }
  sim::RigidBodyState state = body.state;
  double age = body.age;
  for (int i = 0; i < steps; ++i) {
    state = sim::Step(state, body.mass, body.forces, age, dt, body.integrator);
    age += dt;
    points.push_back(state.position);
    // Stop at the ground, where contact would take over.
    if (body.ground_contact && state.position.y < body.collision_radius) break;
  }
  return points;
}

void AddDebugVisualization(const Scene& scene, const DebugViewOptions& options,
                           double alpha, EntityId selected,
                           DebugDrawList& lines, EntityId hidden) {
  if (options.world_axes) lines.AddAxes(Mat4::Identity(), 2.0);

  scene.ForEachEntity([&](EntityId id, const Entity& entity_ref) {
    const Entity* entity = &entity_ref;
    if (IsSet(hidden) && scene.IsAncestorOrSelf(hidden, id)) return;
    const Mat4 world = RenderWorldMatrix(scene, id, alpha);
    const Vec3 position = world.TranslationPart();

    if (options.bounding_boxes && entity->render.has_value()) {
      lines.AddOrientedBox(MeshLocalBounds(entity->render->mesh), world,
                           kBoundsColor);
    }

    if (options.trails && entity->trail.has_value() && entity->trail->enabled &&
        !entity->trail->points.empty()) {
      AddTrail(entity->trail->points, position, entity->trail->color, lines);
    }

    if (!entity->body.has_value()) return;
    const BodyComponent& body = *entity->body;
    const sim::RigidBodyState& state = body.state;

    if (options.local_frames) lines.AddAxes(world, 1.2);

    if (options.drop_lines && position.y > 0.05) {
      lines.AddLine(position, {position.x, 0.0, position.z}, kDropLineColor);
      lines.AddCircle({position.x, 0.0, position.z}, Vec3::UnitY(), 0.35,
                      kDropLineColor, 20);
    }

    const Vec3 velocity = Lerp(body.previous.velocity, state.velocity, alpha);
    if (options.velocity_vectors) {
      lines.AddArrow(position, velocity * options.velocity_scale,
                     colors::kVelocity);
    }
    if (options.acceleration_vectors) {
      lines.AddArrow(position, body.acceleration * options.acceleration_scale,
                     colors::kAcceleration);
    }
    // Agents ignore their force model, so its anchors mean nothing there.
    if (options.attractors && body.motion != MotionType::kAutonomous) {
      AddAttractors(body, position, lines);
    }
    if (options.agent_plans && body.motion == MotionType::kAutonomous) {
      AddAgentPlan(scene, body, position, id == selected,
                   options.velocity_scale, lines);
    }

    if (id != selected) return;
    if (options.force_vectors && body.motion == MotionType::kDynamic) {
      AddForceVectors(body, position, options.acceleration_scale, lines);
    }
    if (options.predicted_path) {
      // Agents forecast against what they sense right now.
      std::vector<sim::AgentObstacle> shapes;
      if (body.motion == MotionType::kAutonomous) {
        std::vector<SceneObstacle> all;
        std::vector<EntityId> ids;
        CollectObstacles(scene, all);
        ObstaclesFor(all, id, shapes, ids);
      }
      const std::vector<Vec3> prediction = PredictTrajectory(
          body, options.prediction_seconds, options.prediction_step, shapes);
      AddPrediction(prediction, position, lines);
    }
  });
}

}  // namespace aerolab
