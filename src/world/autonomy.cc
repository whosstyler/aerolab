#include "world/autonomy.h"

#include <algorithm>
#include <cmath>
#include <format>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "geometry/mesh_library.h"
#include "math/aabb.h"
#include "math/quat.h"
#include "math/random.h"
#include "math/transform.h"
#include "math/vec3.h"
#include "sim/agent_vehicle.h"

namespace aerolab {
namespace {

// Walks the hierarchy; static parts of a body are skipped.
void CollectFrom(const Scene& scene, EntityId id, bool inside_body,
                 std::vector<SceneObstacle>& out) {
  const Entity* entity = scene.Get(id);
  if (entity == nullptr) return;
  const bool is_body = entity->body.has_value();
  if (is_body || (!inside_body && entity->render.has_value())) {
    if (const std::optional<sim::AgentObstacle> shape =
            ObstacleShape(scene, id)) {
      out.push_back({id, *shape});
    }
  }
  for (EntityId child : entity->children) {
    CollectFrom(scene, child, inside_body || is_body, out);
  }
}

double ScaledMeshRadius(const Entity& entity, const Vec3& scale) {
  if (!entity.render.has_value()) return 0.0;
  const Aabb& bounds = MeshLocalBounds(entity.render->mesh);
  if (IsEmpty(bounds)) return 0.0;
  return Length(Hadamard(HalfExtents(bounds), Abs(scale)));
}

}  // namespace

std::optional<sim::AgentObstacle> ObstacleShape(const Scene& scene,
                                                EntityId id) {
  const Entity* entity = scene.Get(id);
  if (entity == nullptr) return std::nullopt;
  const Transform world = scene.WorldTransform(id);
  std::optional<sim::AgentObstacle> shape;
  if (entity->render.has_value() &&
      !IsEmpty(MeshLocalBounds(entity->render->mesh))) {
    // The sphere around the scaled box does not depend on rotation.
    const Aabb& bounds = MeshLocalBounds(entity->render->mesh);
    shape = sim::AgentObstacle{
        world.position +
            Rotate(world.rotation, Hadamard(world.scale, Center(bounds))),
        ScaledMeshRadius(*entity, world.scale), Vec3::Zero()};
  } else if (entity->body.has_value()) {
    shape = sim::AgentObstacle{entity->body->state.position,
                               entity->body->collision_radius, Vec3::Zero()};
  }
  // Moving bodies report their velocity so agents can anticipate them.
  if (shape.has_value() && entity->body.has_value()) {
    shape->velocity = entity->body->state.velocity;
  }
  // Absurd values (e.g. a scale typed as 1e999) are not obstacles.
  if (shape.has_value() &&
      (!IsFinite(shape->center) || !std::isfinite(shape->radius) ||
       !IsFinite(shape->velocity))) {
    return std::nullopt;
  }
  return shape;
}

void CollectObstacles(const Scene& scene, std::vector<SceneObstacle>& out) {
  out.clear();
  for (EntityId root : scene.roots()) CollectFrom(scene, root, false, out);
}

void ObstaclesFor(std::span<const SceneObstacle> all, EntityId agent,
                  std::vector<sim::AgentObstacle>& shapes,
                  std::vector<EntityId>& ids) {
  shapes.clear();
  ids.clear();
  for (const SceneObstacle& obstacle : all) {
    if (obstacle.id == agent) continue;
    shapes.push_back(obstacle.shape);
    ids.push_back(obstacle.id);
  }
}

sim::FlightStyle NaturalFlightStyle(const Entity& entity) {
  if (entity.render.has_value() &&
      (entity.render->mesh == MeshKind::kFixedWing ||
       entity.render->mesh == MeshKind::kDart)) {
    return sim::FlightStyle::kWing;
  }
  return sim::FlightStyle::kHover;
}

sim::AgentParams AgentParamsFor(const Scene& scene, EntityId id,
                                sim::FlightStyle style,
                                sim::FlightModel model) {
  sim::AgentParams params = model == sim::FlightModel::kPhysical
                                ? sim::PhysicalAgentParams(style)
                                : sim::DefaultAgentParams(style);
  const Entity* entity = scene.Get(id);
  if (entity == nullptr) return params;
  const Transform world = scene.WorldTransform(id);
  params.area.center = {world.position.x, 0.0, world.position.z};
  params.seed = MixSeed(id.index + 1u, id.generation);
  const double radius = ScaledMeshRadius(*entity, world.scale);
  if (radius > 0.0) params.body_radius = std::max(radius, 0.3);
  return params;
}

std::string DescribeDecision(const std::string& name, sim::FlightStyle style,
                             const sim::AgentDecision& decision) {
  switch (decision.kind) {
    case sim::AgentDecision::Kind::kNewDestination:
      return std::format("{}: heading to a point {:.0f} m away at {:.1f} m/s",
                         name, decision.distance, decision.speed);
    case sim::AgentDecision::Kind::kLoiter:
      return std::format(
          "{}: {} for {:.1f} s", name,
          style == sim::FlightStyle::kWing ? "circling" : "hovering",
          decision.duration);
    case sim::AgentDecision::Kind::kReplan:
      return std::format("{}: changing plans; heading to a point {:.0f} m away",
                         name, decision.distance);
  }
  return name;
}

}  // namespace aerolab
