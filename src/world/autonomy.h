#ifndef AEROLAB_WORLD_AUTONOMY_H_
#define AEROLAB_WORLD_AUTONOMY_H_

#include <optional>
#include <span>
#include <string>
#include <vector>

#include "scene/entity.h"
#include "scene/scene.h"
#include "sim/flight_agent.h"

namespace aerolab {

// Something autonomous agents keep away from: a bounding sphere around an
// entity's current pose.
struct SceneObstacle {
  EntityId id;
  sim::AgentObstacle shape;
};

// Bounding sphere of an entity at its current pose (its mesh bounds, or the
// body's collision radius when it has no mesh). A sphere is crude for long
// thin objects: a tall tower becomes a large keep-out volume. Returns
// nothing for non-finite results.
std::optional<sim::AgentObstacle> ObstacleShape(const Scene& scene,
                                                EntityId id);

// Everything agents sense in `scene`: every body, plus every free-standing
// object with a mesh. Parts attached to a body (a quadrotor's rotors) move
// with it and are not obstacles in their own right. Replaces `out`.
void CollectObstacles(const Scene& scene, std::vector<SceneObstacle>& out);

// The obstacles as seen by `agent`: everything except the agent itself.
// `ids[i]` names the entity behind `shapes[i]`. Replaces both outputs.
void ObstaclesFor(std::span<const SceneObstacle> all, EntityId agent,
                  std::vector<sim::AgentObstacle>& shapes,
                  std::vector<EntityId>& ids);

// The flight style that suits an entity's shape: winged meshes (aircraft,
// darts) fly like wings, everything else hovers.
sim::FlightStyle NaturalFlightStyle(const Entity& entity);

// Default settings for `style`, fitted to the entity: the flight area is
// centred under it, the seed derives from its id and the body radius from
// its mesh. Copying these settings to another agent copies the seed, so
// both would make identical choices; give each agent its own seed.
sim::AgentParams AgentParamsFor(const Scene& scene, EntityId id,
                                sim::FlightStyle style);

// One-line description of a decision for the event log, e.g.
// "Quadrotor: hovering for 3.2 s".
std::string DescribeDecision(const std::string& name, sim::FlightStyle style,
                             const sim::AgentDecision& decision);

}  // namespace aerolab

#endif  // AEROLAB_WORLD_AUTONOMY_H_
