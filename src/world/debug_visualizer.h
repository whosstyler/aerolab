#ifndef AEROLAB_WORLD_DEBUG_VISUALIZER_H_
#define AEROLAB_WORLD_DEBUG_VISUALIZER_H_

#include <span>
#include <vector>

#include "geometry/debug_draw.h"
#include "math/vec3.h"
#include "scene/entity.h"
#include "scene/scene.h"
#include "sim/flight_agent.h"

namespace aerolab {

// Switches for each debug visualization layer.
struct DebugViewOptions {
  bool world_axes = true;
  bool trails = true;
  bool velocity_vectors = true;
  bool acceleration_vectors = false;
  bool force_vectors = false;  // Per-term forces of the selected body.
  bool local_frames = false;   // Orientation axes of every body.
  bool bounding_boxes = false;
  bool drop_lines = true;      // Vertical line from each body to the ground.
  bool attractors = true;      // Spring anchors and central attractors.
  bool predicted_path = true;  // Look-ahead of the selected body.
  // Autonomous agents: destinations, plus the selected agent's flight
  // area, loiter circle, steering request and what it is avoiding.
  bool agent_plans = true;

  double velocity_scale = 0.25;      // Metres drawn per m/s.
  double acceleration_scale = 0.15;  // Metres drawn per m/s^2.
  double prediction_seconds = 6.0;
  // Step used for the look-ahead; set it to the simulation's fixed step so
  // the forecast matches what the integrator will actually do.
  double prediction_step = 1.0 / 120.0;
};

// Integrates a copy of a dynamic body forward with its own integrator and
// returns sampled positions, starting at the current state. Ground contact
// is not modelled: the forecast stops where the body would touch down. For
// kinematic bodies the closed-form path is sampled instead. Autonomous
// bodies run a copy of their own brain (random generator included) against
// `obstacles` moving on at their current velocities, so the forecast shows
// the decisions the agent will actually make unless something changes
// course (static scenes are forecast exactly). Halted bodies forecast
// nothing, and agents at most 2400 steps.
std::vector<Vec3> PredictTrajectory(
    const BodyComponent& body, double seconds, double dt,
    std::span<const sim::AgentObstacle> obstacles = {});

// Appends the enabled debug layers for the whole scene. Body positions use
// the same interpolation (`alpha`) as rendering, so lines stay attached to
// the drawn objects.
void AddDebugVisualization(const Scene& scene, const DebugViewOptions& options,
                           double alpha, EntityId selected,
                           DebugDrawList& lines);

}  // namespace aerolab

#endif  // AEROLAB_WORLD_DEBUG_VISUALIZER_H_
