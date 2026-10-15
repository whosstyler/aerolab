#ifndef AEROLAB_WORLD_SIMULATION_H_
#define AEROLAB_WORLD_SIMULATION_H_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "core/fixed_timestep.h"
#include "math/aabb.h"
#include "math/mat4.h"
#include "scene/entity.h"
#include "scene/scene.h"

namespace aerolab {

// Something noteworthy that happened during a simulation step.
struct SimulationEvent {
  enum class Kind { kInfo, kBurnout, kImpact, kDivergence };
  Kind kind = Kind::kInfo;
  double time = 0.0;  // Simulation time, s.
  EntityId entity;
  std::string message;
};

struct SimulationStats {
  int steps_last_frame = 0;
  double step_milliseconds = 0.0;  // CPU time of the last frame's steps.
  std::int64_t dropped_steps = 0;  // Steps skipped to stay real time.
};

// Advances every BodyComponent in a scene with a fixed timestep.
//
// Each frame, the real elapsed time is scaled by the time scale and fed to a
// FixedTimestep accumulator; whole steps are then executed with the same dt,
// so results are identical at any frame rate. Pausing stops the clock;
// StepOnce() advances exactly one step; Reset() restores launch conditions.
class SimulationSystem {
 public:
  static constexpr double kTelemetryInterval = 1.0 / 20.0;  // 20 Hz.
  // Pending events beyond this are dropped (oldest first) if nobody drains
  // them with TakeEvents().
  static constexpr std::size_t kMaxPendingEvents = 256;

  explicit SimulationSystem(double fixed_step = FixedTimestep::kDefaultStep);

  // Advances by `real_dt` seconds of wall-clock time. Returns the number of
  // fixed steps executed.
  int Update(Scene& scene, double real_dt);

  // Executes exactly one fixed step (typically while paused).
  void StepOnce(Scene& scene);

  // Restores every body and spinner to its initial state and clears trails,
  // telemetry, statistics and the clock.
  void Reset(Scene& scene);

  bool paused() const { return paused_; }
  void set_paused(bool paused) { paused_ = paused; }

  double time_scale() const { return time_scale_; }
  // Clamped to [0, 16].
  void set_time_scale(double scale);

  double fixed_step() const { return timestep_.step(); }
  void set_fixed_step(double step) { timestep_.set_step(step); }

  double time() const { return time_; }
  std::int64_t step_count() const { return step_count_; }

  // Interpolation factor between the last two states for rendering. After a
  // single step or reset the latest state is shown (1); pausing freezes the
  // current blend so the picture does not jump.
  double alpha() const { return show_latest_ ? 1.0 : timestep_.alpha(); }

  const SimulationStats& stats() const { return stats_; }

  // Returns and clears the events produced since the last call.
  std::vector<SimulationEvent> TakeEvents();

 private:
  void Step(Scene& scene, double dt);
  void StepBody(EntityId id, const std::string& name, BodyComponent& body,
                double dt);
  void ApplyGroundContact(EntityId id, const std::string& name,
                          BodyComponent& body);
  void RecordHistory(BodyComponent& body);
  void PushEvent(SimulationEvent::Kind kind, EntityId id, std::string message);

  FixedTimestep timestep_;
  double time_scale_ = 1.0;
  bool paused_ = false;
  bool show_latest_ = true;
  double time_ = 0.0;
  std::int64_t step_count_ = 0;
  double telemetry_clock_ = 0.0;
  SimulationStats stats_;
  std::vector<SimulationEvent> events_;
};

// Uses the entity's current world pose as the body's launch pose (keeping
// the launch velocities already set in `body.initial`) and resets the body
// to it. Kinematic bodies have their path re-centred so it starts there.
// Call after configuring a body or editing its initial conditions.
void PrepareBody(Scene& scene, EntityId id);

// Moves a body's launch pose and resets the body to it.
void SetLaunchPose(Scene& scene, EntityId id, const Vec3& position,
                   const Quat& orientation);

// World matrix used for rendering: bodies are drawn at a pose interpolated
// between their last two simulation states, spinning parts at their
// interpolated angle, and descendants follow their parents.
Mat4 RenderWorldMatrix(const Scene& scene, EntityId id, double alpha);

// World-space bounds of the entity's mesh at its rendered pose.
std::optional<Aabb> RenderWorldBounds(const Scene& scene, EntityId id,
                                      double alpha);

// Total mechanical energy (kinetic + potential) of a body.
double MechanicalEnergy(const BodyComponent& body);

}  // namespace aerolab

#endif  // AEROLAB_WORLD_SIMULATION_H_
