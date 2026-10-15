#ifndef AEROLAB_SCENE_ENTITY_H_
#define AEROLAB_SCENE_ENTITY_H_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "core/ring_buffer.h"
#include "geometry/color.h"
#include "geometry/material.h"
#include "geometry/mesh_library.h"
#include "math/quat.h"
#include "math/transform.h"
#include "math/vec3.h"
#include "sim/force_model.h"
#include "sim/integrator.h"
#include "sim/kinematic_path.h"
#include "sim/rigid_body.h"

namespace aerolab {

// Generational handle to a scene entity. A handle to a destroyed entity
// never aliases a newer entity that reuses the same slot.
struct EntityId {
  static constexpr std::uint32_t kInvalidIndex = 0xFFFFFFFFu;

  std::uint32_t index = kInvalidIndex;
  std::uint32_t generation = 0;

  constexpr bool operator==(const EntityId&) const = default;
};

inline constexpr EntityId kNoEntity{};

constexpr bool IsSet(EntityId id) {
  return id.index != EntityId::kInvalidIndex;
}

// Visual representation of an entity.
struct RenderComponent {
  MeshKind mesh = MeshKind::kCube;
  Material material;
  bool visible = true;
  bool cast_shadow = true;
  // Clicks on this entity select its parent instead (e.g. rotor blades).
  bool pick_parent = false;
};

// Constant-rate rotation about a local axis; a purely kinematic animation
// used for spinning parts. The simulation advances `angle` with its fixed
// step and keeps `previous_angle` so rendering can interpolate; the entity's
// local rotation is `rest_rotation` followed by the spin.
struct SpinComponent {
  Vec3 axis = Vec3::UnitY();
  double rate = 0.0;  // rad/s
  double angle = 0.0;
  double previous_angle = 0.0;
  Quat rest_rotation;
};

enum class MotionType {
  kDynamic,    // Integrated from forces and torques.
  kKinematic,  // Follows a closed-form path; no forces.
};

// One sample of a body's flight data, recorded at a fixed rate.
struct TelemetrySample {
  double time = 0.0;          // Simulation time, s.
  double speed = 0.0;         // m/s
  double altitude = 0.0;      // m above the ground plane
  double acceleration = 0.0;  // m/s^2
  // Kinetic + potential energy in J; NaN for kinematic bodies, whose
  // motion is scripted and has no meaningful energy.
  double energy = 0.0;
};

// A simulated rigid body. The simulation owns `state` and writes the
// entity's transform from it after every fixed step; `previous` is kept so
// rendering can interpolate between steps.
struct BodyComponent {
  MotionType motion = MotionType::kDynamic;
  sim::RigidBodyState state;
  sim::RigidBodyState previous;
  // Launch conditions restored by a simulation reset.
  sim::RigidBodyState initial;

  sim::MassProperties mass;
  sim::ForceModel forces;
  sim::Integrator integrator = sim::Integrator::kRungeKutta4;
  sim::KinematicPath path;  // Used when motion == kKinematic.

  double age = 0.0;   // Seconds since launch (or the last reset).
  Vec3 acceleration;  // Most recent linear acceleration, m/s^2.

  // Simple ground interaction with the y = 0 plane.
  bool ground_contact = true;
  double collision_radius = 0.5;  // m, lowest extent below the body origin
  double restitution = 0.3;       // Bounce: fraction of normal speed kept.
  // Coulomb friction coefficient: each contact removes up to mu times the
  // normal speed change from the sliding speed, so a resting body slides
  // to a stop at about mu * g regardless of the time step.
  double ground_friction = 0.5;

  // Set when the integrator blew up; cleared by a reset.
  bool diverged = false;

  // Flight history for plots; sampled at a fixed simulated rate.
  RingBuffer<TelemetrySample> telemetry{1200};
};

// World-space breadcrumb trail of an entity's motion.
struct TrailComponent {
  bool enabled = true;
  RingBuffer<Vec3> points{900};
  double sample_interval = 1.0 / 30.0;  // Simulated seconds between points.
  double since_last_sample = 0.0;
  Color color = FromHex(0x7DD3FC);
};

// A node of the scene graph. `local` is relative to the parent (or to the
// world for root entities).
struct Entity {
  std::string name;
  Transform local;
  // Maintained by Scene; use Scene::SetParent() to change.
  EntityId parent;
  std::vector<EntityId> children;

  std::optional<RenderComponent> render;
  std::optional<SpinComponent> spin;
  std::optional<BodyComponent> body;
  std::optional<TrailComponent> trail;
};

}  // namespace aerolab

#endif  // AEROLAB_SCENE_ENTITY_H_
