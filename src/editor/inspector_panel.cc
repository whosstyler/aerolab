// Inspector: transform (in world, parent and camera space), live dynamics,
// launch conditions and every configurable physical parameter of the
// selected entity.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <format>
#include <string>
#include <string_view>

#include "camera/camera.h"
#include "editor/panels.h"
#include "editor/theme.h"
#include "editor/widgets.h"
#include "imgui.h"
#include "math/quat.h"
#include "math/random.h"
#include "math/scalar.h"
#include "math/transform.h"
#include "sim/aerodynamics.h"
#include "sim/agent_vehicle.h"
#include "sim/atmosphere.h"
#include "sim/fixed_wing.h"
#include "sim/flight_agent.h"
#include "sim/flight_control.h"
#include "sim/force_model.h"
#include "sim/integrator.h"
#include "sim/kinematic_path.h"
#include "sim/multirotor.h"
#include "sim/rigid_body.h"
#include "world/autonomy.h"
#include "world/presets.h"
#include "world/simulation.h"

namespace aerolab::editor {
namespace {

constexpr ImGuiTreeNodeFlags kOpen = ImGuiTreeNodeFlags_DefaultOpen;

std::string Format3(const Vec3& v, const char* unit) {
  return std::format("{:.2f}, {:.2f}, {:.2f} {}", v.x, v.y, v.z, unit);
}

void DrawHeader(EditorFrame& frame, EntityId id, Entity& entity) {
  std::array<char, 128> name{};
  entity.name.copy(name.data(), name.size() - 1);
  ImGui::SetNextItemWidth(-1.0f);
  if (ImGui::InputText("##name", name.data(), name.size())) {
    entity.name = name.data();
  }
  const Entity* parent = frame.scene->Get(entity.parent);
  ImGui::TextDisabled("id %u.%u   parent: %s", id.index, id.generation,
                      parent != nullptr ? parent->name.c_str() : "(world)");
  if (ImGui::Button("Focus")) FocusSelection(frame);
  ImGui::SameLine();
  if (ImGui::Button("Delete")) DeleteSelection(frame);
}

void DrawTransform(EditorFrame& frame, EntityId id, Entity& entity) {
  if (!ImGui::CollapsingHeader("Transform", kOpen)) return;
  const Scene& scene = *frame.scene;
  const Transform world = scene.WorldTransform(id);
  if (BeginProperties("##transform")) {
    if (entity.body.has_value()) {
      PropertyText("Driven by", "simulation",
                   "Edit the launch conditions below to move it");
    } else {
      PropertyVec3("Position", &entity.local.position, 0.05f, "%.2f",
                   "Local position, relative to the parent");
      PropertyRotation("Rotation", &entity.local.rotation);
    }
    Vec3 scale = entity.local.scale;
    if (PropertyVec3("Scale", &scale, 0.01f, "%.2f")) {
      entity.local.scale = Max(scale, Vec3{1e-3, 1e-3, 1e-3});
    }
    // The same point expressed in each coordinate space.
    PropertyVec3ReadOnly("World", world.position, "m",
                         "Position in world space (Y up, ground at y = 0)");
    PropertyVec3ReadOnly("Parent-local", entity.local.position, "m",
                         "Position relative to the parent's frame");
    PropertyVec3ReadOnly(
        "Camera space",
        WorldToCameraSpace(frame.camera->camera(), world.position), "m",
        "Position relative to the camera (camera looks down -Z)");
    const EulerAngles e = ToEuler(world.rotation);
    PropertyText(
        "World attitude",
        std::format("yaw {:.1f}  pitch {:.1f}  roll {:.1f} deg", Degrees(e.yaw),
                    Degrees(e.pitch), Degrees(e.roll)));
    EndProperties();
  }
}

void DrawLiveState(const BodyComponent& body) {
  if (!ImGui::CollapsingHeader("Live state", kOpen)) return;
  const sim::RigidBodyState& s = body.state;
  if (BeginProperties("##live")) {
    PropertyText("Speed", std::format("{:.2f} m/s", Length(s.velocity)));
    PropertyText("Velocity", Format3(s.velocity, "m/s"));
    PropertyText(
        "Acceleration",
        std::format("{:.2f} m/s^2  ({:.2f} g)", Length(body.acceleration),
                    Length(body.acceleration) / sim::kStandardGravity));
    PropertyText("Accel vector", Format3(body.acceleration, "m/s^2"));
    PropertyText("Angular vel.",
                 Format3(s.angular_velocity * (180.0 / kPi), "deg/s"));
    PropertyText("Altitude", std::format("{:.2f} m", s.position.y));
    PropertyText("Age", std::format("{:.2f} s", body.age));
    if (body.motion != MotionType::kKinematic) {
      const double kinetic = sim::KineticEnergy(s, SimulatedMass(body));
      const double potential = PotentialEnergy(body);
      PropertyText("Kinetic energy", std::format("{:.1f} J", kinetic));
      PropertyText("Potential energy", std::format("{:.1f} J", potential),
                   body.motion == MotionType::kAutonomous
                       ? "Gravity only: agents ignore the force model"
                       : "Gravity, spring and central-body terms");
      PropertyText("Total energy",
                   std::format("{:.1f} J", kinetic + potential));
      PropertyText(
          "|Angular mom.|",
          std::format("{:.3f} kg m^2/s",
                      Length(sim::AngularMomentum(s, SimulatedMass(body)))));
    }
    EndProperties();
  }
  if (body.halt != BodyHalt::kNone && body.motion == MotionType::kAutonomous) {
    ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(theme::kWarning),
                       "The autopilot left the valid range and the body "
                       "was stopped. Reset to relaunch it.");
  } else if (body.halt == BodyHalt::kDiverged) {
    ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(theme::kWarning),
                       "Integrator diverged; the body was stopped. "
                       "Try RK4 or a smaller fixed step, then reset.");
  } else if (body.halt == BodyHalt::kOutOfBounds) {
    ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(theme::kWarning),
                       "The body left the simulation bounds and was "
                       "stopped. Reduce thrust or enable drag, then reset.");
  }
}

void DrawSimulationSettings(EditorFrame& frame, EntityId id,
                            BodyComponent& body) {
  if (!ImGui::CollapsingHeader("Simulation", kOpen)) return;
  if (BeginProperties("##simulation")) {
    PropertyLabel("Motion",
                  "Dynamic bodies integrate forces; kinematic bodies "
                  "follow a closed-form path; autonomous bodies fly "
                  "themselves (game-style AI)");
    constexpr std::array<MotionType, 3> kMotions = {
        MotionType::kDynamic, MotionType::kKinematic, MotionType::kAutonomous};
    int motion = static_cast<int>(
        std::find(kMotions.begin(), kMotions.end(), body.motion) -
        kMotions.begin());
    if (ImGui::Combo("##motion", &motion,
                     "Dynamic\0Kinematic path\0Autonomous (AI)\0")) {
      body.motion = kMotions[static_cast<std::size_t>(motion)];
      if (body.motion == MotionType::kAutonomous) {
        if (const Entity* entity = frame.scene->Get(id)) {
          body.agent =
              AgentParamsFor(*frame.scene, id, NaturalFlightStyle(*entity));
          // Centre the area on the launch point it is about to relaunch
          // from, not wherever the body has flown to.
          body.agent.area.center = {body.initial.position.x, 0.0,
                                    body.initial.position.z};
        }
      }
      RelaunchFromInitialPose(*frame.scene, id);
    }
    if (body.motion == MotionType::kDynamic) {
      PropertyLabel("Integrator", "Numerical method for this body");
      const std::string current(sim::IntegratorName(body.integrator));
      if (ImGui::BeginCombo("##integrator", current.c_str())) {
        for (sim::Integrator integrator : sim::AllIntegrators()) {
          const std::string name(sim::IntegratorName(integrator));
          if (ImGui::Selectable(name.c_str(), integrator == body.integrator)) {
            body.integrator = integrator;
          }
        }
        ImGui::EndCombo();
      }
    }
    EndProperties();
  }
}

void DrawLaunchConditions(EditorFrame& frame, EntityId id,
                          BodyComponent& body) {
  if (!ImGui::CollapsingHeader("Launch conditions", kOpen)) return;
  HelpText(
      "Initial state restored by Reset. Moving the launch pose relaunches "
      "the object live; velocity edits relaunch it when you release.");
  bool relaunch = false;
  bool pose_changed = false;
  Vec3 position = body.initial.position;
  Quat orientation = body.initial.orientation;
  if (BeginProperties("##launch")) {
    pose_changed |= PropertyVec3("Position", &position, 0.05f, "%.2f");
    pose_changed |= PropertyRotation("Orientation", &orientation);
    if (body.motion == MotionType::kDynamic) {
      // Velocities update the stored launch state while dragging, and the
      // body is relaunched once the edit is released.
      PropertyVec3("Velocity", &body.initial.velocity, 0.05f, "%.2f",
                   "World-frame launch velocity, m/s");
      relaunch |= ImGui::IsItemDeactivatedAfterEdit();
      Vec3 spin_deg = body.initial.angular_velocity * (180.0 / kPi);
      if (PropertyVec3("Angular vel.", &spin_deg, 0.5f, "%.1f",
                       "World-frame angular velocity, deg/s")) {
        body.initial.angular_velocity = spin_deg * (kPi / 180.0);
      }
      relaunch |= ImGui::IsItemDeactivatedAfterEdit();
    }
    EndProperties();
  }
  if (ImGui::Button("Relaunch now")) relaunch = true;
  if (pose_changed) {
    if (body.motion == MotionType::kAutonomous) {
      // The flight area moves with the launch point.
      body.agent.area.center.x += position.x - body.initial.position.x;
      body.agent.area.center.z += position.z - body.initial.position.z;
    }
    SetLaunchPose(*frame.scene, id, position, orientation);
  } else if (relaunch) {
    RelaunchFromInitialPose(*frame.scene, id);
  }
}

void DrawMass(EditorFrame& frame, EntityId id, BodyComponent& body) {
  if (!ImGui::CollapsingHeader("Mass and inertia")) return;
  if (BeginProperties("##mass")) {
    PropertyDouble("Mass", &body.mass.mass, 0.05f, 0.01, 1e5, "%.2f kg");
    Vec3 inertia = body.mass.principal_inertia;
    if (PropertyVec3("Principal I", &inertia, 0.005f, "%.3f",
                     "Moments of inertia about body X, Y, Z (kg m^2)")) {
      body.mass.principal_inertia = Max(inertia, Vec3{1e-5, 1e-5, 1e-5});
    }
    EndProperties();
  }
  if (ImGui::Button("Box inertia from scale")) {
    const Vec3 size = Abs(frame.scene->Get(id)->local.scale);
    body.mass = sim::BoxMassProperties(body.mass.mass, size);
  }
}

void DrawForces(sim::ForceModel& f) {
  if (!ImGui::CollapsingHeader("Forces and torques")) return;
  if (BeginProperties("##forces")) {
    PropertyBool("Gravity", &f.gravity_enabled);
    if (f.gravity_enabled) PropertyVec3("  g", &f.gravity, 0.05f, "%.2f");
    PropertyDouble("Linear drag", &f.linear_drag, 0.005f, 0.0, 100.0,
                   "%.3f N/(m/s)");
    PropertyDouble("Quadratic drag", &f.quadratic_drag, 0.0005f, 0.0, 10.0,
                   "%.4f N/(m/s)^2");
    PropertyVec3("Wind", &f.wind, 0.05f, "%.2f",
                 "Air velocity; drag acts on velocity relative to it");
    PropertyDouble("Thrust", &f.thrust, 0.5f, 0.0, 1e5, "%.1f N",
                   "Constant force along body forward (-Z)");
    PropertyDouble("Burn time", &f.thrust_duration, 0.05f, 0.0, 1e4, "%.2f s");
    PropertyDouble("Lift coeff.", &f.lift_coefficient, 0.001f, 0.0, 10.0,
                   "%.4f N/(m/s)^2", "Lift = c * forward_airspeed^2 (body up)");
    PropertyBool("Spring", &f.spring_enabled);
    if (f.spring_enabled) {
      PropertyVec3("  Anchor", &f.spring_anchor, 0.05f, "%.2f");
      PropertyDouble("  Stiffness", &f.spring_stiffness, 0.05f, 0.0, 1e5,
                     "%.2f N/m");
      PropertyDouble("  Damping", &f.spring_damping, 0.01f, 0.0, 1e4,
                     "%.2f N/(m/s)");
    }
    PropertyBool("Central body", &f.central_enabled,
                 "Inverse-square attraction toward a point");
    if (f.central_enabled) {
      PropertyVec3("  Centre", &f.central_point, 0.05f, "%.2f");
      PropertyDouble("  mu", &f.central_mu, 1.0f, 0.0, 1e9, "%.1f m^3/s^2");
      PropertyDouble("  Softening", &f.central_softening, 0.005f, 0.0, 10.0,
                     "%.3f m");
    }
    PropertyDouble("Weathervane", &f.weathervane, 0.001f, 0.0, 10.0, "%.4f",
                   "Torque turning the nose into the direction of travel");
    PropertyDouble("Angular damping", &f.angular_damping, 0.01f, 0.0, 1e4,
                   "%.3f N m s");
    PropertyVec3("Body torque", &f.body_torque, 0.01f, "%.3f",
                 "Constant torque in body axes (N m)");
    EndProperties();
  }
}

void DrawPath(EditorFrame& frame, EntityId id, BodyComponent& body) {
  if (!ImGui::CollapsingHeader("Kinematic path", kOpen)) return;
  sim::KinematicPath& path = body.path;
  bool changed = false;
  if (BeginProperties("##path")) {
    PropertyLabel("Shape");
    const std::string current(sim::PathShapeName(path.shape));
    if (ImGui::BeginCombo("##shape", current.c_str())) {
      for (sim::PathShape shape : sim::AllPathShapes()) {
        const std::string name(sim::PathShapeName(shape));
        if (ImGui::Selectable(name.c_str(), shape == path.shape)) {
          path.shape = shape;
          changed = true;
        }
      }
      ImGui::EndCombo();
    }
    changed |=
        PropertyDouble("Radius", &path.radius, 0.1f, 0.5, 1000.0, "%.1f m");
    changed |=
        PropertyDouble("Period", &path.period, 0.05f, 0.5, 600.0, "%.2f s");
    if (path.shape == sim::PathShape::kHelix) {
      changed |= PropertyDouble("Climb rate", &path.climb_rate, 0.01f, -50.0,
                                50.0, "%.2f m/s");
    }
    changed |= PropertyBool("Clockwise", &path.clockwise);
    EndProperties();
  }
  // Keep the path passing through the launch point.
  if (changed) RelaunchFromInitialPose(*frame.scene, id);
}

std::string AgentActivity(const BodyComponent& body) {
  const sim::AgentBrain& brain = body.brain;
  if (!brain.has_destination) return "Choosing a destination";
  if (brain.mode == sim::AgentMode::kLoiter) {
    return body.agent.style == sim::FlightStyle::kWing ? "Circling"
                                                       : "Hovering";
  }
  return "Flying to its destination";
}

std::string AgentAvoidance(const Scene& scene, const BodyComponent& body) {
  std::string text;
  const auto add = [&text](std::string_view part) {
    if (!text.empty()) text += ", ";
    text += part;
  };
  if (const Entity* other = scene.Get(body.avoiding)) add(other->name);
  if (body.brain.avoidance.altitude) add("altitude limits");
  if (body.brain.avoidance.boundary) add("area edge");
  return text.empty() ? "nothing" : text;
}

// Live view of the agent's decisions.
void DrawAutopilot(const Scene& scene, const BodyComponent& body) {
  if (!ImGui::CollapsingHeader("Autopilot", kOpen)) return;
  HelpText(
      "Game-style AI: picks its own destinations inside its flight area, "
      "flies each leg at a speed it chooses, sometimes loiters, and steers "
      "clear of the ground, obstacles and other objects. The same seed "
      "always gives the same flight.");
  const sim::AgentBrain& brain = body.brain;
  if (BeginProperties("##autopilot")) {
    PropertyText("Doing", AgentActivity(body));
    if (brain.has_destination) {
      PropertyText(
          "Destination",
          std::format("{:.1f}, {:.1f}, {:.1f} m  ({:.0f} m away)",
                      brain.destination.x, brain.destination.y,
                      brain.destination.z,
                      Distance(brain.destination, body.state.position)));
      PropertyText("Leg speed", std::format("{:.1f} m/s", brain.leg_speed));
      PropertyText(
          brain.mode == sim::AgentMode::kLoiter ? "Loiter time" : "Leg time",
          std::format("{:.1f} / {:.1f} s", brain.mode_time,
                      brain.mode_duration),
          brain.mode == sim::AgentMode::kLoiter
              ? nullptr
              : "A leg that takes this long is abandoned for a new one");
    }
    PropertyText("Keeping clear of", AgentAvoidance(scene, body));
    PropertyText("Legs flown", std::format("{}", brain.legs_completed));
    EndProperties();
  }
}

// Live state of a physical agent's airframe: what the autopilot is doing
// with the actuators and how the air is meeting the vehicle.
void DrawAirframe(const BodyComponent& body) {
  if (body.agent.model != sim::FlightModel::kPhysical) return;
  if (!ImGui::CollapsingHeader("Airframe", kOpen)) return;
  const sim::AgentVehicle& vehicle = body.vehicle;
  const sim::Atmosphere air = sim::StandardAtmosphere(body.state.position.y);
  const Vec3 wind =
      IsFinite(body.forces.wind) ? body.forces.wind : Vec3::Zero();
  const sim::AirData flow = sim::ComputeAirData(body.state, wind, air);
  const sim::AttitudeAngles attitude = sim::AttitudeOf(body.state.orientation);
  const double density_ratio = air.density / sim::kSeaLevelDensity;
  if (BeginProperties("##airframe")) {
    PropertyText("Airspeed", std::format("{:.1f} m/s", flow.airspeed),
                 "Speed through the air (ground speed minus wind)");
    PropertyText("Air density", std::format("{:.3f} kg/m^3", air.density),
                 "Standard atmosphere at the current altitude");
    if (body.agent.style == sim::FlightStyle::kWing) {
      const sim::FixedWingActuators& act = vehicle.actuators;
      const double stall = sim::SeaLevelStallSpeed(vehicle) *
                           std::sqrt(sim::kSeaLevelDensity / air.density);
      PropertyText("Stall speed", std::format("{:.1f} m/s", stall),
                   "Slowest level flight at this altitude; the autopilot "
                   "keeps at least 30% above it");
      PropertyText(
          "Angle of attack",
          std::format("{:.1f} deg  (stalls at {:.0f})", Degrees(flow.alpha),
                      Degrees(vehicle.wing.aero.alpha_stall)));
      PropertyText("Sideslip", std::format("{:.1f} deg", Degrees(flow.beta)));
      PropertyText("Bank / pitch",
                   std::format("{:.1f} / {:.1f} deg", Degrees(attitude.roll),
                               Degrees(attitude.pitch)));
      PropertyText(
          "Throttle",
          std::format(
              "{:.0f} %  ({:.1f} N thrust)", act.throttle * 100.0,
              sim::PropellerThrust(vehicle.wing, act.throttle, flow.airspeed) *
                  density_ratio));
      PropertyText(
          "Elevator / aileron",
          std::format("{:.1f} / {:.1f} deg", Degrees(act.surfaces.elevator),
                      Degrees(act.surfaces.aileron)),
          "Elevator down (positive) lowers the nose; aileron positive rolls "
          "right");
      PropertyText("Rudder",
                   std::format("{:.1f} deg", Degrees(act.surfaces.rudder)));
    } else {
      const sim::MultirotorParams& quad = vehicle.quad;
      double thrust = 0.0;
      for (double w : vehicle.motors.rotor_speed) {
        thrust += quad.thrust_coefficient * w * w * density_ratio;
      }
      const auto& w = vehicle.motors.rotor_speed;
      PropertyText("Rotor speeds",
                   std::format("{:.0f}, {:.0f}, {:.0f}, {:.0f} rad/s", w[0],
                               w[1], w[2], w[3]),
                   "Front-right, front-left, rear-left, rear-right");
      PropertyText(
          "Thrust / weight",
          std::format("{:.2f}",
                      thrust / (quad.mass.mass * sim::kStandardGravity)),
          "Total rotor thrust over the weight: 1 carries it in a hover");
      const double tilt = std::acos(std::clamp(
          Rotate(body.state.orientation, Vec3::UnitY()).y, -1.0, 1.0));
      PropertyText("Tilt", std::format("{:.1f} deg", Degrees(tilt)));
    }
    EndProperties();
  }
}

// Tunable behaviour. Most settings apply on the next step; style and seed
// changes restart the agent's decision making.
void DrawAutopilotSettings(EditorFrame& frame, EntityId id,
                           BodyComponent& body) {
  if (!ImGui::CollapsingHeader("Autopilot settings")) return;
  sim::AgentParams& agent = body.agent;
  const bool wing = agent.style == sim::FlightStyle::kWing;
  bool relaunch = false;
  bool reseed = false;
  bool replan = false;
  if (BeginProperties("##autopilot_settings")) {
    PropertyLabel("Flight style",
                  "Hover: can stop and tilts to accelerate. Wing: never "
                  "stops and banks to turn. Switching loads that style's "
                  "defaults (keeping the area centre and seed).");
    const std::string current(sim::FlightStyleName(agent.style));
    if (ImGui::BeginCombo("##style", current.c_str())) {
      for (sim::FlightStyle style :
           {sim::FlightStyle::kHover, sim::FlightStyle::kWing}) {
        const std::string name(sim::FlightStyleName(style));
        if (ImGui::Selectable(name.c_str(), style == agent.style) &&
            style != agent.style) {
          sim::AgentParams fresh =
              AgentParamsFor(*frame.scene, id, style, agent.model);
          fresh.area.center = agent.area.center;
          fresh.seed = agent.seed;
          agent = fresh;
          relaunch = true;
        }
      }
      ImGui::EndCombo();
    }
    PropertyLabel("Flight model",
                  "Physical: a real airframe (a 1.2 kg quadrotor or a 2 kg "
                  "trainer airplane) with rotor or wing aerodynamics, flown "
                  "by an autopilot that only moves its rotors or throttle "
                  "and control surfaces. Simple: an idealized point mass "
                  "that follows the request within its envelope. Switching "
                  "loads that model's envelope (keeping the area centre and "
                  "seed).");
    const std::string model(sim::FlightModelName(agent.model));
    if (ImGui::BeginCombo("##flight_model", model.c_str())) {
      for (sim::FlightModel option :
           {sim::FlightModel::kPhysical, sim::FlightModel::kSimple}) {
        const std::string name(sim::FlightModelName(option));
        if (ImGui::Selectable(name.c_str(), option == agent.model) &&
            option != agent.model) {
          // Each model has its own envelope; keep the area centre and seed.
          sim::AgentParams fresh =
              AgentParamsFor(*frame.scene, id, agent.style, option);
          fresh.area.center = agent.area.center;
          fresh.seed = agent.seed;
          agent = fresh;
          relaunch = true;
        }
      }
      ImGui::EndCombo();
    }
    PropertyVec3("Area centre", &agent.area.center, 0.1f, "%.1f",
                 "Centre of the flight area (its height is ignored)");
    PropertyDouble("Area radius", &agent.area.radius, 0.2f, 2.0, 5000.0,
                   "%.1f m");
    PropertyDouble("Min altitude", &agent.area.min_altitude, 0.1f, 0.0, 5000.0,
                   "%.1f m");
    PropertyDouble("Max altitude", &agent.area.max_altitude, 0.1f, 0.0, 5000.0,
                   "%.1f m");
    if (wing) {
      PropertyDouble("Min speed", &agent.min_speed, 0.05f, 1.0, 300.0,
                     "%.1f m/s", "Never flies slower than this");
    }
    PropertyDouble("Cruise min", &agent.cruise_speed_min, 0.05f, 0.0, 300.0,
                   "%.1f m/s", "Each leg picks a cruise speed in this range");
    PropertyDouble("Cruise max", &agent.cruise_speed_max, 0.05f, 0.0, 300.0,
                   "%.1f m/s");
    PropertyDouble("Max speed", &agent.max_speed, 0.05f, 0.5, 300.0,
                   "%.1f m/s");
    PropertyDouble("Max accel.", &agent.max_acceleration, 0.05f, 0.1, 100.0,
                   "%.1f m/s^2");
    double tilt = Degrees(agent.max_tilt);
    if (PropertyDouble(wing ? "Max bank" : "Max tilt", &tilt, 0.2f, 1.0, 75.0,
                       "%.0f deg")) {
      agent.max_tilt = Radians(tilt);
    }
    PropertyDouble("Max climb", &agent.max_climb_rate, 0.05f, 0.1, 100.0,
                   "%.1f m/s");
    double chance = agent.loiter_chance * 100.0;
    if (PropertyDouble("Loiter chance", &chance, 0.5f, 0.0, 100.0, "%.0f %%",
                       "Chance of loitering at each destination")) {
      agent.loiter_chance = chance / 100.0;
    }
    PropertyDouble("Loiter min", &agent.loiter_min, 0.05f, 0.0, 600.0,
                   "%.1f s");
    PropertyDouble("Loiter max", &agent.loiter_max, 0.05f, 0.0, 600.0,
                   "%.1f s");
    PropertyDouble("Meander", &agent.meander, 0.01f, 0.0, 1.0, "%.2f",
                   "How much the heading wanders on the way");
    PropertyDouble("Clearance", &agent.clearance, 0.05f, 0.0, 100.0, "%.1f m",
                   "Gap kept between the agent and obstacles");
    PropertyDouble("Look-ahead", &agent.lookahead, 0.05f, 0.2, 30.0, "%.1f s",
                   "Seconds of travel scanned ahead for obstacles");
    PropertyLabel("Seed", "The same seed always gives the same flight");
    ImGui::SetNextItemWidth(-1.0f);
    reseed |= ImGui::InputScalar("##seed", ImGuiDataType_U64, &agent.seed);
    EndProperties();
  }
  if (ImGui::Button("New seed")) {
    agent.seed = MixSeed(agent.seed, 0x5EEDu);
    reseed = true;
  }
  ImGui::SameLine();
  if (ImGui::Button("Centre area here")) {
    agent.area.center = {body.state.position.x, 0.0, body.state.position.z};
  }
  ImGui::SameLine();
  if (ImGui::Button("Re-plan")) replan = true;

  if (relaunch) {
    RelaunchFromInitialPose(*frame.scene, id);
  } else if (reseed) {
    // A new random stream, starting from where the agent is now.
    sim::ResetBrain(agent,
                    {.position = body.state.position,
                     .velocity = body.state.velocity,
                     .acceleration = body.acceleration,
                     .orientation = body.state.orientation},
                    body.brain);
  } else if (replan) {
    sim::Replan(body.brain);
  }
}

void DrawGround(BodyComponent& body) {
  if (!ImGui::CollapsingHeader("Ground contact")) return;
  if (BeginProperties("##ground")) {
    PropertyBool("Enabled", &body.ground_contact);
    PropertyDouble("Contact radius", &body.collision_radius, 0.01f, 0.01, 100.0,
                   "%.2f m",
                   "Distance from origin to the ground "
                   "when resting");
    PropertyDouble("Restitution", &body.restitution, 0.01f, 0.0, 1.0, "%.2f",
                   "Fraction of normal speed kept on bounce");
    PropertyDouble("Friction (mu)", &body.ground_friction, 0.01f, 0.0, 2.0,
                   "%.2f", "Coulomb friction coefficient");
    EndProperties();
  }
}

void DrawTrail(TrailComponent& trail) {
  if (!ImGui::CollapsingHeader("Trail")) return;
  if (BeginProperties("##trail")) {
    PropertyBool("Enabled", &trail.enabled);
    PropertyColor("Colour", &trail.color);
    PropertyDouble("Interval", &trail.sample_interval, 0.005f, 0.005, 5.0,
                   "%.3f s", "Simulated time between trail points");
    // Resizing reallocates the buffer, so the dragged value is kept in
    // widget storage and applied when the drag ends.
    const ImGuiID key = ImGui::GetID("##max_points_pending");
    ImGuiStorage* storage = ImGui::GetStateStorage();
    double length = storage->GetBool(key, false)
                        ? storage->GetFloat(key + 1)
                        : static_cast<double>(trail.points.capacity());
    PropertyDouble("Max points", &length, 5.0f, 2.0, 20000.0, "%.0f");
    storage->SetBool(key, ImGui::IsItemActive());
    storage->SetFloat(key + 1, static_cast<float>(length));
    if (ImGui::IsItemDeactivatedAfterEdit()) {
      trail.points.SetCapacity(static_cast<std::size_t>(length));
    }
    EndProperties();
  }
  if (ImGui::Button("Clear trail")) trail.points.Clear();
}

void DrawMaterial(Entity& entity) {
  if (!entity.render.has_value() || !ImGui::CollapsingHeader("Appearance")) {
    return;
  }
  RenderComponent& render = *entity.render;
  if (BeginProperties("##material")) {
    PropertyText("Mesh", MeshKindName(render.mesh));
    PropertyBool("Visible", &render.visible);
    PropertyBool("Casts shadow", &render.cast_shadow);
    PropertyColor("Base colour", &render.material.base_color);
    double specular = render.material.specular;
    if (PropertyDouble("Specular", &specular, 0.01f, 0.0, 1.0, "%.2f")) {
      render.material.specular = static_cast<float>(specular);
    }
    double shininess = render.material.shininess;
    if (PropertyDouble("Shininess", &shininess, 0.5f, 1.0, 512.0, "%.0f")) {
      render.material.shininess = static_cast<float>(shininess);
    }
    double emissive = render.material.emissive;
    if (PropertyDouble("Emissive", &emissive, 0.01f, 0.0, 1.0, "%.2f")) {
      render.material.emissive = static_cast<float>(emissive);
    }
    EndProperties();
  }
}

void DrawObserver(EditorFrame& frame, EntityId id) {
  if (!ImGui::CollapsingHeader("First person", kOpen)) return;
  HelpText(
      "A standing figure to look through. In first person, WASD walks "
      "(Shift runs, Ctrl creeps), right-drag looks around, the wheel sets "
      "walking speed and F turns to face the selection. The figure walks "
      "with you; other camera modes show where it is.");
  if (ImGui::Button("Look through (4)")) LookThrough(frame, id);
}

void DrawAddBody(EditorFrame& frame, EntityId id) {
  if (!ImGui::CollapsingHeader("Simulation", kOpen)) return;
  HelpText("This object is static. Give it a motion preset to simulate it.");
  const float right_edge =
      ImGui::GetCursorScreenPos().x + ImGui::GetContentRegionAvail().x;
  for (MotionPreset preset : AllMotionPresets()) {
    const std::string name(MotionPresetName(preset));
    if (ImGui::Button(name.c_str())) {
      ApplyMotionPreset(*frame.scene, id, preset);
    }
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
      const std::string description(MotionPresetDescription(preset));
      ImGui::SetTooltip("%s", description.c_str());
    }
    // Flow buttons across the panel width.
    const float next = ImGui::GetItemRectMax().x +
                       ImGui::GetStyle().ItemSpacing.x + 110.0f * Dpi();
    if (next < right_edge) {
      ImGui::SameLine();
    }
  }
  ImGui::NewLine();
}

}  // namespace

void DrawInspectorPanel(EditorFrame& frame) {
  if (!ImGui::Begin(kInspectorWindow)) {
    ImGui::End();
    return;
  }
  const EntityId id = frame.selection->selected;
  Entity* entity = frame.scene->Get(id);
  if (entity == nullptr) {
    HelpText(
        "Select an object in the viewport or the hierarchy to inspect "
        "its transform, motion and physical parameters.");
    ImGui::End();
    return;
  }

  DrawHeader(frame, id, *entity);
  // The header may have deleted the entity.
  entity = frame.scene->Get(id);
  if (entity == nullptr) {
    ImGui::End();
    return;
  }
  ImGui::Separator();
  DrawTransform(frame, id, *entity);
  if (entity->body.has_value()) {
    BodyComponent& body = *entity->body;
    DrawLiveState(body);
    DrawSimulationSettings(frame, id, body);
    DrawLaunchConditions(frame, id, body);
    if (body.motion == MotionType::kKinematic) {
      DrawPath(frame, id, body);
    } else if (body.motion == MotionType::kAutonomous) {
      DrawAutopilot(*frame.scene, body);
      DrawAirframe(body);
      DrawAutopilotSettings(frame, id, body);
    } else {
      DrawMass(frame, id, body);
      DrawForces(body.forces);
      DrawGround(body);
    }
  } else if (IsObserver(*entity)) {
    DrawObserver(frame, id);
  } else {
    DrawAddBody(frame, id);
  }
  if (entity->trail.has_value()) DrawTrail(*entity->trail);
  if (entity->spin.has_value() && ImGui::CollapsingHeader("Spin")) {
    if (BeginProperties("##spin")) {
      double rpm = entity->spin->rate * 60.0 / kTwoPi;
      if (PropertyDouble("Rate", &rpm, 5.0f, -20000.0, 20000.0, "%.0f rpm")) {
        entity->spin->rate = rpm * kTwoPi / 60.0;
      }
      EndProperties();
    }
  }
  DrawMaterial(*entity);
  ImGui::End();
}

}  // namespace aerolab::editor
