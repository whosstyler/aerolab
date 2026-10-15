#ifndef AEROLAB_SIM_AERODYNAMICS_H_
#define AEROLAB_SIM_AERODYNAMICS_H_

#include "math/scalar.h"
#include "math/vec3.h"
#include "sim/atmosphere.h"
#include "sim/force_model.h"
#include "sim/rigid_body.h"

namespace aerolab::sim {

// Axes. The engine's body frame is -Z forward, +Y up, +X right. Aerodynamics
// uses the aviation frame: x forward, y right, z down, so
//   x_aero = -z_body, y_aero = +x_body, z_aero = -y_body.
// Angle of attack is positive when the air meets the wing from below, and
// sideslip is positive when the air comes from the right. Rates follow the
// aviation frame: p rolls the right wing down, q pitches the nose up, r
// yaws the nose right.

// Control surface deflections, radians. Each sign names the motion it
// produces:
//   elevator > 0  nose down (trailing edge down)
//   aileron  > 0  roll right (right wing down)
//   rudder   > 0  nose right
struct ControlSurfaces {
  double elevator = 0.0;
  double aileron = 0.0;
  double rudder = 0.0;
};

// Non-dimensional aerodynamic model of a conventional airplane about its
// centre of mass, in stability-derivative form: linear in angle of attack
// up to the stall, blending to a flat plate beyond it. Rate derivatives are
// per unit non-dimensional rate (p b / 2V, q c / 2V, r b / 2V). Defaults
// describe a small trainer-style UAV (1.8 m span).
struct AeroCoefficients {
  double area = 0.45;   // S, wing reference area, m^2
  double span = 1.8;    // b, m
  double chord = 0.25;  // c, mean aerodynamic chord, m

  // Lift: CL = cl0 + cl_alpha a + cl_elevator de below the stall.
  double cl0 = 0.25;
  double cl_alpha = 4.8;  // per rad
  // Where the flow separates; lift peaks a degree or two before it.
  double alpha_stall = Radians(14.0);
  double cl_elevator = 0.3;  // per rad

  // Drag: CD = cd0 + induced_drag CL^2 below the stall.
  double cd0 = 0.03;
  double induced_drag = 0.055;  // 1 / (pi e AR)

  // Side force.
  double cy_beta = -0.4;
  double cy_rudder = -0.15;  // Tail pushed left when the nose yaws right.

  // Pitching moment.
  double cm0 = 0.02;
  double cm_alpha = -0.6;  // Negative: statically stable.
  double cm_q = -10.0;     // Pitch damping.
  double cm_elevator = -1.0;

  // Rolling moment.
  double cl_beta = -0.08;  // Dihedral effect.
  double cl_p = -0.45;     // Roll damping.
  double cl_r = 0.1;
  double cl_aileron = 0.25;

  // Yawing moment.
  double cn_beta = 0.06;  // Weathervane stability.
  double cn_r = -0.12;    // Yaw damping.
  double cn_p = -0.03;
  double cn_rudder = 0.08;
};

// Flow conditions around a body.
struct AirData {
  Vec3 relative_wind;  // Velocity of the body through the air (v - wind), m/s
  double airspeed = 0.0;
  double alpha = 0.0;             // Angle of attack, rad
  double beta = 0.0;              // Sideslip, rad
  double dynamic_pressure = 0.0;  // 0.5 rho V^2, Pa
  double density = kSeaLevelDensity;
  double mach = 0.0;
};

AirData ComputeAirData(const RigidBodyState& state, const Vec3& wind,
                       const Atmosphere& atmosphere);

// Lift and drag coefficients at an angle of attack (stall blended in).
double LiftCoefficient(const AeroCoefficients& aero, double alpha);
double DragCoefficient(const AeroCoefficients& aero, double alpha);

// Aerodynamic force and moment on the airframe, world frame, about the
// centre of mass. Zero when the airspeed is negligible.
Wrench ComputeAeroWrench(const AeroCoefficients& aero,
                         const RigidBodyState& state, const AirData& air,
                         const ControlSurfaces& surfaces);

}  // namespace aerolab::sim

#endif  // AEROLAB_SIM_AERODYNAMICS_H_
