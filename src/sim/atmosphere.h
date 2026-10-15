#ifndef AEROLAB_SIM_ATMOSPHERE_H_
#define AEROLAB_SIM_ATMOSPHERE_H_

namespace aerolab::sim {

// Air properties at one altitude.
struct Atmosphere {
  double temperature = 288.15;     // K
  double pressure = 101325.0;      // Pa
  double density = 1.225;          // kg/m^3
  double speed_of_sound = 340.29;  // m/s
};

inline constexpr double kSeaLevelDensity = 1.225;  // kg/m^3

// International Standard Atmosphere: a troposphere cooling at 6.5 K/km up
// to 11 km, then an isothermal layer to 20 km. Altitudes outside
// [0, 20000] m are clamped to that range.
Atmosphere StandardAtmosphere(double altitude);

}  // namespace aerolab::sim

#endif  // AEROLAB_SIM_ATMOSPHERE_H_
