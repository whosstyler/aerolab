#include "sim/atmosphere.h"

#include <algorithm>
#include <cmath>

namespace aerolab::sim {
namespace {

// ISA constants.
constexpr double kSeaLevelTemperature = 288.15;  // K
constexpr double kSeaLevelPressure = 101325.0;   // Pa
constexpr double kLapseRate = 0.0065;            // K/m in the troposphere
constexpr double kTropopause = 11000.0;          // m
constexpr double kTopAltitude = 20000.0;         // m, top of the model
constexpr double kGasConstant = 287.05287;       // J/(kg K), dry air
constexpr double kHeatRatio = 1.4;
constexpr double kGravity = 9.80665;  // m/s^2 (standard)

}  // namespace

Atmosphere StandardAtmosphere(double altitude) {
  const double h =
      std::isfinite(altitude) ? std::clamp(altitude, 0.0, kTopAltitude) : 0.0;
  Atmosphere air;
  const double exponent = kGravity / (kGasConstant * kLapseRate);
  if (h <= kTropopause) {
    air.temperature = kSeaLevelTemperature - kLapseRate * h;
    air.pressure = kSeaLevelPressure *
                   std::pow(air.temperature / kSeaLevelTemperature, exponent);
  } else {
    // Isothermal above the tropopause: pressure decays exponentially.
    const double tropopause_temperature =
        kSeaLevelTemperature - kLapseRate * kTropopause;
    const double tropopause_pressure =
        kSeaLevelPressure *
        std::pow(tropopause_temperature / kSeaLevelTemperature, exponent);
    air.temperature = tropopause_temperature;
    air.pressure =
        tropopause_pressure * std::exp(-kGravity * (h - kTropopause) /
                                       (kGasConstant * tropopause_temperature));
  }
  air.density = air.pressure / (kGasConstant * air.temperature);
  air.speed_of_sound = std::sqrt(kHeatRatio * kGasConstant * air.temperature);
  return air;
}

}  // namespace aerolab::sim
