#ifndef AEROLAB_WORLD_PRESETS_H_
#define AEROLAB_WORLD_PRESETS_H_

#include <array>
#include <string_view>

#include "geometry/color.h"
#include "math/vec3.h"
#include "scene/entity.h"
#include "scene/scene.h"
#include "sim/integrator.h"

namespace aerolab {

// Ready-made motion setups built from the generic force-model terms. Each
// is a classroom physics scenario, not a model of any real system.
enum class MotionPreset {
  kBallistic,    // Launched at an angle; gravity + drag, nose follows flow.
  kPowered,      // Short constant-thrust burn, then a ballistic coast.
  kGlider,       // Wing lift and drag: glides, with phugoid oscillation.
  kOrbit,        // Circular orbit around an inverse-square attractor.
  kOscillator,   // 3D isotropic spring: an elliptical loop.
  kTumble,       // Torque-free spin about the intermediate axis.
  kCirclePath,   // Kinematic banked circle.
  kFigureEight,  // Kinematic figure-eight.
  kHelix,        // Kinematic climbing spiral.
  kCount,
};

inline constexpr int kMotionPresetCount =
    static_cast<int>(MotionPreset::kCount);

std::string_view MotionPresetName(MotionPreset preset);
std::string_view MotionPresetDescription(MotionPreset preset);

constexpr std::array<MotionPreset, kMotionPresetCount> AllMotionPresets() {
  std::array<MotionPreset, kMotionPresetCount> all{};
  for (int i = 0; i < kMotionPresetCount; ++i) {
    all[i] = static_cast<MotionPreset>(i);
  }
  return all;
}

// Adds (or replaces) a body and trail on `id`, configured for `preset` and
// launched from the entity's current world pose along its forward axis.
void ApplyMotionPreset(Scene& scene, EntityId id, MotionPreset preset);

// Populates an empty scene with a showcase of every preset.
void BuildDemoScene(Scene& scene);

// Adds one orbiting body per integrator, all starting from identical
// conditions, so their trails diverge visibly. Returns the group entity.
EntityId SpawnIntegratorComparison(Scene& scene, const Vec3& center);

// Distinct colour per integrator, shared by trails and UI plots.
Color IntegratorColor(sim::Integrator integrator);

}  // namespace aerolab

#endif  // AEROLAB_WORLD_PRESETS_H_
