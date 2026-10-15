#ifndef AEROLAB_GEOMETRY_MATERIAL_H_
#define AEROLAB_GEOMETRY_MATERIAL_H_

#include "geometry/color.h"

namespace aerolab {

// Surface parameters for the Blinn-Phong lighting model. The base colour
// multiplies each vertex's own tint, so one composite mesh can be recoloured.
struct Material {
  Color base_color = colors::kWhite;
  // Strength of the specular highlight, 0 (matte) to 1 (glossy).
  float specular = 0.35f;
  // Blinn-Phong exponent; larger values give tighter highlights.
  float shininess = 48.0f;
  // Fraction of the base colour emitted regardless of lighting.
  float emissive = 0.0f;
};

}  // namespace aerolab

#endif  // AEROLAB_GEOMETRY_MATERIAL_H_
