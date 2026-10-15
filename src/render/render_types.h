#ifndef AEROLAB_RENDER_RENDER_TYPES_H_
#define AEROLAB_RENDER_RENDER_TYPES_H_

#include <cstdint>

#include "geometry/color.h"
#include "geometry/material.h"
#include "geometry/mesh_library.h"
#include "math/mat4.h"
#include "math/vec3.h"

namespace aerolab {

// Everything the renderer needs to know about the camera for one frame.
struct RenderView {
  Mat4 view;
  Mat4 projection;
  Vec3 camera_position;
  int width = 1;
  int height = 1;
  double near_plane = 0.05;
  double far_plane = 5000.0;
};

// One mesh instance to draw. The application builds these from the scene;
// the renderer knows nothing about scenes or simulation. Meshes are treated
// as opaque: the pass writes depth and does not sort, so a material alpha
// below 1 only gives a rough "ghost" look, not correct transparency.
struct RenderItem {
  MeshKind mesh = MeshKind::kCube;
  Mat4 model;
  Material material;
  // Rim highlight strength in [0, 1] (selection = 1, hover ~ 0.5).
  float highlight = 0.0f;
  Color highlight_color = colors::kSelection;
  bool cast_shadow = true;
};

// User-adjustable look of the world. Colours are sRGB.
struct RenderSettings {
  // Sky and horizon.
  Color sky_zenith = FromHex(0x2E5A8C);
  Color sky_horizon = FromHex(0xB8CCDD);
  Color ground_horizon = FromHex(0x8E9AA3);
  Color ground_color = FromHex(0x4A5560);

  // Ground plane and world grid.
  bool show_ground = true;
  float ground_opacity = 1.0f;
  bool show_grid = true;
  float grid_cell_size = 1.0f;  // Metres per minor cell.
  int grid_major_every = 10;    // Minor cells per major line.
  float grid_fade_distance = 350.0f;
  Color grid_minor_color = FromHex(0xC6CFD6, 0.22f);
  Color grid_major_color = FromHex(0xE3E8EC, 0.45f);

  // Lighting.
  float sun_azimuth_degrees = 135.0f;   // Clockwise from +Z toward +X.
  float sun_elevation_degrees = 48.0f;  // Above the horizon.
  Color sun_color = FromHex(0xFFF4E0);
  float sun_intensity = 1.1f;
  float ambient_intensity = 0.45f;

  // Atmosphere and effects.
  bool fog = true;
  float fog_density = 0.0012f;  // Per metre.
  bool blob_shadows = true;
  bool wireframe = false;
  bool show_axis_gizmo = true;
  int msaa_samples = 4;
};

// Per-frame counters reported to the profiler panel.
struct RenderStats {
  int draw_calls = 0;
  std::int64_t triangles = 0;
  std::int64_t line_segments = 0;
};

// Unit vector pointing from the scene toward the sun.
Vec3 SunDirection(const RenderSettings& settings);

}  // namespace aerolab

#endif  // AEROLAB_RENDER_RENDER_TYPES_H_
