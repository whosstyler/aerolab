#include "render/shader_sources.h"

#include <string_view>

namespace aerolab::shaders {

// --- Sky and horizon ------------------------------------------------------
// A fullscreen triangle generated from gl_VertexID. Each pixel reconstructs
// its world-space view direction from the inverse of (projection * rotation)
// and shades a gradient sky above the horizon and a hazy ground band below.

const std::string_view kSkyVertex = R"(#version 330 core
out vec2 v_ndc;
void main() {
  vec2 corner = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
  v_ndc = corner * 2.0 - 1.0;
  gl_Position = vec4(v_ndc, 0.0, 1.0);
}
)";

const std::string_view kSkyFragment = R"(#version 330 core
in vec2 v_ndc;
out vec4 frag_color;
uniform mat4 u_inverse_rotation_projection;
uniform vec3 u_sun_direction;
uniform vec3 u_sun_color;
uniform vec3 u_sky_zenith;
uniform vec3 u_sky_horizon;
uniform vec3 u_ground_horizon;
uniform vec3 u_ground_color;
void main() {
  vec4 far_point = u_inverse_rotation_projection * vec4(v_ndc, 1.0, 1.0);
  vec3 dir = normalize(far_point.xyz / far_point.w);
  float h = dir.y;
  vec3 sky = mix(u_sky_horizon, u_sky_zenith, pow(clamp(h, 0.0, 1.0), 0.5));
  vec3 ground = mix(u_ground_horizon, u_ground_color,
                    pow(clamp(-h, 0.0, 1.0), 0.4));
  vec3 color = mix(ground, sky, smoothstep(-0.0015, 0.0015, h));
  float s = max(dot(dir, u_sun_direction), 0.0);
  color += u_sun_color * (pow(s, 1200.0) * 1.2 + pow(s, 16.0) * 0.10) *
           step(0.0, h);
  frag_color = vec4(color, 1.0);
}
)";

// --- Ground plane and infinite grid ---------------------------------------
// A large quad on y = 0 that follows the camera horizontally. Grid lines are
// computed analytically per pixel with screen-space derivatives, so they stay
// anti-aliased and thin at any distance, then fade into the horizon haze.

const std::string_view kGroundVertex = R"(#version 330 core
layout(location = 0) in vec3 a_position;
uniform mat4 u_view_projection;
uniform vec3 u_camera_position;
uniform float u_extent;
out vec3 v_world;
void main() {
  vec3 world = vec3(u_camera_position.x + a_position.x * u_extent, 0.0,
                    u_camera_position.z + a_position.z * u_extent);
  v_world = world;
  gl_Position = u_view_projection * vec4(world, 1.0);
}
)";

const std::string_view kGroundFragment = R"(#version 330 core
in vec3 v_world;
out vec4 frag_color;
uniform vec3 u_camera_position;
uniform vec3 u_ground_color;
uniform vec3 u_ground_horizon;
uniform float u_ground_opacity;
uniform int u_show_grid;
uniform float u_cell;
uniform float u_major_every;
uniform float u_fade_distance;
uniform vec4 u_minor_color;
uniform vec4 u_major_color;
uniform vec3 u_axis_x_color;
uniform vec3 u_axis_z_color;
uniform float u_light;
uniform float u_fog_density;

// Coverage of grid lines with spacing `cell` at this pixel (1 = on a line).
float GridCoverage(vec2 coord, float cell) {
  vec2 scaled = coord / cell;
  vec2 width = fwidth(scaled);
  vec2 grid = abs(fract(scaled - 0.5) - 0.5) / max(width, vec2(1e-6));
  float line = min(grid.x, grid.y);
  // Fade a level out once its lines get denser than about two pixels.
  float density = max(width.x, width.y);
  return (1.0 - min(line, 1.0)) * (1.0 - smoothstep(0.25, 0.5, density));
}

float AxisCoverage(float coord) {
  float width = max(fwidth(coord), 1e-6);
  return 1.0 - min(abs(coord) / (width * 1.5), 1.0);
}

void main() {
  vec2 coord = v_world.xz;
  float distance = length(v_world - u_camera_position);
  vec3 color = u_ground_color * u_light;
  if (u_show_grid != 0) {
    float fade = 1.0 - smoothstep(u_fade_distance * 0.35, u_fade_distance,
                                  distance);
    float minor = GridCoverage(coord, u_cell) * u_minor_color.a;
    float major = GridCoverage(coord, u_cell * u_major_every) *
                  u_major_color.a;
    color = mix(color, u_minor_color.rgb, minor * fade);
    color = mix(color, u_major_color.rgb, major * fade);
    // World X axis runs along z = 0, world Z axis along x = 0.
    float axis_fade = 1.0 - smoothstep(u_fade_distance, u_fade_distance * 2.5,
                                       distance);
    color = mix(color, u_axis_x_color, AxisCoverage(coord.y) * axis_fade);
    color = mix(color, u_axis_z_color, AxisCoverage(coord.x) * axis_fade);
  }
  float haze = 1.0 - exp(-pow(distance * u_fog_density, 1.5));
  color = mix(color, u_ground_horizon, clamp(haze, 0.0, 1.0));
  frag_color = vec4(color, u_ground_opacity);
}
)";

// --- Lit meshes -------------------------------------------------------------
// Blinn-Phong with a directional sun, hemispheric ambient (sky above, ground
// below), optional rim highlight for selection, and distance fog. Lighting is
// computed in linear space; the material and vertex tints are multiplied in
// sRGB first, a common simplification for flat-coloured models. The sky and
// ground passes above stay in display (sRGB) space on purpose: they are
// stylized backdrops rather than lit surfaces.

const std::string_view kMeshVertex = R"(#version 330 core
layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;
layout(location = 2) in vec3 a_color;
uniform mat4 u_model;
uniform mat3 u_normal_matrix;
uniform mat4 u_view_projection;
out vec3 v_world;
out vec3 v_normal;
out vec3 v_color;
void main() {
  vec4 world = u_model * vec4(a_position, 1.0);
  v_world = world.xyz;
  v_normal = u_normal_matrix * a_normal;
  v_color = a_color;
  gl_Position = u_view_projection * world;
}
)";

const std::string_view kMeshFragment = R"(#version 330 core
in vec3 v_world;
in vec3 v_normal;
in vec3 v_color;
out vec4 frag_color;
uniform vec3 u_camera_position;
uniform vec3 u_sun_direction;
uniform vec3 u_sun_color;
uniform float u_sun_intensity;
uniform float u_ambient;
uniform vec3 u_sky_color;
uniform vec3 u_ground_color;
uniform vec4 u_base_color;
uniform float u_specular;
uniform float u_shininess;
uniform float u_emissive;
uniform float u_highlight;
uniform vec3 u_highlight_color;
uniform vec3 u_fog_sky;
uniform vec3 u_fog_ground;
uniform float u_fog_density;

vec3 ToLinear(vec3 c) { return pow(c, vec3(2.2)); }
vec3 ToSrgb(vec3 c) { return pow(max(c, vec3(0.0)), vec3(1.0 / 2.2)); }

void main() {
  vec3 n = normalize(v_normal);
  if (!gl_FrontFacing) n = -n;  // Thin parts are visible from both sides.
  vec3 v = normalize(u_camera_position - v_world);
  vec3 l = normalize(u_sun_direction);
  vec3 h = normalize(l + v);

  vec3 albedo = ToLinear(u_base_color.rgb * v_color);
  vec3 sun = ToLinear(u_sun_color) * u_sun_intensity;
  float diffuse = max(dot(n, l), 0.0);
  float specular = diffuse > 0.0
      ? pow(max(dot(n, h), 0.0), u_shininess) * u_specular : 0.0;
  vec3 hemisphere = mix(ToLinear(u_ground_color), ToLinear(u_sky_color),
                        n.y * 0.5 + 0.5) * u_ambient;

  vec3 color = albedo * (hemisphere + sun * diffuse) + sun * specular +
               albedo * u_emissive;
  float rim = pow(1.0 - max(dot(n, v), 0.0), 2.5);
  vec3 highlight = ToLinear(u_highlight_color);
  color += highlight * u_highlight * (rim * 1.2 + 0.06);

  // Fog toward the sky haze above the horizon and the ground haze below,
  // so distant objects blend into whatever lies behind them.
  float distance = length(u_camera_position - v_world);
  vec3 view_dir = (v_world - u_camera_position) / max(distance, 1e-4);
  vec3 fog_color = mix(u_fog_ground, u_fog_sky,
                       smoothstep(-0.03, 0.03, view_dir.y));
  float fog = 1.0 - exp(-pow(distance * u_fog_density, 1.5));
  color = mix(color, ToLinear(fog_color), clamp(fog, 0.0, 1.0));
  frag_color = vec4(ToSrgb(color), u_base_color.a);
}
)";

// --- Debug lines ------------------------------------------------------------

const std::string_view kLineVertex = R"(#version 330 core
layout(location = 0) in vec3 a_position;
layout(location = 1) in vec4 a_color;
uniform mat4 u_view_projection;
uniform vec3 u_camera_position;
out vec4 v_color;
out vec3 v_offset;
void main() {
  v_color = a_color;
  v_offset = a_position - u_camera_position;
  gl_Position = u_view_projection * vec4(a_position, 1.0);
}
)";

const std::string_view kLineFragment = R"(#version 330 core
in vec4 v_color;
in vec3 v_offset;
out vec4 frag_color;
uniform vec3 u_fog_sky;
uniform vec3 u_fog_ground;
uniform float u_fog_density;
void main() {
  float distance = length(v_offset);
  vec3 fog_color = mix(u_fog_ground, u_fog_sky,
                       smoothstep(-0.03, 0.03, v_offset.y / max(distance, 1e-4)));
  float fog = 1.0 - exp(-pow(distance * u_fog_density, 1.5));
  frag_color = vec4(mix(v_color.rgb, fog_color, clamp(fog, 0.0, 1.0)),
                    v_color.a);
}
)";

// --- Blob shadows -----------------------------------------------------------
// A soft dark disc under each object; cheap but gives strong depth cues.

const std::string_view kShadowVertex = R"(#version 330 core
layout(location = 0) in vec3 a_position;
uniform mat4 u_model;
uniform mat4 u_view_projection;
out vec2 v_local;
void main() {
  v_local = a_position.xz;
  gl_Position = u_view_projection * u_model * vec4(a_position, 1.0);
}
)";

const std::string_view kShadowFragment = R"(#version 330 core
in vec2 v_local;
out vec4 frag_color;
uniform float u_strength;
void main() {
  float r = length(v_local);
  float alpha = (1.0 - smoothstep(0.15, 1.0, r)) * u_strength;
  frag_color = vec4(0.0, 0.0, 0.0, alpha);
}
)";

}  // namespace aerolab::shaders
