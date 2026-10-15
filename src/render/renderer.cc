#include "render/renderer.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <span>
#include <utility>

#include "geometry/primitives.h"
#include "math/intersection.h"
#include "math/mat3.h"
#include "math/projection.h"
#include "math/scalar.h"
#include "render/shader_sources.h"

namespace aerolab {
namespace {

// Strips translation from a view matrix, leaving only camera rotation.
Mat4 RotationOnly(const Mat4& view) {
  return Mat4::FromMat3(view.UpperLeft3x3());
}

MeshData MakeGroundQuad() {
  MeshBuilder builder;
  builder.AddFlatQuad({-1.0, 0.0, 1.0}, {1.0, 0.0, 1.0}, {1.0, 0.0, -1.0},
                      {-1.0, 0.0, -1.0});
  return builder.Build();
}

std::optional<ShaderProgram> Compile(std::string_view name,
                                     std::string_view vertex,
                                     std::string_view fragment) {
  return ShaderProgram::Create(name, vertex, fragment);
}

}  // namespace

Vec3 SunDirection(const RenderSettings& settings) {
  const double azimuth = Radians(settings.sun_azimuth_degrees);
  const double elevation = Radians(settings.sun_elevation_degrees);
  return Normalized(Vec3{std::sin(azimuth) * std::cos(elevation),
                         std::sin(elevation),
                         std::cos(azimuth) * std::cos(elevation)});
}

std::unique_ptr<Renderer> Renderer::Create() {
  std::unique_ptr<Renderer> renderer(new Renderer());

  std::optional<ShaderProgram> sky =
      Compile("sky", shaders::kSkyVertex, shaders::kSkyFragment);
  std::optional<ShaderProgram> ground =
      Compile("ground", shaders::kGroundVertex, shaders::kGroundFragment);
  std::optional<ShaderProgram> mesh =
      Compile("mesh", shaders::kMeshVertex, shaders::kMeshFragment);
  std::optional<ShaderProgram> line =
      Compile("line", shaders::kLineVertex, shaders::kLineFragment);
  std::optional<ShaderProgram> shadow =
      Compile("shadow", shaders::kShadowVertex, shaders::kShadowFragment);
  if (!sky || !ground || !mesh || !line || !shadow) return nullptr;
  renderer->sky_shader_ = std::move(*sky);
  renderer->ground_shader_ = std::move(*ground);
  renderer->mesh_shader_ = std::move(*mesh);
  renderer->line_shader_ = std::move(*line);
  renderer->shadow_shader_ = std::move(*shadow);

  for (MeshKind kind : AllMeshKinds()) {
    renderer->meshes_[static_cast<std::size_t>(kind)] =
        GpuMesh::Upload(BuildMesh(kind));
  }
  renderer->ground_quad_ = GpuMesh::Upload(MakeGroundQuad());
  renderer->shadow_disc_ = GpuMesh::Upload(MakeDisc(1.0, 32));
  renderer->lines_ = LineBatch::Create();
  renderer->fullscreen_vao_ = EmptyVertexArray::Create();

  if (!CheckGlErrors("Renderer::Create")) return nullptr;
  return renderer;
}

const Aabb& Renderer::MeshBounds(MeshKind kind) const {
  return meshes_[static_cast<std::size_t>(kind)].bounds();
}

void Renderer::Render(const RenderView& view, std::span<const RenderItem> items,
                      const DebugDrawList& world_lines,
                      const DebugDrawList& overlay_lines,
                      const RenderSettings& settings) {
  stats_ = RenderStats();
  // Establish a known baseline: other passes (e.g. the UI) share the context.
  glViewport(0, 0, view.width, view.height);
  glDisable(GL_SCISSOR_TEST);
  glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
  glDepthMask(GL_TRUE);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClearDepth(1.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  DrawSky(view, settings);
  DrawMeshes(view, items, settings);
  if (settings.show_ground) DrawGround(view, settings);
  if (settings.blob_shadows && settings.show_ground) DrawShadows(view, items);
  DrawLines(view, world_lines, settings, /*depth_tested=*/true);
  DrawLines(view, overlay_lines, settings, /*depth_tested=*/false);
  if (settings.show_axis_gizmo) DrawAxisGizmo(view);

  // Leave the pipeline in GL's default state for whoever draws next.
  glDisable(GL_BLEND);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_CULL_FACE);
  glDepthMask(GL_TRUE);
  glBindVertexArray(0);
  glUseProgram(0);
}

void Renderer::DrawSky(const RenderView& view, const RenderSettings& settings) {
  glDisable(GL_DEPTH_TEST);
  glDepthMask(GL_FALSE);
  glDisable(GL_BLEND);
  glDisable(GL_CULL_FACE);

  const std::optional<Mat4> inverse =
      Inverse(view.projection * RotationOnly(view.view));
  sky_shader_.Use();
  sky_shader_.SetMat4("u_inverse_rotation_projection",
                      inverse.value_or(Mat4::Identity()));
  sky_shader_.SetVec3("u_sun_direction", SunDirection(settings));
  sky_shader_.SetColor3("u_sun_color", settings.sun_color);
  sky_shader_.SetColor3("u_sky_zenith", settings.sky_zenith);
  sky_shader_.SetColor3("u_sky_horizon", settings.sky_horizon);
  sky_shader_.SetColor3("u_ground_horizon", settings.ground_horizon);
  sky_shader_.SetColor3("u_ground_color", settings.ground_color);
  fullscreen_vao_.Bind();
  glDrawArrays(GL_TRIANGLES, 0, 3);
  ++stats_.draw_calls;
  stats_.triangles += 1;
}

void Renderer::DrawMeshes(const RenderView& view,
                          std::span<const RenderItem> items,
                          const RenderSettings& settings) {
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  glDepthMask(GL_TRUE);
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  glFrontFace(GL_CCW);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  if (settings.wireframe) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glDisable(GL_CULL_FACE);
  }

  mesh_shader_.Use();
  mesh_shader_.SetMat4("u_view_projection", view.projection * view.view);
  mesh_shader_.SetVec3("u_camera_position", view.camera_position);
  mesh_shader_.SetVec3("u_sun_direction", SunDirection(settings));
  mesh_shader_.SetColor3("u_sun_color", settings.sun_color);
  mesh_shader_.SetFloat("u_sun_intensity", settings.sun_intensity);
  mesh_shader_.SetFloat("u_ambient", settings.ambient_intensity);
  mesh_shader_.SetColor3("u_sky_color", settings.sky_horizon);
  mesh_shader_.SetColor3("u_ground_color", settings.ground_color);
  mesh_shader_.SetColor3("u_fog_sky", settings.sky_horizon);
  mesh_shader_.SetColor3("u_fog_ground", settings.ground_horizon);
  mesh_shader_.SetFloat("u_fog_density",
                        settings.fog ? settings.fog_density : 0.0f);

  for (const RenderItem& item : items) {
    const GpuMesh& mesh = meshes_[static_cast<std::size_t>(item.mesh)];
    mesh_shader_.SetMat4("u_model", item.model);
    mesh_shader_.SetMat3("u_normal_matrix", NormalMatrix(item.model));
    mesh_shader_.SetColor4("u_base_color", item.material.base_color);
    mesh_shader_.SetFloat("u_specular", item.material.specular);
    mesh_shader_.SetFloat("u_shininess", item.material.shininess);
    mesh_shader_.SetFloat("u_emissive", item.material.emissive);
    mesh_shader_.SetFloat("u_highlight", item.highlight);
    mesh_shader_.SetColor3("u_highlight_color", item.highlight_color);
    mesh.Draw();
    ++stats_.draw_calls;
    stats_.triangles += mesh.index_count() / 3;
  }

  if (settings.wireframe) glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void Renderer::DrawGround(const RenderView& view,
                          const RenderSettings& settings) {
  glEnable(GL_DEPTH_TEST);
  glDepthMask(GL_TRUE);
  glDisable(GL_CULL_FACE);  // Visible from below as well.
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  const Vec3 sun = SunDirection(settings);
  const float light = static_cast<float>(settings.ambient_intensity +
                                         std::max(sun.y, 0.0) * 0.55);
  // Large enough to reach the horizon haze, but inside the far plane.
  const float extent =
      static_cast<float>(std::min(view.far_plane * 0.8, 4000.0));

  ground_shader_.Use();
  ground_shader_.SetMat4("u_view_projection", view.projection * view.view);
  ground_shader_.SetVec3("u_camera_position", view.camera_position);
  ground_shader_.SetFloat("u_extent", extent);
  ground_shader_.SetColor3("u_ground_color", settings.ground_color);
  ground_shader_.SetColor3("u_ground_horizon", settings.ground_horizon);
  ground_shader_.SetFloat("u_ground_opacity", settings.ground_opacity);
  ground_shader_.SetInt("u_show_grid", settings.show_grid ? 1 : 0);
  ground_shader_.SetFloat("u_cell", std::max(settings.grid_cell_size, 1e-3f));
  ground_shader_.SetFloat("u_major_every", static_cast<float>(std::max(
                                               settings.grid_major_every, 1)));
  ground_shader_.SetFloat("u_fade_distance", settings.grid_fade_distance);
  ground_shader_.SetColor4("u_minor_color", settings.grid_minor_color);
  ground_shader_.SetColor4("u_major_color", settings.grid_major_color);
  ground_shader_.SetColor3("u_axis_x_color", colors::kAxisX);
  ground_shader_.SetColor3("u_axis_z_color", colors::kAxisZ);
  ground_shader_.SetFloat("u_light", light);
  ground_shader_.SetFloat("u_fog_density",
                          settings.fog ? settings.fog_density : 0.0f);
  ground_quad_.Draw();
  ++stats_.draw_calls;
  stats_.triangles += 2;
}

void Renderer::DrawShadows(const RenderView& view,
                           std::span<const RenderItem> items) {
  glEnable(GL_DEPTH_TEST);
  glDepthMask(GL_FALSE);
  glDisable(GL_CULL_FACE);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_POLYGON_OFFSET_FILL);
  glPolygonOffset(-1.0f, -4.0f);

  shadow_shader_.Use();
  shadow_shader_.SetMat4("u_view_projection", view.projection * view.view);
  for (const RenderItem& item : items) {
    if (!item.cast_shadow) continue;
    const Aabb world = TransformAabb(MeshBounds(item.mesh), item.model);
    if (IsEmpty(world) || world.max.y < 0.0) continue;
    const double height = std::max(world.min.y, 0.0);
    // Shadows soften, spread and fade as the object climbs.
    const double strength = 0.55 * (1.0 - std::min(height / 80.0, 1.0));
    if (strength <= 0.01) continue;
    const Vec3 half = HalfExtents(world);
    const double radius = std::max(half.x, half.z) * (1.15 + height * 0.015);
    const Vec3 center = Center(world);
    shadow_shader_.SetMat4("u_model",
                           Mat4::Translation({center.x, 0.0, center.z}) *
                               Mat4::Scale({radius, 1.0, radius}));
    shadow_shader_.SetFloat("u_strength", static_cast<float>(strength));
    shadow_disc_.Draw();
    ++stats_.draw_calls;
    stats_.triangles += shadow_disc_.index_count() / 3;
  }
  glDisable(GL_POLYGON_OFFSET_FILL);
  glDepthMask(GL_TRUE);
}

void Renderer::DrawLines(const RenderView& view, const DebugDrawList& lines,
                         const RenderSettings& settings, bool depth_tested) {
  if (lines.empty()) return;
  if (depth_tested) {
    glEnable(GL_DEPTH_TEST);
  } else {
    glDisable(GL_DEPTH_TEST);
  }
  glDepthMask(GL_FALSE);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  line_shader_.Use();
  line_shader_.SetMat4("u_view_projection", view.projection * view.view);
  line_shader_.SetVec3("u_camera_position", view.camera_position);
  line_shader_.SetColor3("u_fog_sky", settings.sky_horizon);
  line_shader_.SetColor3("u_fog_ground", settings.ground_horizon);
  line_shader_.SetFloat("u_fog_density", settings.fog && depth_tested
                                             ? settings.fog_density * 0.6f
                                             : 0.0f);
  lines_.Draw(lines.vertices());
  ++stats_.draw_calls;
  stats_.line_segments += static_cast<long long>(lines.segment_count());
  glDepthMask(GL_TRUE);
}

void Renderer::DrawAxisGizmo(const RenderView& view) {
  // A small orthographic view of the world axes, rotated like the camera,
  // in the bottom-left corner of the viewport.
  const int size = std::clamp(view.height / 7, 64, 140);
  const int margin = size / 6;
  glViewport(margin, margin, size, size);
  glDisable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);

  DebugDrawList gizmo;
  gizmo.AddArrow(Vec3::Zero(), Vec3::UnitX(), colors::kAxisX, 0.3);
  gizmo.AddArrow(Vec3::Zero(), Vec3::UnitY(), colors::kAxisY, 0.3);
  gizmo.AddArrow(Vec3::Zero(), Vec3::UnitZ(), colors::kAxisZ, 0.3);
  gizmo.AddCircle(Vec3::Zero(), Vec3::UnitY(), 1.0,
                  WithAlpha(colors::kWhite, 0.15f), 40);

  const Mat4 gizmo_view =
      Mat4::Translation({0.0, 0.0, -3.0}) * RotationOnly(view.view);
  line_shader_.Use();
  line_shader_.SetMat4(
      "u_view_projection",
      Orthographic(-1.35, 1.35, -1.35, 1.35, 0.1, 10.0) * gizmo_view);
  line_shader_.SetVec3("u_camera_position", Vec3::Zero());
  line_shader_.SetFloat("u_fog_density", 0.0f);
  lines_.Draw(gizmo.vertices());
  ++stats_.draw_calls;
  glViewport(0, 0, view.width, view.height);
}

}  // namespace aerolab
