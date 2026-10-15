#ifndef AEROLAB_RENDER_RENDERER_H_
#define AEROLAB_RENDER_RENDERER_H_

#include <array>
#include <memory>
#include <span>

#include "geometry/debug_draw.h"
#include "geometry/mesh_library.h"
#include "math/aabb.h"
#include "render/axis_gizmo.h"
#include "render/gpu_mesh.h"
#include "render/render_types.h"
#include "render/shader.h"

namespace aerolab {

// Draws one frame of the 3D world into the currently bound framebuffer:
//
//   1. sky and horizon (fullscreen pass)
//   2. opaque meshes (Blinn-Phong)
//   3. ground plane with the infinite world grid
//   4. blob shadows
//   5. world-space debug lines (depth tested)
//   6. overlay debug lines (always on top)
//   7. corner axis gizmo (camera orientation)
//
// The renderer only consumes flat data (RenderItem, DebugDrawList); it has
// no knowledge of scenes, entities or simulation.
class Renderer {
 public:
  // Compiles shaders and uploads every built-in mesh. Requires a current GL
  // context. Returns nullptr (after logging) on failure.
  static std::unique_ptr<Renderer> Create();

  Renderer(const Renderer&) = delete;
  Renderer& operator=(const Renderer&) = delete;

  void Render(const RenderView& view, std::span<const RenderItem> items,
              const DebugDrawList& world_lines,
              const DebugDrawList& overlay_lines,
              const RenderSettings& settings);

  // Local-space bounds of a built-in mesh (used for picking and gizmos).
  const Aabb& MeshBounds(MeshKind kind) const;

  const RenderStats& stats() const { return stats_; }

 private:
  Renderer() = default;

  void DrawSky(const RenderView& view, const RenderSettings& settings);
  void DrawMeshes(const RenderView& view, std::span<const RenderItem> items,
                  const RenderSettings& settings);
  void DrawGround(const RenderView& view, const RenderSettings& settings);
  void DrawShadows(const RenderView& view, std::span<const RenderItem> items);
  void DrawLines(const RenderView& view, const DebugDrawList& lines,
                 const RenderSettings& settings, bool depth_tested);
  void DrawAxisGizmo(const RenderView& view);

  ShaderProgram sky_shader_;
  ShaderProgram ground_shader_;
  ShaderProgram mesh_shader_;
  ShaderProgram line_shader_;
  ShaderProgram shadow_shader_;

  std::array<GpuMesh, kMeshKindCount> meshes_;
  GpuMesh ground_quad_;
  GpuMesh shadow_disc_;
  LineBatch lines_;
  EmptyVertexArray fullscreen_vao_;

  RenderStats stats_;
};

}  // namespace aerolab

#endif  // AEROLAB_RENDER_RENDERER_H_
