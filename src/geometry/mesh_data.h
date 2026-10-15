#ifndef AEROLAB_GEOMETRY_MESH_DATA_H_
#define AEROLAB_GEOMETRY_MESH_DATA_H_

#include <cstdint>
#include <vector>

#include "geometry/color.h"
#include "math/aabb.h"
#include "math/mat4.h"
#include "math/vec3.h"

namespace aerolab {

// Interleaved vertex layout shared by every lit mesh. Single precision is
// sufficient for local-space model data.
struct MeshVertex {
  float px = 0.0f, py = 0.0f, pz = 0.0f;  // Position.
  float nx = 0.0f, ny = 1.0f, nz = 0.0f;  // Unit normal.
  float r = 1.0f, g = 1.0f, b = 1.0f;     // Per-vertex tint (sRGB).
};

// CPU-side indexed triangle mesh in local space. Triangles wind
// counter-clockwise when seen from outside (OpenGL's default front face).
struct MeshData {
  std::vector<MeshVertex> vertices;
  std::vector<std::uint32_t> indices;
  Aabb bounds;
};

// Incrementally assembles a MeshData. Composite models are built by
// appending transformed primitives.
class MeshBuilder {
 public:
  MeshBuilder() = default;

  // Colour applied to subsequently added vertices.
  void SetColor(const Color& color) { color_ = color; }

  // Adds a vertex and returns its index.
  std::uint32_t AddVertex(const Vec3& position, const Vec3& normal);
  void AddTriangle(std::uint32_t a, std::uint32_t b, std::uint32_t c);
  // Adds the quad a-b-c-d (counter-clockwise) as two triangles.
  void AddQuad(std::uint32_t a, std::uint32_t b, std::uint32_t c,
               std::uint32_t d);

  // Flat-shaded polygon with its own vertices and a computed face normal.
  void AddFlatTriangle(const Vec3& p0, const Vec3& p1, const Vec3& p2);
  void AddFlatQuad(const Vec3& p0, const Vec3& p1, const Vec3& p2,
                   const Vec3& p3);

  // Appends `mesh` transformed by `transform`, multiplying its vertex colours
  // by `tint`. Normals use the inverse-transpose so non-uniform scale works.
  void Append(const MeshData& mesh, const Mat4& transform,
              const Color& tint = colors::kWhite);

  std::size_t vertex_count() const { return mesh_.vertices.size(); }

  // Finalizes the mesh (computes bounds). The builder is left empty.
  MeshData Build();

 private:
  MeshData mesh_;
  Color color_ = colors::kWhite;
};

// Bounding box of the mesh's vertex positions.
Aabb ComputeBounds(const MeshData& mesh);

}  // namespace aerolab

#endif  // AEROLAB_GEOMETRY_MESH_DATA_H_
