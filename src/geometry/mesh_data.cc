#include "geometry/mesh_data.h"

#include <cstdint>
#include <utility>

#include "math/mat3.h"

namespace aerolab {

std::uint32_t MeshBuilder::AddVertex(const Vec3& position, const Vec3& normal) {
  MeshVertex v;
  v.px = static_cast<float>(position.x);
  v.py = static_cast<float>(position.y);
  v.pz = static_cast<float>(position.z);
  v.nx = static_cast<float>(normal.x);
  v.ny = static_cast<float>(normal.y);
  v.nz = static_cast<float>(normal.z);
  v.r = color_.r;
  v.g = color_.g;
  v.b = color_.b;
  mesh_.vertices.push_back(v);
  return static_cast<std::uint32_t>(mesh_.vertices.size() - 1);
}

void MeshBuilder::AddTriangle(std::uint32_t a, std::uint32_t b,
                              std::uint32_t c) {
  mesh_.indices.push_back(a);
  mesh_.indices.push_back(b);
  mesh_.indices.push_back(c);
}

void MeshBuilder::AddQuad(std::uint32_t a, std::uint32_t b, std::uint32_t c,
                          std::uint32_t d) {
  AddTriangle(a, b, c);
  AddTriangle(a, c, d);
}

void MeshBuilder::AddFlatTriangle(const Vec3& p0, const Vec3& p1,
                                  const Vec3& p2) {
  const Vec3 normal = Normalized(Cross(p1 - p0, p2 - p0));
  AddTriangle(AddVertex(p0, normal), AddVertex(p1, normal),
              AddVertex(p2, normal));
}

void MeshBuilder::AddFlatQuad(const Vec3& p0, const Vec3& p1, const Vec3& p2,
                              const Vec3& p3) {
  const Vec3 normal = Normalized(Cross(p1 - p0, p2 - p0));
  AddQuad(AddVertex(p0, normal), AddVertex(p1, normal), AddVertex(p2, normal),
          AddVertex(p3, normal));
}

void MeshBuilder::Append(const MeshData& mesh, const Mat4& transform,
                         const Color& tint) {
  const Mat3 normal_matrix = NormalMatrix(transform);
  // A mirroring transform flips triangle orientation; swap two indices to
  // keep counter-clockwise front faces.
  const bool mirrored = Determinant(transform.UpperLeft3x3()) < 0.0;
  const auto base = static_cast<std::uint32_t>(mesh_.vertices.size());
  for (const MeshVertex& v : mesh.vertices) {
    const Vec3 p = TransformPoint(transform, Vec3{v.px, v.py, v.pz});
    const Vec3 n = Normalized(normal_matrix * Vec3{v.nx, v.ny, v.nz});
    MeshVertex out;
    out.px = static_cast<float>(p.x);
    out.py = static_cast<float>(p.y);
    out.pz = static_cast<float>(p.z);
    out.nx = static_cast<float>(n.x);
    out.ny = static_cast<float>(n.y);
    out.nz = static_cast<float>(n.z);
    out.r = v.r * tint.r;
    out.g = v.g * tint.g;
    out.b = v.b * tint.b;
    mesh_.vertices.push_back(out);
  }
  for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
    const std::uint32_t a = base + mesh.indices[i];
    const std::uint32_t b = base + mesh.indices[i + 1];
    const std::uint32_t c = base + mesh.indices[i + 2];
    if (mirrored) {
      AddTriangle(a, c, b);
    } else {
      AddTriangle(a, b, c);
    }
  }
}

MeshData MeshBuilder::Build() {
  mesh_.bounds = ComputeBounds(mesh_);
  MeshData result = std::move(mesh_);
  mesh_ = MeshData();
  return result;
}

Aabb ComputeBounds(const MeshData& mesh) {
  Aabb bounds;
  for (const MeshVertex& v : mesh.vertices) {
    bounds = Expanded(bounds, Vec3{v.px, v.py, v.pz});
  }
  return bounds;
}

}  // namespace aerolab
