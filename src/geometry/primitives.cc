#include "geometry/primitives.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "math/scalar.h"

namespace aerolab {
namespace {

// Point on the unit circle in the XZ plane. Increasing angle runs from +X
// toward -Z, i.e. counter-clockwise when viewed from +Y.
Vec3 CirclePoint(double angle) {
  return {std::cos(angle), 0.0, -std::sin(angle)};
}

// Fan of triangles closing a ring of vertices at height y. `up` selects the
// winding so the cap faces +Y (true) or -Y (false).
void AddCap(MeshBuilder& builder, double radius, double y, int segments,
            bool up) {
  const Vec3 normal = up ? Vec3::UnitY() : -Vec3::UnitY();
  const std::uint32_t center = builder.AddVertex({0.0, y, 0.0}, normal);
  const std::uint32_t first =
      static_cast<std::uint32_t>(builder.vertex_count());
  for (int i = 0; i <= segments; ++i) {
    const Vec3 p = CirclePoint(kTwoPi * i / segments) * radius;
    builder.AddVertex({p.x, y, p.z}, normal);
  }
  for (int i = 0; i < segments; ++i) {
    const auto a = first + static_cast<std::uint32_t>(i);
    if (up) {
      builder.AddTriangle(center, a, a + 1);
    } else {
      builder.AddTriangle(center, a + 1, a);
    }
  }
}

}  // namespace

MeshData MakeBox(const Vec3& half_extents) {
  const Vec3 h = half_extents;
  MeshBuilder builder;
  // Each face lists its corners counter-clockwise as seen from outside.
  builder.AddFlatQuad({h.x, -h.y, h.z}, {h.x, -h.y, -h.z}, {h.x, h.y, -h.z},
                      {h.x, h.y, h.z});  // +X
  builder.AddFlatQuad({-h.x, -h.y, -h.z}, {-h.x, -h.y, h.z}, {-h.x, h.y, h.z},
                      {-h.x, h.y, -h.z});  // -X
  builder.AddFlatQuad({-h.x, h.y, h.z}, {h.x, h.y, h.z}, {h.x, h.y, -h.z},
                      {-h.x, h.y, -h.z});  // +Y
  builder.AddFlatQuad({-h.x, -h.y, -h.z}, {h.x, -h.y, -h.z}, {h.x, -h.y, h.z},
                      {-h.x, -h.y, h.z});  // -Y
  builder.AddFlatQuad({-h.x, -h.y, h.z}, {h.x, -h.y, h.z}, {h.x, h.y, h.z},
                      {-h.x, h.y, h.z});  // +Z
  builder.AddFlatQuad({h.x, -h.y, -h.z}, {-h.x, -h.y, -h.z}, {-h.x, h.y, -h.z},
                      {h.x, h.y, -h.z});  // -Z
  return builder.Build();
}

MeshData MakeUvSphere(double radius, int slices, int stacks) {
  slices = std::max(slices, 3);
  stacks = std::max(stacks, 2);
  MeshBuilder builder;
  for (int stack = 0; stack <= stacks; ++stack) {
    // Polar angle from +Y (north pole) down to -Y.
    const double phi = kPi * stack / stacks;
    const double ring_radius = std::sin(phi);
    const double y = std::cos(phi);
    for (int slice = 0; slice <= slices; ++slice) {
      const Vec3 ring = CirclePoint(kTwoPi * slice / slices);
      const Vec3 normal{ring.x * ring_radius, y, ring.z * ring_radius};
      builder.AddVertex(normal * radius, normal);
    }
  }
  const auto row = static_cast<std::uint32_t>(slices + 1);
  for (int stack = 0; stack < stacks; ++stack) {
    for (int slice = 0; slice < slices; ++slice) {
      const auto top = static_cast<std::uint32_t>(stack) * row +
                       static_cast<std::uint32_t>(slice);
      const std::uint32_t bottom = top + row;
      // Skip the degenerate triangles that touch the poles.
      if (stack != 0) builder.AddTriangle(top, bottom, top + 1);
      if (stack != stacks - 1) builder.AddTriangle(top + 1, bottom, bottom + 1);
    }
  }
  return builder.Build();
}

MeshData MakeCylinder(double radius, double half_height, int segments) {
  segments = std::max(segments, 3);
  MeshBuilder builder;
  const auto first = static_cast<std::uint32_t>(builder.vertex_count());
  for (int i = 0; i <= segments; ++i) {
    const Vec3 dir = CirclePoint(kTwoPi * i / segments);
    builder.AddVertex({dir.x * radius, -half_height, dir.z * radius}, dir);
    builder.AddVertex({dir.x * radius, half_height, dir.z * radius}, dir);
  }
  for (int i = 0; i < segments; ++i) {
    const std::uint32_t bottom = first + 2 * static_cast<std::uint32_t>(i);
    builder.AddQuad(bottom, bottom + 2, bottom + 3, bottom + 1);
  }
  AddCap(builder, radius, half_height, segments, /*up=*/true);
  AddCap(builder, radius, -half_height, segments, /*up=*/false);
  return builder.Build();
}

MeshData MakeCone(double radius, double height, int segments) {
  segments = std::max(segments, 3);
  MeshBuilder builder;
  // The side normal tilts up by the cone's half-angle: (cos, r/h, sin).
  const double slope = radius / height;
  for (int i = 0; i < segments; ++i) {
    const double a0 = kTwoPi * i / segments;
    const double a1 = kTwoPi * (i + 1) / segments;
    const double mid = 0.5 * (a0 + a1);
    const Vec3 d0 = CirclePoint(a0);
    const Vec3 d1 = CirclePoint(a1);
    const Vec3 dm = CirclePoint(mid);
    const std::uint32_t v0 =
        builder.AddVertex(d0 * radius, Normalized(Vec3{d0.x, slope, d0.z}));
    const std::uint32_t v1 =
        builder.AddVertex(d1 * radius, Normalized(Vec3{d1.x, slope, d1.z}));
    // One apex vertex per segment so each facet gets a sensible normal.
    const std::uint32_t apex = builder.AddVertex(
        {0.0, height, 0.0}, Normalized(Vec3{dm.x, slope, dm.z}));
    builder.AddTriangle(v0, v1, apex);
  }
  AddCap(builder, radius, 0.0, segments, /*up=*/false);
  return builder.Build();
}

MeshData MakeDisc(double radius, int segments) {
  MeshBuilder builder;
  AddCap(builder, radius, 0.0, std::max(segments, 3), /*up=*/true);
  return builder.Build();
}

}  // namespace aerolab
