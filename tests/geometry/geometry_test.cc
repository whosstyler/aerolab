#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <vector>

#include "geometry/debug_draw.h"
#include "geometry/mesh_data.h"
#include "geometry/mesh_library.h"
#include "geometry/primitives.h"
#include "math/mat4.h"
#include "math/quat.h"
#include "test_util.h"

namespace aerolab {
namespace {

Vec3 Position(const MeshVertex& v) { return {v.px, v.py, v.pz}; }
Vec3 Normal(const MeshVertex& v) { return {v.nx, v.ny, v.nz}; }

// For a closed mesh that is star-shaped around the origin, every triangle's
// counter-clockwise face normal must point away from the centre and agree
// with the vertex normals.
void ExpectOutwardFacing(const MeshData& mesh) {
  ASSERT_EQ(mesh.indices.size() % 3, 0u);
  ASSERT_FALSE(mesh.indices.empty());
  const Vec3 center = Center(mesh.bounds);
  for (std::size_t i = 0; i < mesh.indices.size(); i += 3) {
    const MeshVertex& a = mesh.vertices[mesh.indices[i]];
    const MeshVertex& b = mesh.vertices[mesh.indices[i + 1]];
    const MeshVertex& c = mesh.vertices[mesh.indices[i + 2]];
    const Vec3 face =
        Cross(Position(b) - Position(a), Position(c) - Position(a));
    if (Length(face) < 1e-9) continue;  // Degenerate sliver.
    const Vec3 centroid = (Position(a) + Position(b) + Position(c)) / 3.0;
    EXPECT_GT(Dot(face, centroid - center), 0.0) << "triangle " << i / 3;
    EXPECT_GT(Dot(face, Normal(a) + Normal(b) + Normal(c)), 0.0)
        << "triangle " << i / 3;
  }
}

void ExpectUnitNormals(const MeshData& mesh) {
  for (const MeshVertex& v : mesh.vertices) {
    EXPECT_NEAR(Length(Normal(v)), 1.0, 1e-5);
  }
}

TEST(PrimitivesTest, BoxIsClosedOutwardAndSized) {
  const MeshData box = MakeBox({1.0, 2.0, 3.0});
  EXPECT_EQ(box.vertices.size(), 24u);  // Flat shading: 4 per face.
  EXPECT_EQ(box.indices.size(), 36u);
  EXPECT_VEC3_NEAR(box.bounds.min, (Vec3{-1, -2, -3}), 1e-6);
  EXPECT_VEC3_NEAR(box.bounds.max, (Vec3{1, 2, 3}), 1e-6);
  ExpectOutwardFacing(box);
  ExpectUnitNormals(box);
}

TEST(PrimitivesTest, SphereVerticesLieOnSurface) {
  const MeshData sphere = MakeUvSphere(2.0, 24, 12);
  for (const MeshVertex& v : sphere.vertices) {
    EXPECT_NEAR(Length(Position(v)), 2.0, 1e-5);
  }
  ExpectOutwardFacing(sphere);
  ExpectUnitNormals(sphere);
}

TEST(PrimitivesTest, CylinderAndConeAreOutwardFacing) {
  const MeshData cylinder = MakeCylinder(0.5, 1.0, 16);
  EXPECT_VEC3_NEAR(cylinder.bounds.max, (Vec3{0.5, 1.0, 0.5}), 1e-6);
  ExpectOutwardFacing(cylinder);
  ExpectUnitNormals(cylinder);
  const MeshData cone = MakeCone(0.5, 2.0, 16);
  EXPECT_NEAR(cone.bounds.max.y, 2.0, 1e-6);
  EXPECT_NEAR(cone.bounds.min.y, 0.0, 1e-6);
  ExpectOutwardFacing(cone);
  ExpectUnitNormals(cone);
}

TEST(PrimitivesTest, DiscFacesUp) {
  const MeshData disc = MakeDisc(1.0, 8);
  for (std::size_t i = 0; i < disc.indices.size(); i += 3) {
    const Vec3 a = Position(disc.vertices[disc.indices[i]]);
    const Vec3 b = Position(disc.vertices[disc.indices[i + 1]]);
    const Vec3 c = Position(disc.vertices[disc.indices[i + 2]]);
    EXPECT_GT(Cross(b - a, c - a).y, 0.0);
  }
}

TEST(MeshBuilderTest, AppendTransformsPositionsNormalsAndTint) {
  MeshBuilder builder;
  builder.Append(MakeBox({0.5, 0.5, 0.5}),
                 Mat4::Translation({10, 0, 0}) * Mat4::Scale({2, 1, 1}),
                 FromHex(0x808080));
  const MeshData mesh = builder.Build();
  EXPECT_VEC3_NEAR(mesh.bounds.min, (Vec3{9, -0.5, -0.5}), 1e-6);
  EXPECT_VEC3_NEAR(mesh.bounds.max, (Vec3{11, 0.5, 0.5}), 1e-6);
  EXPECT_NEAR(mesh.vertices[0].r, 128.0f / 255.0f, 1e-6);
  ExpectUnitNormals(mesh);
}

TEST(MeshBuilderTest, MirroringKeepsTrianglesOutward) {
  MeshBuilder builder;
  builder.Append(MakeBox({0.5, 0.5, 0.5}), Mat4::Scale({-1, 1, 1}));
  ExpectOutwardFacing(builder.Build());
}

TEST(MeshLibraryTest, EveryKindBuildsValidGeometry) {
  for (MeshKind kind : AllMeshKinds()) {
    const MeshData mesh = BuildMesh(kind);
    EXPECT_FALSE(mesh.vertices.empty()) << MeshKindName(kind);
    EXPECT_EQ(mesh.indices.size() % 3, 0u) << MeshKindName(kind);
    for (std::uint32_t index : mesh.indices) {
      ASSERT_LT(index, mesh.vertices.size()) << MeshKindName(kind);
    }
    EXPECT_FALSE(IsEmpty(mesh.bounds)) << MeshKindName(kind);
    ExpectUnitNormals(mesh);
    // The cached bounds match a fresh build.
    EXPECT_VEC3_NEAR(MeshLocalBounds(kind).min, mesh.bounds.min, 1e-9);
    EXPECT_VEC3_NEAR(MeshLocalBounds(kind).max, mesh.bounds.max, 1e-9);
    EXPECT_NE(MeshKindName(kind), "Unknown");
  }
}

TEST(MeshLibraryTest, VehiclesFaceNegativeZ) {
  // Slender bodies are longer along Z, with the nose toward -Z.
  const Aabb dart = MeshLocalBounds(MeshKind::kDart);
  EXPECT_GT(dart.max.z - dart.min.z, 4.0 * (dart.max.x - dart.min.x) * 0.4);
  EXPECT_LT(dart.min.z, -1.0);
  const Aabb wing = MeshLocalBounds(MeshKind::kFixedWing);
  EXPECT_GT(wing.max.x - wing.min.x, wing.max.z - wing.min.z);
}

TEST(DebugDrawTest, ShapeSegmentCounts) {
  DebugDrawList list;
  list.AddOrientedBox(MakeAabb({-1, -1, -1}, {1, 1, 1}), Mat4::Identity(),
                      colors::kWhite);
  EXPECT_EQ(list.segment_count(), 12u);
  list.Clear();
  list.AddAxes(Mat4::Identity(), 1.0);
  EXPECT_EQ(list.segment_count(), 3u);
  list.Clear();
  list.AddArrow(Vec3::Zero(), {0, 0, 5}, colors::kWhite);
  EXPECT_EQ(list.segment_count(), 5u);
  list.Clear();
  list.AddArrow(Vec3::Zero(), Vec3::Zero(), colors::kWhite);
  EXPECT_TRUE(list.empty());
  list.AddCircle(Vec3::Zero(), Vec3::UnitY(), 1.0, colors::kWhite, 16);
  EXPECT_EQ(list.segment_count(), 16u);
  list.Clear();
  list.AddGrid(Vec3::Zero(), Vec3::UnitX(), Vec3::UnitZ(), 1.0, 5,
               colors::kWhite);
  EXPECT_EQ(list.segment_count(), 22u);  // 11 lines in each direction.
}

TEST(DebugDrawTest, PolylinesAndFading) {
  DebugDrawList list;
  const std::vector<Vec3> points = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}};
  list.AddPolyline(points, colors::kWhite);
  EXPECT_EQ(list.segment_count(), 2u);
  list.Clear();
  list.AddPolyline(points, colors::kWhite, /*closed=*/true);
  EXPECT_EQ(list.segment_count(), 3u);
  list.Clear();
  list.AddFadingPolyline(points, WithAlpha(colors::kWhite, 1.0f), 0.0f);
  ASSERT_EQ(list.vertices().size(), 4u);
  EXPECT_FLOAT_EQ(list.vertices().front().color.a, 0.0f);
  EXPECT_FLOAT_EQ(list.vertices().back().color.a, 1.0f);
  // Alpha is continuous across the joint between segments.
  EXPECT_FLOAT_EQ(list.vertices()[1].color.a, list.vertices()[2].color.a);
}

TEST(DebugDrawTest, AxesFollowTransformRotation) {
  DebugDrawList list;
  const Mat4 frame =
      Mat4::Trs({5, 0, 0}, FromAxisAngle(Vec3::UnitZ(), kHalfPi), {3, 3, 3});
  list.AddAxes(frame, 2.0);
  const auto& v = list.vertices();
  // X axis endpoint: rotated onto +Y, length unaffected by scale.
  EXPECT_NEAR(v[1].x, 5.0, 1e-5);
  EXPECT_NEAR(v[1].y, 2.0, 1e-5);
  EXPECT_EQ(v[0].color, colors::kAxisX);
}

}  // namespace
}  // namespace aerolab
