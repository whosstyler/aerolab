#ifndef AEROLAB_GEOMETRY_PRIMITIVES_H_
#define AEROLAB_GEOMETRY_PRIMITIVES_H_

#include "geometry/mesh_data.h"
#include "math/vec3.h"

namespace aerolab {

// Procedural primitives in local space, centred on the origin unless noted.
// Rotationally symmetric shapes use +Y as their axis.

// Axis-aligned box with flat-shaded faces.
MeshData MakeBox(const Vec3& half_extents);

// Latitude/longitude sphere with smooth normals.
MeshData MakeUvSphere(double radius, int slices, int stacks);

// Cylinder along Y spanning [-half_height, +half_height] with flat caps.
MeshData MakeCylinder(double radius, double half_height, int segments);

// Cone with its base disc at y = 0 and apex at y = height.
MeshData MakeCone(double radius, double height, int segments);

// Upward-facing (+Y) flat disc at y = 0.
MeshData MakeDisc(double radius, int segments);

}  // namespace aerolab

#endif  // AEROLAB_GEOMETRY_PRIMITIVES_H_
