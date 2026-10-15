#ifndef AEROLAB_CAMERA_CAMERA_H_
#define AEROLAB_CAMERA_CAMERA_H_

#include "math/mat4.h"
#include "math/quat.h"
#include "math/scalar.h"
#include "math/vec3.h"

namespace aerolab {

// A perspective camera. `orientation` maps camera space to world space; the
// camera looks down its local -Z axis with +Y up (OpenGL convention).
struct Camera {
  Vec3 position{0.0, 2.0, 10.0};
  Quat orientation;
  double vertical_fov_radians = Radians(60.0);
  double near_plane = 0.05;
  double far_plane = 5000.0;
};

// World space -> camera (view) space.
Mat4 ViewMatrix(const Camera& camera);

// Camera space -> clip space for a viewport of the given aspect ratio.
Mat4 ProjectionMatrix(const Camera& camera, double aspect);

// Camera basis vectors in world space.
Vec3 CameraForward(const Camera& camera);
Vec3 CameraRight(const Camera& camera);
Vec3 CameraUp(const Camera& camera);

// Coordinate-space conversions between world and camera space.
Vec3 WorldToCameraSpace(const Camera& camera, const Vec3& world_point);
Vec3 CameraToWorldSpace(const Camera& camera, const Vec3& camera_point);

// World-space size of one pixel at `distance` in front of the camera, for a
// viewport `viewport_height` pixels tall. Used for pixel-accurate panning
// and constant-screen-size gizmos.
double WorldUnitsPerPixel(const Camera& camera, double distance,
                          double viewport_height);

}  // namespace aerolab

#endif  // AEROLAB_CAMERA_CAMERA_H_
