#include "camera/camera.h"

#include <algorithm>
#include <cmath>

#include "math/mat3.h"
#include "math/projection.h"

namespace aerolab {

Mat4 ViewMatrix(const Camera& camera) {
  // The view matrix is the inverse of the camera's rigid world transform:
  // V = R^T * T(-p).
  const Mat3 rotation_inverse = Transpose(ToMat3(camera.orientation));
  Mat4 view = Mat4::FromMat3(rotation_inverse);
  const Vec3 t = -(rotation_inverse * camera.position);
  view(0, 3) = t.x;
  view(1, 3) = t.y;
  view(2, 3) = t.z;
  return view;
}

Mat4 ProjectionMatrix(const Camera& camera, double aspect) {
  return Perspective(camera.vertical_fov_radians, std::max(aspect, 1e-3),
                     camera.near_plane, camera.far_plane);
}

Vec3 CameraForward(const Camera& camera) {
  return Rotate(camera.orientation, -Vec3::UnitZ());
}

Vec3 CameraRight(const Camera& camera) {
  return Rotate(camera.orientation, Vec3::UnitX());
}

Vec3 CameraUp(const Camera& camera) {
  return Rotate(camera.orientation, Vec3::UnitY());
}

Vec3 WorldToCameraSpace(const Camera& camera, const Vec3& world_point) {
  return Rotate(Conjugate(camera.orientation), world_point - camera.position);
}

Vec3 CameraToWorldSpace(const Camera& camera, const Vec3& camera_point) {
  return camera.position + Rotate(camera.orientation, camera_point);
}

double WorldUnitsPerPixel(const Camera& camera, double distance,
                          double viewport_height) {
  const double visible_height =
      2.0 * distance * std::tan(0.5 * camera.vertical_fov_radians);
  return visible_height / std::max(viewport_height, 1.0);
}

}  // namespace aerolab
