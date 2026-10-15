#include "math/mat3.h"

#include <cmath>
#include <optional>

namespace aerolab {

Mat3 operator*(const Mat3& a, const Mat3& b) {
  Mat3 result = Mat3::Zero();
  for (int col = 0; col < 3; ++col) {
    for (int row = 0; row < 3; ++row) {
      double sum = 0.0;
      for (int k = 0; k < 3; ++k) sum += a(row, k) * b(k, col);
      result(row, col) = sum;
    }
  }
  return result;
}

Vec3 operator*(const Mat3& m, const Vec3& v) {
  return {m(0, 0) * v.x + m(0, 1) * v.y + m(0, 2) * v.z,
          m(1, 0) * v.x + m(1, 1) * v.y + m(1, 2) * v.z,
          m(2, 0) * v.x + m(2, 1) * v.y + m(2, 2) * v.z};
}

Mat3 operator*(const Mat3& m, double s) {
  Mat3 result = m;
  for (int col = 0; col < 3; ++col) {
    for (int row = 0; row < 3; ++row) result(row, col) *= s;
  }
  return result;
}

Mat3 operator+(const Mat3& a, const Mat3& b) {
  Mat3 result = a;
  for (int col = 0; col < 3; ++col) {
    for (int row = 0; row < 3; ++row) result(row, col) += b(row, col);
  }
  return result;
}

Mat3 operator-(const Mat3& a, const Mat3& b) { return a + b * -1.0; }

Mat3 Transpose(const Mat3& m) {
  return Mat3::FromRows(m.Column(0), m.Column(1), m.Column(2));
}

double Determinant(const Mat3& m) {
  // Scalar triple product of the columns.
  return Dot(m.Column(0), Cross(m.Column(1), m.Column(2)));
}

std::optional<Mat3> Inverse(const Mat3& m) {
  const Vec3 c0 = m.Column(0);
  const Vec3 c1 = m.Column(1);
  const Vec3 c2 = m.Column(2);
  // The rows of the inverse are the cross products of column pairs divided by
  // the determinant (adjugate formula).
  const Vec3 r0 = Cross(c1, c2);
  const Vec3 r1 = Cross(c2, c0);
  const Vec3 r2 = Cross(c0, c1);
  const double det = Dot(c0, r0);
  const double scale = Length(c0) * Length(c1) * Length(c2);
  if (std::abs(det) <= kSingularTolerance * scale) return std::nullopt;
  const double inv_det = 1.0 / det;
  return Mat3::FromRows(r0 * inv_det, r1 * inv_det, r2 * inv_det);
}

Mat3 CrossMatrix(const Vec3& v) {
  return Mat3::FromRows({0.0, -v.z, v.y}, {v.z, 0.0, -v.x}, {-v.y, v.x, 0.0});
}

Mat3 OuterProduct(const Vec3& a, const Vec3& b) {
  return Mat3::FromColumns(a * b.x, a * b.y, a * b.z);
}

bool NearlyEqual(const Mat3& a, const Mat3& b, double tolerance) {
  for (int i = 0; i < 9; ++i) {
    if (!NearlyEqual(a.elements()[i], b.elements()[i], tolerance)) {
      return false;
    }
  }
  return true;
}

}  // namespace aerolab
