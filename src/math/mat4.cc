#include "math/mat4.h"

#include <array>
#include <cmath>
#include <optional>

#include "math/quat.h"

namespace aerolab {

Mat4 Mat4::Rotation(const Quat& rotation) { return FromMat3(ToMat3(rotation)); }

Mat4 Mat4::Trs(const Vec3& translation, const Quat& rotation,
               const Vec3& scale) {
  const Mat3 r = ToMat3(rotation);
  // Columns of R * S are the rotation columns scaled per axis.
  Mat4 m = FromMat3(Mat3::FromColumns(
      r.Column(0) * scale.x, r.Column(1) * scale.y, r.Column(2) * scale.z));
  m(0, 3) = translation.x;
  m(1, 3) = translation.y;
  m(2, 3) = translation.z;
  return m;
}

std::array<float, 16> Mat4::ToFloatArray() const {
  std::array<float, 16> result;
  for (int i = 0; i < 16; ++i) result[i] = static_cast<float>(elements_[i]);
  return result;
}

Mat4 operator*(const Mat4& a, const Mat4& b) {
  Mat4 result = Mat4::Zero();
  for (int col = 0; col < 4; ++col) {
    for (int row = 0; row < 4; ++row) {
      double sum = 0.0;
      for (int k = 0; k < 4; ++k) sum += a(row, k) * b(k, col);
      result(row, col) = sum;
    }
  }
  return result;
}

Vec4 operator*(const Mat4& m, const Vec4& v) {
  Vec4 result;
  for (int row = 0; row < 4; ++row) {
    result[row] =
        m(row, 0) * v.x + m(row, 1) * v.y + m(row, 2) * v.z + m(row, 3) * v.w;
  }
  return result;
}

Mat4 Transpose(const Mat4& m) {
  Mat4 result;
  for (int col = 0; col < 4; ++col) {
    for (int row = 0; row < 4; ++row) result(row, col) = m(col, row);
  }
  return result;
}

namespace {

// 2x2 sub-determinants shared by Determinant() and Inverse(). Following the
// Laplace expansion by complementary minors, the 4x4 determinant is built from
// the 2x2 minors of the top two rows (s) and the bottom two rows (c).
struct Minors {
  double s0, s1, s2, s3, s4, s5;
  double c0, c1, c2, c3, c4, c5;
};

Minors ComputeMinors(const Mat4& m) {
  Minors n;
  n.s0 = m(0, 0) * m(1, 1) - m(1, 0) * m(0, 1);
  n.s1 = m(0, 0) * m(1, 2) - m(1, 0) * m(0, 2);
  n.s2 = m(0, 0) * m(1, 3) - m(1, 0) * m(0, 3);
  n.s3 = m(0, 1) * m(1, 2) - m(1, 1) * m(0, 2);
  n.s4 = m(0, 1) * m(1, 3) - m(1, 1) * m(0, 3);
  n.s5 = m(0, 2) * m(1, 3) - m(1, 2) * m(0, 3);
  n.c5 = m(2, 2) * m(3, 3) - m(3, 2) * m(2, 3);
  n.c4 = m(2, 1) * m(3, 3) - m(3, 1) * m(2, 3);
  n.c3 = m(2, 1) * m(3, 2) - m(3, 1) * m(2, 2);
  n.c2 = m(2, 0) * m(3, 3) - m(3, 0) * m(2, 3);
  n.c1 = m(2, 0) * m(3, 2) - m(3, 0) * m(2, 2);
  n.c0 = m(2, 0) * m(3, 1) - m(3, 0) * m(2, 1);
  return n;
}

double DeterminantFromMinors(const Minors& n) {
  return n.s0 * n.c5 - n.s1 * n.c4 + n.s2 * n.c3 + n.s3 * n.c2 - n.s4 * n.c1 +
         n.s5 * n.c0;
}

}  // namespace

double Determinant(const Mat4& m) {
  return DeterminantFromMinors(ComputeMinors(m));
}

std::optional<Mat4> Inverse(const Mat4& m) {
  const Minors n = ComputeMinors(m);
  const double det = DeterminantFromMinors(n);
  double scale = 1.0;
  for (int col = 0; col < 4; ++col) {
    const Vec4 c = m.Column(col);
    scale *= std::sqrt(Dot(c, c));
  }
  if (std::abs(det) <= kSingularTolerance * scale) return std::nullopt;
  const double inv = 1.0 / det;

  Mat4 r;
  r(0, 0) = (m(1, 1) * n.c5 - m(1, 2) * n.c4 + m(1, 3) * n.c3) * inv;
  r(0, 1) = (-m(0, 1) * n.c5 + m(0, 2) * n.c4 - m(0, 3) * n.c3) * inv;
  r(0, 2) = (m(3, 1) * n.s5 - m(3, 2) * n.s4 + m(3, 3) * n.s3) * inv;
  r(0, 3) = (-m(2, 1) * n.s5 + m(2, 2) * n.s4 - m(2, 3) * n.s3) * inv;

  r(1, 0) = (-m(1, 0) * n.c5 + m(1, 2) * n.c2 - m(1, 3) * n.c1) * inv;
  r(1, 1) = (m(0, 0) * n.c5 - m(0, 2) * n.c2 + m(0, 3) * n.c1) * inv;
  r(1, 2) = (-m(3, 0) * n.s5 + m(3, 2) * n.s2 - m(3, 3) * n.s1) * inv;
  r(1, 3) = (m(2, 0) * n.s5 - m(2, 2) * n.s2 + m(2, 3) * n.s1) * inv;

  r(2, 0) = (m(1, 0) * n.c4 - m(1, 1) * n.c2 + m(1, 3) * n.c0) * inv;
  r(2, 1) = (-m(0, 0) * n.c4 + m(0, 1) * n.c2 - m(0, 3) * n.c0) * inv;
  r(2, 2) = (m(3, 0) * n.s4 - m(3, 1) * n.s2 + m(3, 3) * n.s0) * inv;
  r(2, 3) = (-m(2, 0) * n.s4 + m(2, 1) * n.s2 - m(2, 3) * n.s0) * inv;

  r(3, 0) = (-m(1, 0) * n.c3 + m(1, 1) * n.c1 - m(1, 2) * n.c0) * inv;
  r(3, 1) = (m(0, 0) * n.c3 - m(0, 1) * n.c1 + m(0, 2) * n.c0) * inv;
  r(3, 2) = (-m(3, 0) * n.s3 + m(3, 1) * n.s1 - m(3, 2) * n.s0) * inv;
  r(3, 3) = (m(2, 0) * n.s3 - m(2, 1) * n.s1 + m(2, 2) * n.s0) * inv;
  return r;
}

std::optional<Mat4> AffineInverse(const Mat4& m) {
  const std::optional<Mat3> linear_inverse = Inverse(m.UpperLeft3x3());
  if (!linear_inverse.has_value()) return std::nullopt;
  Mat4 result = Mat4::FromMat3(*linear_inverse);
  const Vec3 t = -(*linear_inverse * m.TranslationPart());
  result(0, 3) = t.x;
  result(1, 3) = t.y;
  result(2, 3) = t.z;
  return result;
}

Vec3 TransformPoint(const Mat4& m, const Vec3& point) {
  return Xyz(m * MakePoint(point));
}

Vec3 TransformPointProjective(const Mat4& m, const Vec3& point) {
  const Vec4 clip = m * MakePoint(point);
  // Keep the sign of w so points behind a projective camera stay mirrored
  // consistently instead of flipping when |w| is tiny.
  const double w =
      std::abs(clip.w) > kEpsilon ? clip.w : std::copysign(kEpsilon, clip.w);
  return Xyz(clip) / w;
}

Vec3 TransformDirection(const Mat4& m, const Vec3& direction) {
  return Xyz(m * MakeDirection(direction));
}

Mat3 NormalMatrix(const Mat4& model) {
  const std::optional<Mat3> inverse = Inverse(model.UpperLeft3x3());
  return inverse.has_value() ? Transpose(*inverse) : Mat3::Identity();
}

bool NearlyEqual(const Mat4& a, const Mat4& b, double tolerance) {
  for (int i = 0; i < 16; ++i) {
    if (!NearlyEqual(a.elements()[i], b.elements()[i], tolerance)) {
      return false;
    }
  }
  return true;
}

}  // namespace aerolab
