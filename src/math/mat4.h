#ifndef AEROLAB_MATH_MAT4_H_
#define AEROLAB_MATH_MAT4_H_

#include <array>
#include <cstddef>
#include <optional>

#include "math/mat3.h"
#include "math/scalar.h"
#include "math/vec3.h"
#include "math/vec4.h"

namespace aerolab {

struct Quat;

// 4x4 matrix of doubles, column-major, acting on column vectors (M * v).
// The memory layout matches OpenGL, so ToFloatArray() can be uploaded with
// transpose = GL_FALSE.
class Mat4 {
 public:
  // Identity by default.
  constexpr Mat4()
      : elements_{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1} {}

  static constexpr Mat4 Identity() { return Mat4(); }
  static constexpr Mat4 Zero() {
    Mat4 m;
    m.elements_ = {};
    return m;
  }
  static constexpr Mat4 FromColumns(const Vec4& c0, const Vec4& c1,
                                    const Vec4& c2, const Vec4& c3) {
    Mat4 m;
    m.elements_ = {c0.x, c0.y, c0.z, c0.w, c1.x, c1.y, c1.z, c1.w,
                   c2.x, c2.y, c2.z, c2.w, c3.x, c3.y, c3.z, c3.w};
    return m;
  }
  static constexpr Mat4 Translation(const Vec3& t) {
    Mat4 m;
    m(0, 3) = t.x;
    m(1, 3) = t.y;
    m(2, 3) = t.z;
    return m;
  }
  static constexpr Mat4 Scale(const Vec3& s) {
    Mat4 m;
    m(0, 0) = s.x;
    m(1, 1) = s.y;
    m(2, 2) = s.z;
    return m;
  }
  // Embeds a 3x3 linear part (rotation, scale) with zero translation.
  static constexpr Mat4 FromMat3(const Mat3& linear) {
    Mat4 m;
    for (int col = 0; col < 3; ++col) {
      for (int row = 0; row < 3; ++row) m(row, col) = linear(row, col);
    }
    return m;
  }
  static Mat4 Rotation(const Quat& rotation);
  // Translation * Rotation * Scale.
  static Mat4 Trs(const Vec3& translation, const Quat& rotation,
                  const Vec3& scale);

  constexpr double operator()(int row, int col) const {
    return elements_[static_cast<std::size_t>(col * 4 + row)];
  }
  constexpr double& operator()(int row, int col) {
    return elements_[static_cast<std::size_t>(col * 4 + row)];
  }

  constexpr Vec4 Column(int col) const {
    return {elements_[col * 4], elements_[col * 4 + 1], elements_[col * 4 + 2],
            elements_[col * 4 + 3]};
  }
  constexpr Vec4 Row(int row) const {
    return {elements_[row], elements_[4 + row], elements_[8 + row],
            elements_[12 + row]};
  }

  constexpr Vec3 TranslationPart() const {
    return {elements_[12], elements_[13], elements_[14]};
  }
  constexpr Mat3 UpperLeft3x3() const {
    return Mat3::FromColumns(Xyz(Column(0)), Xyz(Column(1)), Xyz(Column(2)));
  }

  constexpr const std::array<double, 16>& elements() const { return elements_; }

  // Single-precision copy in the same column-major order, for the GPU.
  std::array<float, 16> ToFloatArray() const;

  constexpr bool operator==(const Mat4&) const = default;

 private:
  std::array<double, 16> elements_;
};

Mat4 operator*(const Mat4& a, const Mat4& b);
Vec4 operator*(const Mat4& m, const Vec4& v);

Mat4 Transpose(const Mat4& m);
double Determinant(const Mat4& m);

// General inverse by cofactor expansion; nullopt for (near-)singular input.
// Singularity is judged relative to the matrix scale (see kSingularTolerance),
// so tiny but well-conditioned matrices still invert.
std::optional<Mat4> Inverse(const Mat4& m);

// Inverse of an affine matrix (bottom row 0,0,0,1) via its 3x3 block.
// Cheaper and better conditioned than the general inverse.
std::optional<Mat4> AffineInverse(const Mat4& m);

// Applies M to a point (w = 1) without a perspective divide.
Vec3 TransformPoint(const Mat4& m, const Vec3& point);

// Applies M to a point (w = 1) and divides by the resulting w.
Vec3 TransformPointProjective(const Mat4& m, const Vec3& point);

// Applies the linear part of M to a direction (w = 0).
Vec3 TransformDirection(const Mat4& m, const Vec3& direction);

// Inverse-transpose of the upper 3x3 block: transforms surface normals
// correctly under non-uniform scale. Falls back to the identity if singular.
Mat3 NormalMatrix(const Mat4& model);

bool NearlyEqual(const Mat4& a, const Mat4& b, double tolerance = kEpsilon);

}  // namespace aerolab

#endif  // AEROLAB_MATH_MAT4_H_
