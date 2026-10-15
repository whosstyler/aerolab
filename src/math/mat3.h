#ifndef AEROLAB_MATH_MAT3_H_
#define AEROLAB_MATH_MAT3_H_

#include <array>
#include <cstddef>
#include <optional>

#include "math/scalar.h"
#include "math/vec3.h"

namespace aerolab {

// 3x3 matrix of doubles, column-major, acting on column vectors (M * v).
// Used for rotations, inertia tensors and normal matrices.
class Mat3 {
 public:
  // Identity by default.
  constexpr Mat3() : elements_{1, 0, 0, 0, 1, 0, 0, 0, 1} {}

  static constexpr Mat3 Identity() { return Mat3(); }
  static constexpr Mat3 Zero() {
    Mat3 m;
    m.elements_ = {};
    return m;
  }
  static constexpr Mat3 FromColumns(const Vec3& c0, const Vec3& c1,
                                    const Vec3& c2) {
    Mat3 m;
    m.elements_ = {c0.x, c0.y, c0.z, c1.x, c1.y, c1.z, c2.x, c2.y, c2.z};
    return m;
  }
  static constexpr Mat3 FromRows(const Vec3& r0, const Vec3& r1,
                                 const Vec3& r2) {
    Mat3 m;
    m.elements_ = {r0.x, r1.x, r2.x, r0.y, r1.y, r2.y, r0.z, r1.z, r2.z};
    return m;
  }
  static constexpr Mat3 Diagonal(const Vec3& d) {
    Mat3 m = Zero();
    m(0, 0) = d.x;
    m(1, 1) = d.y;
    m(2, 2) = d.z;
    return m;
  }

  constexpr double operator()(int row, int col) const {
    return elements_[static_cast<std::size_t>(col * 3 + row)];
  }
  constexpr double& operator()(int row, int col) {
    return elements_[static_cast<std::size_t>(col * 3 + row)];
  }

  constexpr Vec3 Column(int col) const {
    return {elements_[col * 3], elements_[col * 3 + 1], elements_[col * 3 + 2]};
  }
  constexpr Vec3 Row(int row) const {
    return {elements_[row], elements_[3 + row], elements_[6 + row]};
  }

  // Column-major element storage.
  constexpr const std::array<double, 9>& elements() const { return elements_; }

  constexpr bool operator==(const Mat3&) const = default;

 private:
  std::array<double, 9> elements_;
};

Mat3 operator*(const Mat3& a, const Mat3& b);
Vec3 operator*(const Mat3& m, const Vec3& v);
Mat3 operator*(const Mat3& m, double s);
Mat3 operator+(const Mat3& a, const Mat3& b);
Mat3 operator-(const Mat3& a, const Mat3& b);

Mat3 Transpose(const Mat3& m);
double Determinant(const Mat3& m);

// Returns nullopt for (near-)singular matrices, judged relative to the matrix
// scale (see kSingularTolerance).
std::optional<Mat3> Inverse(const Mat3& m);

// Skew-symmetric matrix [v]x such that [v]x * u == Cross(v, u).
Mat3 CrossMatrix(const Vec3& v);

// a * b^T.
Mat3 OuterProduct(const Vec3& a, const Vec3& b);

bool NearlyEqual(const Mat3& a, const Mat3& b, double tolerance = kEpsilon);

}  // namespace aerolab

#endif  // AEROLAB_MATH_MAT3_H_
