#include <gtest/gtest.h>

#include <optional>

#include "math/mat3.h"
#include "math/mat4.h"
#include "math/quat.h"
#include "math/vec3.h"
#include "test_util.h"

namespace aerolab {
namespace {

Mat4 SampleAffine() {
  return Mat4::Trs({3.0, -2.0, 7.5}, FromAxisAngle(Vec3{1.0, 2.0, -0.5}, 0.83),
                   {2.0, 0.5, 1.25});
}

TEST(Mat3Test, DefaultIsIdentityAndStorageIsColumnMajor) {
  const Mat3 identity;
  EXPECT_EQ(identity, Mat3::Identity());
  const Mat3 m = Mat3::FromRows({1, 2, 3}, {4, 5, 6}, {7, 8, 9});
  EXPECT_EQ(m(0, 1), 2.0);
  EXPECT_EQ(m(1, 0), 4.0);
  EXPECT_EQ(m.elements()[1], 4.0);  // Column 0, row 1.
  EXPECT_EQ(m.Column(2), (Vec3{3, 6, 9}));
  EXPECT_EQ(m.Row(2), (Vec3{7, 8, 9}));
}

TEST(Mat3Test, MultiplyMatchesHandComputation) {
  const Mat3 a = Mat3::FromRows({1, 2, 0}, {0, 1, 0}, {0, 0, 2});
  const Mat3 b = Mat3::FromRows({1, 0, 0}, {3, 1, 0}, {0, 0, 1});
  const Mat3 expected = Mat3::FromRows({7, 2, 0}, {3, 1, 0}, {0, 0, 2});
  EXPECT_EQ(a * b, expected);
  EXPECT_EQ(a * Vec3(1, 1, 1), (Vec3{3, 1, 2}));
}

TEST(Mat3Test, InverseAndDeterminant) {
  const Mat3 m = Mat3::FromRows({2, -1, 0}, {-1, 2, -1}, {0, -1, 2});
  EXPECT_NEAR(Determinant(m), 4.0, 1e-12);
  const std::optional<Mat3> inv = Inverse(m);
  ASSERT_TRUE(inv.has_value());
  EXPECT_TRUE(NearlyEqual(m * *inv, Mat3::Identity(), 1e-12));
  EXPECT_TRUE(NearlyEqual(*inv * m, Mat3::Identity(), 1e-12));
}

TEST(Mat3Test, SingularMatrixHasNoInverse) {
  const Mat3 singular = Mat3::FromRows({1, 2, 3}, {2, 4, 6}, {0, 1, 1});
  EXPECT_FALSE(Inverse(singular).has_value());
  EXPECT_FALSE(Inverse(Mat3::Zero()).has_value());
}

TEST(Mat3Test, TinyButWellConditionedMatrixInverts) {
  // e.g. the inertia tensor of a small part, in kg m^2.
  const Mat3 tiny = Mat3::Diagonal({2e-6, 3e-6, 1e-6});
  const std::optional<Mat3> inv = Inverse(tiny);
  ASSERT_TRUE(inv.has_value());
  EXPECT_TRUE(NearlyEqual(tiny * *inv, Mat3::Identity(), 1e-12));
}

TEST(Mat3Test, CrossMatrixReproducesCrossProduct) {
  const Vec3 a{0.3, -1.2, 2.5};
  const Vec3 b{4.0, 0.1, -0.7};
  EXPECT_VEC3_NEAR(CrossMatrix(a) * b, Cross(a, b), 1e-12);
}

TEST(Mat3Test, TransposeAndOuterProduct) {
  const Mat3 m = Mat3::FromRows({1, 2, 3}, {4, 5, 6}, {7, 8, 9});
  EXPECT_EQ(Transpose(m), Mat3::FromRows({1, 4, 7}, {2, 5, 8}, {3, 6, 9}));
  const Mat3 outer = OuterProduct({1, 2, 3}, {4, 5, 6});
  EXPECT_EQ(outer(1, 2), 12.0);
  EXPECT_EQ(outer(2, 0), 12.0);
  EXPECT_EQ(Mat3::Diagonal({1, 2, 3}) * Vec3(1, 1, 1), (Vec3{1, 2, 3}));
}

TEST(Mat4Test, TranslationAndScaleAffectPointsNotDirections) {
  const Mat4 t = Mat4::Translation({1, 2, 3});
  EXPECT_EQ(TransformPoint(t, Vec3::Zero()), (Vec3{1, 2, 3}));
  EXPECT_EQ(TransformDirection(t, Vec3::UnitX()), Vec3::UnitX());
  const Mat4 s = Mat4::Scale({2, 3, 4});
  EXPECT_EQ(TransformPoint(s, Vec3::One()), (Vec3{2, 3, 4}));
  EXPECT_EQ(t.TranslationPart(), (Vec3{1, 2, 3}));
}

TEST(Mat4Test, TrsAppliesScaleThenRotationThenTranslation) {
  const Quat r = FromAxisAngle(Vec3::UnitY(), kHalfPi);
  const Mat4 m = Mat4::Trs({10, 0, 0}, r, {2, 2, 2});
  // +X scaled to 2, rotated 90 deg about +Y onto -Z, then translated.
  EXPECT_VEC3_NEAR(TransformPoint(m, Vec3::UnitX()), (Vec3{10, 0, -2}), 1e-12);
  const Mat4 composed = Mat4::Translation({10, 0, 0}) * Mat4::Rotation(r) *
                        Mat4::Scale({2, 2, 2});
  EXPECT_TRUE(NearlyEqual(m, composed, 1e-12));
}

TEST(Mat4Test, GeneralInverseOfAffineAndProjectiveMatrices) {
  const Mat4 affine = SampleAffine();
  const std::optional<Mat4> inv = Inverse(affine);
  ASSERT_TRUE(inv.has_value());
  EXPECT_TRUE(NearlyEqual(affine * *inv, Mat4::Identity(), 1e-12));

  Mat4 projective = affine;
  projective(3, 0) = 0.1;
  projective(3, 2) = -0.3;
  const std::optional<Mat4> pinv = Inverse(projective);
  ASSERT_TRUE(pinv.has_value());
  EXPECT_TRUE(NearlyEqual(projective * *pinv, Mat4::Identity(), 1e-11));
  EXPECT_TRUE(NearlyEqual(*pinv * projective, Mat4::Identity(), 1e-11));
}

TEST(Mat4Test, AffineInverseMatchesGeneralInverse) {
  const Mat4 m = SampleAffine();
  const std::optional<Mat4> general = Inverse(m);
  const std::optional<Mat4> affine = AffineInverse(m);
  ASSERT_TRUE(general.has_value());
  ASSERT_TRUE(affine.has_value());
  EXPECT_TRUE(NearlyEqual(*general, *affine, 1e-12));
}

TEST(Mat4Test, DeterminantOfTrsIsProductOfScales) {
  EXPECT_NEAR(Determinant(SampleAffine()), 2.0 * 0.5 * 1.25, 1e-12);
  EXPECT_FALSE(Inverse(Mat4::Scale({1, 0, 1})).has_value());
}

TEST(Mat4Test, RankDeficientMatrixHasNoInverse) {
  // Column 2 equals column 0 plus column 1, so the matrix has rank 3.
  const Mat4 m =
      Mat4::FromColumns({1, 2, 3, 0}, {0, 1, 4, 0}, {1, 3, 7, 0}, {5, 6, 7, 1});
  EXPECT_FALSE(Inverse(m).has_value());
  EXPECT_FALSE(AffineInverse(m).has_value());
}

TEST(Mat4Test, TinyUniformScaleStillInverts) {
  const Mat4 tiny = Mat4::Trs({1e-3, 0, 0}, FromAxisAngle(Vec3::UnitY(), 0.4),
                              {1e-6, 1e-6, 1e-6});
  const std::optional<Mat4> inv = Inverse(tiny);
  const std::optional<Mat4> affine_inv = AffineInverse(tiny);
  ASSERT_TRUE(inv.has_value());
  ASSERT_TRUE(affine_inv.has_value());
  EXPECT_TRUE(NearlyEqual(tiny * *inv, Mat4::Identity(), 1e-9));
  EXPECT_TRUE(NearlyEqual(tiny * *affine_inv, Mat4::Identity(), 1e-9));
}

TEST(Mat4Test, TransposeTwiceIsIdentityOperation) {
  const Mat4 m = SampleAffine();
  EXPECT_EQ(Transpose(Transpose(m)), m);
  EXPECT_EQ(Transpose(m)(0, 3), m(3, 0));
}

TEST(Mat4Test, NormalMatrixKeepsNormalsPerpendicularUnderNonUniformScale) {
  const Mat4 model = Mat4::Scale({4.0, 1.0, 1.0});
  // A surface tangent and its normal on a 45-degree slope.
  const Vec3 tangent = Normalized(Vec3{1.0, 1.0, 0.0});
  const Vec3 normal = Normalized(Vec3{-1.0, 1.0, 0.0});
  const Vec3 world_tangent = TransformDirection(model, tangent);
  const Vec3 world_normal = NormalMatrix(model) * normal;
  EXPECT_NEAR(Dot(world_tangent, world_normal), 0.0, 1e-12);
}

TEST(Mat4Test, FloatArrayPreservesColumnMajorOrder) {
  const Mat4 t = Mat4::Translation({1, 2, 3});
  const auto floats = t.ToFloatArray();
  EXPECT_FLOAT_EQ(floats[12], 1.0f);
  EXPECT_FLOAT_EQ(floats[13], 2.0f);
  EXPECT_FLOAT_EQ(floats[14], 3.0f);
  EXPECT_FLOAT_EQ(floats[15], 1.0f);
}

TEST(Mat4Test, ProjectiveTransformDividesByW) {
  Mat4 m;
  m(3, 3) = 2.0;
  EXPECT_VEC3_NEAR(TransformPointProjective(m, {2, 4, 6}), (Vec3{1, 2, 3}),
                   1e-12);
}

}  // namespace
}  // namespace aerolab
