#ifndef AEROLAB_TESTS_TEST_UTIL_H_
#define AEROLAB_TESTS_TEST_UTIL_H_

#include <gtest/gtest.h>

#include <format>

#include "math/mat3.h"
#include "math/mat4.h"
#include "math/quat.h"
#include "math/vec3.h"

namespace aerolab::test_util {

inline ::testing::AssertionResult Vec3Near(const char* a_expr,
                                           const char* b_expr,
                                           const char* /*tol_expr*/,
                                           const Vec3& a, const Vec3& b,
                                           double tolerance) {
  if (NearlyEqual(a, b, tolerance)) return ::testing::AssertionSuccess();
  return ::testing::AssertionFailure() << std::format(
             "{} = ({:.12g}, {:.12g}, {:.12g}) vs {} = "
             "({:.12g}, {:.12g}, {:.12g}), tolerance {}",
             a_expr, a.x, a.y, a.z, b_expr, b.x, b.y, b.z, tolerance);
}

inline ::testing::AssertionResult SameRotationPred(const char* a_expr,
                                                   const char* b_expr,
                                                   const char* /*tol_expr*/,
                                                   const Quat& a, const Quat& b,
                                                   double tolerance) {
  if (SameRotation(a, b, tolerance)) return ::testing::AssertionSuccess();
  return ::testing::AssertionFailure() << std::format(
             "{} = ({:.9g}, {:.9g}, {:.9g}, {:.9g}) and {} = "
             "({:.9g}, {:.9g}, {:.9g}, {:.9g}) differ",
             a_expr, a.w, a.x, a.y, a.z, b_expr, b.w, b.x, b.y, b.z);
}

}  // namespace aerolab::test_util

#define EXPECT_VEC3_NEAR(a, b, tol) \
  EXPECT_PRED_FORMAT3(::aerolab::test_util::Vec3Near, a, b, tol)
#define EXPECT_SAME_ROTATION(a, b, tol) \
  EXPECT_PRED_FORMAT3(::aerolab::test_util::SameRotationPred, a, b, tol)

#endif  // AEROLAB_TESTS_TEST_UTIL_H_
