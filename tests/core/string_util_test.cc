#include "core/string_util.h"

#include <gtest/gtest.h>

namespace aerolab {
namespace {

TEST(StringUtilTest, ContainsIgnoringCase) {
  EXPECT_TRUE(ContainsIgnoringCase("Ballistic dart", "DART"));
  EXPECT_TRUE(ContainsIgnoringCase("Ballistic dart", "llis"));
  EXPECT_TRUE(ContainsIgnoringCase("anything", ""));
  EXPECT_FALSE(ContainsIgnoringCase("Glider", "orbit"));
  EXPECT_FALSE(ContainsIgnoringCase("", "x"));
}

}  // namespace
}  // namespace aerolab
