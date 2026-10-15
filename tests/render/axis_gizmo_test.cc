#include "render/axis_gizmo.h"

#include <gtest/gtest.h>

namespace aerolab {
namespace {

TEST(AxisGizmoTest, SizeScalesWithViewportWithinBounds) {
  EXPECT_EQ(ComputeAxisGizmoLayout(700, 700).size, 100);
  // Clamped for tiny and huge viewports; the smaller side decides.
  EXPECT_EQ(ComputeAxisGizmoLayout(100, 100).size, AxisGizmoLayout::kMinSize);
  EXPECT_EQ(ComputeAxisGizmoLayout(4000, 3000).size, AxisGizmoLayout::kMaxSize);
  EXPECT_EQ(ComputeAxisGizmoLayout(3000, 560).size, 80);
}

TEST(AxisGizmoTest, MarginIsAFractionOfSize) {
  const AxisGizmoLayout layout = ComputeAxisGizmoLayout(840, 840);
  EXPECT_EQ(layout.size, 120);
  EXPECT_EQ(layout.x, 20);
  EXPECT_EQ(layout.y, 20);
}

}  // namespace
}  // namespace aerolab
