#include "app/scene_presenter.h"

#include <gtest/gtest.h>

#include <vector>

#include "geometry/color.h"
#include "geometry/debug_draw.h"
#include "scene/scene.h"
#include "world/spawner.h"

namespace aerolab {
namespace {

int CountHighlighted(const std::vector<RenderItem>& items, float level) {
  int count = 0;
  for (const RenderItem& item : items) {
    if (item.highlight == level) ++count;
  }
  return count;
}

TEST(ScenePresenterTest, OneItemPerVisibleMesh) {
  Scene scene;
  Spawn(scene, Archetype::kCube, {0, 0, 0});
  const EntityId hidden = Spawn(scene, Archetype::kSphere, {2, 0, 0});
  scene.Get(hidden)->render->visible = false;
  scene.Create("Empty group");  // No mesh: not drawn.
  const std::vector<RenderItem> items =
      BuildRenderItems(scene, SelectionState{});
  ASSERT_EQ(items.size(), 1u);
  EXPECT_EQ(items[0].mesh, MeshKind::kCube);
  EXPECT_EQ(items[0].highlight, 0.0f);
}

TEST(ScenePresenterTest, SelectionHighlightsWholeSubtree) {
  Scene scene;
  const EntityId quad = Spawn(scene, Archetype::kQuadrotor, {0, 3, 0});
  Spawn(scene, Archetype::kCube, {5, 0, 0});
  SelectionState selection;
  selection.selected = quad;
  const std::vector<RenderItem> items = BuildRenderItems(scene, selection);
  // Frame + 4 rotors selected; the cube is not.
  EXPECT_EQ(CountHighlighted(items, 1.0f), 5);
  EXPECT_EQ(CountHighlighted(items, 0.0f), 1);
}

TEST(ScenePresenterTest, SelectionWinsOverHover) {
  Scene scene;
  const EntityId cube = Spawn(scene, Archetype::kCube, {0, 0, 0});
  SelectionState selection{cube, cube};
  const std::vector<RenderItem> items = BuildRenderItems(scene, selection);
  ASSERT_EQ(items.size(), 1u);
  EXPECT_EQ(items[0].highlight, 1.0f);
  EXPECT_EQ(items[0].highlight_color, colors::kSelection);
}

TEST(ScenePresenterTest, OverlayDrawsBoxesAndFrame) {
  Scene scene;
  const EntityId a = Spawn(scene, Archetype::kCube, {0, 0, 0});
  const EntityId b = Spawn(scene, Archetype::kSphere, {3, 0, 0});
  DebugDrawList lines;
  AddSelectionOverlay(scene, SelectionState{a, b}, lines);
  // Selected box (12) + selected axes (3) + hovered box (12).
  EXPECT_EQ(lines.segment_count(), 27u);
  lines.Clear();
  AddSelectionOverlay(scene, SelectionState{}, lines);
  EXPECT_TRUE(lines.empty());
}

}  // namespace
}  // namespace aerolab
