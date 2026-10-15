#ifndef AEROLAB_GEOMETRY_DEBUG_DRAW_H_
#define AEROLAB_GEOMETRY_DEBUG_DRAW_H_

#include <span>
#include <vector>

#include "geometry/color.h"
#include "math/aabb.h"
#include "math/mat4.h"
#include "math/vec3.h"

namespace aerolab {

// One end of a debug line segment, in world space.
struct DebugVertex {
  float x = 0.0f, y = 0.0f, z = 0.0f;
  Color color;
};

// Immediate-mode list of world-space line segments: polylines, grids, axes,
// boxes, arrows. Pure CPU data; the renderer uploads and draws it.
//
// Typical use: Clear() once per frame, add shapes, hand to the renderer.
class DebugDrawList {
 public:
  DebugDrawList() = default;

  void Clear() { vertices_.clear(); }
  bool empty() const { return vertices_.empty(); }

  // Pairs of vertices; each pair is one GL_LINES segment.
  std::span<const DebugVertex> vertices() const { return vertices_; }
  std::size_t segment_count() const { return vertices_.size() / 2; }

  void AddLine(const Vec3& a, const Vec3& b, const Color& color);
  // A segment whose colour (and alpha) blends from `color_a` to `color_b`.
  void AddLine(const Vec3& a, const Vec3& b, const Color& color_a,
               const Color& color_b);

  // Connected segments through `points`; optionally closed into a loop.
  void AddPolyline(std::span<const Vec3> points, const Color& color,
                   bool closed = false);

  // Polyline whose alpha ramps from `tail_alpha` at the first point to the
  // colour's own alpha at the last point. Used for fading motion trails.
  void AddFadingPolyline(std::span<const Vec3> points, const Color& color,
                         float tail_alpha);

  // Square grid on the plane through `center` spanned by unit `axis_u` and
  // `axis_v`, with `half_cells` cells on each side of the centre.
  void AddGrid(const Vec3& center, const Vec3& axis_u, const Vec3& axis_v,
               double cell_size, int half_cells, const Color& color);

  // X/Y/Z axes of the frame `transform` as red/green/blue lines of `length`.
  void AddAxes(const Mat4& transform, double length);

  void AddAabb(const Aabb& box, const Color& color);

  // Box `local_box` transformed by `transform` (an oriented bounding box).
  void AddOrientedBox(const Aabb& local_box, const Mat4& transform,
                      const Color& color);

  void AddCircle(const Vec3& center, const Vec3& normal, double radius,
                 const Color& color, int segments = 48);

  // Three orthogonal great circles.
  void AddWireSphere(const Vec3& center, double radius, const Color& color,
                     int segments = 48);

  // Arrow from `origin` along `vector` with a four-fin head. The head size
  // scales with the arrow but is capped at `max_head_length`.
  void AddArrow(const Vec3& origin, const Vec3& vector, const Color& color,
                double max_head_length = 0.4);

  // Small 3D cross marking a point.
  void AddCross(const Vec3& point, double half_size, const Color& color);

  // Appends every segment of `other`.
  void Append(const DebugDrawList& other);

 private:
  std::vector<DebugVertex> vertices_;
};

}  // namespace aerolab

#endif  // AEROLAB_GEOMETRY_DEBUG_DRAW_H_
