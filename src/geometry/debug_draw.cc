#include "geometry/debug_draw.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <span>

#include "math/scalar.h"

namespace aerolab {
namespace {

DebugVertex MakeVertex(const Vec3& p, const Color& color) {
  return {static_cast<float>(p.x), static_cast<float>(p.y),
          static_cast<float>(p.z), color};
}

}  // namespace

void DebugDrawList::AddLine(const Vec3& a, const Vec3& b, const Color& color) {
  AddLine(a, b, color, color);
}

void DebugDrawList::AddLine(const Vec3& a, const Vec3& b, const Color& color_a,
                            const Color& color_b) {
  vertices_.push_back(MakeVertex(a, color_a));
  vertices_.push_back(MakeVertex(b, color_b));
}

void DebugDrawList::AddPolyline(std::span<const Vec3> points,
                                const Color& color, bool closed) {
  if (points.size() < 2) return;
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    AddLine(points[i], points[i + 1], color);
  }
  if (closed && points.size() > 2) {
    AddLine(points.back(), points.front(), color);
  }
}

void DebugDrawList::AddFadingPolyline(std::span<const Vec3> points,
                                      const Color& color, float tail_alpha) {
  if (points.size() < 2) return;
  const auto last = static_cast<float>(points.size() - 1);
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const float t0 = static_cast<float>(i) / last;
    const float t1 = static_cast<float>(i + 1) / last;
    AddLine(points[i], points[i + 1],
            WithAlpha(color, tail_alpha + (color.a - tail_alpha) * t0),
            WithAlpha(color, tail_alpha + (color.a - tail_alpha) * t1));
  }
}

void DebugDrawList::AddGrid(const Vec3& center, const Vec3& axis_u,
                            const Vec3& axis_v, double cell_size,
                            int half_cells, const Color& color) {
  const double extent = cell_size * half_cells;
  for (int i = -half_cells; i <= half_cells; ++i) {
    const double offset = cell_size * i;
    AddLine(center + axis_u * offset - axis_v * extent,
            center + axis_u * offset + axis_v * extent, color);
    AddLine(center + axis_v * offset - axis_u * extent,
            center + axis_v * offset + axis_u * extent, color);
  }
}

void DebugDrawList::AddAxes(const Mat4& transform, double length) {
  const Vec3 origin = transform.TranslationPart();
  AddLine(origin,
          origin +
              Normalized(TransformDirection(transform, Vec3::UnitX())) * length,
          colors::kAxisX);
  AddLine(origin,
          origin +
              Normalized(TransformDirection(transform, Vec3::UnitY())) * length,
          colors::kAxisY);
  AddLine(origin,
          origin +
              Normalized(TransformDirection(transform, Vec3::UnitZ())) * length,
          colors::kAxisZ);
}

void DebugDrawList::AddAabb(const Aabb& box, const Color& color) {
  AddOrientedBox(box, Mat4::Identity(), color);
}

void DebugDrawList::AddOrientedBox(const Aabb& local_box, const Mat4& transform,
                                   const Color& color) {
  if (IsEmpty(local_box)) return;
  std::array<Vec3, 8> corners;
  for (int i = 0; i < 8; ++i) {
    const Vec3 local{(i & 1) ? local_box.max.x : local_box.min.x,
                     (i & 2) ? local_box.max.y : local_box.min.y,
                     (i & 4) ? local_box.max.z : local_box.min.z};
    corners[i] = TransformPoint(transform, local);
  }
  // The 12 edges connect corners whose indices differ in exactly one bit.
  for (int i = 0; i < 8; ++i) {
    for (int bit = 1; bit < 8; bit <<= 1) {
      if ((i & bit) == 0) AddLine(corners[i], corners[i | bit], color);
    }
  }
}

void DebugDrawList::AddCircle(const Vec3& center, const Vec3& normal,
                              double radius, const Color& color, int segments) {
  segments = std::max(segments, 3);
  const Vec3 n = NormalizedOr(normal, Vec3::UnitY());
  const Vec3 u = AnyPerpendicular(n);
  const Vec3 v = Cross(n, u);
  Vec3 previous = center + u * radius;
  for (int i = 1; i <= segments; ++i) {
    const double angle = kTwoPi * i / segments;
    const Vec3 point =
        center + (u * std::cos(angle) + v * std::sin(angle)) * radius;
    AddLine(previous, point, color);
    previous = point;
  }
}

void DebugDrawList::AddWireSphere(const Vec3& center, double radius,
                                  const Color& color, int segments) {
  AddCircle(center, Vec3::UnitX(), radius, color, segments);
  AddCircle(center, Vec3::UnitY(), radius, color, segments);
  AddCircle(center, Vec3::UnitZ(), radius, color, segments);
}

void DebugDrawList::AddArrow(const Vec3& origin, const Vec3& vector,
                             const Color& color, double max_head_length) {
  const double length = Length(vector);
  if (length < kEpsilon) return;
  const Vec3 tip = origin + vector;
  AddLine(origin, tip, color);
  const Vec3 direction = vector / length;
  const double head = std::min(length * 0.25, max_head_length);
  const Vec3 u = AnyPerpendicular(direction);
  const Vec3 v = Cross(direction, u);
  const Vec3 base = tip - direction * head;
  const double spread = head * 0.35;
  AddLine(tip, base + u * spread, color);
  AddLine(tip, base - u * spread, color);
  AddLine(tip, base + v * spread, color);
  AddLine(tip, base - v * spread, color);
}

void DebugDrawList::AddCross(const Vec3& point, double half_size,
                             const Color& color) {
  AddLine(point - Vec3::UnitX() * half_size, point + Vec3::UnitX() * half_size,
          color);
  AddLine(point - Vec3::UnitY() * half_size, point + Vec3::UnitY() * half_size,
          color);
  AddLine(point - Vec3::UnitZ() * half_size, point + Vec3::UnitZ() * half_size,
          color);
}

void DebugDrawList::Append(const DebugDrawList& other) {
  vertices_.insert(vertices_.end(), other.vertices_.begin(),
                   other.vertices_.end());
}

}  // namespace aerolab
