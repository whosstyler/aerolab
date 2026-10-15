#include "geometry/mesh_library.h"

#include <array>
#include <cstddef>
#include <string_view>

#include "geometry/color.h"
#include "geometry/primitives.h"
#include "math/mat4.h"
#include "math/quat.h"
#include "math/scalar.h"

namespace aerolab {
namespace {

// Rotates a +Y-axis primitive so its axis points along local -Z (forward).
const Quat kYToForward = FromAxisAngle(Vec3::UnitX(), -kHalfPi);

Mat4 Place(const Vec3& position, const Quat& rotation = Quat::Identity(),
           const Vec3& scale = Vec3::One()) {
  return Mat4::Trs(position, rotation, scale);
}

MeshData BuildDart() {
  constexpr double kRadius = 0.11;
  constexpr double kBodyHalfLength = 0.75;
  const MeshData body = MakeCylinder(kRadius, kBodyHalfLength, 24);
  const MeshData nose = MakeCone(kRadius, 0.35, 24);
  const MeshData fin = MakeBox({0.008, 0.16, 0.12});
  const MeshData nozzle = MakeCylinder(kRadius * 0.7, 0.04, 20);

  MeshBuilder builder;
  builder.Append(body, Place(Vec3::Zero(), kYToForward), FromHex(0xE8EAED));
  builder.Append(nose, Place({0.0, 0.0, -kBodyHalfLength}, kYToForward),
                 FromHex(0xE5674D));
  builder.Append(nozzle, Place({0.0, 0.0, kBodyHalfLength + 0.04}, kYToForward),
                 FromHex(0x3A3F47));
  // Four fins in a cross at the tail, offset outward from the body surface.
  for (int i = 0; i < 4; ++i) {
    const Quat roll = FromAxisAngle(Vec3::UnitZ(), kHalfPi * i);
    const Vec3 offset = Rotate(roll, Vec3{0.0, kRadius + 0.15, 0.0});
    builder.Append(fin,
                   Place(offset + Vec3{0.0, 0.0, kBodyHalfLength - 0.12}, roll),
                   FromHex(0x2F5D8C));
  }
  return builder.Build();
}

MeshData BuildFixedWing() {
  const MeshData fuselage = MakeCylinder(0.09, 0.62, 20);
  const MeshData nose = MakeUvSphere(0.09, 20, 10);
  const MeshData tail_cone = MakeCone(0.09, 0.35, 20);
  const MeshData wing = MakeBox({1.2, 0.015, 0.14});
  const MeshData wing_tip = MakeBox({0.06, 0.016, 0.141});
  const MeshData stabilizer = MakeBox({0.36, 0.01, 0.08});
  const MeshData fin = MakeBox({0.01, 0.16, 0.09});
  const MeshData blade = MakeBox({0.16, 0.004, 0.02});

  const Color body_color = FromHex(0xD9DCE1);
  const Color dark = FromHex(0x3A3F47);
  MeshBuilder builder;
  builder.Append(fuselage, Place(Vec3::Zero(), kYToForward), body_color);
  builder.Append(nose, Place({0.0, 0.0, -0.62}), body_color);
  // Tail cone tapers backward (+Z): rotate the +Y apex onto +Z.
  builder.Append(tail_cone,
                 Place({0.0, 0.0, 0.62}, FromAxisAngle(Vec3::UnitX(), kHalfPi)),
                 body_color);
  builder.Append(wing, Place({0.0, 0.05, -0.12}), FromHex(0xF1F3F5));
  // Navigation colours: green on starboard (+X), red on port (-X).
  builder.Append(wing_tip, Place({1.17, 0.05, -0.12}), FromHex(0x46A758));
  builder.Append(wing_tip, Place({-1.17, 0.05, -0.12}), FromHex(0xE5484D));
  builder.Append(stabilizer, Place({0.0, 0.03, 0.88}), FromHex(0xF1F3F5));
  builder.Append(fin, Place({0.0, 0.16, 0.9}), FromHex(0x2F5D8C));
  // Pusher propeller behind the tail cone.
  builder.Append(blade, Place({0.0, 0.0, 0.99}), dark);
  builder.Append(blade,
                 Place({0.0, 0.0, 0.99}, FromAxisAngle(Vec3::UnitZ(), kHalfPi)),
                 dark);
  return builder.Build();
}

MeshData BuildQuadrotor() {
  constexpr double kArmLength = 0.36;
  const MeshData hub = MakeBox({0.11, 0.04, 0.11});
  const MeshData canopy = MakeUvSphere(0.08, 16, 8);
  const MeshData arm = MakeBox({kArmLength * 0.5, 0.012, 0.018});
  const MeshData motor = MakeCylinder(0.03, 0.03, 14);
  const MeshData leg = MakeBox({0.008, 0.06, 0.008});

  MeshBuilder builder;
  builder.Append(hub, Place(Vec3::Zero()), FromHex(0x3A3F47));
  builder.Append(canopy,
                 Place({0.0, 0.04, 0.0}, Quat::Identity(), {1.0, 0.5, 1.3}),
                 FromHex(0xD9DCE1));
  for (int i = 0; i < 4; ++i) {
    // Arms at 45, 135, 225, 315 degrees (an "X" configuration).
    const double angle = kPi * 0.25 + kHalfPi * i;
    const Quat yaw = FromAxisAngle(Vec3::UnitY(), angle);
    const Vec3 tip = Rotate(yaw, Vec3{kArmLength, 0.0, 0.0});
    builder.Append(arm, Place(tip * 0.5, yaw), FromHex(0x4A5059));
    // Front motors (local -Z side) are coloured for orientation at a glance.
    const Color motor_color =
        tip.z < 0.0 ? FromHex(0xE5674D) : FromHex(0x2F5D8C);
    builder.Append(motor, Place(tip + Vec3{0.0, 0.02, 0.0}), motor_color);
    builder.Append(leg, Place(tip * 0.45 + Vec3{0.0, -0.08, 0.0}),
                   FromHex(0x4A5059));
  }
  return builder.Build();
}

MeshData BuildRotor() {
  const MeshData blade = MakeBox({0.13, 0.004, 0.016});
  const MeshData cap = MakeCylinder(0.015, 0.01, 10);
  MeshBuilder builder;
  builder.Append(blade, Place(Vec3::Zero()), FromHex(0x1F2328));
  builder.Append(blade,
                 Place(Vec3::Zero(), FromAxisAngle(Vec3::UnitY(), kHalfPi)),
                 FromHex(0x1F2328));
  builder.Append(cap, Place({0.0, 0.01, 0.0}), FromHex(0x9AA0A6));
  return builder.Build();
}

}  // namespace

std::string_view MeshKindName(MeshKind kind) {
  switch (kind) {
    case MeshKind::kCube:
      return "Cube";
    case MeshKind::kSphere:
      return "Sphere";
    case MeshKind::kCylinder:
      return "Cylinder";
    case MeshKind::kCone:
      return "Cone";
    case MeshKind::kDart:
      return "Dart";
    case MeshKind::kFixedWing:
      return "Fixed wing";
    case MeshKind::kQuadrotor:
      return "Quadrotor";
    case MeshKind::kRotor:
      return "Rotor";
    case MeshKind::kCount:
      break;
  }
  return "Unknown";
}

const Aabb& MeshLocalBounds(MeshKind kind) {
  // Built on first use; function-local statics initialize thread-safely.
  static const std::array<Aabb, kMeshKindCount> kBounds = [] {
    std::array<Aabb, kMeshKindCount> bounds;
    for (MeshKind k : AllMeshKinds()) {
      bounds[static_cast<std::size_t>(k)] = BuildMesh(k).bounds;
    }
    return bounds;
  }();
  const auto index = static_cast<std::size_t>(kind);
  return kBounds[index < kBounds.size() ? index : 0];
}

MeshData BuildMesh(MeshKind kind) {
  switch (kind) {
    case MeshKind::kCube:
      return MakeBox({0.5, 0.5, 0.5});
    case MeshKind::kSphere:
      return MakeUvSphere(0.5, 32, 16);
    case MeshKind::kCylinder:
      return MakeCylinder(0.5, 0.5, 32);
    case MeshKind::kCone:
      return MakeCone(0.5, 1.0, 32);
    case MeshKind::kDart:
      return BuildDart();
    case MeshKind::kFixedWing:
      return BuildFixedWing();
    case MeshKind::kQuadrotor:
      return BuildQuadrotor();
    case MeshKind::kRotor:
      return BuildRotor();
    case MeshKind::kCount:
      break;
  }
  return MakeBox({0.5, 0.5, 0.5});
}

}  // namespace aerolab
