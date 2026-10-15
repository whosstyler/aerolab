# AeroLab Architecture

AeroLab is a small, purpose-built 3D simulation engine and editor for exploring
the mathematics, physics, and graphics programming behind fast-moving aerial
objects. It is an **educational sandbox**: every object is a generic rigid body
or kinematic path driven by configurable toy force models. There is no
guidance, targeting, or interception logic anywhere in the code base.

## Goals

* A genuine 3D viewport (perspective projection, lighting, materials, a
  navigable camera) is the centerpiece.
* Each subsystem is an independent CMake library with an explicit dependency
  direction, so math, simulation, rendering, and UI can evolve separately.
* Simulation is deterministic, runs at a fixed timestep, and is covered by unit
  tests that do not need a GPU or a window.

## Module graph

```
                  +-----------+
                  |   math    |  Vec2/3/4, Mat3/4, Quat, Transform, Ray, Aabb
                  +-----+-----+
          +-------------+--------------+-------------+
          v             v              v             v
     +---------+   +----------+   +---------+   +----------+
     |  core   |   | geometry |   |   sim   |   |  camera  |<-- core
     +----+----+   +----+-----+   +----+----+   +----+-----+
          |             |              |             |
          |             v              v             |
          |        +----------------------+          |
          +------->|        scene         |          |
                   +----------+-----------+          |
                              v                      |
                   +----------------------+          |
                   |        world         |<---------+
                   +----------+-----------+
                              |        +-----------+
                              |        |  render   |<-- math, geometry (GL)
                              v        +-----+-----+
                   +----------------------+  |
                   |   editor (ImGui)     |<-+
                   +----------+-----------+
                              v
                   +----------------------+
                   |   app (SDL3, main)   |
                   +----------------------+
```

| Library            | Depends on                     | Knows about GL? | Knows about ImGui? |
|--------------------|--------------------------------|-----------------|--------------------|
| `aerolab_math`     | -                              | no              | no                 |
| `aerolab_core`     | math                           | no              | no                 |
| `aerolab_geometry` | math                           | no              | no                 |
| `aerolab_sim`      | math                           | no              | no                 |
| `aerolab_camera`   | math, core                     | no              | no                 |
| `aerolab_scene`    | math, core, geometry, sim      | no              | no                 |
| `aerolab_world`    | scene, sim, geometry, camera, core | no          | no                 |
| `aerolab_render`   | math, core, geometry           | **yes**         | no                 |
| `aerolab_editor`   | everything above + ImGui       | via render API  | **yes**            |
| `aerolab` (app)    | everything + SDL3              | via render API  | backends only      |

The physics code never includes an OpenGL or ImGui header. The renderer never
includes a scene or simulation header: the application translates the scene
into a flat list of `RenderItem`s and a `DebugDrawList` each frame.

Because all libraries share the `src/` include root, the linker alone cannot
stop a module from including a header it should not depend on. The
`module_dependencies` test (`cmake/CheckModuleDependencies.cmake`) scans every
`#include` and fails the build's test run on any violation of the table above,
including SDL use outside `app` and ImGui use outside `editor`/`app`.

## Conventions

* **Units:** SI (metres, seconds, kilograms, radians internally; degrees only in
  the UI).
* **World space:** right-handed, **+Y up**, +X right/east, +Z toward the default
  viewer. The ground is the `y = 0` plane.
* **Local (body) space:** +X right, +Y up, **-Z forward**. The same convention
  as camera space, so "forward" means the same thing for cameras and bodies.
* **Camera (view) space:** OpenGL convention; the camera looks down -Z.
* **Clip space:** OpenGL NDC with depth in [-1, 1].
* **Matrices:** column vectors, column-major storage (`M * v`), uploaded to GL
  with `transpose = GL_FALSE`.
* **Quaternions:** Hamilton convention `(w, x, y, z)`, rotating `v' = q v q*`,
  representing body-to-world orientation.
* **Scalar type:** `double` on the CPU; converted to `float` only at the GPU
  boundary.
* **Euler angles:** intrinsic yaw (Y) -> pitch (X') -> roll (Z''); prefer
  designated initializers such as `EulerAngles{.pitch = 0.2}`.

### Style exceptions

The code follows the Google C++ Style Guide. One deliberate exception: the
OpenGL declarations in `render/gl.h` keep the C API spelling (`glDrawArrays`,
`GL_TRIANGLES`) so they read like every GL reference. They live in namespace
`aerolab` and are generated from a single X-macro list.

## Frame anatomy

```
Application::RunFrame()
  1. Pump SDL events -> ImGui backend + InputState
  2. Update camera rig with real (unscaled) frame time
  3. SimulationSystem::Update(frame_dt)  -> 0..N fixed steps (accumulator)
  4. Build RenderItems + DebugDrawList from the scene (interpolated poses)
  5. Render the 3D scene into the viewport framebuffer (MSAA, resolved)
  6. Build editor UI (viewport panel shows the framebuffer texture)
  7. Render ImGui draw data, swap buffers
```

Simulation uses the "fix your timestep" pattern: the frame time (scaled by the
user's time scale) is accumulated and consumed in fixed `dt` steps. Rendering
interpolates between the previous and current body states with the leftover
`alpha`, so motion is smooth at any frame rate and the physics result never
depends on frame rate.

## Implementation stages

Each stage leaves the application runnable.

1. CMake project, SDL3 window, OpenGL 3.3 core context, main loop, timing.
2. Math library (vectors, matrices, quaternions, transforms, projections,
   coordinate-space helpers) with GoogleTest coverage.
3. Renderer: GL loader, shaders, buffers, meshes, procedural geometry, sky and
   horizon, world grid, axes, directional lighting, materials.
4. Free-fly and orbit camera controllers, frame-rate independent.
5. Scene: entities, transform hierarchy, components, spawning/removal,
   ray picking and selection.
6. Fixed-timestep simulation: rigid-body state, composable force models,
   explicit Euler / semi-implicit Euler / velocity Verlet / RK4 integrators,
   kinematic paths, deterministic tests.
7. Trajectory trails and 3D debug visualization (velocity, acceleration,
   frames, bounds, drop lines, predicted paths).
8. Dear ImGui docking editor shell (viewport panel, hierarchy, inspector,
   bottom panel, toolbar).
9. Object inspection, simulation controls, telemetry plots, integrator
   comparison tooling.
10. Rendering polish, GPU/CPU profiling, tests, documentation.
