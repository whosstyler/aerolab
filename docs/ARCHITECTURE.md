# AeroLab Architecture

AeroLab is a small, purpose-built 3D simulation engine and editor for exploring
the mathematics, physics, and graphics programming behind fast-moving aerial
objects. It is an **educational sandbox**: every object is a generic rigid body,
a kinematic path or a self-flying game-AI agent driven by configurable toy
models. Agents only head for points they choose themselves and only ever
steer away from other objects; there is no guidance, targeting, or
interception logic anywhere in the code base.

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
                   |        world         |          |
                   +----------+-----------+          |
                              |        +-----------+ |
                              |        |  render   | |  render <- math,
                              v        +-----+-----+ |  core, geometry (GL)
                   +----------------------+  |       |
                   |   editor (ImGui)     |<-+-------+
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
| `aerolab_world`    | scene, sim, geometry, core     | no              | no                 |
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
Application::Run() per frame
  1. ProcessEvents     SDL events -> ImGui backend + InputState
  2. UpdateSimulation  SimulationSystem::Update -> 0..N fixed steps;
                       events -> EventLog (stamped with sim time)
  3. UpdateCamera      CameraRig with real (unscaled) frame time; mouse only
                       over the viewport, keyboard only when no text field
                       is active (state reported by last frame's UI). Runs
                       after the simulation so follow mode tracks the pose
                       drawn this frame
  4. RenderScene       ScenePresenter -> RenderItems, debug visualizer ->
                       DebugDrawList, Renderer -> MSAA RenderTarget,
                       resolve (GPU-timed)
  5. RenderUi          Editor::Draw (viewport panel shows the resolved
                       texture, does picking and overlays with the same
                       camera), ImGui -> default framebuffer
  6. SwapBuffers
```

The viewport size the UI asks for is applied on the next frame. Overlays,
picking and the 3D image are all built from the same camera state within a
frame, so labels never lag the geometry.

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

## Design notes

**Fixed step and interpolation.** `FixedTimestep` turns variable frame time
into whole steps (capped per frame; the backlog is dropped rather than
allowed to snowball). Each body keeps its previous and current state;
`RenderWorldMatrix` blends them with the accumulator fraction, and spinning
parts blend their angle the same way. Picking, focus and the follow camera
use the same interpolated pose as the drawn image.

**Bodies in a hierarchy.** Rigid bodies are integrated in world space and
are independent of one another: a body parented under another body keeps
its own world-space motion. After all bodies step, their poses are written
back to the scene parents-first with `Scene::SetWorldPose`, so the scene
graph agrees with the simulation, and non-body descendants (rotors,
markers) follow their parent's pose.

**Scene storage.** Entities live in a slot array addressed by generational
`EntityId`s; a handle to a destroyed entity never aliases a newer one.
Components are `std::optional` members of `Entity` (render, spin, body,
trail) - a deliberately simple alternative to a full ECS.

**Renderer contract.** The renderer draws whatever framebuffer is bound
from three flat inputs: `RenderView`, a span of `RenderItem`s and two
`DebugDrawList`s (depth-tested and overlay). It establishes and restores a
known GL state, so ImGui can share the context.

**Editor contract.** `Editor::Draw(EditorFrame&)` receives non-owning
pointers to the application state for one frame and returns
`EditorRequests` (quit, vsync). The editor owns only UI state; all scene,
simulation and camera changes go through their public APIs.

**Numerics worth knowing.** See README "Physics notes": measured
convergence orders, RK4's second-order orientation, and why the
"symplectic" label of semi-implicit Euler does not cover gyroscopic
rotation. Divergence (a non-finite state) and runaway states past the
simulation bounds are detected per body; the body is frozen (`BodyHalt`)
until the next reset instead of spreading NaNs or float overflow.

**First person.** `CameraRig` walks on the ground plane in first person
and is told what it may not walk through as `WalkBlocker` circles; the
app derives them each frame from the same bounding spheres the agents
sense (`WalkBlockerFromSphere` keeps only spheres within body height).
`ViewpointState` (world/selection.h, owned by the application) names the
observer being looked through: the app moves that figure with the camera
(and moves the camera if something else moves the figure, such as an
inspector edit) and leaves it out of rendering, the selection and debug
overlays, labels and picking.

**Autonomous agents.** `sim/flight_agent` is pure decision and motion
logic with no scene types. `PlanAgent(params, obstacles, dt, motion,
brain)` senses, decides and steers, leaving the wanted velocity in the
brain; `StepAgent` adds the simple point-mass move. With
`FlightModel::kPhysical`, `StepPhysicalAgent` (sim/agent_vehicle) runs the
same plan and then flies an `AgentVehicle`: an autopilot from
sim/flight_control sets the actuators of a sim/multirotor or sim/fixed_wing
airframe, whose wrench (sim/aerodynamics, sim/atmosphere) is integrated
with the generic `StepRk4(state, mass, WrenchFunction, time, dt)`. The
vehicle, like the brain, is plain copyable data, so the forecast copies
both. `world/autonomy` is the sensing side: it turns the scene into
bounding-sphere obstacles (bodies plus free-standing props; a body's
attached parts such as rotors are skipped) once per fixed step, before any
body moves, so stepping order does not matter. The brain, including its
SplitMix64 generator, is plain copyable data: reset re-seeds it, and the
look-ahead copies it and steps the copy (moving obstacles along their
current velocities), which is why the forecast line is exact in a static
scene. Avoidance feelers are cast in each obstacle's frame (relative
velocity), and `SanitizeAgentParams` makes any parameter set safe.
Decisions surface as `SimulationEvent::Kind::kDecision` entries in
the event log (not on stderr, they are frequent).

## Status

All ten stages are implemented. Each stage is a separate commit (or a
library commit followed by an integration commit) on the feature branch,
reviewed before commit; see `git log` for details.
