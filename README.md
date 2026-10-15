# AeroLab

AeroLab is a small, purpose-built **3D simulation engine and editor** for
learning the mathematics, physics, graphics programming and software
architecture behind fast-moving aerial objects. It is written from the
ground up in C++20 on SDL3, OpenGL 3.3 and Dear ImGui, with its own math
library and a GoogleTest suite.

It is an educational sandbox. Every object is a generic rigid body, a
scripted path or a self-flying "game AI" agent driven by configurable toy
models: balls, boxes, model darts, a glider, a quadrotor frame, a small
plane, springs, orbits. Agents only fly to points they pick themselves and
only ever steer away from other objects; there is no guidance, targeting or
interception logic anywhere in the code base.

![AeroLab editor: hierarchy, 3D viewport, inspector and telemetry](docs/images/editor.png)

## Features

**3D viewport (the centrepiece)**
* Perspective camera with **orbit**, **free-fly** and **follow** modes,
  frame-rate independent movement, smooth focus, exponential zoom and
  pixel-accurate panning.
* Gradient sky with a visible horizon, an analytically anti-aliased infinite
  world grid with X/Z axis lines, directional sun with Blinn-Phong materials
  and hemispheric ambient light, distance fog, blob shadows, 4x MSAA.
* Procedural meshes: primitives plus generic dart, fixed-wing, quadrotor
  (with spinning rotor children) models.
* Debug layers, each switchable: motion trails, velocity, acceleration and
  per-term force arrows, local coordinate frames, oriented bounding boxes,
  drop lines to the ground, spring/attractor markers and a look-ahead of the
  selected body, plus a corner axis gizmo, object labels and a HUD.
* Click to select, hover highlighting, F to focus.

**Simulation**
* Fixed-timestep loop (default 120 Hz) with render interpolation: results
  are bit-identical at any frame rate.
* Rigid bodies with mass and principal inertia; Euler's equations for
  rotation; orientation integrated with the quaternion exponential map.
* Composable force model: gravity, linear and quadratic drag (with wind),
  timed thrust, simple wing lift, spring-damper, inverse-square attractor,
  weathervane torque, angular damping, body torque.
* Four integrators to compare: explicit Euler, semi-implicit Euler,
  velocity Verlet, RK4. An *integrator comparison* scene launches four
  identical orbits and plots their energy drift.
* Kinematic paths (circle, figure-eight, helix) with analytic derivatives.
* Ground contact with restitution and Coulomb friction; events for thrust
  burnout, impacts and numerical divergence.
* Play, pause, single step, reset and a 0.05x to 16x time scale.

**Autonomous flight (game AI)**
* The demo's quadrotor and patrol plane fly themselves; any object can be
  switched to *Autonomous (AI)* in the inspector or spawned with the *AI
  hover flight* / *AI wing flight* presets.
* Each agent runs sense -> decide -> steer -> move every fixed step: a small
  state machine (transit, loiter) picks destinations inside its flight area
  by scoring random candidates (leg length, turn needed, clear line of
  flight, novelty), chooses a cruise speed per leg, and hovers or circles
  for a while on arrival.
* Steering behaviours: arrive, meander (filtered random turning), stay in
  area, altitude band, and ray-cast obstacle avoidance ("context steering":
  fly the smallest clear deviation). Feelers work in each obstacle's moving
  frame, so agents anticipate other moving objects; wings also check the
  arc they are turning on, and anything too close pushes the agent away.
  An agent whose destination becomes occupied picks another one.
* Two flight envelopes: hover (stops, tilts up to a limit to accelerate,
  jerk-limited) and wing (minimum airspeed, bank-limited turns with a roll
  rate limit, climb and path-angle limits, coordinated-turn attitude).
* Deterministic: a seeded generator means the same seed gives the same
  flight, and the look-ahead line replays the agent's own future
  decisions (exactly, as long as nothing else changes course).
* Inspector *Autopilot* section: what it is doing, its destination, leg
  speed and timers, what it is keeping clear of, plus its main tuning knobs.
  The *AI* toolbar toggle draws destinations, the flight area, the
  steering request and the object being avoided.

**Editor**
* Docking layout: viewport centre, scene hierarchy left, inspector and
  settings right, telemetry / event log / performance at the bottom,
  toolbar on top.
* Inspector shows a body's position in **world, parent-local and camera
  space**, live velocity/acceleration/energy/angular momentum, and edits
  launch conditions, mass and inertia, every force term, paths, ground
  contact, trails and materials.
* Telemetry: simulation timeline with event markers and speed / altitude /
  acceleration / energy plots.
* Performance: FPS, frame-time graph, CPU timings per stage, GPU time
  (timer queries), draw calls, triangles, steps per frame.

## Building

Requirements: CMake 3.25+, Ninja and a C++20 compiler (tested with
llvm-mingw Clang 22 on Windows; GCC/Clang on Linux and MSVC should work).
SDL3, Dear ImGui and GoogleTest are downloaded automatically by CMake and
verified against pinned SHA-256 hashes.

```sh
cmake --preset release            # or: debug
cmake --build --preset release
./build/release/bin/aerolab
```

Run the tests (unit tests, a module-boundary check and GPU smoke tests):

```sh
cmake --preset debug && cmake --build --preset debug
ctest --preset debug              # add -LE gpu on machines without a GPU
cmake --preset ubsan && cmake --build --preset ubsan && ctest --preset ubsan
cmake --preset asan && cmake --build --preset asan && ctest --preset asan
```

`asan` builds the app and tests with AddressSanitizer + UBSan (the smoke
tests then run the editor with a selection under the sanitizers). With
llvm-mingw the ASan runtime DLL ships in the toolchain's `bin` directory,
which must be on `PATH`.

`tests-only` builds just the libraries and tests, without SDL or a GPU.

### Command line

| Option | Effect |
|---|---|
| `--frames N` | Exit after N frames (smoke testing) |
| `--screenshot PATH` | Save the final frame as PNG (needs `--frames`) |
| `--select NAME` | Select and follow an object at startup |
| `--hidden` | Do not show the window |
| `--no-vsync` | Disable vertical sync |

Example: `aerolab --frames 300 --select Glider --screenshot glider.png`.

## Controls

| Input | Action |
|---|---|
| Right drag | Orbit / look around |
| Middle drag | Pan |
| Wheel | Zoom (orbit) or cruise speed (free-fly) |
| W A S D, Q E | Move (free-fly) or slide the pivot (orbit) |
| Shift / Ctrl | Faster / slower |
| Left click | Select (click empty space to deselect) |
| F | Focus the selection |
| 1 / 2 / 3 | Free-fly / orbit / follow camera |
| Space, `.`, R | Play/pause, single step, reset |
| Delete, Esc | Remove selection, clear selection |
| F1 | Controls overlay |

Spawn objects from **Scene > Spawn** (archetype x motion preset) or the
hierarchy's **+ Add** button; drag entities in the hierarchy to re-parent.

## Architecture

```
math  ->  core, geometry, sim, camera      (foundations)
          scene    <- math, core, geometry, sim
          world    <- scene, sim, geometry, core
          render   <- math, core, geometry   (never sees scene or sim)
          editor   <- world, scene, camera, render, ImGui
          app      <- everything + SDL3
```

Each directory under `src/` is an independent static library. The physics
(`sim`) knows nothing about OpenGL or ImGui; the renderer only consumes flat
`RenderItem` and `DebugDrawList` data and never sees scene or simulation
types; the editor is the only module that uses ImGui. These rules are
enforced by the `module_dependencies` test. See
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for the module graph,
coordinate conventions, frame anatomy and design notes.

| Directory | Contents |
|---|---|
| `src/math` | Vec2/3/4, Mat3/4, Quat, Transform, projections, rays, AABBs |
| `src/core` | Logging, timing, fixed timestep, input, ring buffer, stats |
| `src/geometry` | Procedural meshes, mesh library, debug line lists |
| `src/sim` | Rigid-body state, force model, integrators, kinematic paths, flight agents |
| `src/camera` | Camera and the free-fly/orbit/follow rig |
| `src/scene` | Entities, generational handles, transform hierarchy |
| `src/world` | Simulation system, agent sensing, presets, picking, debug visualization |
| `src/render` | OpenGL 3.3 loader, shaders, meshes, MSAA target, renderer |
| `src/editor` | Dear ImGui editor panels and widgets |
| `src/app` | SDL3 platform layer, main loop, command line |
| `tests/` | GoogleTest suites per module |

## Physics notes

* **Determinism.** The simulation advances in identical fixed steps, so
  60 Hz and 144 Hz displays produce the same trajectory step for step.
* **Integrator accuracy.** Measured convergence orders (tested): explicit
  and semi-implicit Euler 1, velocity Verlet 2, RK4 4 for position. RK4's
  *orientation* is second order because it applies one averaged angular
  velocity (rotations do not commute); Lie-group methods would fix that.
* **Energy.** Symplectic Euler and Verlet keep oscillator and orbit energy
  bounded; explicit Euler gains energy; RK4 drifts very slowly. These
  guarantees apply to translation: on a fast asymmetric tumble with a coarse
  step, semi-implicit Euler can diverge, which the engine detects and stops.
* **Simulation bounds.** A body that produces a non-finite state, or passes
  1e6 m from the origin or 1e5 m/s (or rad/s), is frozen and stays put until
  the next reset, with an event explaining which limit it hit. This keeps
  single-precision rendering and the UI sane.
* **Toy models.** Drag, lift and the weathervane torque are textbook
  idealizations for teaching, not aerodynamic models of any real vehicle.
* **Agents.** Flight agents are point masses with simple envelopes, tuned
  to look plausible: they demonstrate game-AI decision making and steering,
  not the flight control of any real aircraft. Tests check their envelope,
  area and altitude limits on every step, and that they never touch an
  obstacle across many seeds. Obstacles are bounding spheres: a long
  thin prop becomes a large keep-out volume, and one big enough to engulf
  an agent's whole flight area is ignored by that agent.

## Third-party components

SDL3 (zlib), Dear ImGui (MIT), GoogleTest (BSD-3-Clause) and the Roboto font
(Apache 2.0, shipped with Dear ImGui) are fetched at build time.
