# Physics Simulator

A from-scratch rigid-body physics engine: spheres and boxes, gravity,
sequential-impulse contact resolution with friction, and box-box collision
via the Separating Axis Theorem (SAT).

## Project layout

```
physics-sim/
├── CMakeLists.txt
├── include/
│   ├── math/
│   │   ├── vec3.h        Vec3: add/sub/scale/dot/cross
│   │   └── quat.h         Quat + Mat3 (orientation, rotation matrix)
│   └── physics/
│       ├── shape.h        Sphere / Box shape data
│       ├── rigidbody.h    Pose, velocity, mass, inertia, integration
│       ├── contact.h       Contact struct (normal, point, penetration)
│       ├── collision.h     Narrow-phase collision function declarations
│       ├── solver.h        Sequential impulse + positional correction
│       └── world.h         World: owns bodies, runs the step loop
└── src/
    ├── rigidbody.cpp
    ├── collision.cpp       Sphere-sphere, sphere-box, box-box (SAT), vs ground
    ├── solver.cpp
    ├── world.cpp
    └── main.cpp             Demo scene: spheres + a box stack
```

## Build & run

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
./physics_sim
```

## What's implemented

- Semi-implicit Euler integration for position, orientation (quaternion-based)
- Sphere-sphere, sphere-box, and box-box (SAT, face axes) collision detection
- Sphere-plane and box-plane (per-corner) contact vs. an implicit ground
- Sequential impulse solver with restitution, applied at the actual contact
  point (so boxes get correct torque, not just spheres)
- Coulomb friction, clamped to the normal impulse magnitude
- Baumgarte-style positional correction to prevent sinking

## Verified status (tested by actually running it, including step-by-step debugging)

- **Spheres**: fall, bounce with restitution, settle correctly. Verified.
- **Single box vs. ground**: settles correctly. Verified.
- **Sphere vs. box**: settles correctly. Verified.
- **Box-vs-box stacking: NOW STABLE.** Previously exploded (see bug history
  below) — fixed by correcting the solver's effective-mass calculation.
  Verified with an isolated 2-box test and the full 3-box demo scene; boxes
  settle with near-zero velocity instead of gaining energy.

## Bugs found and fixed during testing (real debugging history, not hypothetical)

**Bug 1 — redundant ground contacts.** `boxPlaneContact` originally emitted
one contact per penetrating corner (up to 4). The solver treated these as
independent, fully redundant constraints on the same vertical motion, and
corrections compounded instead of converging — a box on flat ground shot
upward at 30+ units/sec within a few frames. Fixed by reducing all
penetrating corners to one contact per box (deepest penetration, averaged
point).

**Bug 2 — missing rotational term in effective mass (the big one).** After
adding a real contact manifold for box-box (4 clipped points instead of 1
approximate point), boxes exploded even worse than before. Root-caused by
building an isolated single-contact reproduction case and inspecting
before/after velocities directly (see git history / commit notes if you
want the actual debug session). The bug: `resolveContact`'s impulse
formula divided by `invMassA + invMassB` only. That's only correct for
contacts that pass through the center of mass (true for spheres, since
r=0 there). For any **off-center** contact — every box corner — resisting
motion also fights rotational inertia, which adds a term per body:
`normal . ( (I^-1 * (r x normal)) x r )`. Skipping it meant the solver
underestimated how much each contact resists motion, so impulses
overshot and the box spun and flew apart. Fixed in `solver.cpp` via
`effectiveMassDenominator()`, using the new `RigidBody::applyWorldInvInertia()`
helper (also used by `applyImpulseAtPoint`, so both paths are consistent).
This is a classic physics-engine bug — it's silent for sphere-only scenes
and only appears once contacts stop passing through the center of mass.

## What's now implemented in the box-box collision path

- Full 15-axis SAT (6 face axes + 9 edge-edge cross-product axes) —
  previously only tested the 6 face axes.
- Proper contact manifold generation for face-face contacts via
  Sutherland-Hodgman clipping (up to 4 points), instead of one
  approximate midpoint. See `getFaceVerts`/`clipPolygonAgainstPlane`/
  `boxBoxContact` in `collision.cpp`.
- Single-point resolution for genuine edge-edge contacts (physically
  correct — an edge-edge collision is a single point in exact geometry).
- Solver correctly accounts for rotational inertia at every contact point,
  not just linear mass (see Bug 2 above).

## Remaining known simplifications

1. **Small positional drift** — in a 2-box resting test, the top box drifts
   laterally by ~0.1 units over several seconds even though it should stay
   put. This traces to using the box's *current* position each solver
   iteration to compute `r` for the effective-mass term, rather than a
   fixed reference frame for the whole timestep — a small, bounded
   inaccuracy, not a stability problem.
2. **No warm starting.** `accumNormalImpulse`/`accumTangentImpulse` reset to
   zero every frame instead of carrying over from the previous frame's
   matching contact point. Real engines cache impulses across frames
   (matched by contact ID) for faster convergence and less jitter in
   persistent stacks.
3. **Edge-edge contact point is approximate**, not the exact closest-point-
   between-two-segments solution. Fine for stability, slightly imprecise
   for exact contact placement in edge-on-edge configurations.
4. **No broad-phase.** Every pair of bodies is tested every frame (O(n^2)).
5. **No joints/constraints yet.** The `Contact`/solver pattern generalizes
   directly — a joint is just another constraint type solved the same way.

## Windowed viewer (physics_sim_gl)

A live OpenGL viewer is included, built as a **separate executable** so the
dependency-free console demo above still builds even without GLFW installed.

```
include/render/renderer.h     Window/camera/draw-box/draw-sphere/draw-grid API
src/render/renderer.cpp        Implementation (legacy fixed-function GL + GLFW)
src/render/main_gl.cpp          Windowed main: runs the same demo scene, renders it live
```

It uses **legacy fixed-function OpenGL** (`glBegin`/`glEnd`, `glLoadMatrixf`)
rather than a modern shader pipeline — deliberately, to keep the renderer
small. A hand-written `lookAt`/perspective-frustum setup avoids needing GLU
as a dependency.

### Install GLFW first

```bash
# Ubuntu/Debian
sudo apt install libglfw3-dev libgl1-mesa-dev

# macOS
brew install glfw

# Windows
vcpkg install glfw3
```

### Build

CMake auto-detects GLFW and only builds `physics_sim_gl` if found:

```bash
mkdir build && cd build
cmake ..
cmake --build .
./physics_sim_gl
```

Or compile directly without CMake:

```bash
g++ -O2 -std=c++17 -Iinclude src/render/main_gl.cpp src/render/renderer.cpp \
    src/rigidbody.cpp src/collision.cpp src/solver.cpp src/world.cpp \
    -lglfw -lGL -o physics_sim_gl
./physics_sim_gl
```

### Verified

This was actually compiled and run (not just written) — under Xvfb (a
virtual display, since this dev environment has no GPU/monitor) with a
screenshot captured to confirm real rendering, not just "it didn't crash."
See `viewer-screenshot.png` in this folder: shaded box, orange sphere,
ground grid, correct perspective and lighting all confirmed working from
the actual compiled binary.

### Known limitations of the viewer

- **Sphere orientation isn't drawn.** Spheres are visually symmetric, so
  the mesh ignores `orientation` when drawing (physics still tracks it
  correctly internally, it's just not visualized — add a marker line/texture
  seam if you want to see spin).
- **No camera controls.** The camera is fixed (see `Camera cam` in
  `main_gl.cpp`). Adding mouse-look/orbit controls via GLFW's cursor
  callbacks is a natural next step.
- **Box-on-box stacking is still unstable** (see "Known simplifications"
  above) — you'll see this visually now instead of just in printed numbers.


