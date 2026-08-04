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

## Verified status (tested by actually running it)

- **Spheres**: fall, bounce with restitution, and settle correctly at rest. Verified.
- **Single box vs. ground**: settles correctly at rest. Verified — this
  required a real bug fix (see below).
- **Sphere vs. box**: settles correctly. Verified.
- **Box-vs-box stacking is NOT stable yet.** Boxes in the demo scene gain
  energy and fly apart rather than settling. Root cause below — this is a
  known, understood limitation, not a mystery bug, but it is not fixed in
  this version.

## Bug found and fixed during testing

The original `boxPlaneContact` emitted one contact **per penetrating
corner** (up to 4 for a box resting flat). The sequential-impulse solver
treated these as four independent, fully redundant constraints on the same
vertical motion; each pass corrected motion the others had just corrected,
compounding instead of converging, and boxes shot upward at ~30+ units/sec
within a few frames. Fixed by reducing all penetrating corners to a single
contact per box (deepest penetration, averaged point) — see the comment
block above `boxPlaneContact` in `collision.cpp` for the full explanation.

## Known simplifications (the box-box instability above traces to these)

1. **Box-box contact is a single approximate point** (midpoint between
   centers), not a clipped contact manifold. A real engine generates a
   stable 2-4 point manifold by clipping the incident face against the
   reference face (Sutherland-Hodgman style) using the SAT reference axis.
   A single point can't resist rotation the way a real face-face contact
   needs to, so torque from the solver overcorrects — this is the direct
   cause of the box-stacking explosion in the demo. **This is the most
   valuable next piece of code to write** if you want a stable stack.
2. **Box-box SAT only tests face axes (6), not edge-edge axes (9 more).**
   Misses/mispositions some edge-on-edge collision configurations. Add the
   9 cross-product axes of each edge-pair to `boxBoxContact` for full SAT.
3. **No broad-phase.** Every pair of bodies is tested every frame (O(n^2)).
   Fine below ~50 bodies; add a spatial hash grid or BVH beyond that.
4. **No joints/constraints yet** (hinge, point-to-point). The `Contact`/solver
   pattern in `solver.cpp` generalizes directly — a joint is just another
   constraint type solved the same way.

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


