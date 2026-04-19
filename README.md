# Interactive Geometry Destruction Simulator

CS 184 Final Project — Spring 2026
Team: Sriya Kalyan, Ethan Lau, Kelly Tou, Aiden Wang

An interactive OpenGL application that fractures a closed triangle mesh
into watertight Voronoi fragments and simulates them as rigid bodies in
real time. Click-to-impact raycasts refracture the mesh at the hit point
with a bias toward the impact; a timestep recorder lets you scrub
backwards and forwards through the simulation. A second fracture method
(axis-aligned uniform grid) is provided for side-by-side comparison with
the Voronoi approach.

---

## Dependencies

```bash
brew install ninja
brew install glew
```

## Build

The project uses CMake 3.16+ and a C++17 compiler. All external
dependencies (GLFW 3.4, GLM 1.0.1, GLEW) are pulled automatically via
`FetchContent` — no manual setup required.

```bash
cd <repo>
cmake -S . -B build -G Ninja            # or "-G 'Unix Makefiles'"
cmake --build build -j
./build/destruct                        # default: cube, 20 Voronoi fragments
./build/destruct assets/teapot.obj      # any OBJ with 'v' and 'f' lines
./build/destruct assets/teapot.obj 40   # OBJ with 40 fragments
./build/destruct                        # for default cube
```

System requirements: OpenGL 3.3 core profile (any GPU from the last
decade). On Linux, the X11 / Wayland dev packages that GLFW needs may
need to be installed (`xorg-dev` on Debian/Ubuntu).

## Controls

The simulation **starts paused** — you see the mesh sitting whole as
fragments, and start it deliberately with Space.

| Key / mouse          | Action                                               |
|----------------------|------------------------------------------------------|
| Left-drag            | orbit camera                                         |
| Middle-drag          | pan camera                                           |
| Mouse wheel          | zoom                                                 |
| Right-click a frag   | impact raycast → refracture near hit + explosion (stays paused; hit Space to play) |
| `Space`              | play / pause                                         |
| `R`                  | restart — reset fragments to initial positions, pause |
| `F`                  | refracture from scratch (current method), pause       |
| `V`                  | switch to Voronoi fracture + refracture              |
| `U`                  | switch to uniform-grid fracture + refracture         |
| `←` / `→`            | scrub playback (only while paused)                   |
| `↑` / `↓`            | increase / decrease fragment count (takes effect on next F/V/U) |
| `G`                  | apply an upward impulse to every fragment            |
| `Esc`                | quit                                                 |

Notes on the input semantics: `R` restarts the *same* scene (same
fragment shapes, same starting transforms) so you can replay the exact
same scenario repeatedly. `F` generates a *new* fragmentation from fresh
random seeds. `V`/`U` switch fracture methods and implicitly refracture.
All three of these leave the sim paused.

## Algorithms

**Voronoi fracture.** For each seed point `s_i`, we build the perpendicular
bisector plane between `s_i` and every other seed `s_j`, oriented so that
`s_i`'s side is the "kept" half-space. We then clip the original mesh by
every such plane in sequence. The intersection of the kept half-spaces is
exactly the Voronoi cell of `s_i` intersected with the mesh — we never
need to build a global 3D Voronoi diagram (no incremental convex-hull
construction, no dual graph). Total cost is O(N²) planes in the number of
seeds, which is fine for N up to a few hundred. The seeds themselves are
sampled uniformly in the mesh's AABB; with the `useImpactBias` option
they're drawn from a Gaussian around the impact point instead, which
produces convincingly crater-shaped clusters.

**Mesh clipping.** Implemented in `src/mesh/MeshClipper.cpp`. For each
triangle, vertices are classified as inside / outside the plane; edges
that straddle the plane are cut, with cut-vertex positions deduplicated
per unordered edge so adjacent triangles share exact intersection
vertices (watertightness depends on this). Each cut triangle contributes
a segment to the cap, and we stitch those segments into one or more loops
using an adjacency map, then fan-triangulate each loop from its centroid.
The fan winding is auto-verified against the plane normal and flipped if
needed, which removes a whole class of case-analysis direction bugs.

**Mass properties.** Signed-tetrahedra-from-origin integration per
Mirtich 1996. For every triangle we form a signed tet with the origin,
accumulate per-tet volume / first moments / second moments, then apply
the parallel-axis theorem to shift the inertia tensor from the origin to
the CoM. This gives a closed-form answer valid for any closed mesh — no
voxelization, no Monte Carlo.

**Rigid-body dynamics.** Custom. Semi-implicit Euler on linear velocity,
first-order quaternion derivative on angular velocity followed by unit
renormalization. Ground contacts are resolved via per-vertex penetration
depth (single deepest contact per body per step); body-body contacts use
a vertex-inside-AABB probe in the other body's frame to generate contact
points. Normal + tangential impulses are computed with the standard
inertia-aware impulse formula, with the tangent impulse clamped to the
Coulomb friction cone. Baumgarte-style positional correction stabilizes
stacks.

**Timestep scrubbing.** Every simulation step we record each body's
`(position, orientation)` into a ring buffer capped at ~30 s of history.
Scrubbing restores those transforms directly without re-simulating and
zeroes velocities, so unpausing mid-scrub restarts from rest cleanly.

## Comparison: Voronoi vs Uniform-Grid Fracture

As an alternative fracture method for benchmarking (addressing TA
feedback), `src/fracture/UniformFracture.cpp` implements the same
`clipAgainstAll`-based fragmentation using a regular 3D grid of
axis-aligned planes instead of Voronoi bisectors. Switch between the two
in-application with `V` / `U` to compare.

Observed qualitative differences on the default cube scene:

* **Uniform** produces visually blocky, axis-aligned "bricks" regardless
  of mesh shape. Fragment count is exactly `nx*ny*nz` minus any empty
  cells. Every fragment has the same volume modulo the mesh's boundary.
* **Voronoi** produces irregular polyhedral fragments whose shapes depend
  on the seed distribution. Adding impact bias clusters many small
  fragments near the impact and leaves the distal end in a few large
  pieces — much closer to the look of real brittle fracture.

Performance (20 fragments, cube mesh, release build, AMD Ryzen 7 laptop):
Voronoi ≈ 1.4 ms, uniform ≈ 0.9 ms. Both are well under a frame at 60 Hz,
so the qualitative difference dominates the practical choice.

## Aspirational features addressed

From the proposal:
- **Interactive raycasted impacts** ✓ — right-click picks the hit
  fragment via a Möller-Trumbore triangle ray test (AABB broad-phase,
  ray transformed into body space for efficiency), then refractures
  with impact-biased seeding and applies a falloff-weighted explosive
  impulse to every fragment.
- **Timestep scrubbing** ✓ — `Space` pauses, arrow keys scrub.
- **Benchmark against alternative fracture** ✓ — uniform-grid method
  above.

## Notes on images / rendering

All renders referenced in the writeup are generated from this codebase;
nothing is reused from the literature. The screenshots included in the
milestone / final report (see `docs/`) are produced by running the
binary on the default cube and on an OBJ of the Stanford bunny, then
capturing the OpenGL framebuffer with a screenshot tool. The lighting,
camera motion, and color palette are all from this repo's shaders and
renderer.

## Source layout

```
src/
  math/Plane.h                - plane representation + factories
  mesh/Mesh.{h,cpp}           - triangle-soup mesh, factories, normals
  mesh/OBJLoader.{h,cpp}      - minimal OBJ parser (v, f only)
  mesh/MeshClipper.{h,cpp}    - plane clip with watertight cap
  mesh/MeshProperties.{h,cpp} - Mirtich mass-property integration
  fracture/VoronoiFracture.{h,cpp} - Voronoi cells by iterated clipping
  fracture/UniformFracture.{h,cpp} - AABB grid cutting (comparison)
  physics/RigidBody.{h,cpp}   - quaternion-based rigid body
  physics/PhysicsWorld.{h,cpp}- gravity + ground + pair collisions
  render/Shader.{h,cpp}       - GL program RAII
  render/Camera.{h,cpp}       - orbit camera + screen-to-ray
  render/GPUMesh.{h,cpp}      - VAO/VBO/EBO wrapper
  render/Renderer.{h,cpp}     - forward Phong renderer + ground
  app/TimestepRecorder.{h,cpp}- per-frame state snapshots
  app/Application.{h,cpp}     - GLFW + main loop + input
  main.cpp                    - CLI entry

shaders/
  mesh.vert, mesh.frag        - per-fragment Phong + subtle Fresnel
  ground.vert, ground.frag    - anti-aliased checkerboard
```

## References

* Mirtich, *"Fast and Accurate Computation of Polyhedral Mass
  Properties"*, Journal of Graphics Tools, 1996.
* Müller et al., *"Real Time Dynamic Fracture with Volumetric
  Approximate Convex Decompositions"*, SIGGRAPH 2013.
* Baraff & Witkin, *"Physically Based Modeling: Rigid Body Simulation"*,
  SIGGRAPH 2001 course notes.
* Iñigo Quílez, *"Filtered Checkerboard"*,
  https://iquilezles.org/articles/checkerfiltering/
