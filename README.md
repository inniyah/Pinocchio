# Pinocchio — automatic rigging

Pinocchio automatically fits a skeleton inside a 3D character mesh and computes
the skinning weights that bind the mesh to it. You give it a closed mesh and a
template skeleton (human, quadruped, …). It returns the joint positions inside
the mesh and, for every vertex, how much each bone influences it.

This repository is a maintained version of:

- **Pinocchio** by Ilya Baran and Jovan Popović, the reference implementation
  of *"Automatic Rigging and Animation of 3D Characters"* (SIGGRAPH 2007).
- The **CMPUT414 (Winter 2015, Group F)** extension, which adds dual quaternion
  skinning (DQS) and an LBS/DQS blend using Rodolphe Vaillant-David's dual
  quaternion library. See [`README_CMPUT414.md`](README_CMPUT414.md).

On top of those it adds:

- More mesh formats.
- Extra skeletons.
- Two OpenGL viewers.
- Sample models.
- A set of [bug fixes](#recent-fixes) for wrong results, crashes and hangs.

The original documentation is in [`README.txt`](README.txt).

---

## Contents

- [Repository layout](#repository-layout)
- [Building](#building)
- [AttachWeights](#attachweights)
- [DemoUI](#demoui)
- [Viewer and ViewObj](#viewer-and-viewobj)
- [Skeleton files](#skeleton-files)
- [Mesh requirements](#mesh-requirements)
- [Using the library](#using-the-library)
- [Recent fixes](#recent-fixes)
- [Known limitations](#known-limitations)
- [Licenses and credits](#licenses-and-credits)
- [References](#references)

## Repository layout

```
Pinocchio/      the library: libpinocchio.a and libpinocchio.so (LGPL 2.1)
AttachWeights/  command-line tool: rigging and skinning weights
DemoUI/         FLTK viewer that rigs a mesh and plays a motion
  data/         sample meshes (test.obj, cheb.obj) and motion files
Viewer/         GLUT/GLEW viewer (not built by default)
ViewObj/        GLFW/GLEW viewer (not built by default)
models/         sample human models (CC0)
tests/          regression tests (make check)
```

## Building

Requirements:

- A C++17 compiler (tested with g++ 13). The code also compiles as C++20.
- `make`.
- For DemoUI: FLTK 1.3 with OpenGL.
- For the optional viewers: see [Viewer and ViewObj](#viewer-and-viewobj).

On Debian/Ubuntu, the DemoUI dependencies are:

```bash
sudo apt-get install libfltk1.3-dev libgl-dev
```

Build:

```bash
make            # library, AttachWeights and DemoUI
make check      # regression tests (no display needed)
make test       # runs DemoUI on DemoUI/data/test.obj with a walking motion
make clean
```

The programs link against the shared library, so run them with
`LD_LIBRARY_PATH=Pinocchio` from the top directory (or install
`libpinocchio.so` somewhere the loader can find it).

The build is warning-free with `-Wall`.

## AttachWeights

`AttachWeights` computes the skinning weights for a mesh and, with `-fit`,
also fits the skeleton inside it.

```bash
LD_LIBRARY_PATH=Pinocchio AttachWeights/AttachWeights models/human_male.obj -fit \
    -skelOut skeleton.out -weightOut attachment.out
```

```
AttachWeights mesh.{obj|ply|off|gts|stl}
              [-skel human|horse|quad|centaur|file] [-rot x y z deg]* [-scale s]
              [-fit | -nofit] [-stiffness s] [-meshonly | -mo]
              [-skelOut file] [-weightOut file]
```

> **`-fit` is not the default.** Without it, AttachWeights assumes the skeleton
> is already placed inside the mesh, typically one read with `-skel file`, and
> only computes weights. To fit a template such as the default `human`, pass
> `-fit`.

| Option | Description |
|---|---|
| `-skel` | `human` (default), `horse`, `quad`, `centaur`, or a [skeleton file](#skeleton-files). |
| `-fit` / `-nofit` | Fit the skeleton to the mesh, or use it as given (default). |
| `-rot x y z deg` | Rotates the mesh around axis (x, y, z) before rigging. Repeatable. Use it if the model is not upright (+Y) and facing +Z. |
| `-scale s` | Scales the template skeleton (on top of the built-in 0.7 factor). |
| `-stiffness s` | Weight of the heat term in the bone-heat equation (default 1). Higher values give stiffer, more local weights. |
| `-skelOut` | Joint output file (default `skeleton.out`). |
| `-weightOut` | Weight output file (default `attachment.out`). |
| `-meshonly` | Loads and validates the mesh, then exits. |

It exits with status 0 on success and 1 on any error (unreadable mesh, invalid
mesh, no embedding found, malformed skeleton file…).

### Output formats

`skeleton.out` has one line per joint, in the mesh's original coordinates:

```
index x y z parent_index        (parent_index is -1 for the root)
```

It is itself a valid [skeleton file](#skeleton-files). This lets you fit
once, correct the joints by hand if needed, and then recompute only the
weights:

```bash
AttachWeights/AttachWeights mesh.obj -skel skeleton.out -weightOut attachment.out
```

`attachment.out` has one line per mesh vertex, with one weight per bone,
separated by spaces. Bone *j* is the segment that ends at joint *j* + 1, in the
order of `skeleton.out`. Each line sums to 1.

Vertices not used by any face are dropped when the mesh is loaded, so the rows
skip them.

## DemoUI

DemoUI rigs a mesh and shows it. With `-motion`, it plays an animation using
the selected skinning algorithm.

```bash
LD_LIBRARY_PATH=Pinocchio DemoUI/DemoUI DemoUI/data/test.obj \
    -motion DemoUI/data/walk.txt -algo DQS
```

```
DemoUI mesh.{obj|ply|off|gts|stl}
       [-skel human|horse|quad|centaur|file] [-rot x y z deg]* [-scale s]
       [-meshonly | -mo] [-nofit] [-motion file] [-algo LBS|DQS|MIX [blend_weight]]
```

- `-algo LBS`: linear blend skinning (default).
- `-algo DQS`: dual quaternion skinning.
- `-algo MIX w`: `w · LBS + (1 − w) · DQS`. `w` is optional and defaults to
  0.5.

Motion files (`walk.txt`, `runAround.txt`, `jumpAround.txt`, …) are in
`DemoUI/data/`. Their format is hard-coded for the human skeleton; see
`DemoUI/DemoUI.txt`. DemoUI also writes `skeleton.out` and `attachment.out` in
the format described above.

**Controls:** left mouse pans, the wheel zooms, right mouse rotates. Keys:
`S` skeleton, `F` flat shading, `G` ground, `T` reset view, `Z` reset
transform, `P` pause, `1`–`6` preset cameras, `R` print the current camera
transform.

## Viewer and ViewObj

Two extra OpenGL viewers that rig a model and animate it with
`paths/walk.txt` using DQS. They are not part of the default build.

**Viewer** (GLUT + GLEW + GLM):

```bash
sudo apt-get install freeglut3-dev libglew-dev libglm-dev
make -C Pinocchio && make -C Viewer
cd Viewer && LD_LIBRARY_PATH=../Pinocchio ./Viewer ../models/human_male.obj \
    shaders/phong-shading.v.glsl shaders/phong-shading.f.glsl
```

**ViewObj** (GLFW + GLEW + stb), based on Anton Gerdelan's OpenGL `.obj`
viewer:

```bash
sudo apt-get install libglfw3-dev libglew-dev libstb-dev
make -C Pinocchio && make -C ViewObj
cd ViewObj && LD_LIBRARY_PATH=../Pinocchio ./ViewObj -o ../models/human_male.obj
```

Run `./ViewObj --help` for its options.

## Skeleton files

`-skel file` reads a plain-text skeleton, one joint per line:

```
name x y z parent
```

- `parent` is the name of the parent joint, or `-1` for the root.
- Parents must appear before their children.
- Lines starting with `#` are comments.
- `skeleton.out` uses joint indices as names, which is why it can be read back.

Coordinates are in the mesh's coordinates, so a skeleton file is mainly meant
for computing weights without `-fit`. With `-fit`, only the proportions and
directions of the skeleton matter, and the templates built into the code
(`human`, `quad`, `horse`, `centaur`) work best. Those templates also mark
which joints touch the ground and which sit in thick regions.

Errors are reported rather than silently accepted:

- A missing file.
- An unknown parent name.
- An unreadable number.
- A line with fewer than five fields.

Each error message names the file and the line. Unknown parents used to be
silently attached to the root.

## Mesh requirements

- **Formats:** OBJ, PLY, OFF, GTS and STL.
  - OBJ: polygons are triangulated, negative indices are supported, and per-vertex
    texture coordinates are taken from the faces.
  - OFF and PLY: polygonal faces are accepted.
- **Closed and manifold:** no holes, every edge shared by exactly two faces,
  consistent orientation. Invalid meshes are rejected with a message; they
  used to hang the program.
- **One connected component.**
- **Pose:** upright (+Y), facing +Z, in a pose reasonably close to the template
  (T-pose or A-pose for humans). Use `-rot` to fix the orientation.

Duplicate faces, degenerate faces and unreferenced vertices are cleaned up
automatically.

## Using the library

```cpp
#include "Pinocchio/pinocchioApi.h"
using namespace Pinocchio;

Mesh mesh("character.obj");
mesh.normalizeBoundingBox();
mesh.computeVertexNormals();

Skeleton skeleton = HumanSkeleton();   // or FileSkeleton("skeleton.txt")
skeleton.scale(0.7);

PinocchioOutput out = autorig(skeleton, mesh);
if (out.embedding.empty()) { /* no fit found */ }

// Joints in the mesh's original coordinates
for (Vector3 &p : out.embedding) p = (p - mesh.toAdd) / mesh.scale;

// Weights of vertex i: one entry per bone (bone j ends at joint j + 1)
Vector<double, -1> w = out.attachment->getWeights(i);

// Deform with one Transform<> per bone
Mesh posed = out.attachment->deform(mesh, transforms, DQS);   // LBS, DQS or MIX
delete out.attachment;
```

`autorig` runs the individual steps declared in `Pinocchio/pinocchioApi.h`:
`constructDistanceField`, `sampleMedialSurface`, `packSpheres`,
`connectSamples`, `discreteEmbed`, `refineEmbedding`, and so on. You can call
them yourself, for example to constrain a joint to a given sphere by editing
the `possibilities` vector before `discreteEmbed`.

## Recent fixes

### Wrong results

- **Voxelized-mesh weight transfer:** `barycentricCoord` used vertex `v2` twice,
  so `autorigVoxelTransfer` and `autoRigSkinWeightOnly` transferred weights
  incorrectly.
- **Refinement gradient:** the per-bone gradient was scaled by the coordinate
  values.
- **Line search:** it could return a worse position than its input.
- **`fixDupFaces`:** it misplaced vertices when vertex 0 was unused, and
  corrupted the mesh when duplicates were near the end of the face list. The
  function was rewritten.
- **`splitPaths`:** it relied on one particular joint declaration order.
- **Vertices with no visible bone:** they got all-zero weights and collapsed to
  the origin. They are now bound to their closest bone.
- **Failed heat-equation factorization:** it aborted the program through an
  `assert`, or returned garbage weights. It now falls back to the closest bone
  with a warning.
- **Scale in DQS/MIX:** transform scale was silently ignored.
- **OBJ texture coordinates:** each `vt` was assigned to the last `v` read
  instead of being taken from the faces.
- **`packSpheres`:** it returned one sphere more than the maximum.

### Crashes, hangs and undefined behavior

- **Open or non-manifold meshes:** they caused an infinite loop.
- **Unbounded search:** the A* search could exhaust memory. It now stops at
  2 million partial matches.
- **OBJ parsing:**
  - A `vt` line before the first `v` crashed the reader.
  - Texture indices were left uninitialized.
  - Faces were limited to 14 vertices.
  - Negative indices were not supported.
- **Out-of-range reads in the other readers:** GTS and STL could read out of
  range, and OFF and PLY only accepted triangles.
- **Out-of-bounds accesses:**
  - In `mixedBlend` for vertices without bones.
  - In `setFoot`/`setFat` on mid-chain joints.
  - In DemoUI's `-algo` parsing.
- **Root with exactly two children:** it was dropped from the compressed
  skeleton.
- **Unknown parent or symmetry names:** they silently mapped to the root.
- **Uninitialized members:** `MeshEdge::tvertex`, `Tri3Object::triIdx`, and
  `Mesh::withTexture` and `Mesh::blendWeight` in the default constructor.

### Build and tooling

- **Removed C++17 APIs replaced:** `std::binary_function`,
  `std::unary_function` and `std::bind2nd`.
- **Shared library:** `libpinocchio.so` failed to link because
  `libcode.version` was missing.
- **Makefiles:**
  - They no longer fail on a missing `.depend`.
  - The top-level build stops at the first failing directory instead of
    carrying on.
  - New `make check` target.
- **Viewer and ViewObj:** they compile again; they had not been updated after
  the skinning algorithm moved out of `Mesh`.
- **Exit codes:** errors now exit with status 1 instead of 0, and exceptions are
  reported.

On the sample models, the fixed version places joints within about 1 % of the
model size of the previous version, and the dominant bone of 95–98 % of the
vertices is unchanged.

## Known limitations

- It is a 2007 algorithm: it expects a closed character mesh in a pose close to
  the template, and it fails (with exit status 1) when the template does not
  fit.
- The dual quaternion library computes in single precision (`float`).
- DemoUI's motion reader only supports its own hard-coded format.
- The `-circlesonly` option is parsed but has no effect.
- `Viewer/skeletons/` contains skeleton descriptions in another format that the
  code in this repository does not read.

## Licenses and credits

- **Library** (`Pinocchio/`): GNU LGPL 2.1 or later, © 2007 Ilya Baran. See
  `Pinocchio/COPYING`.
- **DemoUI:** MIT license, © 2007 Ilya Baran.
- **Dual quaternion skinning library** (`dual_quat_cu.h`, `quat_cu.h`,
  `mat3.h`, `vec3.h`, `point3.h`, `transfo.h`): by Rodolphe Vaillant-David. The
  files carry no license header; check the original source before
  redistributing them separately.
- **DQS / LBS–DQS blend integration:** CMPUT414 Winter 2015, Group F.
- **mini-yaml** (`Pinocchio/yaml.*`): MIT license, © 2018 Jimmie Bergmann.
- **ViewObj:** based on Anton Gerdelan's OpenGL `.obj` viewer, MIT license (see
  `ViewObj/LICENSE`).
- **Sample models in `models/`:** from OpenGameArt, CC0 (see `models/URLs`):
  - `human_male.obj` and `human_female.obj` by VideroBoy.
  - `anime_male.obj` and `anime_female.obj` by adrix89.
- **`DemoUI/data/`:** sample meshes and motions from the original Pinocchio
  release.

If you use Pinocchio in research or a product, please acknowledge the original
authors.

## References

- Ilya Baran and Jovan Popović. *Automatic Rigging and Animation of 3D
  Characters.* ACM Transactions on Graphics 26(3), SIGGRAPH 2007.
  Project page: <http://www.mit.edu/~ibaran/autorig/pinocchio.html>
- Ladislav Kavan, Steven Collins, Jiří Žára and Carol O'Sullivan. *Skinning
  with Dual Quaternions.* I3D 2007.
- Rodolphe Vaillant. Dual quaternion skinning tutorial and code:
  <http://rodolphe-vaillant.fr/?e=29>
