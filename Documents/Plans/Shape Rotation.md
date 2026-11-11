# Shape Rotation

Plan written 2026-11-11 for making Cuboid and Ellipsoid honour their rotation in the Managed, Burst and GPU implementations.

**Status:** in progress. The user asked for it on 2026-11-11 and started it on 2026-11-12. Steps 0 to 2 are done. Until step 3, the GPU still contours shapes unrotated. Unrotated Burst contours are 3 to 5% slower; the user still has to decide whether that is acceptable (see Measurements).

## Goal

A rotated shape is contoured rotated, by every algorithm and every implementation. Shapes without rotation give exactly the meshes they give today.

## What exists today

- **Interface:** `DensityFunction` describes a shape by `type`, `origin`, `extents`, `combination` and `materialIndex`. There is no rotation.
- **Sampling,** written three times:
  - Managed: `Cuboid.sample` and `Ellipsoid.sample` (static, taking position, origin and extents), called by `Volume` and by `DensityFunction.Snapshot`.
  - Burst: `BurstDensityFunction.sampleCuboid` and `sampleEllipsoid`, the same arithmetic operation for operation.
  - GPU: `sampleCuboid` and `sampleEllipsoid` in `UniformDualContouring.compute`, fed by `DensityFunction.Data`.
- **Volumes:** `Volume.origin` and `Volume.extents` read the transform's local position and scale. Its rotation is not read.

### Findings that shape the work

1. **Rotation only touches sampling.** Intersections are found by searching the density along an edge, normals are finite differences of the density, and material boundaries use the density of a subset of the shapes. The QEFs work from those. Nothing outside the sample functions needs to know a shape is rotated.
2. **No existing scene is affected.** Checked again on 2026-11-12, with the interactive editor's scene added: none of the 171 shapes in the 68 scene files is rotated, and neither is any group or voxelizer above one. Reading the rotation changes no existing mesh.
3. **Four classes besides `Volume` implement `DensityFunction`:** `DensityFunction.Snapshot`, the interactive editor's `Shape`, and the test helpers `CountingSphere` and `FixedDensityFunction`. Each gains the new member. `DensityFunction.snapshot` only replaces volumes, so the editor's shapes and the test helpers sample themselves and have to honour the rotation on their own.
4. **The GPU layout is declared twice.** `DensityFunction.Data` in C# and `struct DensityFunction` in the shader have to change together; the buffer's stride comes from the C# struct.
5. **Sampling corners is the largest fixed cost of the Burst implementations.** Whatever rotation adds per sample is paid at every corner for every shape.
6. **A turned cube can leave cells that Uniform and Adaptive Dual Contouring cannot contour cleanly.** Found in step 1.
   - Where an edge of the cube crosses a cell diagonally, the cell's inside corners fall into two groups that no edge of the cell connects. The corner signs then look like two separate pieces of surface.
   - Uniform and Adaptive Dual Contouring give such a cell one vertex. It lands off the cube, 0.01 to 0.015 at 16³, the two pieces meet in it, and the surface is closed but not manifold there.
   - On the cube turned 45° about a diagonal, this happens in 4 cells, the same 4 for both algorithms. Manifold Dual Contouring gives each piece a vertex of its own and is manifold.
   - This is the limit of one vertex per cell that the Manifold paper sets out to fix, not an error in the rotation. Unrotated cubes never produce such cells: their edges run along the grid's axes, so the inside corners of a cell always stay connected.

## Design

- **Interface:** `DensityFunction` gains `rotation`, the shape's rotation in voxelizer object space.
- **Sampling:** the position is taken into the shape's own frame, by undoing the rotation about the origin, and the existing formula is applied there. The formulas themselves do not change.
- **Unrotated shapes take today's path.** Each snapshot notes once whether its rotation is identity, and if so samples exactly as now. Existing meshes stay bit for bit the same, and unrotated shapes cost nothing extra.
- **Stored form:** each implementation's snapshot works the inverse rotation out once per contour and keeps it as three rows, so a rotated sample costs nine multiplications. The rows are written out the same way in C#, Burst and the shader.
  - The rows are worked out in one place, a small shared type in `DensityFunction.cs`, on the CPU. The managed snapshot, the Burst struct and the GPU upload all copy them from there, so the three implementations undo a rotation with the same numbers.
  - The rotation is normalised first. A zero quaternion, which a density function that never sets one would return, counts as no rotation.
  - Whether a shape is rotated is decided from the rows: it is not when they are exactly the three axes. Unity's `==` on quaternions is approximate, so it is not used.
  - The GPU struct keeps that flag as a `uint`. A C# `bool` is copied as one byte but counted as four in the stride.
- **Volumes:** `Volume.rotation` is the transform's local rotation, in line with how `origin` reads the local position.
- **Interactive editor:** its shape conversion passes the rotation on, relative to the root.
  - For a direct child that is its local rotation, and the result is exact under any root scale.
  - Inside groups the rotations compose. One case cannot be expressed: a rotated shape under a group with unequal scale on its axes is sheared. The inspector warns about it.
  - The signature includes the rotation, the outlines are drawn rotated, and the "rotated shape" warning is removed.

## Tests

- **Sampling (managed):**
  - a rotated shape's density at a point equals the unrotated shape's density at the point rotated back;
  - a quarter turn about an axis equals the shape with two extents swapped, to rounding;
  - a sphere is unchanged by any rotation.
- **Identity:** densities with an identity rotation are bit-identical to today's, in Managed and Burst, and the meshes of the existing configurations hash the same before and after.
- **Parity:**
  - Burst against Managed on rotated shapes, within the tolerances already in use. The random shapes of `BurstTestSupport` get random rotations, from a generator of their own: the existing parity tests keep exactly the shapes they have, and rotated runs are added beside them.
  - GPU against CPU vertices on a rotated model, in `UniformDualContouringGPUTest`.
- **Contours:** a cube turned 45° about one axis and about a diagonal, for all three algorithms with Managed and Burst. This is `RotatedCubeTest`, on 16³ cells, with Adaptive and Manifold simplifying as the "Simplification" scenes do.
  - **Closed:** the mesh is closed for every algorithm. It is also manifold, as one piece without handles, for Manifold.
  - **On the cube:** every vertex lies within 0.002 of the cube. That is √3 times the 1/1024 that six binary search steps leave. For Uniform and Adaptive, the cells of finding 6 are left out.
  - **Corners:** every corner of the cube has a vertex.
  - **Edges:** the mesh runs along every edge of the cube, within the same 0.002. For Uniform and Adaptive, points within a cell of the cells of finding 6 are left out.
  - **Unity's convention:** the corners and distances are taken from the cube's own `Transform`, so the test also holds the rotation to the way Unity shows it.
  - **Normals:** `HermiteNormalTest` holds the normals of a turned cube to 0.1°. It leaves out intersections near an edge, where the central difference reaches the other face.
- **Snapshots:** `DensityFunctionSnapshotTest` covers the rotation.
- **Performance:** `ContouringPerformanceTest` on the unrotated model stays within the 5% the runs differ by, and the same model with every shape rotated is recorded beside it.

## Steps

Each step compiles, passes every test, and is reviewed before the next one starts.

0. **Baseline (small).** Both suites green, and mesh hashes of the existing configurations recorded. **Done 2026-11-12:**
   - Edit mode 764 of 764, play mode 21 of 21.
   - The `Start` hashes of the 416 configurations of the example scenes are identical to the interactive editor's baseline.
   - The Burst runs of `ContouringPerformanceTest` are recorded under Measurements, with Unity in front.
   - Findings 2 and 3 checked again and brought up to date.
1. **Interface and Managed (medium).** `rotation` on `DensityFunction`, `Volume` and `Snapshot`; rotation in `Cuboid.sample` and `Ellipsoid.sample`. `ContouringTest.Shape` and the two test helpers gain a rotation. Sampling, identity, snapshot and managed contour tests. **Done 2026-11-12:**
   - **`DensityFunction.Frame`** holds the origin, the three rows (`xAxis`, `yAxis`, `zAxis`) and `rotated`. Its `local( position )` gives the offset from the origin along the shape's axes; without a rotation that is the plain offset.
   - **The shapes:** `Cuboid.sample` and `Ellipsoid.sample` now take a position in the shape's own frame and its extents. A volume samples through a frame of its own, and a snapshot keeps one.
   - **The other implementers:** the editor's `Shape`, `FixedDensityFunction` and `ContouringTest.Shape` take a rotation; the last two also have `rotated( rotation )`. The editor passes the identity until step 4.
   - **New tests:**
     - `ShapeRotationTest` (13): the turn is Unity's; quarter turns swap extents; spheres don't change; which frames count as rotated; the length of the quaternion doesn't matter; unrotated volumes, snapshots and zero quaternions sample bit for bit like the old formulas.
     - `ManagedRotatedCubeTest` (24): see Tests.
     - The turned cube in `HermiteNormalTest`, for Managed.
     - Rotated volumes in `DensityFunctionSnapshotTest`.
   - **The tests catch a mistake:** with the rotation deliberately inverted, they failed.
     - The sampling test, every diagonal-turn cube test and the normals test failed.
     - A cube turned 45° about one axis looks the same turned the other way, so those cases still passed.
     - The snapshot test only compares snapshots with their volumes, so it still passed.
   - **Verification:** edit mode 802 of 802, play mode 21 of 21. The 416 `Start` hashes are identical, and the open scene is clean.
2. **Burst (small).** `BurstDensityFunction`. Parity tests, Burst contour tests, performance check. **Done 2026-11-12:**
   - **`BurstDensityFunction`** copies the rows and `rotated` from `DensityFunction.Frame`. Its `local( position )` is `Frame.local` written out the same way, and `sampleCuboid` and `sampleEllipsoid` take a position in the shape's own frame.
   - **Parity, random shapes:** the two parity tests of `BurstSurfaceExtractorTest` run a second time with every shape turned. The rotations come from a generator of their own (`BurstTestSupport.randomlyRotated`), so the unturned run has exactly the shapes it had. The tolerances already in use hold.
   - **Parity, whole meshes:** a turned model, `TopologyTest.turnedShapes`, is contoured by Managed and Burst for all three algorithms, every solver and every amount of simplification or clustering (16 cases).
     - The model is a cube turned about a diagonal, an ellipsoid turned about all three axes cut out of it, and a turned box of a second material.
     - Both give the same mesh within `BurstParity`'s tolerances.
   - **Identity:** `withoutRotationDensitiesAreExactlyAsBefore` compares unrotated Burst densities with the old formulas compiled by Burst, bit for bit, including zero quaternions.
   - **Contours:** `BurstRotatedCubeTest` (24) passes, and so does the turned cube of `HermiteNormalTest` for Burst.
   - **The tests catch a mistake:** with Burst made to ignore the rotation again, every turned parity test, 18 of the 24 cube tests and the Burst normals test failed.
   - **Performance:** see Measurements.
   - **Verification:** edit mode 846 of 846, play mode 21 of 21, and the 416 `Start` hashes are identical.
3. **GPU (small).** `DensityFunction.Data` and the shader struct and sample functions. GPU tests.
4. **Interactive editor (small).** Conversion, signature, outlines, warnings. Unity's rotate tool then works on shapes, with a live remesh for Burst and GPU.
5. **Documentation (small).** README, and this plan marked complete.

## Measurements

### Before any change (2026-11-12)

`ContouringPerformanceTest` on the three balls, Burst, with Unity in front. Each figure is the median of three contours, in ms.

| Algorithm | Cells | Voxelize | SampleCorners | SolveVertices |
|---|---|---|---|---|
| Uniform | 64³ | 4.6 | 0.8 | 2.6 |
| Uniform | 128³ | 23.9 | 5.6 | 10.9 |
| Uniform | 256³ | 157.9 | 49.3 | 52.8 |
| Adaptive | 64³ | 6.8 | 1.0 | 3.2 |
| Adaptive | 128³ | 30.6 | 6.3 | 13.5 |
| Manifold | 64³ | 7.6 | 1.0 | 3.4 |
| Manifold | 128³ | 29.7 | 5.7 | 12.6 |

`SolveVertices` is in the table because it samples too: it finds the intersections along the edges and takes the normals there.

### Burst with rotation (step 2, 2026-11-12)

**Comparing against step 0 is not good enough.** On the same code, contours vary from session to session by up to 13% (Uniform 128³: 25.3, 28.4, 25.0 and 25.0 ms). So the old and new sampling were timed in the same session, alternately, twice each, and this was done in four sessions.
- **The old variant:** the file from before step 2, with an unused `rotated` field so the new tests still compile.
- **Checking the swap:** a probe confirmed which variant was compiled before each timing. An early attempt had silently timed the new code throughout, because Unity had not compiled the swapped file.

**Unrotated model, Voxelize in ms, old against new, one A/B session.** Every session showed the same pattern.

| Algorithm | Cells | Old | New | Change | Change over all four sessions |
|---|---|---|---|---|---|
| Uniform | 128³ | 23.3, 23.7 | 24.4, 24.7 | +4.5% | +4 to +7% |
| Uniform | 256³ | 148.1, 152.6 | 155.1, 157.7 | +4.0% | +3 to +7% |
| Adaptive | 128³ | 28.5, 28.8 | 29.8, 28.8 | +2.3% | +2 to +5% |
| Manifold | 128³ | 29.3, 28.4 | 29.8, 29.7 | +3.1% | +3 to +4% |

By stage, SampleCorners is 3 to 5% slower, and SolveVertices, which samples 12 times per edge for the intersection and the normal, 6 to 9%.

**Where the cost comes from**, from the variants timed the same way:
- **Not the larger struct.** It grew from 36 to 76 bytes. The new struct sampled without the rotation branch was as fast as the old one.
- **Not inlining.** Forcing `sample` and `local` inline made no difference.
- **Worse alternatives.** Moving the rotation into a function that is never inlined, or folding the rotation into the type switch as four kinds, were slower for both unrotated and turned shapes: +7 to +12% unrotated.
- **What is left** is the branch on `rotated` in every sample, as written.

So the plan's aim of staying within 5% is met only at the edge: 3 to 5% usually, 7% in the worst session. Making it free would mean compiling every sampling job twice, with and without rotation, and choosing per contour. That touches every Burst job of the three algorithms and is not done.

**Turned model** (the same three balls, each turned), from one run:

| Algorithm | Cells | Unrotated | Turned | Change |
|---|---|---|---|---|
| Uniform | 128³ | 24.8 | 27.1 | +9% |
| Uniform | 256³ | 159.4 | 173.9 | +9% |
| Adaptive | 128³ | 31.5 | 33.5 | +6% |
| Manifold | 128³ | 29.5 | 32.7 | +11% |

SampleCorners is about 25% slower on turned shapes. Turned balls give the same meshes, except that Adaptive and Manifold at 128³ collapse slightly differently (2604 against 2586 vertices, 2371 against 2359). Turning a ball changes its densities in the last bits, and simplification decides some nodes right at the threshold.

## Out of scope

- **New primitives.**
- **Sheared shapes.** A shape is an origin, a rotation and extents; a full affine transform per shape would change the sample formulas.
- **Rotating the grid.** The root's own rotation already turns the whole mesh.
