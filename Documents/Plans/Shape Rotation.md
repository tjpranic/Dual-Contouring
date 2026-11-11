# Shape Rotation

Plan written 2026-11-11 for making Cuboid and Ellipsoid honour their rotation in the Managed, Burst and GPU implementations.

**Status:** in progress. The user asked for it on 2026-11-11 and started it on 2026-11-12. Steps 0 and 1 are done. Until steps 2 and 3, Burst and the GPU still contour shapes unrotated.

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
2. **Burst (small).** `BurstDensityFunction`. Parity tests, Burst contour tests, performance check.
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

## Out of scope

- **New primitives.**
- **Sheared shapes.** A shape is an origin, a rotation and extents; a full affine transform per shape would change the sample formulas.
- **Rotating the grid.** The root's own rotation already turns the whole mesh.
