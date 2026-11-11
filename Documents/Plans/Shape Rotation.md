# Shape Rotation

Plan written 2026-11-11 for making Cuboid and Ellipsoid honour their rotation in the Managed, Burst and GPU implementations.

**Status:** planned, to follow `Interactive Editor.md`. The user asked for it on 2026-11-11. No code written.

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
2. **No existing scene is affected.** None of the 167 shapes in the 60 scenes is rotated, and every group above a shape is at identity. Reading the rotation changes no existing mesh.
3. **Three classes besides `Volume` implement `DensityFunction`:** `DensityFunction.Snapshot`, and the test helpers `CountingSphere` and `FixedDensityFunction`. Each gains the new member.
4. **The GPU layout is declared twice.** `DensityFunction.Data` in C# and `struct DensityFunction` in the shader have to change together; the buffer's stride comes from the C# struct.
5. **Sampling corners is the largest fixed cost of the Burst implementations.** Whatever rotation adds per sample is paid at every corner for every shape.

## Design

- **Interface:** `DensityFunction` gains `rotation`, the shape's rotation in voxelizer object space.
- **Sampling:** the position is taken into the shape's own frame, by undoing the rotation about the origin, and the existing formula is applied there. The formulas themselves do not change.
- **Unrotated shapes take today's path.** Each snapshot notes once whether its rotation is identity, and if so samples exactly as now. Existing meshes stay bit for bit the same, and unrotated shapes cost nothing extra.
- **Stored form:** each implementation's snapshot works the inverse rotation out once per contour and keeps it as three rows, so a rotated sample costs nine multiplications. The rows are written out the same way in C#, Burst and the shader.
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
  - Burst against Managed on rotated shapes, within the tolerances already in use. The random shapes of `BurstTestSupport` get random rotations.
  - GPU against CPU vertices on a rotated model, in `UniformDualContouringGPUTest`.
- **Contours:** a cube turned 45° about one axis and about a diagonal, for all three algorithms with Managed and Burst.
  - Every vertex lies on the rotated cube, the mesh is closed and manifold, and the SVD solver puts vertices on its edges and corners.
  - `HermiteNormalTest` holds its normals to 0.1°.
- **Snapshots:** `DensityFunctionSnapshotTest` covers the rotation.
- **Performance:** `ContouringPerformanceTest` on the unrotated model stays within the 5% the runs differ by, and the same model with every shape rotated is recorded beside it.

## Steps

Each step compiles, passes every test, and is reviewed before the next one starts.

0. **Baseline (small).** Both suites green, and mesh hashes of the existing configurations recorded.
1. **Interface and Managed (medium).** `rotation` on `DensityFunction`, `Volume` and `Snapshot`; rotation in `Cuboid.sample` and `Ellipsoid.sample`. `ContouringTest.Shape` and the two test helpers gain a rotation. Sampling, identity, snapshot and managed contour tests.
2. **Burst (small).** `BurstDensityFunction`. Parity tests, Burst contour tests, performance check.
3. **GPU (small).** `DensityFunction.Data` and the shader struct and sample functions. GPU tests.
4. **Interactive editor (small).** Conversion, signature, outlines, warnings. Unity's rotate tool then works on shapes, with a live remesh for Burst and GPU.
5. **Documentation (small).** README, and this plan marked complete.

## Out of scope

- **New primitives.**
- **Sheared shapes.** A shape is an origin, a rotation and extents; a full affine transform per shape would change the sample formulas.
- **Rotating the grid.** The root's own rotation already turns the whole mesh.
