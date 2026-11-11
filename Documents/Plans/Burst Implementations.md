# Burst Implementations

Plan written 2026-11-11 for adding a Burst implementation of each dual contouring algorithm next to its managed implementation.

**Status:** steps 0 and 1 done and reviewed 2026-11-11, tolerances accepted. Step 2 done 2026-11-11 and awaiting review. Steps 3–5 not started.

## Goal

The CPU implementations are slow. Uniform, Adaptive and Manifold dual contouring each get a Burst implementation. It is kept separate from the managed implementation, which stays in place as the reference.

Uniform keeps its GPU implementation. GPU implementations of Adaptive and Manifold are not planned.

## Decisions

- **Implementation selection:** `SurfaceExtractor.Implementation.Type` becomes `{ Managed = 0, GPU = 1, Burst = 2 }`. `CPU` is renamed to `Managed`. Scenes save this field as a number, so every existing scene keeps the implementation it has now.
- **Managed hot spots:** the three found below are fixed in the managed code in step 0. None of them changes the contour.
- **Index format:** every path, Managed, Burst and GPU readback alike, switches to 32-bit indices when a mesh has more than 65,535 vertices.
- **Sub-mesh numbering:** every path numbers sub-meshes by material bit (`findHighestMaterialBit`), as the GPU direct path already does.
- **Parity:** Burst results are compared with managed results within a tolerance, not bit for bit.

## What exists today

- **Implementation dispatch:** only `UniformDualContouring` selects its implementation through the `Implementation` interface, with nested CPU and GPU classes. `AdaptiveDualContouring` and `ManifoldDualContouring` override `voxelize` directly, and their `implementation` property throws.
- **Data model:**
  - The managed code uses a class per corner, edge and voxel.
  - Every voxel owns its own 8 corners and 12 edges. Each grid point is sampled 8 times and each edge is solved up to 4 times.
  - It also relies on LINQ and on recursion that allocates arrays at every call. `SimpleQEF` keeps a `List<Plane>` that grows with every combine, and the SVD matrices are classes.
  - None of this goes into Burst as it is. Each Burst implementation is a rewrite over flat arrays that keeps the managed math.
- **Packages:** Burst 1.8.29 and Collections 6.4.0 are resolved as dependencies of other packages, not listed in `Packages/manifest.json`. `Assets/Scripts/GameAssembly.asmdef` references Unity.Mathematics but does not use it yet, and it references neither Burst nor Collections. The performance testing package (`com.unity.test-framework.performance`) is in the package cache as a dependency of Collections.
- **Grid sizes:** the `resolution` field is an exponent.
  - Uniform: 2^resolution cells per axis, 4 to 256.
  - Adaptive and Manifold: an octree of depth `resolution` with 2^(resolution−1) leaves per axis, 2 to 128. Their octree is always built full.

### Findings that shape the work

1. **Split normals:** `Voxelizer.Start` converts to split normals in a loop that reads `mesh.vertices` once per triangle. That property returns a fresh copy each time, which makes the loop O(triangles × vertices). Every scene except the four Sphere scenes uses split normals, and both Managed and Burst go through this code.
2. **Volume transforms:** `Volume.origin` and `Volume.extents` call `GetComponentInParent` on every density sample.
3. **Corner sampling:** in Adaptive and Manifold, corner sampling has a leftover inner `foreach( var densityFunction in densityFunctions )` loop. Density and material are evaluated N times per corner instead of once.
4. **Index format:** meshes use the default 16-bit index format. A mesh with more than 65,535 vertices breaks, which Uniform at resolution 8 reaches.
5. **Sub-mesh order:** the CPU paths and the GPU readback (`convert`) number sub-meshes in the order each material first appears. The GPU direct path colours by material bit. Material colours can therefore swap between paths.
6. **Out of scope:** the compute shader dispatches `numthreads( 1, 1, 1 )`. This skews any Burst vs GPU comparison, but optimizing the shader is not part of this plan.

## Architecture

### Selection and layout

- **Dispatch:** each voxelizer switches on `implementationType`. The Adaptive and Manifold editors switch GPU back to Managed, the way they switch it back to CPU today. `VoxelizerEditor` still forces the SVD solver only for GPU, because Burst supports both solvers.
- **Code layout:** the Burst code lives in `Assets/Scripts/Burst/`, inside GameAssembly. The asmdef gains references to Unity.Burst and Unity.Collections, and the manifest lists both packages explicitly.
- **Per-algorithm files:** each algorithm's Burst part is a partial class, for example `Assets/Scripts/Burst/AdaptiveDualContouring.Burst.cs`. The managed files gain only the dispatch switch and the step 0 fixes.
- **Lifetime:** Burst implementations are created on first use, so tests that call `voxelize` without `Start` keep working. Their native buffers are freed in an `OnDestroy` that moves from `UniformDualContouring` up to `Voxelizer`. `ContouringTest.tearDown` calls it for every voxelizer, not only Uniform ones.

### Data layout

- **Corners:** one grid of (n+1)³ corners, each sampled once. In Adaptive and Manifold, the corners of a node at any level are points of the leaf corner grid.
- **Edges:** each unique edge is solved once, three per grid point.
- **Octree:** stored as one grid per level. Node (L, x, y, z) has children (L+1, 2x+i, 2y+j, 2z+k), so there are no pointers. Node state is kept per level in flat arrays.
- **Determinism:** every job writes by index. `ParallelWriter` is avoided wherever its ordering would leak into the output.

### Shared Burst core (`Assets/Scripts/Burst/`)

- **Density:**
  - A blittable snapshot of the volumes (type, combination, material, origin and extents as `float3`) is taken on the main thread before any job runs.
  - It comes with ports of `calculateDensity`, `calculateMaterial`, `approximateIntersection`, `calculateNormal` and `calculateMaterialBoundary`. The compute shader's material-masked overloads show the pattern for the last one. `findHighestMaterialBit` becomes a leading-zero count.
  - Two details must match the managed code:
    - `Vector3.Normalize` returns zero for lengths below 1e-5, whereas `math.normalize` returns NaN.
    - The `==` guard in `Ellipsoid.sample` is Unity's approximate comparison, a squared distance below 1e-10.
- **QEFs:**
  - The SVD solver becomes a struct with the 6 floats of the symmetric matrix, AtB, BtB, the mass point, the sample count and the feature dimension. `solve` takes the struct by `ref`, because it records the feature dimension that `combine` reads later.
  - The Simple solver's step `v -= Σ(n·v − d) n / N` equals `(AtA·v − AtB) / N`, and its error equals the SVD error formula, so the Burst Simple solver uses the same fixed-size accumulator. The math is the same as the managed `List<Plane>` version, but the rounding is different.
- **Tables:**
  - **Dual marching cubes:** constant data in `BurstTables`, generated from `DualMarchingCubes`. For each of the 256 configurations it holds the vertex count, and for each of the 12 edges the index of the vertex that edge belongs to. Burst reads static readonly arrays of constants directly, so there is no native container to keep alive. (The plan first said a persistent native array.)
  - **Octree contouring:** `OctreeContouringTables` becomes child-index tables. Only the `Root` position is ever used.
  - **Topology:** `OctreeTopology.isQuasiManifold` and `collapsePreservesTopology` are ported with fixed-size local arrays.
- **Mesh output:**
  - Vertices, normals and indices are written into a `Mesh.MeshDataArray` from a job. Indices are 32-bit when there are more than 65,535 vertices, and sub-meshes go in as `SubMeshDescriptor`s numbered by material bit.
  - Vertices are emitted in first-use order, as in the managed code. The Manifold voxel test compares `mesh.vertices` with the contour voxels, and `MeshTopology` counts only referenced vertices.
- **Debug data:**
  - Corners, edges and voxels are copied out of the native buffers only when debug drawing is on or a test asks for them. Otherwise the enumerables are empty, so benchmarks measure contouring rather than debug capture.
  - The copies are wrapped in small read-only classes that implement `SurfaceExtractor.Corner`, `Edge` and `Voxel`. They cover the same sets the managed implementation returns.
- **Burst settings:**
  - Jobs use `CompileSynchronously = true`, so the first call and every edit-mode test runs Burst code rather than the Mono fallback.
  - The float mode starts as `FloatMode.Strict`. The faster modes are measured in step 5.
  - Jobs complete inside `voxelize`, which stays synchronous like `Start`. Asynchronous voxelization is out of scope.

## Per algorithm

### Uniform (medium)

As built in step 2, in `Assets/Scripts/Burst/UniformDualContouring.Burst.cs`:

1. **Corners** (parallel over (n+1)³): density and material, from one sample of each density function.
2. **Cells** (parallel over n³):
   - Skips cells that are all void or all solid.
   - Works out the hermite data of the cell's own 12 edges in the order of the edge layout, adds it to a QEF, and places the vertex as `QEFSolver.placeVertex` does. Then applies surface correction and sets the averaged normal.
   - An edge the contour crosses is worked out by each of the four cells around it, as in the managed implementation. The plan first had a pass over every edge of the grid, but that needs storage for every edge, about 1.3 GB at 256³, of which almost none cross the contour.
3. **Quads** (parallel over n³): each cell gets a bit for each positive-axis edge (edges 3, 7 and 11) where all four cells around it have a vertex and the edge crosses the contour. Quads on the grid boundary are skipped, as in managed.
4. **Contour** (one job): goes through the cells in the managed order and builds the quads with the managed winding. Vertices are created when a quad first uses them, so the mesh is the managed one, vertex for vertex and triangle for triangle.
5. **Mesh:** written by `BurstMesh`.
6. **Debug data:** copied only when debug drawing is on or `Voxelizer.keepDebugInformation` is set; tests set it. Every corner, every edge and every cell is copied, as the managed implementation has them.
   - Hermite data of the edges is worked out in a separate pass over every edge, for contour edges and for material-boundary edges in cells the contour passes through.

### Adaptive (medium to large)

1. **Leaf pass:** reuses Uniform's corner, edge and cell jobs on the leaf grid, keeping each leaf's QEF and error.
   - The managed code also solves a QEF from every internal node's own coarse edges. Nothing reads it afterwards:
     - simplification overwrites it;
     - contouring only reaches nodes that are not internal;
     - the debug data leaves internal nodes out.
   - The Burst version skips that QEF. A test confirms this before the code relies on it.
2. **Simplification:** one parallel job per level, bottom-up, following the managed climb:
   - A node collapses only when all eight children are already non-internal.
   - Children with no QEF give an empty Pseudo leaf.
   - The topology check, when enabled, comes before the error threshold.
   - "`children = null`" becomes "type Pseudo". The children's data stays in place and is ignored by contouring.
3. **Contouring:**
   - A literal port of `contourCell`, `contourFace` and `contourEdge`, run as one job with an explicit stack.
   - The minimal edge is taken from the smallest node. Vertices are indexed by first use, and the half of a quad that repeats a vertex is dropped.
   - The traversal is O(nodes) and cheap next to sampling, so it is only parallelised over subtrees if profiling says so.

### Manifold (large)

1. **Leaf pass:**
   - Vertices are counted per leaf from the dual marching cubes table, then prefix-summed.
   - The fill pass gives each vertex its QEF, normal, 12 intersection counts, Euler characteristic (1) and face flag (true), and gives each leaf its edge-to-vertex map.
   - Material-boundary samples go to the vertices whose edges reach a corner of the boundary edge, or to all vertices when none does.
   - Each vertex is solved with `solve`, not `placeVertex`, followed by surface correction.
2. **Clustering:** one parallel job per level, bottom-up.
   - **Members:** the children's vertices, in child order.
   - **Union-find:** runs over the leaf pairs on the node's 12 internal faces, the explicit form of `findLeavesWithinFacePair`. In the full octree these pairs form a square block of leaves per face, and the leaves' vertices are followed through parent indices.
   - **Output:** clusters are written into per-node slot ranges, sized from the children's vertex counts and found by prefix sum. Clusters come out in the order of their first member, which keeps the output deterministic.
   - **Per-cluster values:** intersection counts, the Euler characteristic, the face condition and the QEF solve all follow the managed code.
   - **Parallel writes:** each node writes only the parent indices of its own children's vertices, so the writes are safe under `[NativeDisableParallelForRestriction]`.
3. **Collapse flags:** `collapsible` per vertex and `collapsed` per cell, set per level, bottom-up and in parallel.
4. **Contour vertices:** `findContourVertex` runs per leaf vertex, in parallel.
5. **Contouring:**
   - A literal traversal port, including both skips: `contourCell` on a collapsed node, and `contourFace` when both nodes are collapsed.
   - Triangles are de-duplicated in insertion order on their three indices, to match `Distinct()`.
6. **Validation:** the debug-build check that every child vertex has a parent in the enclosing cell becomes a test.

## Scenes

- **Move to `Managed/`:**
  - `Assets/Scenes/Uniform Dual Contouring/CPU/*.unity` (8 scenes) → `Assets/Scenes/Uniform Dual Contouring/CPU/Managed/`
  - `Assets/Scenes/Adaptive Dual Contouring/*.unity` (9 scenes) → `Assets/Scenes/Adaptive Dual Contouring/Managed/`
  - `Assets/Scenes/Manifold Dual Contouring/*.unity` (11 scenes) → `Assets/Scenes/Manifold Dual Contouring/Managed/`
  - `Assets/Scenes/Uniform Dual Contouring/GPU/` stays as it is.
- **Copy to `Burst/`:** each of the 28 scenes is copied into the sibling `Burst/` folder under the same name, with `_implementation: 2`. The copies for an algorithm are made in the step that adds its Burst implementation.
- **Method:**
  - Moves and copies go through the editor's asset database. That keeps the GUIDs of the moved scenes and gives the copies new ones.
  - `_implementation` is then set by a one-line text edit of the scene file, so no scene gets opened.
  - This is done while none of these scenes is open with unsaved changes.
- **References:** nothing refers to scene paths. The build scene list is empty and no test loads a scene.
- **Optional:** one high-resolution benchmark scene per algorithm, with debug drawing off.

## Tests and measurement

- **Baseline (step 0):**
  - Profiler markers for each stage of each managed implementation, with no change in behaviour.
  - Edit-mode performance tests: Uniform at resolutions 4 to 8, Adaptive and Manifold at 5 to 8, with debug capture off. Managed sizes are capped where a run stops finishing in reasonable time; the cap is recorded.
  - `voxelize` is timed separately from the mesh post-processing in `Start` (split normals, `RecalculateBounds`, `OptimizeReorderVertexBuffer`).
  - Results note whether Burst safety checks were on.
- **Existing fixtures:**
  - The Adaptive and Manifold fixtures run for `[Values( ImplementationType.Managed, ImplementationType.Burst )]`, which roughly doubles their test count.
  - Uniform's CPU-only tests also run for Burst.
  - Tests that already take `[Values] ImplementationType` pick Burst up automatically.
- **Parity tests:** one fixture per algorithm comparing Burst with Managed within a tolerance, as `UniformDualContouringGPUTest` does for the GPU.
  - Uniform: the vertex per cell index.
  - Adaptive: node type and vertex per (level, index).
  - Manifold: equal vertex and triangle counts, the same topology, and matching vertex positions.
- **Leak test:** voxelize twice, destroy the voxelizer, and expect no native leak warnings.
- **Core unit tests:** the Burst density, normal, intersection, QEF and table functions are compared with the managed functions on random and edge-case inputs. Each test runs the Burst code in a Burst-compiled job and fails if the job ran as managed code instead.
- **Why parity needs a tolerance (found in step 1):**
  - Mono keeps the intermediate results of an expression at a higher precision than float, and rounds only when a value is stored. In the editor, `a * a + c` with a = 1 + 2⁻¹² and c = −(1 + 2⁻¹¹) gives 2⁻²⁴, where float arithmetic gives 0. `Vector3.Magnitude` matches the square root of the double sum, not of the float sum.
  - Burst in `FloatMode.Strict` rounds every operation to float and fuses nothing.
  - The same operations in the same order therefore differ in the last bits. About a quarter of the densities differ by one unit in the last place.
  - Bit-exact parity would mean emulating where Mono rounds, so the tests use tolerances instead:

    | Result | Tolerance |
    |---|---|
    | Densities | relative 1e-6 |
    | Normals near a surface | 1e-3 |
    | Intersections and surface corrections | 1e-5 |
    | QEF vertices and errors | relative 1e-5 |
    | Materials away from surfaces, mass points, tables, topology and meshes | exact |

  - Normals take differences of densities 2e-4 apart. Far from a surface the densities are large and so are their last bits, which is why normals are compared near a surface, where the algorithms take them.
- **Where tests run:** the standalone dotnet harness that runs the managed sources outside Unity cannot run Burst or Jobs code. Burst code is verified only in the editor. Test runs block the editor, and play-mode runs enter play mode.

## Steps

Each step is done tests first and reviewed before the next one starts.

0. **Groundwork (small). Done 2026-11-11.**
   - Add Burst, Collections and the performance testing package to the manifest, and Unity.Burst and Unity.Collections to the asmdef.
   - Rename the enum and add the dispatch. Burst throws `NotImplementedException` for now.
   - Move `OnDestroy` to `Voxelizer` and make test teardown call it for every voxelizer.
   - Fix hot spots 1–3.
   - Switch to 32-bit indices when needed and number sub-meshes by material bit in every path. Check that each multi-material scene's MeshRenderer material list lines up with the material bits it uses, and that tests iterating sub-meshes tolerate empty ones.
   - Add the profiler markers and performance tests, and record the baseline in the measurements section below.
   - Move the 28 CPU scenes into `Managed/`.
   - Done when the edit-mode and play-mode suites are green (171 and 20 tests at the last full run) and the baseline is recorded.
1. **Burst core (medium). Done 2026-11-11.** Density, QEFs, tables, the mesh writer and debug data. Done when the core unit tests are green.
   - Code in `Assets/Scripts/Burst/`: `BurstDensityFunction`, `BurstSurfaceExtractor`, `BurstQEF`, `BurstTables`, `BurstOctreeTopology`, `BurstMesh`, `BurstDebugInformation`.
   - Tests: `BurstSurfaceExtractorTest`, `BurstQEFTest`, `BurstTablesTest`, `BurstOctreeTopologyTest`, `BurstMeshTest` and `BurstDebugInformationTest`, with shared helpers in `BurstTestSupport`. Edit mode 207/207, play mode 20/20.
2. **Uniform (medium). Done 2026-11-11.** The Burst implementation, the 8 Burst scenes, parity tests and the benchmark. Done when the Uniform fixtures and parity tests are green for Burst and the benchmark is recorded.
   - Code: `Assets/Scripts/Burst/UniformDualContouring.Burst.cs`, with `UniformDualContouring` made partial, and `Voxelizer.keepDebugInformation`.
   - Scenes: `Assets/Scenes/Uniform Dual Contouring/CPU/Burst/`, 8 copies of the managed scenes with `_implementation: 2`. Each one contours to the same vertex, triangle and voxel counts as its managed scene.
   - Tests: `UniformDualContouringBurstTest`, 18 parity tests (meshes identical triangle for triangle, vertices and hermite data within the tolerances), including 256³ with 32-bit indices. The Uniform topology, hermite normal and sub-mesh tests run for Burst too. Edit mode 239/239, play mode 20/20.
3. **Adaptive (medium to large).** The same, with 9 scenes. Done on the same terms.
4. **Manifold (large).** The same, with 11 scenes. Done on the same terms.
5. **Tuning.**
   - Float mode, batch sizes, and a parallel traversal if profiling calls for it.
   - Update `README.md` to list the Burst implementations and their package requirements.

## Measurements

Measured with `ContouringPerformanceTest` (edit mode, explicit, category `Performance`). Run it by name with explicit tests included.

**Setup:**
- Model: the three balls of the Simplification scenes, contoured with the SVD solver and no surface correction.
- Adaptive simplifies at 2e-4, topology-safe. Manifold clusters at 2e-4 with the manifold criterion on.
- Times are medians of 3 runs after a warm-up, except at 128³, which is a single run.
- All times are in milliseconds, in the editor (Mono), on a 64 GB Mac.
- Stage times come from the profiler markers. The "voxelize" column is the whole `voxelize` call.

### Baseline: Managed, after the step 0 fixes

**Uniform**

| Cells | Vertices | Triangles | Voxelize | Build grid | Sample corners | Solve vertices | Contour | Build mesh | Debug data |
|------:|---------:|----------:|---------:|-----------:|---------------:|---------------:|--------:|-----------:|-----------:|
| 16³   | 760      | 1,516     | 104.5    | 14.6       | 48.5           | 31.5           | 1.3     | 0.1        | 8.1        |
| 32³   | 3,216    | 6,428     | 678.6    | 75.2       | 355.6          | 183.6          | 6.9     | 0.5        | 56.7       |
| 64³   | 13,000   | 26,036    | 4,818.9  | 681.5      | 2,944.0        | 666.8          | 36.9    | 1.8        | 459.8      |
| 128³  | 52,224   | 104,500   | 38,554.3 | 6,333.1    | 24,735.5       | 3,384.0        | 221.4   | 14.3       | 3,866.0    |

**Adaptive**

| Leaves | Vertices | Triangles | Voxelize | Build octree | Sample corners | Solve vertices | Simplify | Contour | Build mesh | Debug data |
|-------:|---------:|----------:|---------:|-------------:|---------------:|---------------:|---------:|--------:|-----------:|-----------:|
| 16³    | 628      | 1,252     | 119.7    | 15.9         | 54.2           | 40.8           | 2.7      | 3.5     | 0.1        | 2.4        |
| 32³    | 852      | 1,700     | 757.9    | 93.5         | 459.8          | 181.2          | 14.3     | 4.6     | 0.2        | 3.1        |
| 64³    | 1,056    | 2,148     | 5,449.8  | 797.1        | 3,760.1        | 819.5          | 69.9     | 5.6     | 0.2        | 3.7        |
| 128³   | 2,594    | 5,240     | 47,104.2 | 13,715.0     | 28,778.0       | 4,235.6        | 352.0    | 14.8    | 0.5        | 8.2        |

**Manifold**

| Leaves | Vertices | Triangles | Voxelize | Build octree | Sample corners | Solve vertices | Cluster | Validate | Mark collapsed | Contour | Build mesh | Debug data |
|-------:|---------:|----------:|---------:|-------------:|---------------:|---------------:|--------:|---------:|---------------:|--------:|-----------:|-----------:|
| 16³    | 628      | 1,252     | 117.9    | 18.3         | 53.2           | 34.0           | 6.7     | 0.5      | 0.3            | 3.2     | 0.4        | 1.5        |
| 32³    | 852      | 1,700     | 762.9    | 92.9         | 420.1          | 181.8          | 39.1    | 4.1      | 2.2            | 7.8     | 0.5        | 10.9       |
| 64³    | 944      | 1,884     | 5,280.5  | 890.1        | 3,296.3        | 629.6          | 262.9   | 35.1     | 21.1           | 22.2    | 0.5        | 99.0       |
| 128³   | 2,368    | 4,732     | 56,792.3 | 21,746.0     | 28,427.6       | 3,504.1        | 1,905.8 | 274.6    | 165.5          | 78.2    | 1.1        | 689.3      |

### Burst Uniform (step 2)

Same model and settings as the baseline. Times are medians of 3 runs after a warm-up, in the editor with Burst safety checks on (the editor default).

| Cells | Vertices | Triangles | Voxelize | Sample corners | Solve vertices | Contour | Build mesh | Managed voxelize | Speed-up |
|------:|---------:|----------:|---------:|---------------:|---------------:|--------:|-----------:|-----------------:|---------:|
| 16³   | 760      | 1,516     | 0.5      | 0.1            | 0.2            | 0.1     | 0.1        | 104.5            | ~200×    |
| 32³   | 3,216    | 6,428     | 1.3      | 0.2            | 0.7            | 0.2     | 0.1        | 678.6            | ~520×    |
| 64³   | 13,000   | 26,036    | 5.0      | 0.8            | 2.8            | 1.0     | 0.3        | 4,818.9          | ~960×    |
| 128³  | 52,224   | 104,500   | 34.6     | 7.5            | 16.5           | 8.0     | 1.3        | 38,554.3         | ~1,110×  |
| 256³  | 208,900  | 417,876   | 199.1    | 50.9           | 78.1           | 46.6        | 4.6        | out of memory    | —        |

- **Same output:** vertex and triangle counts equal the managed ones at every size managed can contour.
- **Mesh post-processing:** at 256³, splitting vertices takes 22.7 ms and `RecalculateBounds` with `OptimizeReorderVertexBuffer` 18.1 ms. Both are now a sizeable part of the total.
- **Contour stage:** the single-threaded pass is a quarter of the time at 128³ and 256³, which makes it the first candidate for step 5.

### What the baseline shows

- **Sampling corners is 45–69% of the time** in every algorithm. Every voxel samples its own 8 corners, and in the octrees every internal node samples its corners too. Each sample goes through LINQ and interface calls.
- **Building the grid or octree is another 11–38%**, made up of a class per voxel, corner and edge. The share grows with the grid; at 128³ it is 16% for Uniform, 29% for Adaptive and 38% for Manifold.
- **The algorithm-specific stages are small** next to those two: simplification, clustering and contouring.
- **Memory limits the managed implementations.** A single 128³ contour takes about 10 GB of managed memory, and the editor process went from 5.4 GB to 15 GB over the 128³ runs. 256³ would need about 80 GB, so the performance test skips managed Uniform above 128³. Octrees cannot go past 128³ leaves anyway.
- **Time grows 6–9× per doubling of the grid**, close to the 8× of a cost proportional to the number of cells. Manifold goes from 64³ to 128³ at 11×, most of it in building the octree.
- **Mesh post-processing is now negligible:** at most 6 ms to split vertices and 4 ms for `RecalculateBounds` and `OptimizeReorderVertexBuffer`, at 128³.
  - The split loop before step 0 read `mesh.vertices` once per triangle corner. On real Uniform meshes it took 270 ms at 32³ (5,740 triangles) and 3.6 s at 64³ (23,100 triangles), against 0.8 ms and 1.5 ms now, with identical output.
  - Its cost grows with triangles × vertices, so at 128³ it would have been close to a minute.
- **The other two step 0 fixes have no before-and-after timing.**
  - Adaptive and Manifold used to evaluate the density functions once per function at every corner. `DensitySamplingTest` counted 2.46× (Adaptive) and 2.57× (Manifold) as many samples per volume with three volumes as with one.
  - The cost of `Volume.origin` and `Volume.extents` searching the hierarchy on every sample was not measured on its own.
- **Found in step 2, fixed 2026-11-11 at the user's request:** the debug corners and edges were slow to tell apart.
  - `Edge.GetHashCode` XORed the hashes of the edge's two corners. For an axis-aligned edge the two coordinates that agree cancel out, so every edge between the same two planes of the grid got the same hash: 31 hashes for the 104,544 edges of a 32³ grid. `Distinct()` over them took about 30 s.
  - `Vector3.GetHashCode`, which the corners used, shifts the coordinate hashes by two bits and XORs them. On the few mantissa bits of grid coordinates that gave 3,447 hashes for 35,937 corners.
  - The managed implementations hand their corners and edges out through `Distinct()`, and `Voxelizer.Start` goes through them when debug drawing is on. Managed scenes with debug drawing on paid for it at start-up.
  - Now corners hash with `SurfaceExtractor.hashOf`, which combines the three coordinates, and edges combine their two corner hashes in order. This is done in the Uniform, Adaptive, Manifold and Burst debug classes.
  - At 32³, `Distinct()` takes 82 ms on the edges and 33 ms on the corners. Voxel hashes are unchanged: voxel equality uses Unity's approximate `==`, which no exact hash can follow, and voxels never go through `Distinct()`.
  - Test: `DebugInformationHashTest`, which needs at least 99% distinct hashes for the corners and edges of each implementation.
