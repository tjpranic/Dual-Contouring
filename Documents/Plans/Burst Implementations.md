# Burst Implementations

Plan written 2026-11-11 for adding a Burst implementation of each dual contouring algorithm next to its managed implementation.

**Status:** complete. All six steps, 0 to 5, were done and reviewed on 2026-11-11. Uniform, Adaptive and Manifold dual contouring each have a Burst implementation next to the managed one.

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

As built in step 3, in `Assets/Scripts/Burst/AdaptiveDualContouring.Burst.cs`. The grid, settings, corner job and cell solve that Uniform and Adaptive share moved to `Assets/Scripts/Burst/BurstGrid.cs`.

1. **Octree:** a grid of nodes per level in one array. Node (x, y, z) of a level has children (2x + i, 2y + j, 2z + k) on the next, in the managed child order. Every node starts internal; a collapsed node becomes Pseudo, and the nodes below it are ignored from then on.
2. **Leaves:** the corner and cell code shared with Uniform, on the leaf grid. Each leaf keeps the QEF as `placeVertex` left it, since solving it sets the feature dimension that combining reads.
   - The managed code also solves a QEF for every internal node from its own corners. Nothing reads that QEF, so Burst skips it.
   - The parity tests show the result is unchanged: the same nodes collapse into the same vertices, and the debug voxels list the same nodes.
3. **Simplification:** one parallel job per level, bottom-up, each node combining its children in order. A node without surface becomes Pseudo; otherwise come the topology test, then `placeVertex`, then the threshold.
4. **Contouring:** a literal port of `contourCell`, `contourFace` and `contourEdge` in one job, recursive, which Burst compiles. The plan first said an explicit stack; recursion keeps the managed order with no extra bookkeeping.
5. **Debug data:** one job walks the nodes in the managed list order. It collects the non-internal nodes, their corners and their edges, each the first time it is seen. An edge gets material-boundary hermite data when the first node that has it is one the contour passes through, which is exactly what the managed implementation shows.

### Manifold (large)

As built in step 4, in `Assets/Scripts/Burst/ManifoldDualContouring.Burst.cs`. The octree struct that Adaptive and Manifold share moved to `Assets/Scripts/Burst/BurstOctree.cs`.

1. **Vertices:** a node can hold several, so they live in arrays of their own, and each node has a start and a count into them.
   - The leaves' vertices come first, then those of each level above in turn.
   - A node above the leaves gets room for as many vertices as its children have between them, which is the most clusters it can end up with. Room is added one level at a time, once the level below has been clustered, so the arrays hold about as many vertices as there are.
   - Each vertex keeps its position, normal, QEF, parent, collapsible flag, Euler characteristic, node and 12 intersection counts.
2. **Leaf pass:**
   - One byte per leaf records which corners are solid. The dual marching cubes table turns that into the leaf's vertex count and into the vertex of each intersected edge, so no edge-to-vertex map is stored.
   - Each vertex gets the QEF of its edges in edge order, then the material boundaries that reach it, or all of them where none reaches any vertex.
   - It is solved with `solve`, not `placeVertex`, followed by surface correction.
3. **Clustering:** one parallel job per level, bottom-up.
   - **Members:** the children's vertices, in child order.
   - **Union-find:** runs over the leaf pairs on the node's 12 internal faces. In the full octree these are two squares of leaves per face. A leaf vertex is followed up its parents to the member it belongs to.
   - **Output:** clusters come out in the order of their first member, and members add to their cluster in member order. That is the managed order, whatever the union-find did in between.
   - **Per-cluster values:** intersection counts, the Euler characteristic, the face condition and the QEF solve all follow the managed code.
   - **Collapse flags:** `collapsible` per vertex and `collapsed` per node are set in the same job. The managed code does this in a pass of its own.
   - **Parallel writes:** each node writes only its own vertices and the parents of its children's vertices.
4. **Contouring:** a literal port of the traversal, recursive as in Adaptive, including both skips: `contourCell` on a collapsed node, and `contourFace` when both nodes are collapsed.
   - The contour vertex of a leaf vertex is looked up when a polygon needs it, not in a pass of its own.
   - Triangles are de-duplicated on their three indices in insertion order, to match `Distinct()`. Only triangles with a cluster vertex go through the set: a triangle between three leaf vertices can only come from one edge.
5. **Validation:** the managed debug-build check, that every child vertex was clustered into a vertex of the enclosing node, is a job that also runs only in debug builds, which includes the editor.
   - The plan first said it would become a test. A test can't see the vertex tree, and as an assertion in the editor it fails any test that trips it, which is how the managed check works too.
6. **Debug data:** one job walks the leaves in the managed order and shows each contour vertex once, as a voxel in the node it was created in. Only leaves have hermite data on their edges, as in the managed implementation.

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
3. **Adaptive (medium to large). Done 2026-11-11.** The same as step 2, with 9 scenes, done on the same terms.
   - Code: `Assets/Scripts/Burst/AdaptiveDualContouring.Burst.cs` and the shared `Assets/Scripts/Burst/BurstGrid.cs`; `AdaptiveDualContouring` is now partial.
   - Scenes: `Assets/Scenes/Adaptive Dual Contouring/Burst/`, 9 copies with `_implementation: 2`. Each one contours to the same triangles and voxel count as its managed scene.
   - Tests:
     - `AdaptiveDualContouringTopologyTest` is now an abstract base, and its 59 tests run as `ManagedAdaptiveDualContouringTopologyTest` and `BurstAdaptiveDualContouringTopologyTest`. Parameterized `[TestFixture]`s weren't picked up by the test runner.
     - `AdaptiveDualContouringBurstTest` has 53 parity tests: every model, both solvers, no, light and heavy simplification, with and without the topology test, four grid sizes, 128³ leaves, and debug data on request. The parity checks are shared in `BurstParity`.
     - Edit mode 363/363, play mode 20/20.
   - The parity vertex tolerance went from 1e-3 to 5e-3 leaf cells. A collapsed node's vertex comes from the QEFs of all its leaves added up, and the farthest apart seen was 0.002 leaves at 32³.
4. **Manifold (large). Done 2026-11-11.** The same, with 11 scenes, done on the same terms, with two scenes that differ as explained below.
   - Code: `Assets/Scripts/Burst/ManifoldDualContouring.Burst.cs` and the shared `Assets/Scripts/Burst/BurstOctree.cs`; `ManifoldDualContouring` is now partial. `BurstTables` gained the edges around each face.
   - Scenes: `Assets/Scenes/Manifold Dual Contouring/Burst/`, 11 copies with `_implementation: 2`.
     - 9 contour to the same triangles and voxel count as their managed scene, with vertices at most 0.0013 leaves apart.
     - "Multiple Materials" and "Sphere" don't. Both use the default error threshold of 6e-12, which is below what the error formula can resolve. See the findings.
   - Tests:
     - `ManifoldDualContouringTopologyTest` is now an abstract base, and its 45 tests run as `ManagedManifoldDualContouringTopologyTest` and `BurstManifoldDualContouringTopologyTest`.
     - `ManifoldDualContouringBurstTest` has 87 parity tests: every model, both solvers, four clustering thresholds, with and without the manifold criterion, four grid sizes, the special models of the topology tests, surface correction, 128³ leaves, and debug data on request.
     - `SubMeshTest` and `DebugInformationHashTest` include Burst Manifold. `BurstTablesTest` checks the two new lookups.
     - Edit mode 501/501, play mode 20/20.
5. **Tuning. Done 2026-11-11.** Measured first, then changed what the measurements pointed at. The numbers are in "Tuning" under Measurements.
   - **Float mode: stays Strict.** Fast measured 3–5% less time and moved more collapse decisions away from managed. The mode is now one constant, `BurstFloats.mode`, that every job and test job uses.
   - **Batch sizes: unchanged.** Four times larger or smaller made no measurable difference. They are named constants in `BurstBatches`.
   - **Empty cells: the change that mattered.** More than half the time went into walking cells the contour doesn't pass through. Fixing that took 23–35% off the 128³ and 256³ timings.
     - `BurstSolidCornersJob` in `BurstGrid.cs` classifies every cell in one sweep per row and writes a byte of solid corners per cell. The leaf passes of all three algorithms read that byte and return at once for an empty cell.
     - Uniform and Adaptive no longer write a vertex, normal or QEF for empty cells. What those arrays hold there is never read.
     - Manifold lays its leaf vertices out per row in parallel, and counts each level's members in parallel, in place of two single-threaded passes over every node.
     - The Uniform contour pass skips eight cells at a time where none starts a quad.
   - **Parallel traversal: not done.** After the above, the single-threaded Uniform contour pass is 5 ms of 24 at 128³ and 32 of 153 at 256³. What is left of it is per-quad work in first-use order, which is what keeps the mesh identical to managed.
   - **Same output:** 99 configurations (three algorithms, both solvers, two models, several sizes and thresholds) hash to the same meshes and debug data before and after.
   - Tests: `BurstGridTest` (12) for the new job. Edit mode 513/513, play mode 20/20.
   - `README.md` lists the implementations and the packages they need.

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

### Burst Adaptive (step 3)

Same model and settings as the baseline: simplification at 2e-4, topology-safe.

| Leaves | Vertices | Triangles | Voxelize | Sample corners | Solve vertices | Simplify | Contour | Build mesh | Managed voxelize | Speed-up |
|-------:|---------:|----------:|---------:|---------------:|---------------:|---------:|--------:|-----------:|-----------------:|---------:|
| 16³    | 628      | 1,252     | 0.8      | 0.1            | 0.2            | 0.1      | 0.4     | 0.0        | 119.7            | ~150×    |
| 32³    | 852      | 1,700     | 1.8      | 0.2            | 0.7            | 0.3      | 0.5     | 0.1        | 757.9            | ~420×    |
| 64³    | 1,056    | 2,148     | 6.7      | 0.8            | 3.6            | 1.0      | 0.6     | 0.1        | 5,449.8          | ~810×    |
| 128³   | 2,586    | 5,224     | 44.4     | 6.5            | 20.4           | 5.0      | 1.6     | 0.1        | 47,104.2         | ~1,060×  |

- Up to 64³ the counts equal the managed ones.
- At 128³, managed has 2,594 vertices and 5,240 triangles: a handful of nodes collapse in one implementation and not the other. That's the precision limit of the error test, explained in the findings below, not a difference in the algorithm.

### Burst Manifold (step 4)

Same model and settings as the baseline: clustering at 2e-4 with the manifold criterion on.

| Leaves | Vertices | Triangles | Voxelize | Sample corners | Solve vertices | Cluster | Validate | Contour | Build mesh | Managed voxelize | Speed-up |
|-------:|---------:|----------:|---------:|---------------:|---------------:|--------:|---------:|--------:|-----------:|-----------------:|---------:|
| 16³    | 628      | 1,252     | 1.0      | 0.1            | 0.3            | 0.3     | 0.0      | 0.3     | 0.0        | 117.9            | ~120×    |
| 32³    | 852      | 1,700     | 2.3      | 0.2            | 0.9            | 0.5     | 0.1      | 0.6     | 0.1        | 762.9            | ~330×    |
| 64³    | 944      | 1,884     | 7.9      | 0.8            | 3.8            | 1.9     | 0.3      | 1.0     | 0.1        | 5,280.5          | ~670×    |
| 128³   | 2,359    | 4,714     | 42.5     | 6.8            | 20.2           | 10.0    | 1.7      | 3.9     | 0.1        | 56,792.3         | ~1,340×  |

- Up to 64³ the counts equal the managed ones.
- At 128³, managed has 2,368 vertices and 4,732 triangles. 22 clusters 8 leaves across collapse in one implementation and not the other: 12 only in Burst, 10 only in managed. The managed errors of all 22 are between 0.99 and 1.18 times the threshold. That's the same precision limit as in Adaptive.
- The mesh at 128³ is closed and manifold with no clustering, at 2e-4 and with the threshold out of the way. On the same model, plain dual contouring has 56 non-manifold edges.
- Collapse flags are part of the Cluster stage. Validate only runs in the editor and in development builds.

### Tuning (step 5)

Same model and settings as above, in the editor with Burst safety checks on. These replace the "Voxelize" columns of the three Burst tables above, which were measured before tuning.

**Voxelize after tuning, in ms**

| Cells or leaves | Uniform | Before | Adaptive | Before | Manifold | Before |
|----------------:|--------:|-------:|---------:|-------:|---------:|-------:|
| 16³             | 0.5     | 0.5    | 0.8      | 0.8    | 1.2      | 1.0    |
| 32³             | 1.3     | 1.3    | 2.1      | 1.8    | 2.6      | 2.3    |
| 64³             | 5.3     | 5.0    | 6.4      | 6.7    | 7.0      | 7.9    |
| 128³            | 24.3    | 34.6   | 28.8     | 44.4   | 29.0     | 42.5   |
| 256³            | 152.5   | 199.1  | —        | —      | —        | —      |

Against managed at 128³ that is about 1,590× for Uniform, 1,640× for Adaptive and 1,960× for Manifold. Vertex and triangle counts are unchanged.

**Stages after tuning, in ms**

| Run           | Sample corners | Solve vertices | Simplify or cluster | Validate | Contour | Build mesh |
|---------------|---------------:|---------------:|--------------------:|---------:|--------:|-----------:|
| Uniform 128³  | 6.0            | 11.1           | —                   | —        | 5.3     | 1.2        |
| Uniform 256³  | 48.0           | 54.3           | —                   | —        | 31.9    | 4.6        |
| Adaptive 128³ | 6.1            | 12.8           | 4.9                 | —        | 1.6     | 0.1        |
| Manifold 128³ | 6.2            | 11.9           | 5.5                 | 1.6      | 3.6     | 0.1        |

**What was measured on the way**

- **Empty cells.** A model that touches almost no cells took 17.1 ms of Uniform's 31.5 ms at 128³ and 134.5 of 196.8 ms at 256³, with Adaptive and Manifold at 24 of 37–39 ms. After the change the same model takes 8.8 ms at 128³ and 77 ms at 256³, most of it sampling corners.
- **Burst safety checks.** With the checks off, as in a build, the untuned code took 15–38% less time (Uniform 22 against 31.5 ms, Manifold 25 against 40.5 ms at 128³). All tables here have them on, the editor default.
- **Float mode.** Fast took 3–5% less time. Adaptive went from 2,586 to 2,574 vertices and Manifold from 2,359 to 2,344 at 128³.
- **Batch sizes.** Five combinations from a quarter to four times the current sizes all landed within the 5% the runs differ by anyway.
- **After a domain reload** the first benchmark run measured 9.6 ms at 16³ where a warm run takes 0.5 ms. The cause wasn't looked into; it looks like the jobs running before Burst has finished compiling them.

**What is left, and was left alone**

- **Sampling corners** is now the largest fixed cost: every corner samples every volume, about 9 divisions and square roots per ellipsoid. Making it cheaper means changing the arithmetic, which changes the last bits of every density, or culling parts of the grid, which is a different algorithm.
- **Hermite data** is worked out by each cell for its own edges, so a crossing edge is worked out by the four cells around it. Working each edge out once would take an estimated 10–15% off the total, for a compact per-edge store with its own counting and layout passes in all three leaf passes.
- **The Uniform contour pass** is single-threaded: 32 ms at 256³.
- **Allocation** of the dense per-cell arrays is 12–15 ms at 256³ for Uniform, about 640 MB for a mesh of 209 thousand vertices. Compact vertex arrays would shrink both.

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
- **Found in step 3, not fixed: the error test of simplification is at the limit of float precision on large nodes.**
  - The error is xᵀAx − 2xᵀb + c, where the terms nearly cancel. For a node 8 leaves across, with 380 samples, c (BtB) was 70.4 against a threshold of 2e-4. Float rounding of the sum is then a few times 10⁻⁶, several percent of the threshold.
  - Example at 128³ leaves on the three balls: a node whose error in double precision is 2.033e-4, 1.7% above the threshold, was collapsed by the managed code. Its mirror image, with the same samples, was kept by managed and collapsed by Burst.
  - At 128³ a few dozen nodes fall in that band at any threshold, so the two implementations differ by a few nodes at 1e-4 and 2e-4 alike. At 32³ there are about 150 candidates, and none fell in the band.
  - It would go away in both implementations if the QEF sums were kept relative to a local origin, such as the node's center, which keeps c small. That changes the numerics of the algorithm, so it is left for a decision.
- **Found in step 4, left as it is: an error threshold of 6e-12, the default of Manifold, is below what the error formula can resolve.**
  - The error is xᵀAx − 2xᵀb + c in float. With c (BtB) near 1, its smallest step is about 6e-8, ten thousand times the threshold. A cluster whose true error is zero comes out as 0 or as one such step, depending on the last bits of the sums. So at 6e-12, "below the threshold" means "rounded to exactly zero".
  - Two of the 11 Manifold scenes use 6e-12 and get a different contour from Burst than from managed. Both contours are closed and manifold.
    - **Sphere:** each of the 8 leaves on the diagonals is alone in its cluster, with three samples and BtB 0.75. Managed computes an error of 5.96e-8 (2⁻²⁴) and keeps the leaf vertex. Burst computes 0 and uses the cluster vertex, which is not moved onto the surface, so it sits 0.058 leaves from the managed one.
    - **Multiple Materials:** two clusters half the grid across have an error of exactly 0 in managed. The same sums give 3.8e-6 with every operation rounded to float and 9.5e-7 in double. Burst keeps them apart: 52 vertices against 40.
  - Neither implementation is the right one here. Managed's zero in Multiple Materials is as much a matter of rounding as Burst's zero on the sphere.
  - The other 9 scenes have thresholds from 2e-4 up, or shapes whose sums are exact, and match.
  - Keeping the QEF sums in double would make the error meaningful down to about 1e-15 in both implementations. A local origin alone would not: it shrinks the step to about 1e-9, still above 6e-12. Either changes the numerics of the managed solvers.
  - **Decided 2026-11-11: the solvers stay in float.** That level of precision isn't needed for this project. The two scenes keep their Burst contours, and the few clusters at 128³ that collapse differently are accepted.
- **Found in step 3, expected:** plain dual contouring of the three balls has edges shared by more than two triangles from 64³ cells on, 40 at 64³ and 56 at 128³, in Uniform and in unsimplified Adaptive, managed and Burst alike. Where the balls meet in creases, a cell holds two sheets of surface and has only one vertex for both. That is what the manifold variant is for. The large-grid closedness tests use the sphere.
- **Found in step 2, fixed 2026-11-11 at the user's request:** the debug corners and edges were slow to tell apart.
  - `Edge.GetHashCode` XORed the hashes of the edge's two corners. For an axis-aligned edge the two coordinates that agree cancel out, so every edge between the same two planes of the grid got the same hash: 31 hashes for the 104,544 edges of a 32³ grid. `Distinct()` over them took about 30 s.
  - `Vector3.GetHashCode`, which the corners used, shifts the coordinate hashes by two bits and XORs them. On the few mantissa bits of grid coordinates that gave 3,447 hashes for 35,937 corners.
  - The managed implementations hand their corners and edges out through `Distinct()`, and `Voxelizer.Start` goes through them when debug drawing is on. Managed scenes with debug drawing on paid for it at start-up.
  - Now corners hash with `SurfaceExtractor.hashOf`, which combines the three coordinates, and edges combine their two corner hashes in order. This is done in the Uniform, Adaptive, Manifold and Burst debug classes.
  - At 32³, `Distinct()` takes 82 ms on the edges and 33 ms on the corners. Voxel hashes are unchanged: voxel equality uses Unity's approximate `==`, which no exact hash can follow, and voxels never go through `Distinct()`.
  - Test: `DebugInformationHashTest`, which needs at least 99% distinct hashes for the corners and edges of each implementation.
