# Manifold Dual Contouring on the GPU

Plan written 2026-11-13 for a GPU implementation of Manifold Dual Contouring (`Documents/Papers/Manifold Dual Contouring.md`), next to the managed and Burst ones.

**Status:** decided, not started. The user answered the five decisions at the end on 2026-11-13, each as recommended, and asked for the plan to be updated with no code changes yet. Nothing is built and nothing was measured for it. Step 0 waits for the go-ahead.

## Goal

`ManifoldDualContouring` runs on the GPU when its implementation is set to GPU: corners, leaf vertices, clustering and polygons all in a compute shader, with the mesh read back or drawn straight from the GPU as the other two GPU implementations have it. The mesh is closed and manifold wherever the Burst one is. While only the error threshold changes, a new contour reuses the vertex tree that is already on the GPU. The managed and Burst implementations, and the other algorithms, give the meshes they give today.

## What the paper does, and what of it is parallel

- **Leaves (section III).** One vertex for every piece of the contour in a cell, from the dual marching cubes table, each with the QEF of the edges its piece crosses. Every cell by itself.
- **Clustering (section IV.A).** Bottom-up: in every node, the vertices of its eight children that the contour connects across the twelve faces between them become one vertex, whose QEF is the sum of theirs. A node reads only what is below it, so the nodes of a level are independent, and the levels are a chain as long as the octree is deep: 7 at 128³.
- **Manifold criterion (section V).** Thirteen whole numbers a vertex: an Euler characteristic and an intersection count for each edge of its node, summed from its members (equation 2). No floats.
- **Polygons (section IV.B).** A polygon around every crossed leaf edge, between the vertices that stand in for the four leaf vertices: for each, the last collapsible one on the way up its vertex tree. The paper finds the edges with a recursive walk of the octree. This project's octree is always full, so the edges are those of the leaf grid, each by itself.

## What exists today

- **Burst** (`Assets/Scripts/Burst/ManifoldDualContouring.Burst.cs`): `SolveLeavesJob`, one `ClusterLevelJob` per level, a single-threaded recursive `ContourJob`, `DebugInformationJob`, `ValidateJob`. Vertices of all levels are in one set of arrays, with a start and a count per node.
- **Two GPU implementations** to take from:
  - `OccupancyDualContouring.compute` has what the leaves need for their topology: `classifyCells` (solid corners, the complement rule, a count and room for each cell's vertices, the polygon count), one readback of two totals to size the buffers, `contourShared` and `contourSplit`.
  - `UniformDualContouring.compute` has what the leaves need for their geometry: density with a material mask, `approximateIntersection`, `calculateNormal`, `surfaceCorrection`, Hermite data on boundaries between materials, and `QEF.hlsl`.
- **The frame around them:** `GPUVoxelization` (vertices, normals, quads of six indices and a sub-mesh, counts, indirect arguments), `SurfaceExtractor.readMesh`, drawing straight from the GPU in `Voxelizer.contour`, and `VoxelizerEditor` forcing the SVD solver on the GPU.
- **`ManifoldDualContouring`** throws for GPU, and its inspector turns GPU into Managed. The interactive editor gives the octree algorithms Burst when GPU is asked for (`implementationFor`).

### Findings that shape the work

1. **Nothing in the algorithm is serial.** See above. The longest chain is the levels, one dispatch each.

2. **The topology is exact on the GPU.** Which vertices are clustered, the Euler characteristics and the intersection counts are whole numbers, so they are Burst's bit for bit. Only positions, normals and the error of a cluster are floats.

3. **Collapse decisions near the threshold will differ from Burst's,** as Burst's differ from managed (Burst plan, step 4). The error is a difference of nearly equal sums, and the GPU rounds differently again.
   - **At the default threshold of 6e-12** "below the threshold" means "rounded to exactly zero", which falls either way. The GPU will give a third contour on the two scenes where managed and Burst already disagree.
   - **Every such contour is closed and manifold.** The criterion does not depend on the error.
   - **For the tests** this means: exact parity with Burst where no decision hangs on a float (threshold zero, and a threshold out of the way), and invariants in between.

4. **Where Burst's time goes, 128³, three balls, clustering at 2e-4** (Burst plan, after tuning): corners 6.2 ms, leaf vertices 11.9, clustering 5.5, validation 1.6, polygons 3.6. A whole contour is about 34 ms since the new shapes and rotation. Two thirds of it is corners and leaves, which are what a GPU is best at.

5. **What to expect, as an estimate and nothing more.** Occupancy's GPU compute is about four times faster than its Burst equivalent, and its readback and mesh building take about 17 ms of its 32 at 128³.
   - **Simplified:** the mesh is small (2,359 vertices at 2e-4), so readback is cheap. Perhaps 10 to 15 ms for a whole remesh.
   - **Not simplified:** readback dominates as it does for Occupancy. Perhaps 25 to 30 ms.
   - **Drawn straight from the GPU:** no readback at all.
   - **Clustering is the unknown.** Many small dispatches with little to do near the root. Step 3 measures it.
   - **Burst is already under the 50 ms** up to which the editor follows a drag, so speed at 128³ is not what this buys. See finding 6.

6. **The vertex tree outlives a contour on the GPU.** The paper's section VI: once the tree is built, a new threshold only changes which vertices are collapsible. If the shader keeps a cluster's error and whether it passed the criterion, a threshold change is a few passes over the tree's vertices and the polygon pass (kernels 6 to 10), with nothing sampled or solved. That is what the GPU version offers that the Burst one does not have today.

7. **`ClusterLevelJob` ports nearly line for line, with one change.** A thread per node, working in the room of that node as the job does.
   - **The long loop:** the job walks every pair of leaves on either side of the twelve faces between a node's children. The root's thread would walk 49,152 pairs at 128³, nearly all of them empty. A GPU thread is slow at that.
   - **The change:** descend through the nodes on either side of a face, and only where both have vertices. The counts per node are already there. The order in which pairs are found changes, the clusters do not: a cluster's index is that of its first member, and members add up in their own order.
   - **No compare-and-swap is needed.** Each node's union-find is its thread's alone.

8. **The room a level needs is known only on the GPU.** A node gets room for as many vertices as its children have clusters.
   - **Burst** lays each level out exactly, on the CPU, between jobs.
   - **On the GPU** a node takes its room from a counter with `InterlockedAdd`, as cells do in Occupancy's shader. The order varies from run to run, which nothing depends on.
   - **How large the buffers are** has to be decided before the levels run. At worst every level has as many vertices as the leaves. On a surface a level has about a quarter of the one below, a third more than the leaves in all. Step 0 measures it.
   - **Decided (decision 3):** room for twice the leaf vertices to start with, grown when it runs out (see Room under Design).

9. **Polygons can lose one or both of their triangles, and the quad format has no place for that.** Clustering collapses a quad to a triangle, an edge or a point. A GPU quad is six indices.
   - **In the shader:** a polygon with nothing left is not written. One with a triangle left is written with its other half as one vertex three times, which draws nothing.
   - **On readback:** `ManifoldDualContouring.readMesh` leaves out triangles that use a vertex twice. `SurfaceExtractor.readMesh` and the draw shader stay as they are.

10. **The same triangle can come out of two polygons.** Managed drops repeats with `Distinct`, Burst with a hash set in its single-threaded pass. On the GPU that would take a hash table with compare-and-swap, which this project has not used on Metal.
    - **Decided (decision 2):** repeats are dropped on readback, in the same pass as finding 9. Drawn straight from the GPU a repeated triangle is drawn twice in the same place and cannot be seen.
    - **How often it happens** is not known. Step 0 counts what Burst's set turns away, and if they turn out to be common that is reported before step 3 builds on this.

11. **The shader's QEF cannot be combined as Burst's is.** `QEF.hlsl` has no mass point count or dimension, which `BurstQEF.combine` uses to keep the mass point of the higher dimension. It needs the rank and a combine. Uniform's and Occupancy's use of the struct must not change.

12. **A kernel may write eight buffers.** `ClusterLevelJob` writes fourteen arrays. On the GPU a vertex is a record: position, normal, parent, node, Euler characteristic, error, flags and its twelve intersection counts in one buffer, its QEF in a second, and the three numbers clustering works with in a third. With the node starts, node counts and totals that is six.
    - **Intersection counts as bytes,** as in Burst: at most one for each leaf edge along an edge of a node, 128 at the root.
    - **About 130 bytes a vertex.** The three balls at 128³ have 52,224 leaf vertices.

13. **A wrong parent or cluster index would hang the GPU** where Burst throws. Every walk up a chain gets a bound, the depth of the octree or the members of the node, and sets a flag in the totals when it reaches it. The driver asserts the flag is clear whenever it reads the totals.

14. **The debug view and the validation can run on Burst's own jobs.** `DebugInformationJob` and `ValidateJob` read arrays of starts, counts, parents, nodes, positions and normals. Read back from the GPU into native arrays, the tree can be given to both as it is.

15. **SVD only,** as on the GPU elsewhere. The topology tests that run over both solvers skip the simple one for the GPU.

16. **The grid stays at 128³ at most** (`resolution` 2 to 8, leaves 2^(resolution − 1)). A larger grid is where the GPU would gain most, and is not part of this plan.

## Design

### The component

- `ManifoldDualContouring.voxelize` dispatches GPU to `voxelizeGPU`, in `Assets/Scripts/GPU/ManifoldDualContouring.GPU.cs`, a partial class with a `GPUImplementation` as Occupancy has.
- `convert`, `readMesh` and `release` are implemented. The inspector no longer turns GPU into Managed.
- **Settings:** all of them mean what they mean in Burst. No new ones.

### Shared shader code

Moved, not rewritten, with the output and timing of the two existing shaders checked before and after, as when the shape functions moved:

- **`HermiteData.hlsl`:** density and material at a point, density with a material mask, intersection, normal, surface correction, out of `UniformDualContouring.compute`.
- **`DualMarchingCubes.hlsl`:** the table offsets and the lookups over them, the solid corners of a cell and the complement rule, out of `OccupancyDualContouring.compute`. The tables buffer is built in one place in C#, and gains what clustering needs: the edges of each face, the children on either side of each face inside a node, and the edge two leaves across a face or four around an edge have in common.
- **`QEF.hlsl`:** `calculateRank` and `combineQEF`. The count and dimension of the mass point are kept beside the QEF in this shader's own record (finding 11).

### What the shader keeps

- **For every corner:** density and material.
- **For every node of the octree,** level by level as `BurstOctree` has them, the leaves last: where its vertices start and how many it has. A leaf also has its solid corners and what it looks its vertices up with.
- **For every vertex of the tree:** the three records of finding 12. The leaves' vertices come first.
- **For the mesh:** vertices, normals, quads, counts and arguments, as the other shaders have them.
- **Totals:** leaf vertices, polygons, tree vertices, and the flags for room that ran out and for a chain that hit its bound.

### Kernels

`Assets/Resources/Shaders/ManifoldDualContouring.compute`. Thread groups are 4 × 4 × 4 over corners, cells and nodes, and 64 over vertices.

1. **`sampleCorners`.**
2. **`classifyCells`:** as Occupancy's. Room for each leaf's vertices, and the polygon count.
3. **Readback** of the totals. The driver sizes the tree, at twice the leaf vertices unless it is already larger, and the mesh buffers. Mesh vertices are at most the leaf vertices, and quads at most the polygons.
4. **`solveLeaves`:** `SolveLeavesJob`. Hermite data of the cell's twelve edges, boundaries between materials, a QEF and a vertex for each piece.
5. **`clusterLevel`,** once for each level from the one above the leaves to the root, with the level set before each dispatch: `ClusterLevelJob` with the descent of finding 7. It writes each cluster's error and whether it passed the criterion, not whether it is collapsible, and marks what is left of its room as unused.
6. **`flagVertices`,** over the tree: collapsible is an error under the threshold and, with `enforceManifold`, the criterion.
7. **`resolveVertices`,** over the leaf vertices: the last collapsible vertex on the way up, kept with the leaf vertex.
8. **`markVertices`,** over cells, shared normals only: the four vertices of every polygon are marked as used. Every thread that marks a vertex writes the same thing.
9. **`emitVertices`,** over the tree, shared normals only: a used vertex takes the next place in the mesh.
10. **`contourShared` or `contourSplit`,** over cells: the quad around each crossed edge that starts at the cell, with what clustering left of it (finding 9). With split normals every triangle that is left gets three vertices of its own and the normal of its face.

A threshold change with the tree in place runs 6 to 10.

### Room

The tree's buffers are kept from one contour to the next and only grow. A node that finds no room sets the flag and leaves itself empty. The driver reads the totals after the last level, and when the flag is set makes the buffers as large as the totals say, then runs clustering again. Corners and leaves are not run again.

### Readback

- **`readMesh`:** counts, vertices, normals and quads, then the triangles that neither use a vertex twice nor repeat an earlier one. With split normals a triangle has vertices of its own, so both are judged by position.
- **`convert`:** the mesh, and for the debug view the tree read back and given to Burst's `DebugInformationJob` (finding 14). In the editor and in development builds `ValidateJob` runs on it too.

### Reusing the tree

The driver remembers what the tree on the GPU was built from: the uploaded density functions, the resolution and every setting but `errorThreshold`, `enforceManifold` and the vertex normals. When a contour is asked for and those are the same, it starts at `flagVertices`. `release` forgets it.

### Interactive editor

- `implementationFor` gives Manifold the GPU when asked. Ten variants where there are nine.
- Nothing else changes: the settings are the ones Manifold has.

### Example scenes

`Assets/Scenes/Manifold Dual Contouring/GPU`, copies of Burst scenes with the implementation changed, as the Burst scenes were made from the managed ones. Which ones is decided at that step, with one simplified and one drawn straight from the GPU among them.

## Tests

Burst is the reference. Each new test is shown to fail on a deliberate mistake, as in the earlier plans.

- **Shared shader code:** rank and combine in the play-mode `SVDTest`, against `BurstQEF` on the same samples. The existing GPU tests and the `Start` hashes for the two shaders that lose code to the includes.
- **Topology:** `ManifoldDualContouringTopologyTest` gains a GPU subclass, SVD only. All of it applies: closed and manifold with and without clustering, two pieces in a leaf, joined faces, boundaries between materials, separate surfaces, no degenerate triangles, the cube to its corners, clustering as far as the criterion allows, the tube never clustered.
- **Against Burst** (new `ManifoldDualContouringGPUTest`, compared with `MeshComparison` within a tolerance measured in the step):
  - **Leaves:** threshold zero, every model, turned shapes, the new shapes, surface correction, both intersection approximations.
  - **Clustering without floats:** a threshold out of the way, with and without `enforceManifold`. The same triangles between the same vertices.
  - **The tree:** read back, the same number of clusters in every node as Burst, with the same Euler characteristics and intersection counts, at 16³ and 32³. This needs a way for the test to see Burst's tree, kept internal.
  - **In between** (2e-4 and the scene thresholds): closed, manifold, as many pieces and handles as the unclustered mesh, every vertex within a tolerance of the surface. Equal counts are asserted on the models where step 3 finds them equal.
- **Room:** a contour that starts with buffers too small for its tree gives the mesh of one that does not.
- **Repeats and collapsed polygons:** a mesh read back has no triangle twice and none with a vertex twice, on the models where step 0 found Burst turning triangles away.
- **Reusing the tree:** a contour after a threshold change is the mesh of a fresh contour at that threshold. A change to a volume, the resolution or any other setting builds the tree again.
- **Drawing straight from the GPU and split normals:** as `OccupancyDualContouringGPUTest` has them.
- **The frame:** `SubMeshTest`, `VoxelizerContourTest`, `VoxelizerDefaultsTest` and the editor tests over all variants take the new one in.
- **No change for the others:** the `Start` hashes of step 0 after every step.
- **Performance:** the GPU remesh timing of the Occupancy plan, with Manifold beside it.

## Steps

Each step compiles, passes every test, and is reviewed before the next one starts. Nothing is committed for you.

0. **Baseline and what Burst can tell (small).** Both suites green; the `Start` hashes; Burst Manifold and the two GPU remeshes timed with Unity in front. From a probe on the Burst implementation, over the test models and thresholds and the three balls up to 128³: vertices in the tree for each leaf vertex, clusters per level, and how many triangles the set turns away. Those correct the factor of two that the tree's room starts with (decision 3) and say how much finding 10 matters.
1. **Shared shader code (medium).** The two includes and the QEF additions, with the table builder in one place. Uniform's and Occupancy's GPU output and timings as before.
2. **Leaves (large).** The driver, kernels 1 to 4, and 7 to 10 over a tree that is only leaves, so the unclustered mesh is complete: dispatch, inspector, `readMesh`, drawing straight from the GPU, split normals. The leaf tests and the unclustered topology tests. **Measured here:** GPU time of corners and leaves, and a whole remesh without clustering.
3. **Clustering (large).** `clusterLevel`, `flagVertices`, collapsed polygons, repeats, room that runs out, the chain bounds. The clustered topology tests and the rest of the tests against Burst. **Measured here:** GPU time of each level, and a whole remesh at 2e-4 against Burst.
4. **Debug view and reusing the tree (medium).** `convert` through Burst's jobs, the tree kept between contours, their tests. **Measured here:** a contour after a threshold change.
5. **Interactive editor and scenes (small).** `implementationFor`, the editor tests, the scenes.
6. **Tuning and documentation (medium).** Whatever steps 2 to 4 show to cost most. Candidates: Hermite data once per edge and not once per cell around it; thread group sizes; fewer dispatches for the top levels. README (the table of implementations, the GPU paragraph, the code layout) and this plan marked complete.

## Measurements

None yet.

## Out of scope

- **A larger grid than 128³.**
- **The simple QEF solver on the GPU.**
- **Clustering across the faces in parallel,** with a lock-free union-find. It would need compare-and-swap, and is only worth looking at if step 3 shows the top levels to cost more than the rest together.
- **A threshold that varies over the model,** as in the paper's figure 12. Reusing the tree is what it would be built on.
- **Corners and leaves on the GPU with Burst's clustering** on what is read back (decision 1).
- **Dropping repeated triangles on the GPU** (decision 2).
- **Room for the worst case,** so that clustering never runs twice (decision 3).
- **Reading the mesh back without waiting for the GPU.**
- **The same reuse of the tree in Burst.**
- **An Adaptive Dual Contouring GPU implementation.**

## Decisions

All five were answered by the user on 2026-11-13, each as recommended.

1. **All of it on the GPU, or only corners and leaves?** The other way reads the leaves' vertices and QEFs back and runs Burst's clustering and polygon jobs on them. It is about a third of the work and keeps Burst's output order, but a mesh could not be drawn straight from the GPU and the tree could not be reused there. Recommended: all of it. **Decided: all of it** ("Full GPU").
2. **Repeated triangles: dropped on readback, left in when drawn straight from the GPU?** Recommended: yes (finding 10), unless step 0 finds they are common. **Decided: yes** ("Yes"). What step 0 counts is still reported.
3. **Room for the tree.** Recommended: twice the leaf vertices to start with, grown when it runs out (see Room), with the factor corrected by step 0. The other way is room for the worst case, eight times the leaf vertices at 128³, about 55 MB for the three balls, with no second run ever. **Decided: as recommended** ("Your suggestion").
4. **Reusing the tree for threshold changes in this plan?** Recommended: yes, as part of step 4. The kernels are split for it either way; what the step adds is the driver remembering what the tree was built from. **Decided: yes** ("Include").
5. **Editor and scenes in this plan?** Recommended: yes, as step 5. The scenes are new files; `Interactive Editor.unity` does not change. **Decided: yes** ("Include them").
