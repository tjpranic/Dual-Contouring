# Mesh Simplification

Plan written 2026-11-13 for a mesh simplifier that runs on the finished mesh of any algorithm: quadric error edge collapse, with line quadrics and a progressive record of the collapses.

**Status:** draft. Nothing is built and nothing was measured for it. The seven decisions at the end are open, and step 0 waits for them and for the go-ahead.

**Papers,** all in `Documents/Papers/`:
- *Surface Simplification Using Quadric Error Metrics* (Garland and Heckbert, 1997): the algorithm.
- *Controlling Quadric Error Simplification with Line Quadrics* (Liu, Rahimzadeh and Zordan, 2025): a second error term that settles flat regions.
- *Progressive Meshes* (Hoppe, 1996): the record of collapses, and the rules for boundaries between materials.
- *Topology Preserving Edge Contraction* (Dey, Edelsbrunner, Guha and Nekhayev, 1999): which collapses keep the topology.

## Goal

Every voxelizer has a **Mesh Simplification** setting, off by default. On, the mesh a contour gives is simplified before it is shown: edges are collapsed in order of the error they add, for as long as that error stays under a bound. The simplified mesh has the topology of the contour, the same sub-meshes, and its sharp edges and corners. A **Detail** slider then picks any mesh between the contour and its simplest form without contouring or simplifying again. With the setting off, every mesh is the one it is today, bit for bit.

It applies to all four algorithms and all three implementations, since it works on the mesh and not inside a contourer. Where it gains most is Uniform and Occupancy-Based Dual Contouring, which have no simplification: the three balls at 128³ are 104,500 triangles from Uniform, against 4,732 from Manifold clustering at 2e-4. It also applies after Adaptive and Manifold have simplified, where it can merge across the octree nodes that clustering stops at.

## What the papers do, and what of them is used

- **Quadrics (Garland and Heckbert, sections 4 and 5).** Every vertex keeps a symmetric 4 × 4 matrix, ten numbers, that gives the sum of squared distances from any point to the planes of the triangles the vertex stands for. Collapsing an edge adds the two matrices. The new vertex goes where the sum is least, which is a 3 × 3 linear solve, and the sum there is the cost of the collapse. Edges sit in a heap by cost, and the cheapest is collapsed until the target is reached.
  - **Used:** all of it, for edges only. Joining vertices that share no edge (section 3.2, a threshold above zero) changes the topology and is left out.
  - **Boundaries (section 6):** for every triangle on a boundary edge, a plane through the edge and square to the triangle is added to the quadrics of the edge's ends, with a large weight. Used for the edges between materials, open edges and non-manifold edges.
  - **Inversion (section 6):** a collapse that turns the normal of a neighbouring triangle over is refused.
- **Line quadrics (Liu et al., section 4).** On a flat region every plane quadric is zero, so the 3 × 3 system has no single answer and the order of collapses is arbitrary. A line quadric is the squared distance to the line through a vertex along its normal, written as two plane quadrics (equations 13 to 15). Added with a small weight times the vertex's area (equation 16), it makes every system well-conditioned and places merged vertices at the area-weighted centre of the vertices they replace (lemma 4.1). The paper's default weight is 1e-3, and it recommends staying under 1e-1.
  - **Used:** the line quadric with one weight for the whole mesh.
  - **Left out:** weights that vary over the mesh (sections 4.4.1 and 5.3) and the edge variant (section 4.4.2).
- **Progressive meshes (Hoppe, section 3).** The sequence of collapses is kept. Each one can be undone by a vertex split, so the full mesh and every mesh in between can be had from the record. Every vertex of a finer mesh has one ancestor in a coarser one (section 3.2).
  - **Used:** the record, and meshes at any level through the ancestor map.
  - **Used from section 4.5:** the six tests on sharp edges around an edge that say whether a collapse changes the topology of the curves they form. Here the sharp edges are those between materials.
  - **Left out:** geomorphs, progressive transmission, compression, selective refinement, and the paper's own energy function, which the quadrics replace.
- **Link conditions (Dey et al., sections 4 and 5).** Collapsing edge *ab* keeps the topology of a 2-manifold exactly when the vertices next to both *a* and *b* are only the two opposite the edge (theorem B₀). For a mesh that is not a manifold, theorem B gives two conditions over the mesh and over its edges that do not have exactly two triangles.
  - **Used:** theorem B, since not every contour is a manifold (finding 4). On a manifold it is theorem B₀.

## What exists today

- **One place every contour passes through:** `Voxelizer.contour`. A CPU implementation hands it a `Mesh`; a GPU one hands it buffers, read into a `Mesh` by `readMesh` unless they are drawn straight from the GPU. It then splits vertices for flat shading, recalculates bounds and shows the mesh.
- **The mesh:** one array of vertices with normals. Sub-mesh n holds the triangles of material bit n. The Burst implementations build it from native arrays of positions, normals and `BurstMesh.Triangle` (three indices and a sub-mesh) in `BurstMesh.createMesh`.
- **QEF code:** `BurstQEF` keeps AtA, AtB and BtB in floats, which is a quadric in other letters, with solvers made for Hermite data and a mass point.
- **The interactive editor** builds a signature of everything a contour depends on. `RemeshScheduler` contours when it changes: live while a contour takes at most 50 ms, after the changes settle otherwise, and only when asked after a contour over a second. The editor never draws straight from the GPU.
- **Test helpers:** `MeshTopology` (open, non-manifold and odd edges, winding conflicts, pinched vertices, components, Euler characteristic, for a mesh or one sub-mesh), `MeshComparison`, `MeshIntersections` (crossing triangle pairs), exact distances to every shape in `ShapeDistanceTest`, and the `Start` hash probe.

### Findings that shape the work

1. **One hook serves every algorithm and implementation.** The simplifier takes the mesh in `Voxelizer.contour`, before vertices are split. Managed, Burst and GPU contours all arrive there as a `Mesh`.

2. **A mesh drawn straight from the GPU cannot be simplified,** since it never reaches the CPU. With simplification on, the mesh is read back, as it is for the debug view.

3. **The GPU splits vertices itself for flat shading,** in its contour kernel, and such a mesh has no shared vertices to collapse edges between. With simplification on, the voxelizer asks its implementation for shared normals and splits afterwards on the CPU, as it does for the CPU implementations today.

4. **Not every contour is a closed manifold.**
   - **Uniform and Adaptive** give edges with more than two triangles where a cell holds two sheets of surface: 40 on the three balls at 64³ and 56 at 128³ (Burst plan).
   - **Manifold and Occupancy** give a manifold.
   - **Any algorithm** probably leaves the mesh open where the surface leaves the grid. I have not checked this. Step 0 counts open edges on a model that crosses the grid's bounds.
   - **What it means:** the mesh is kept as vertices with lists of their triangles and edges, as Garland and Heckbert have it, not as half-edges. Legality is theorem B of Dey et al.

5. **In the ranking I said the quadrics could be seeded from the contourer's own planes. I no longer recommend that.**
   - **They are not there to take.** Burst Uniform keeps only a vertex and a normal per cell. The GPU implementations do not read their QEFs back. Occupancy's planes are fitted through searched points. Only Adaptive and Manifold keep a QEF for each vertex.
   - **The line quadric's weight is defined against area-weighted triangle quadrics** (equations 8 and 16). Hermite QEFs count samples, not area, so the published weights would not carry over.
   - **Triangle planes are what both papers use,** and they need nothing but the mesh. See decision 2.

6. **Floats are not enough for a quadric kept about the origin.** The constant term of a vertex 100 cells from the origin is near 10⁴ times its area, where a float resolves about 10⁻³. The costs to be ordered on a flat region are zero or close to it. The Burst plan met the same limit in the octree's error test. Doubles cost Burst nothing in speed here, and the simplifier has no managed or GPU version to match. Step 1 measures the float error before anything is built on either. See decision 3.

7. **With line quadrics the cost of a collapse is no longer a distance to the surface.** On a flat face of a cube it is the line term alone, and it grows as the face is merged down to two triangles. A bound on the whole cost would stop a cube far short of twelve triangles.
   - **So the two sums are kept apart,** twenty numbers a vertex: the heap orders by their sum, as the paper has it, and the bound is tested against the plane part alone.
   - **This is my addition.** The paper stops at a triangle count and does not need it.

8. **The bound can be a length.** A vertex's plane quadric, divided by the area it stands for, is the mean squared distance to its planes weighted by area. Its root is a distance, which the setting gives in cells, as the project's tolerances are. The division is also my addition.

9. **Occupancy's triangles may cross after simplification.** Its split of every polygon is chosen so that triangles do not pass through each other, and a collapse undoes that choice. The inversion test catches a fold around the collapsed edge and nothing further away. Step 3 counts crossing pairs before and after with `MeshIntersections`. See decision 6.

10. **The collapse loop is serial.** Each collapse changes the costs around it, so the heap is worked one edge at a time. Building the quadrics and the first costs is parallel.
    - **An estimate and nothing more:** taking 52,000 vertices to their floor is about 50,000 collapses, each recomputing about ten edges with a solve, an inversion test and a heap move. That is perhaps 50 to 150 ms at 128³ and a quarter of it at 64³.
    - **What it means for the editor:** at 128³ a contour with simplification will probably pass the 50 ms up to which a drag is followed live, and wait for changes to settle instead. The scheduler already does that by itself.

11. **A changed simplification setting needs no new contour.** If the voxelizer keeps the unsimplified mesh, a new error bound is a new run of the simplifier, and a new detail is a walk through the record.

12. **The order must not depend on chance.** Equal costs are common on flat regions when the line weight is zero. Ties go to the lower edge index, so the same mesh always simplifies the same way and hashes can be compared between runs.

13. **Garland and Heckbert do not say how large the "large penalty" on boundary planes is.** Step 2 measures what keeps a straight boundary between two materials straight and a round one round, and sets it.

14. **Degenerate triangles have no plane.** A triangle without area adds nothing to a quadric, and a vertex whose faces cancel has no normal for its line. Liu et al. give such a vertex a point quadric of weight 1e-6 (section 4.2), which is used here too.

## Design

### Settings

On `Voxelizer`, so every algorithm has them, carried over by the editor's `select` and part of its signature:

| Setting | Default | What it does |
|---|---|---|
| Mesh Simplification | Off | Simplifies the mesh of every contour. |
| Simplification Error | measured in step 3 | How far a collapse may leave a vertex from the planes of the surface it stands for, as a root mean square distance in cells. |
| Line Quadric Weight | 1e-3 | The weight of the line quadrics. Zero gives Garland and Heckbert's algorithm as written. |
| Detail | 0 | Which mesh of the record is shown: 0 is the simplest the error allows, 1 is the contour itself. |

### Code

- **`Assets/Scripts/Burst/BurstQuadric.cs`:** a quadric in doubles. From a plane, from a triangle with its area, from a boundary edge and one of its triangles, and from a vertex and its normal as a line. Add, evaluate, and the point of least error.
- **`Assets/Scripts/Burst/BurstMeshSimplifier.cs`:** the connectivity, the legality tests, the heap, the loop and the record, as Burst jobs. It takes positions, normals and `BurstMesh.Triangle`s and gives the same back.
- **`Assets/Scripts/MeshSimplification.cs`:** the settings, the record as managed arrays, and the mesh at a level of detail.
- **`Voxelizer.contour`:** the hook, between the mesh arriving and vertices being split.

### The algorithm

1. **Connectivity.** For each vertex its triangles and its edges. For each edge how many triangles it has, and whether its triangles are of different sub-meshes. Edges that are open, non-manifold or between materials are sharp.
2. **Quadrics,** in parallel. Each triangle adds its plane quadric times a third of its area to its three vertices, and each sharp edge its boundary planes to its two ends. Each vertex adds its line quadric, times the weight and its area, to its second sum.
3. **Costs,** in parallel. For every edge, the point of least error of both sums of both ends, and the error there.
   - **Position:** with a line weight above zero the system is always solvable. At zero the fallbacks are the paper's: the best point on the edge, then the better of its ends and middle.
4. **The loop.** Take the cheapest edge. It is collapsed unless one of these refuses it:
   - **Error:** the plane part at the new position is over the bound (findings 7 and 8).
   - **Topology:** the link conditions of theorem B fail.
   - **Materials:** one of Hoppe's six tests says the curves between materials would change.
   - **Inversion:** a triangle around either end would turn over.
   
   A refused edge leaves the heap and returns when a collapse next to it gives it a new cost. A collapse moves one end to the new position, gives it both ends' triangles, edges, sums and area, removes the triangles that lost a side, and recomputes the edges around it. It ends when the heap is empty.
5. **Normals.** The new vertex takes the sum of the two normals, each weighted by the area its vertex stands for, made unit length. Manifold's clusters add their members' normals the same way, without the weights.
6. **The record.** Each collapse writes the vertex kept, the vertex removed, and the position and normal the kept one got.

### The mesh at a level of detail

A level is a number of collapses. The mesh at that level is the unsimplified mesh with every vertex replaced by its ancestor after that many collapses (Hoppe, section 3.2), at the position its ancestor last got, without the triangles that then use a vertex twice. That is one pass over the record and one over the triangles, with no connectivity, and it is parallel.

- **Detail 0** is every collapse, and the mesh the loop ended with.
- **Detail 1** is none, and the contour itself.

### What the voxelizer keeps

The unsimplified mesh and the record of its last contour, as a `Mesh` and managed arrays, so a script reload leaks nothing. They are dropped by `clear` and replaced by the next contour.

- **`Voxelizer.simplify( )`** shows the last contour again under the settings as they are now: a new walk through the record when only the detail changed, a new run of the simplifier when the error or the weight did.
- **`ContourStatistics`** gains the sizes before simplification.

### Interactive editor

- The signature is split in two: what a contour depends on, and the four simplification settings. A change to the second alone calls `simplify`, not `contour`.
- The line under **Contour** shows the sizes before and after.
- Nothing changes in `Interactive Editor.unity`: the voxelizers in it take the defaults, which are off.

### Example scenes

New scenes in `Assets/Scenes/Mesh Simplification`, copies of Burst scenes with the setting on. Which ones is decided at that step, with a Uniform and an Occupancy scene, one with several materials and one with flat shading among them.

## Tests

There is no second implementation to compare with, so the tests hold the simplifier to answers known in closed form and to invariants, as Occupancy's do. Each new test is shown to fail on a deliberate mistake, as in the earlier plans.

- **Quadrics:**
  - The error of a point is the sum of squared distances to the planes, worked out one plane at a time.
  - A line quadric is the squared distance to its line.
  - The best point of three planes is where they meet.
  - On a flat patch with line quadrics, the best point of a set of vertices is their area-weighted centre (lemma 4.1).
  - A boundary plane holds its edge and is square to its triangle.
- **Legality,** on meshes built by hand:
  - The two cases of figure 2 of Dey et al., one legal and one not.
  - No edge of a tetrahedron collapses.
  - An edge with three triangles, a pinched vertex and an open edge, each against theorem B.
  - Each of Hoppe's six cases around a boundary between materials.
  - A fan that the best position would fold over.
- **Shapes with known answers,** for every algorithm that contours them exactly: an upright cuboid becomes 8 vertices and 12 triangles at its corners, an octahedron 6 and 8, a pyramid 5 and 6, a hexagonal prism 12 and 20. A cuboid of two materials keeps a straight boundary between them, with 12 vertices and 20 triangles. Turned shapes within a tolerance measured in the step.
- **Invariants,** over the models of `TopologyTest`, every algorithm, Burst and GPU, and Managed on small grids:
  - As many components, the same Euler characteristic, and as many open and non-manifold edges and pinched vertices as the contour, for the mesh and for each sub-mesh.
  - No degenerate triangles, no winding conflicts, and no triangle turned over.
  - Every vertex within the bound of the shape, by the exact distances.
  - A lower bound never gives fewer triangles.
  - The torus keeps its handle.
- **The same every time:** two runs give the same mesh bit for bit.
- **The record:**
  - Detail 1 is the contour, triangle for triangle.
  - Detail 0 is the mesh the loop ended with.
  - The mesh at level k is the mesh of a run stopped after k collapses.
  - Every level passes the invariants.
- **The frame:**
  - With the setting off, the `Start` hashes are those of step 0.
  - Flat shading after simplification, and past 65,535 split vertices.
  - A GPU contour is read back and simplified.
  - `simplify` after a change of detail or error gives the mesh of a fresh contour under those settings, without calling `voxelize`.
- **Editor:** the settings carry over in `select`; a change of detail does not contour; undo.
- **Performance:** `ContouringPerformanceTest` gains the simplifier, by stage.

## Steps

Each step compiles, passes every test, and is reviewed before the next one starts. Nothing is committed for you.

0. **Baseline and what the contours are like (small).** Both suites green; the `Start` hashes; Burst timings with Unity in front. A probe over the test models and example scenes, for every algorithm and implementation: vertices, triangles, open edges, non-manifold edges, pinched vertices, edges between materials, degenerate and repeated triangles, and vertices that share a position. That settles finding 4 and says what the connectivity has to cope with.
1. **Quadrics (small).** `BurstQuadric` and its tests. **Measured here:** the error of a float quadric against a double one on a flat face and on a ball at 128³ (finding 6, decision 3).
2. **Connectivity and legality (medium).** The lists, sharp edges, the link conditions, Hoppe's tests, the inversion test, and one collapse with everything it updates, on meshes built by hand. No heap yet. **Measured here:** the weight of boundary planes (finding 13).
3. **The simplifier (large).** Heap, loop, bound, normals, the mesh out; the hook, the first two settings and the inspector; shared normals asked of the GPU. Line weight zero, so this step is Garland and Heckbert's algorithm with its own fallbacks. The shape, invariant and frame tests. **Measured here:** time by stage from 16³ to 128³ and Uniform at 256³, triangles against the error bound for each algorithm, distance to the shapes, and crossing triangle pairs before and after (finding 9). The default error comes out of this.
4. **Line quadrics (small).** The second sum, the weight, the point quadric for degenerate vertices, their tests. **Measured here:** weights 0, 1e-4, 1e-3, 1e-2 and 1e-1 as in the paper's figure 4: triangles, distance to the shapes, the smallest angle of the triangles, and time. The default weight is set from it.
5. **The record (medium).** The record, the mesh at a level, the Detail setting, what the voxelizer keeps, `simplify`, their tests. **Measured here:** a change of detail and a change of error at 128³, against a whole contour.
6. **Interactive editor and scenes (small).** The split signature, the statistics line, `select`, the editor tests, the scenes.
7. **Tuning and documentation (medium).** Whatever steps 3 to 5 show to cost most. Candidates: Burst implementations handing their arrays to the simplifier without building a mesh first; the heap; stopping at the detail asked for when the record is not wanted. README (a section, the settings, the code layout), `AGENTS.md` (the papers) and this plan marked complete.

## Measurements

None yet.

## Out of scope

- **A managed or a GPU simplifier** (decision 1).
- **Quadrics from Hermite data** (decision 2).
- **Joining vertices that share no edge,** and any collapse that changes the topology (decision 5).
- **Keeping triangles from crossing** (decision 6).
- **Vertex splits, geomorphs, selective refinement, compression** (decision 7).
- **Line quadric weights that vary over the mesh,** and the edge variant.
- **Quadrics over colour or texture coordinates.** The meshes have neither.
- **Levels of detail by distance,** `LODGroup`s and chunks.
- **A parallel collapse loop.**
- **Lindstrom and Turk's cost function,** and octree clustering for Occupancy. Both were declined on 2026-11-13.

## Decisions

1. **One Burst simplifier for every implementation?** A managed contour would be simplified by Burst code, and nothing would be simplified on the GPU. The other way is a managed simplifier beside it as a reference, which doubles the algorithm for a mesh the Burst one would give anyway. Recommended: Burst only, with tests against known answers, as Occupancy has.
2. **Quadrics from triangle planes?** Finding 5. The other way seeds them from Hermite QEFs where they exist, which is Burst Adaptive and Manifold only, and needs its own weights. Recommended: triangle planes, with Hermite seeding out of scope.
3. **Doubles for the quadrics?** Finding 6. You kept the QEF solvers in floats on 2026-11-11, since precision was not a must-have. This is different code with nothing to stay in step with, and its ordering depends on it. The other way is floats about a moving origin, which is more code. Recommended: doubles, confirmed by step 1's measurement before step 2 builds on them.
4. **An error bound in cells and a detail slider?** Findings 7 and 8. The other way is one target, a triangle count or a share of the triangles, as both papers have it. That is simpler, but a cube and a ball then need different numbers. Recommended: both settings as designed.
5. **Topology never changes?** No collapse joins, splits, opens or closes anything, on the surface or on the boundaries between materials. A small ball then never goes below a tetrahedron and never vanishes. The other way allows such collapses at a penalty (Hoppe, section 4.6), and gives fewer triangles on models with small features. Recommended: never, for a project whose algorithms are about keeping the surface manifold.
6. **Crossing triangles: counted, not prevented?** Finding 9. Preventing them means testing every collapse against the triangles near it, from a paper that is not in `Documents/Papers/` (Gumhold, Borodin and Klein, 2003). Recommended: step 3 counts them and reports, and the README says Occupancy's guarantee holds with simplification off.
7. **The record as a full mesh and a list of collapses?** The paper's own form is the simplest mesh and a list of vertex splits. That suits sending a mesh piece by piece, and needs connectivity to refine. Recommended: the full mesh and the collapses, read through the ancestor map. Every contour then runs to its floor, whatever the detail, and step 7 can stop early when the record is not wanted.
