# Occupancy-Based Dual Contouring

Plan written 2026-11-12 for adding the algorithm of *Occupancy-Based Dual Contouring* (Hwang and Sung, 2024; `Documents/Papers/Occupancy-Based Dual Contouring.md`) as a fourth algorithm, with a Burst and a GPU implementation and no managed one.

**Status:** in progress. The user accepted the five decisions at the end as recommended on 2026-11-12 ("All decisions as you recommend") and asked for step 0. Step 0 was done the same day. After it the user decided to give Manifold Dual Contouring the complement rule too ("We'll add the rule to the Manifold Dual Contouring implementation") and asked for step 1. Both were done on 2026-11-12 and wait for review.

## Goal

A new voxelizer component, `OccupancyDualContouring`, contours the same volumes as the other three on a uniform grid, in Burst and on the GPU. It asks the volumes only whether a point is inside or outside, never how far away the surface is. It can be picked in the interactive editor like the other algorithms, and has example scenes of its own. Uniform, Adaptive and Manifold Dual Contouring give the meshes they give today.

## What the paper does

The paper (ODC below) starts from Manifold Dual Contouring on a uniform grid, where a cell has one vertex for every piece of the contour that passes through it, and changes three things.

- **1D points (section 5.1).** Where the contour crosses a grid edge is found by bisecting the edge on inside or outside alone, 15 times, instead of interpolating distances.
- **2D points (section 5.2, appendix A.1.3 and A.1.4).** On every face of a cell that the contour crosses, between the two 1D points the crossing connects, three searches along rays find the point where the surface's two straight pieces would meet on that face. A straight crossing gives the middle of the two 1D points. Each search takes a few evenly spaced samples to find the first change, then bisects.
- **3D points (section 5.3).** The plane at a 1D point is the one through it and the two 2D points next to it in the cell. Those planes take the place of the gradient in the QEF, and the vertex of each piece is the QEF's minimizer, as in Manifold Dual Contouring.
- **Polygons (section 5.4, appendix A.1.5).** The quad around an edge is split along the diagonal that keeps both triangles inside an envelope around the edge. When neither diagonal does, it becomes four triangles around the edge's 1D point. This is the rule of Intersection-free Contouring, tested as in Wang's convex/concave analysis.
- **Manifold (appendix A.1.1).** The dual marching cubes table can give a non-manifold edge where two ambiguous cells meet. The paper follows Wenger's fix.
- **No simplification.** The grid is uniform. Adaptive resolution is named as future work.

## What exists today

- **A voxelizer is a component with a switch over implementations.** `ManifoldDualContouring` and `AdaptiveDualContouring` dispatch to `voxelizeManaged` and `voxelizeBurst` and refuse GPU; `UniformDualContouring` has all three. `Implementation.Type` is `{ Managed, GPU, Burst }` and its values are saved in scenes.
- **Pieces of contour per cell.** `DualMarchingCubes` builds the table of which edges of a cell belong to which vertex, for all 256 ways its corners can be solid. `BurstTables` holds it as constant data, and the GPU gets the number of pieces per configuration as a buffer.
- **Inside and outside is already a question of its own.** `calculateMaterial` gives the materials at a point from the signs of the shapes alone, and a corner is solid when it has any material. Every algorithm decides which edges the contour crosses from that. Distances are only used to place the intersection on an edge, for the gradient and for surface correction.
- **Burst on a uniform grid:** `BurstGrid`, `BurstSampleCornersJob`, `BurstSolidCornersJob` (a byte of solid corners per cell), `BurstQEF` (both solvers), `BurstMesh` and `BurstDebugInformation`. Each cell works out the hermite data of its own edges, so nothing is stored per edge.
- **GPU:** `UniformDualContouring.compute` holds the nine shape functions, the density loop and the five kernels of Uniform Dual Contouring, with `QEF.hlsl` (SVD only) beside it. A contour is handed over as vertex, normal and quad buffers, where a quad is six indices and a sub-mesh. `Voxelizer` reads those back or draws them straight from the GPU.
- **Interactive editor:** `InteractiveEditor.Algorithm` is `{ Uniform, Adaptive, Manifold }`, with one voxelizer component of each on the editor object. Seven variants can be picked.

### Findings that shape the work

1. **The project's volumes are already an occupancy function.** Solid means "has a material", which is worked out from signs. ODC uses that one question everywhere, so it never reads a distance, and the sub-mesh of a polygon comes from the same sample. This is also why it would keep working on functions that are not distances, such as the warped and compressed ones in `TODO.md`.
2. **On this project's exact distances, ODC should not be visibly more accurate than Manifold Dual Contouring.** The paper's gains are over gradients of occupancy networks, which are poor normals. Here the gradient is the true normal. ODC's planes are secants through points on the surface where the gradient gives tangents, so I expect vertices about as far off a curved surface, on the other side of it. What ODC brings here is independence from distances, the intersection-free split, the manifold fix, and multi-vertex cells on the GPU. Step 2 measures this instead of assuming it.
3. **The dual marching cubes table gives non-manifold edges on a uniform grid, as the paper warns.** I re-stated the rule of `DualMarchingCubes.findVertexEdges` in Python and contoured 300 random 6³ grids with it.
   - **The cause:** in 36 of the 256 configurations (24 with five solid corners, 12 with six) one face is ambiguous and all four of its edges belong to the same piece. Where two such cells share that face, the edge between their two vertices is used by four quads.
   - **The size of it:** 458 edges that are not used exactly twice, in 68,308 quads.
   - **The fix:** a cell whose such face is shared with a cell that has it too looks its pieces up under the complement of its solid corners, which pairs the edges on ambiguous faces the other way. The complement of each of the 36 has no such face, and no configuration has more than one. With the rule, the same 300 grids gave no edge used other than twice and no pinched vertex.
   - **Manifold Dual Contouring has the same table.** Its leaves can form such a pair. The existing topology tests pass, so their models do not contain one. It has the rule since step 0a.
   - **Confirmed on the project's own table in step 0.** `DualMarchingCubesTest` goes through every pair of cells that share a face, all 12,288 of them. Without the rule, the edge between two vertices is part of four polygons exactly where both cells have the shared face joined, and of two everywhere else. With the rule it is two everywhere.
   - **Why the complement is safe,** also from step 0: in each of the 36, the joined face is the cell's only ambiguous face. So the complement connects edges differently on that face alone, where the neighbour takes its complement too, and the two still agree across every face.
4. **The paper leaves out things the authors' code does.** I read the reference implementation (github.com/KAIST-Visual-AI-Group/ODC) for what the paper does not say.
   - **Vertices are kept inside their cells.** The QEF's minimizer is pulled back along a line from a point inside the cell until it is inside, less a margin of a thousandth of a cell. The intersection-free argument needs that: the envelope is built from the cells' corners.
   - **The QEF is regularized,** by a pull of weight 0.05 toward the average of the piece's 1D points, and solved by least squares. This project's solvers truncate small singular values or iterate instead.
   - **Vertices of the same cell are kept apart** by planes from a table.
   - **Which diagonal when both are allowed:** the one whose triangles pass nearer the edge's 1D point.
   - **A 2D point falls back to the middle** of its two 1D points when they coincide, when the first search finds the surface at once, when the two lines are parallel, or when they meet on the wrong side.
   - **A plane is left out of the QEF** when its three points lie on a line. A piece with no planes left gets the average of its 1D points.
5. **The reference code cannot be copied.** It is licensed CC BY-NC-SA 4.0 and this project is MIT. The implementation is written from the paper. The tables it needs are generated from the project's own `DualMarchingCubes`, and what finding 4 lists is re-derived, not ported.
6. **The reference code works in double precision, with thresholds below what a float resolves.** This project stays in float (decision of 2026-11-11). A float holds 17 bits inside a cell at 256³ at the far side of the grid, so 15 bisections fit with two bits to spare. The searches run in units of a cell from the corner of the edge or face, and only the sample itself is taken to object space. The thresholds are chosen for float in step 2, by measurement.
7. **ODC samples far more than the other algorithms.**

   | What | Samples |
   |---|---|
   | A 1D point: 15 bisections | 15 |
   | A 2D point: its middle, then three searches of 15 | 46 |
   | Manifold today, per intersected edge: 6 bisections and 6 for the gradient | 12 |

   A cell that the surface crosses simply has four intersected edges and four crossed faces.
   - **Each cell working out its own points,** as the Burst implementations do now: 244 samples, against 48.
   - **Sharing them between the cells around an edge or face:** about 107.

   From Manifold's 12.8 ms for its vertices at 128³, I estimate a Burst contour of the three balls at 128³ at about 85 ms without sharing and 45 ms with, against 33 ms. That is an estimate, to be replaced by the measurement of step 2.
8. **A 1D point has up to four planes,** one in each cell around its edge, because each cell has its own two 2D points next to it. The gradient gave an edge one normal.
9. **The quad format fits.** Four triangles around a 1D point are twelve indices, which is two of the GPU's quads, so the draw shader and the mesh readback need no change.
10. **The voxelizer's distance settings have no meaning here.** Binary Search Iterations (6, at most 12), Intersection Approximation and Surface Correction Iterations place points by distance. ODC has its own searches, which need more steps than 12: its planes come from the positions of three points, so those have to be far more exact than an intersection that also has a gradient.
11. **`Interactive Editor.unity` needs the new component on its editor object.** `RequireComponent` only adds it to objects made afterwards.

## Design

### The component

`OccupancyDualContouring : Voxelizer`, in `Assets/Scripts/OccupancyDualContouring.cs`, a partial class like the others.

- **Grid:** uniform, 2^resolution cells across, as `UniformDualContouring`.
- **Implementations:** Burst and GPU. A new component starts as Burst. Managed throws `NotSupportedException`, and the inspector turns it into Burst, as Manifold's turns GPU into Managed. Until step 4 the inspector turns GPU into Burst as well.
- **Settings it uses from `Voxelizer`:** resolution, QEF solver, minimizer iterations, vertex normals, the debug view. On the GPU the solver is SVD, as for Uniform.
- **Settings it hides** (finding 10): binary search iterations, intersection approximation, surface correction iterations.
- **Its own settings,** with the paper's values. Each is added in the step that first uses it:
  - `edgeSearchIterations` = 15: bisections for a 1D point.
  - `faceSearchSamples` = 4 and `faceSearchIterations` = 11: the first search of a 2D point. The two searches along the line take one sample fewer and one bisection more, as in the paper, so all three cost the same.
  - `splitQuads` = true: the intersection-free split. Off gives the plain split of Manifold Dual Contouring, which is the paper's ablation and shows what the split does.
- **Constants:** the ranges of the searches, 0.8 of a cell and √2/2 of a cell, and the margin that keeps vertices inside their cells.

### Inside and outside

A point is inside when `calculateMaterial` gives it any material (finding 1). Corners are sampled for their materials only. The solid corners of a cell, a byte, come from `BurstSolidCornersJob` as now.

### Pieces, and the manifold rule

- **Pieces:** `DualMarchingCubes`' table, as Manifold Dual Contouring uses it.
- **New in `DualMarchingCubes`** (built in step 0):
  - `lookupJoinedFace( solidCorners )`: the face whose four edges belong to one piece, or none (finding 3).
  - `lookupEdgeAcrossFace( solidCorners, edge, side )`: the edge next to an intersected edge around its piece, across either of the edge's two faces, which `lookupEdgeFace( edge, side )` names. A piece is a cycle of 1D and 2D points in turn, and this is the cycle.
  - `complement( solidCorners )`, and `lookupFaceAxis` and `lookupFaceSide`, which say where the cell across a face is.
- **The rule:** a cell with a joined face whose neighbour across it has it joined too uses the table under the complement of its solid corners. Both cells do, so it does not depend on the order cells are visited in, and Burst and the GPU decide alike.
- **In Burst** the rule is `BurstCells.vertexLookupOf`, which gives what a cell looks its vertices up with. It is worked out wherever it is needed instead of being stored: only a cell with a joined face looks at its neighbour, and few have one.
- **Burst and GPU copies:** constant data in `BurstTables`, generated from `DualMarchingCubes` and held to it by `BurstTablesTest`. The GPU gets buffers filled from `DualMarchingCubes`, as it gets its vertex counts now.

### 1D points

Bisect the edge between its solid and its empty corner `edgeSearchIterations` times, keeping one end solid and one empty. The 1D point is the solid end of the last interval. The search is written from the edge's own start corner along its axis, so every cell around the edge finds the same point to the bit.

### 2D points

For a crossing of a face between the 1D points `a` and `b`, in the face's two coordinates and in units of a cell:

1. `m` is the middle of `a` and `b`, and is sampled. If `a` and `b` coincide, the 2D point is `m`.
2. The first search leaves `m` at right angles to the line through `a` and `b`, toward the side where the face's corners differ from `m` in being inside or outside. It gives `q`, the last point like `m` before the first change. If `q` is at `m`, the crossing is straight and the 2D point is `m`.
3. Two searches leave `q` along the line's direction, one toward `a`'s side and one toward `b`'s, and give `q1` and `q2`.
4. The 2D point is where the line through `a` and `q1` meets the line through `b` and `q2`. It is `m` when they are parallel or meet behind `m`.

- **A search** (appendix A.1.3) samples evenly out to its range, takes the first interval whose far end differs from the start, or the last interval when none does, and bisects it. It returns the end nearer the start, which is on the start's side of the surface.
- **Each cell works out the 2D points of its own pieces** in the first version (finding 7), written from the face's own corner so the two cells of a face find the same points. The two always connect the same pairs of 1D points, under the complement rule too (finding 3).

### Vertices

For each piece of a cell:

- **Planes:** at each of its 1D points, the plane through the point and the two 2D points next to it. Its normal is turned to point from the solid end of the edge to the empty one. A plane whose three points lie on a line is left out.
- **QEF:** the planes go into a `BurstQEF`, or the shader's SVD QEF, and the piece's vertex is its minimizer. With no plane, the vertex is the average of the piece's 1D points.
- **Inside the cell:** a minimizer outside the cell, less the margin, is moved back along the line to the average of the piece's 1D points until it is inside (finding 4). This is ODC's alone: the other algorithms keep the Secret Sauce rule the audit gave them.
- **Normal of the vertex:** the normalized sum of the piece's plane normals.
- **Not in the first version:** keeping the vertices of one cell apart (finding 4). Step 3 counts self-intersections on cells with several vertices and reports them. If there are any, planes to keep them apart are derived from `DualMarchingCubes` then.

### Polygons

For each grid edge inside the grid that the contour crosses, the four cells around it give four vertices, in an order that turns from the solid end to the empty one.

- **The test** (appendix A.1.5, equations 3 and 4): a vertex is concave when it lies beyond the triangle of its two neighbours and either end of the edge.
- **Two triangles** along the diagonal 1-3 when neither 2 nor 4 is concave, or along 2-4 when neither 1 nor 3 is. When both are allowed, the diagonal whose triangles pass nearer the edge's 1D point (finding 4).
- **Four triangles** around the edge's 1D point when neither diagonal is allowed. The 1D point becomes a vertex of the mesh, with the normalized sum of the four vertices' normals for its normal.
- **Sub-mesh and facing:** the highest material of the solid end, facing the empty end, as now.
- **With `splitQuads` off:** always the diagonal the other algorithms use.

### Burst

`Assets/Scripts/Burst/OccupancyDualContouring.Burst.cs`. The jobs, in order:

1. Sample the materials of the corners (parallel).
2. Solid corners per cell (`BurstSolidCornersJob`), then the cells that take the complement (parallel, reads neighbours only).
3. Count and lay out the vertices per row, as Manifold's leaves do.
4. Solve the cells: 1D points, 2D points, planes, QEF, clamp (parallel).
5. Contour: the quads each cell starts, then the mesh in a fixed order, so the same input gives the same mesh vertex for vertex.
6. Debug information when wanted, as `UniformDualContouring.Burst.cs` does it.

Profiler markers are named `OccupancyDualContouring.Burst.<Stage>`.

### GPU

`Assets/Scripts/GPU/OccupancyDualContouring.GPU.cs` and `Assets/Resources/Shaders/OccupancyDualContouring.compute`.

- **Shared shader code:** the shape functions and the density function struct move from `UniformDualContouring.compute` into `DensityFunctions.hlsl`, which both shaders include. The `Start` hashes and the remesh timing show whether that changes Uniform's GPU output or speed. It should change neither.
- **Buffers:** a material per corner, a 1D point per edge, a record per cell (lookup index, first vertex, vertex count), and the vertex, normal, quad, count and argument buffers that `Voxelizer` already reads and draws. Vertices are appended with an atomic counter, as now.
- **Kernels:** sample corners; classify cells; search edges; solve cells, each working out its own 2D points; contour, which writes two or four triangles as one or two quads (finding 9) and makes split normals itself, as Uniform's does.
- **Thread groups** of 4³, where Uniform's shader uses groups of one. Measured in step 4.
- **Readback:** `readMesh` is Uniform's, moved to where both can call it. `convert` reads back materials, 1D points and vertices for the debug view.

### Debug view

- **Corners:** their material. ODC has no density, so a corner reports -1 inside and 1 outside.
- **Edges:** the 1D point as the intersection, and the normalized sum of its planes' normals.
- **Voxels:** one for every vertex, in its cell, as Manifold Dual Contouring shows them.
- **2D points** are not shown (Out of scope).

### Interactive editor

- `Algorithm.Occupancy`, and the component on the editor object, which makes nine variants.
- **Switching** carries over what the algorithms share. Managed becomes Burst on the way to Occupancy. The cells across carry over as for Uniform.
- **The signature** gains ODC's four settings.
- **The scene** (finding 11): the component is added to `Interactive Editor.unity` in step 5, with your go-ahead at that step.

### Example scenes

In `Assets/Scenes/Occupancy-Based Dual Contouring/Burst` and `/GPU`, made from the Uniform ones with the component replaced:

- **Burst:** Sphere, Cube, Union, Intersection, Subtraction, CSG Operations, Multiple Materials.
- **GPU:** Sphere, CSG Operations, Multiple Materials, Direct To GPU.

## Tests

There is no managed implementation to compare against, so the tests hold the code to answers that are known in closed form, and Burst and the GPU to each other. Each new test is shown to fail on a deliberate mistake, as in the earlier plans.

- **Tables** (`DualMarchingCubesTest`, `BurstTablesTest`):
  - the joined faces: 36 configurations, one face each, none in their complements;
  - the cycle of every piece visits each of its edges once and crosses a face of the cell between any two;
  - random grids of solid corners give no edge used other than twice and no pinched vertex with the rule, and some without it.
- **Searches** (new `OccupancySearchTest`, through small Burst jobs):
  - a 1D point is within 2^-n of a cell of where a plane, a sphere and a turned cube cross the edge, on the solid side;
  - a search finds the nearest of two crossings, and the last interval when there is none in range;
  - a 2D point on a face crossed by a cube's edge is where that edge meets the face, and on a face crossed by a flat side it is the middle (lemma 5.1);
  - each of the fallbacks of finding 4.
- **Vertices:**
  - a cube off the grid's planes has a vertex at every corner and the mesh along every edge, within a tolerance measured in step 2;
  - a plane at any angle has every vertex on it;
  - every vertex is inside its cell;
  - each shape of `ShapeContourTest`, upright and turned, has every vertex within a tolerance of the shape.
- **Topology** (new `OccupancyDualContouringTopologyTest`, Burst and GPU): closed, manifold and one piece for every model of `TopologyTest`; a model of small cubes that makes two joined cells meet is manifold; the torus has its handle.
- **Polygons:**
  - quads built by hand take each of the three cases;
  - every triangle of every test model passes the envelope test it was chosen by;
  - no test model has two triangles that cross, counted by a new helper beside `MeshTopology`;
  - with `splitQuads` off, at least one model fails one of those two, which shows the split does something.
- **Materials:** `SubMeshTest`'s models give each material its sub-mesh.
- **Burst against GPU:** vertices within a tolerance on the models of `UniformDualContouringGPUTest`, and equal triangle counts.
- **Editor:** the tests that run over all seven variants run over nine; switching to and from Occupancy; the signature.
- **No change for the others:** the 416 `Start` hashes, the GPU lines included, are those of step 0 after every step.
- **Performance:** `ContouringPerformanceTest` gains Occupancy in Burst. A GPU remesh is timed with the probe of the earlier plans.

## Steps

Each step compiles, passes every test, and is reviewed before the next one starts. Nothing is committed for you.

0. **Baseline and tables (small).** Both suites green, the 416 `Start` hashes and the Burst timings recorded with Unity in front. The two additions to `DualMarchingCubes`, their Burst copies and their tests. **Done 2026-11-12:**
   - **Baseline, before any change:**
     - Edit mode 1392 of 1392, play mode 21 of 21.
     - The 416 `Start` hashes are identical to those at the end of New Shapes, the GPU lines included. They are the baseline for every later step.
     - The Burst timings are under Measurements. Unity was in front for every case.
   - **`DualMarchingCubes`:** the lookups listed under Design. They are built once from the table that was there, which is unchanged.
   - **Faces are numbered as the octrees number them,** the order of `BurstTables.lookupFaceEdge`. The generator of the table keeps its own order of the faces beside it. Going through the faces in the octrees' order would number the vertices of 13 configurations differently, which would change meshes.
   - **`BurstTables`:** `lookupJoinedFace`, `lookupEdgeAcrossFace`, `lookupEdgeFace`, `lookupFaceAxis`, `lookupFaceSide` and `complement`, as constant data. The two tables over the 256 configurations were written out by a probe from `DualMarchingCubes`.
   - **New tests:**
     - **`DualMarchingCubesTest`,** 7 tests before and 36 now:
       - every edge lies in the two faces that hold both of its corners;
       - the edge across a face is another edge of the same vertex in that face, and leads back;
       - the edges of every vertex form one ring;
       - the complement cuts off the empty corners of an ambiguous face where the cell cuts off the solid ones;
       - the joined faces, worked out in the test from the vertices: 36 configurations, 12 with six solid corners and 24 with five, never two in a cell;
       - the complement of such a cell has no joined face and differs from the cell on that face only;
       - the two tests over every pair of cells of finding 3;
       - 20 grids of 6³ cells with random corners are manifold with the rule: every edge is part of two polygons and the polygons around every vertex form one fan. Without the rule the same grids are not.
     - **`BurstTablesTest`:** `tablesAreTheManagedOnes` reads the new tables in a Burst job and holds them to `DualMarchingCubes`, 10,308 entries in all. The face test also holds the axis and side of each face and the faces of each edge to the corners.
   - **The tests catch mistakes.**
     - In `DualMarchingCubes`, cutting off the empty corner across an ambiguous face and counting every ambiguous face as joined were put in together. They failed 8 of the 10 new tests, 27 of the 36 cases of the fixture.
     - In `BurstTables`, one entry of each new table was changed. `tablesAreTheManagedOnes` failed.
   - **Verification:**
     - Edit mode 1421 of 1421, play mode 21 of 21.
     - The 416 `Start` hashes are identical to the baseline.
     - Compiling gave no errors.
   - **Not timed again:** no job changed.
   **0a. The rule in Manifold Dual Contouring (small).** Added to the plan on 2026-11-12, when the user decided decision 4 after step 0. **Done 2026-11-12:**
   - **Shared:** `DualMarchingCubes.sharesJoinedFace( solidCorners, solidCornersAcross )` says whether a cell and the one across its joined face have that face joined in both, with a copy in `BurstTables`. `DualMarchingCubes.solidCornersOf` gives the solid corners of a cell from its materials.
   - **Burst:** `BurstCells.vertexLookupOf` (Design). The Burst implementation looks the vertices of a leaf up with it in all four places it used the solid corners for that: counting them, solving them, clustering and contouring.
   - **Managed:** `vertexLookupOf` in `ManifoldDualContouring.Managed.cs`. The managed octree keeps no way from a leaf to the one next to it, so a leaf with a joined face samples the four corners beyond that face. Leaves without one sample nothing more.
   - **At the outside of the grid** a joined face has no leaf across it, and no polygon crosses it, so the leaf keeps its solid corners. Both implementations do this.
   - **Uniform and Adaptive Dual Contouring** are not changed. They have one vertex in a cell and only take the number of pieces from the table.
   - **The test model,** `TopologyTest.hollowsAcrossFaces`: a block with six small hollows, each around one corner of a grid of 16³ cells. They come in three pairs, around corners diagonally across a face of a cell, once for each direction a face can face. The two cells on either side of such a face have it joined.
   - **New tests:**
     - `surfaceIsManifoldWhereTwoLeavesHaveAJoinedFaceInCommon`, for Managed and Burst: the block and its six hollows are seven separate manifold surfaces.
     - `clusteredSurfaceIsManifoldWhereTwoLeavesHaveAJoinedFaceInCommon`, for both and both solvers.
     - `contourIsTheManagedOneWhereTwoLeavesHaveAJoinedFaceInCommon`: Burst against Managed, both solvers and every amount of clustering (8).
     - `twoCellsShareAJoinedFaceWhenTheirJoinedFacesFaceEachOther` in `DualMarchingCubesTest`, over all 65,536 pairs of configurations: 216 share one. `BurstTablesTest` holds the Burst copy to it.
   - **What it was like without the rule.** With `sharesJoinedFace` made to say no, both implementations contour the model with three edges that more than two triangles share and six conflicts of facing, and the hollows of each pair hang together: four pieces of surface instead of seven. The three topology tests fail in each implementation.
   - **The tests catch mistakes.** With the Burst code looking at the cell on the wrong side of the joined face, the three Burst topology tests and all eight parity tests fail, and so does one of the parity tests of the three balls on other grid sizes.
   - **No change for the example models:** the 416 `Start` hashes are identical to step 0.
   - **No cost that shows:** Burst Manifold at 128³ takes 34.8 and 34.7 ms, against 34.4 and 35.2 in step 0 (Measurements).

1. **Component and a first Burst contour (medium).** The component, its settings and inspector, defaults and dispatch. Burst: materials, pieces with the manifold rule, 1D points, a vertex at the average of each piece's 1D points, the plain split, debug information. This is a closed, manifold mesh without sharp features. The table, 1D point, topology and material tests. **Done 2026-11-12:**
   - **The component:** `OccupancyDualContouring` in `Assets/Scripts/OccupancyDualContouring.cs`, with `edgeSearchIterations` (15, from 1 to 20). It starts as Burst. Managed throws `NotSupportedException`, GPU `NotImplementedException` until step 4.
   - **The inspector** leaves out the three settings of `Voxelizer` that place points by distance, and turns Managed and GPU into Burst.
   - **`BurstOccupancy`** (`Assets/Scripts/Burst/BurstOccupancy.cs`): `isSolid`, `positionOnEdge` and `searchEdge`, the 1D point search. The later searches go here too.
   - **`OccupancyDualContouring.Burst.cs`,** its jobs in order:
     - the materials of the corners;
     - solid corners (`BurstSolidCornersJob`), the vertices of each row counted and laid out;
     - the vertices: for each piece of each cell, the 1D points of its edges and their average;
     - the polygons each cell starts, then the mesh in the order of the cells;
     - debug information when wanted.
   - **The vertex's normal** in this step is the sum of the directions of its edges, each from its solid end to its empty one. It is what a vertex without any plane will keep in step 2.
   - **Sample positions are exact.** A point that up to 15 halvings come to is a position a float holds exactly on grids of up to 256³ cells, when it is worked out as the corners are: its number along the axis, divided by the cells across, less a half. So the search samples exactly the points it means to, and the corners of an edge exactly where the grid has them.
   - **Moved to where both can use them:** Uniform's three helpers for the cells around an edge are `BurstQuads` in `BurstGrid.cs`, and Manifold's job that lays out rows is `BurstLayoutRowsJob` there. Neither changed.
   - **The debug view** gets a voxel for every vertex, with the corners and edges of the cells that have one. A corner shows -1 inside and 1 outside. An edge the contour crosses shows its 1D point and the direction from its solid end to its empty one.
   - **New tests:**
     - **`OccupancySearchTest` (15):**
       - the 1D point of every crossed edge of a 16³ grid is within 2^-n of a cell of where the surface is, on its solid side, for a box, a turned box and a sphere and n of 1, 4, 8 and 15. The crossing is worked out from the shape in double precision.
       - points on an edge are exactly where the grid has them, on grids of 4³, 16³ and 256³.
     - **`BurstOccupancyDualContouringTopologyTest` (22),** an abstract fixture that the GPU joins in step 4:
       - every model is one closed manifold surface without handles;
       - the three balls are manifold on grids from 4³ to 128³;
       - two spheres in one cell stay two, the block with six hollows is seven surfaces, the turned shapes and the new shapes are manifold, and a torus keeps its handle;
       - every vertex lies in its cell;
       - the intersections the debug view shows are on their edges and within the last halving of the sphere, from inside, with 3 and with 15 halvings;
       - a voxel for every vertex, corners that show their side, and debug information only when wanted.
     - `thereIsNoManagedImplementation`, `VoxelizerDefaultsTest.occupancyDualContouring`, and `SubMeshTest` with Occupancy in Burst.
     - `ContouringPerformanceTest` has Occupancy, in Burst.
   - **The tests catch mistakes.**
     - Returning the empty end of a search, contouring without the complement and a corner's side the wrong way round were put in together. They failed all 12 search tests and 4 of the topology tests: the intersections, the block with six hollows and the corners.
     - Taking the wrong edge of two of the four cells around an edge failed all 22 topology tests.
   - **Not in this step:** the face searches and `splitQuads`, which steps 2 and 3 add with their settings.
   - **Speed:** a contour of the three balls at 128³ takes 31 ms, about what Uniform Dual Contouring takes (Measurements). The searches of step 2 are what will cost.
   - **Verification of steps 0a and 1:**
     - Edit mode 1477 of 1477, play mode 21 of 21.
     - The 416 `Start` hashes are identical to step 0.
     - Compiling gave no errors.
2. **2D points and the QEF (large).** The searches, 2D points, planes, the QEF and the clamp. The search and vertex tests. Measurements: how far vertices are from each shape against Manifold Dual Contouring on the same grid, the float thresholds, and timings against the estimate of finding 7.
3. **Intersection-free polygons (medium).** The concave test, the choice of diagonal, four triangles around a 1D point, `splitQuads`. The crossing-triangles helper and the polygon tests. The count of crossings on cells with several vertices (Design, Vertices).
4. **GPU (large).** `DensityFunctions.hlsl` first, with Uniform's GPU hashes and timing checked. Then the shader, its buffers, readback, drawing straight from the GPU and split normals. The Burst-against-GPU tests.
5. **Interactive editor and scenes (medium).** The fourth algorithm in the editor and its tests, the component in `Interactive Editor.unity`, the example scenes.
6. **Tuning and documentation (medium).** Sharing 1D and 2D points between cells in Burst if step 2's timings call for it, with the output held bit for bit. README (algorithms, implementations table, code layout, the editor), `AGENTS.md`, and this plan marked complete.

## Measurements

### Before any change (step 0, 2026-11-12)

`ContouringPerformanceTest` on the three balls, Burst, with Unity in front. Each figure is the median of three contours, in ms, from two rounds in one session.

| Algorithm | Cells | Model | Voxelize | SampleCorners | SolveVertices |
|---|---|---|---|---|---|
| Uniform | 128³ | upright | 31.3, 29.2 | 8.2, 8.1 | 14.3, 13.3 |
| Uniform | 256³ | upright | 183.4, 182.4 | 62.9, 63.1 | 64.4, 65.1 |
| Adaptive | 128³ | upright | 33.6, 32.7 | 8.1, 7.9 | 15.5, 15.2 |
| Manifold | 128³ | upright | 34.4, 35.2 | 8.1, 8.2 | 14.4, 14.6 |
| Uniform | 128³ | turned | 31.3, 31.7 | 8.9, 8.9 | 15.0, 14.9 |
| Adaptive | 128³ | turned | 35.7, 35.1 | 9.2, 9.0 | 16.5, 16.1 |
| Manifold | 128³ | turned | 37.8, 37.0 | 9.3, 9.0 | 16.9, 16.3 |

These are 3 to 9% above the last session of New Shapes, which is within what sessions have differed by before (up to 13%). The estimate of finding 7 was made from the older figures.

### After steps 0a and 1 (2026-11-12)

The same test, in a later session, with Unity in front for all 20 cases. Voxelize in ms, the two rounds:

| Algorithm | Cells | Model | Step 0 | Now |
|---|---|---|---|---|
| Uniform | 128³ | upright | 31.3, 29.2 | 28.6, 28.4 |
| Uniform | 256³ | upright | 183.4, 182.4 | 179.5, 178.8 |
| Adaptive | 128³ | upright | 33.6, 32.7 | 32.0, 33.7 |
| Manifold | 128³ | upright | 34.4, 35.2 | 34.8, 34.7 |
| Uniform | 128³ | turned | 31.3, 31.7 | 31.6, 30.4 |
| Adaptive | 128³ | turned | 35.7, 35.1 | 36.5, 34.5 |
| Manifold | 128³ | turned | 37.8, 37.0 | 36.5, 36.6 |

Moving Uniform's helpers and giving Manifold the rule changed neither by more than the rounds differ.

Occupancy-Based Dual Contouring as step 1 leaves it, with 15 halvings for every 1D point and no face searches yet:

| Cells | Model | Vertices | Voxelize | SampleCorners | SolveVertices | Contour |
|---|---|---|---|---|---|---|
| 128³ | upright | 52,252 | 31.1, 31.4 | 6.7, 6.8 | 18.7, 18.8 | 4.4, 4.4 |
| 256³ | upright | 208,940 | 175.6, 174.2 | 54.1, 53.1 | 85.7, 86.0 | 27.0, 28.1 |
| 128³ | turned | 52,252 | 34.2, 34.1 | 8.2, 8.0 | 20.4, 20.3 | 4.2, 4.4 |

- It has 28 vertices more than Uniform Dual Contouring's 52,224 at 128³: the cells along the creases that hold two pieces of surface.
- Sampling the corners takes a sixth less than in the other algorithms, which also work out a density there.

## Out of scope

- **A managed implementation.**
- **Simplification or an octree.** The paper has none.
- **Hermite data on the boundary between two materials.** See decision 2.
- **The gradient as an option for the normals,** the paper's second ablation. It needs distances.
- **2D points in the debug view.**

## Decisions

All five were decided as recommended on 2026-11-12.

1. **A component of its own?** Recommended: yes, `OccupancyDualContouring`, on a uniform grid without simplification, as the paper has it. The other way is an option on `ManifoldDualContouring`, which would put it on an octree with a managed implementation beside it.
2. **Boundaries between materials.** The other algorithms pull a vertex to where two materials meet on the surface, with a gradient. Recommended: ODC gives each material its sub-mesh and leaves that pull out, so a boundary between two materials follows the grid, as it did before the audit's last item. Adding it later, with the existing gradient code or by searching on the material, would be a step of its own.
3. **The QEF.** Recommended: the project's solvers, so the solver setting means what it does elsewhere, plus the clamp into the cell. The other way is the reference code's regularized least squares, which would be a third solver that only ODC uses.
4. **Manifold Dual Contouring's non-manifold case** (finding 3). Recommended: leave it out of this plan, and decide once step 0's test shows it on the project's own table. The rule would then be a small change to its leaves. **Decided after step 0, 2026-11-12:** the rule is added, as step 0a.
5. **Editor and scenes in this plan?** Recommended: yes, as step 5.
