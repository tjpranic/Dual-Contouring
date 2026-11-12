# Occupancy-Based Dual Contouring

Plan written 2026-11-12 for adding the algorithm of *Occupancy-Based Dual Contouring* (Hwang and Sung, 2024; `Documents/Papers/Occupancy-Based Dual Contouring.md`) as a fourth algorithm, with a Burst and a GPU implementation and no managed one.

**Status:** in progress. The user accepted the five decisions at the end as recommended on 2026-11-12 ("All decisions as you recommend") and asked for step 0. Step 0 was done the same day. After it the user decided to give Manifold Dual Contouring the complement rule too ("We'll add the rule to the Manifold Dual Contouring implementation") and asked for step 1. Both were done on 2026-11-12, and the user approved them ("Looks good. Continue."). Step 2 was done the same day; the user had step 3 go ahead and left the question about the clamp (finding 14) for after it ("Continue with step 3 for now, we'll decide the clamp after that"). Step 3 was done on 2026-11-12 and waits for review, with what it measured for that question (finding 15).

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
2. **How accurate ODC is against Manifold Dual Contouring on this project's exact distances.** Before step 2 I expected it to be about as accurate: here the gradient is the true normal, where the paper's gains are over the poor gradients of occupancy networks. **Step 2 measured it, and that expectation was wrong both ways** (findings 12 to 14): ODC is far closer on curved surfaces, as close on flat faces and most edges, and worse at corners and wherever a vertex is moved into its cell.
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

12. **On curved surfaces ODC's vertices are ten to twenty times closer than Manifold Dual Contouring's.** Found in step 2, on single shapes at 16³ (Measurements).
    - A tangent plane touches a curved surface at one point and leaves it everywhere else, so the vertex where tangent planes meet is off the surface by the sag across the cell: 0.039 of a cell on average on a sphere 4.8 cells in radius.
    - ODC's plane at a 1D point goes through two more points that are on or near the surface, its 2D points, so it cuts through the surface instead: 0.0016 of a cell on average on the same sphere.
    - More steps for Manifold's intersections do not help it there: with 12 instead of 6 its average is the same.
13. **Where a face of the grid sees three faces of a shape, ODC has a wrong plane, and it misses corners.** Found in step 2.
    - **The cause** is the paper's own assumption. Lemma 5.1 is about a line across a face that is straight or bends once. Near a corner where three faces meet, a face of the grid can cut all three, and the line across it bends twice. The search then draws one of its two lines through a point on the middle piece, and the 2D point is on neither face.
    - **What it costs:**
      - On the turned cuboid 3 of 500 planes are off by 39 to 98 degrees, all at 1D points within a quarter of a cell of an edge of the cuboid. The other planes are exact.
      - The nearest vertex to a corner is up to 0.44 of a cell away, on the turned cuboid and the pyramid's tip. Manifold Dual Contouring has a vertex within 0.03 of a cell of every corner its tests look at.
    - **Where it does not happen:** on the upright cuboid and hexagonal prism every face of the grid sees a single bend, and every plane, corner and edge is exact to a ten-thousandth of a cell.
    - **Not a mistake in the implementation, as far as I can tell:** the 2D points of the cells around the turned cuboid's corners were printed against the cuboid's planes. Those of single bends are on both faces, and those of double bends are on one.
14. **Keeping vertices in their cells moves one in six to ten of them on turned shapes with edges, by up to 0.4 of a cell.** Found in step 2.
    - **Why they are outside:** an edge of a shape can pass just outside a cell that still sees both of its faces. The QEF rightly puts that cell's vertex on the edge, outside the cell. Manifold Dual Contouring leaves it there: 52 of the turned cuboid's 502 vertices are outside their cells.
    - **What moving them costs:** on the turned cuboid the 50 moved vertices are up to 0.24 of a cell off it, and the mesh is 0.22 of a cell from its edges at one point in ten. Without the clamp those figures are 0.035 and 0.05 (Measurements).
    - **No way of moving them is much better.** Moving toward the average of the 1D points, as planned, and taking the nearest point of the cell instead give the same sizes of error: the edge is simply not in the cell.
    - **The paper does not mention it.** It is what the reference code does (finding 4), and what the argument for polygons without intersections needs. Step 3 will show how many intersections there are with and without it.
    - **Open, for the user:** whether keeping vertices in their cells should be a setting. See the question at the end.
15. **The clamp is what makes the polygons free of intersections, and cells with several vertices need planes between them.** Found in step 3, by counting the pairs of triangles that pass through each other on 27 models at 16³, 32³ and 64³ (Measurements).
    - **With the clamp and the split:** none at 16³ and 32³, and 4 at 64³, all on one model around one cell with two vertices.
    - **With the clamp and without the split,** always the same diagonal as Manifold Dual Contouring takes: 99, 151 and 436, on 11 of the 27 models. The split is what removes them.
    - **Without the clamp** the split does not help: 436, 645 and 1,849 with it, against 401, 745 and 2,238 without. Its test of a diagonal assumes the four vertices are in their own cells.
    - **Cells with several vertices,** which the paper says its argument does not cover. With every vertex only kept in its cell, the two models of shapes that cut into each other had 32 and 7 crossings at 32³, every one between triangles with vertices in the same cell. With each vertex also kept on its own side of a plane between the pieces of its cell, they have none. The reference code keeps them apart with planes from a table (finding 4); here the planes are derived from `DualMarchingCubes`.
    - **So the choice of finding 14 is a real one:** the clamp costs edges on turned shapes, and it is all that stands between this algorithm and the crossings Manifold Dual Contouring has.

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

- **A search** (appendix A.1.3) samples evenly out to its range, takes the first interval whose far end differs from the start, or the last interval when none does, and bisects it. It returns the end nearer the start, which is on the start's side of the surface. It stops sampling at the first sample that differs, where the paper's code, which samples all rays at once, takes them all: the result is the same with fewer samples.
- **Which way the first ray goes** is put to a vote of the four corners of the face: toward a corner that is on the other side of the surface from the middle, away from one that is not. On a face with two lines the corner beyond the other line is outvoted.
- **What counts as nothing,** in cells: lengths below 1e-5, for two 1D points in one place, a first search that ends where it started, and two lines that do not come together. The searches step by 6e-5 to 1e-4, so a search that found the surface at its start returns exactly zero.
- **Each cell works out the 2D points of its own pieces** in the first version (finding 7), written from the face's own corner so the two cells of a face find the same points. The two always connect the same pairs of 1D points, under the complement rule too (finding 3).

### Vertices

For each piece of a cell:

- **Planes:** at each of its 1D points, the plane through the point and the two 2D points next to it. Its normal is turned to point from the solid end of the edge to the empty one. A plane whose three points lie on a line is left out: where the sine of the angle between the two directions is below 1e-4.
- **QEF:** the planes go into a `BurstQEF`, or the shader's SVD QEF, and the piece's vertex is its minimizer. With no plane, the vertex is the average of the piece's 1D points.
- **Inside the cell:** a minimizer outside the cell, less the margin, is moved back along the line to the average of the piece's 1D points until it is inside (finding 4). This is ODC's alone: the other algorithms keep the Secret Sauce rule the audit gave them.
- **Normal of the vertex:** the normalized sum of the piece's plane normals.
- **Apart from the other vertices of its cell** (built in step 3, finding 15): in a cell with several vertices, each is also kept on its own side of a plane between its piece of the contour and each of the others.
  - `DualMarchingCubes.lookupSeparation` gives the plane for every pair of pieces of every configuration: the one with the widest gap between the corners at the ends of their edges, midway in it. Most pairs touch, at a corner or along a diagonal, and then it is the plane through where they touch that faces most squarely from one piece to the other.
  - All 118 pairs have such a plane, with a normal of -1, 0 or 1 along each axis.
  - The average of a piece's 1D points is on its side of every such plane, so the way back from the minimizer to it always comes inside.

### Polygons

For each grid edge inside the grid that the contour crosses, the four cells around it give four vertices, in an order that turns from the solid end to the empty one.

- **The test** (appendix A.1.5, equations 3 and 4): a vertex is concave when it lies beyond the triangle of its two neighbours and either end of the edge.
- **Two triangles** along the diagonal 1-3 when neither 2 nor 4 is concave, or along 2-4 when neither 1 nor 3 is. When both are allowed, the diagonal whose triangles pass nearer the edge's 1D point (finding 4), and the one the other algorithms take when both pass as near, as on a flat polygon.
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
- **Edges:** the 1D point as the intersection, and the normal of its plane in the cell that showed the edge first. As built in step 2: each of the up to four cells around an edge has a plane of its own there (finding 8), and the debug view has one edge for them all. Nothing draws the normal of an edge; the tests read it.
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
2. **2D points and the QEF (large).** The searches, 2D points, planes, the QEF and the clamp. The search and vertex tests. Measurements: how far vertices are from each shape against Manifold Dual Contouring on the same grid, the float thresholds, and timings against the estimate of finding 7. **Done 2026-11-12,** apart from the question of finding 14:
   - **`BurstOccupancy`:**
     - `searchLine`, the line-binary search, and `positionOnFace`;
     - `searchFace`, the 2D point of a line across a face;
     - `planesOf`, the 1D points of a piece in the order they go around it, with the normal of the plane at each;
     - `intoCell`, which moves a vertex back into its cell.
   - **The vertex** (`BurstOccupancyCells.solve` in `OccupancyDualContouring.Burst.cs`): the planes go into a `BurstQEF` of the solver the component has, and its minimizer is moved into the cell. With no plane at all it is the average of the 1D points, as in step 1.
   - **Settings:** `faceSearchSamples` (4, from 1 to 8) and `faceSearchIterations` (11, from 1 to 20).
   - **Searching in cells from the corner of the face** keeps the precision: a point of a face is its corner plus an offset, and only the sample is taken to object space.
   - **Each cell works out its own 2D points,** ordered by the number of the grid's edges so that both cells of a face search alike.
   - **The debug view** now shows the planes: each cell's planes are worked out again for it, with the same function.
   - **New tests:**
     - **`OccupancySearchTest`,** 21 now:
       - a search along a line finds the nearest of two crossings, from inside and from outside and at an angle, and ends at its range when there is none;
       - on a box and a turned box, the 2D point of every straight crossing is exactly the middle of its 1D points, and that of a single bend is within 0.00031 of a cell of the bend (over 100 and over 20 faces of each);
       - on a sphere every 2D point lies between the circle the sphere draws in the face and the point where the tangents at the two 1D points meet: between 0.002 and 0.27 of the way;
       - the three ways a 2D point falls back to the middle.
     - **`BurstOccupancyShapeContourTest` (45),** an abstract fixture that the GPU joins in step 4. Its tolerances are what was measured, and its header says where the algorithm is exact and where it is not:
       - every vertex of a plane at three angles is within 0.002 of a cell of it. The farthest is 0.0006: a vertex nearer a face of its cell than the margin is moved in by that much;
       - every vertex of the nine shapes, upright and turned, in two groups: those that were not moved into their cells, and those that were (finding 14);
       - a vertex at every corner of the upright cuboid and hexagonal prism, and what is missed elsewhere (finding 13);
       - the mesh along the edges and rims;
       - the planes of a cube are its faces: all of them upright, and all but 3 of 500 turned.
     - **In the topology fixture,** 24 now: normals point out of the shape, within five degrees on a sphere, and coarser face searches place vertices worse.
   - **The tests catch mistakes.** The first ray leaving toward the wrong side, normals not turned outward, no clamp and a search returning its far end were put in together. They failed 5 of the search tests, 41 of the 45 shape tests and 4 topology tests, among them every vertex in its cell and the normals.
   - **Measured, not assumed:** findings 12 to 14, and the tables under Measurements.
   - **Speed:** a contour of the three balls at 128³ takes 73 ms, against the 85 ms finding 7 estimated without sharing points between cells. Of that, 61 ms are the vertices. Manifold Dual Contouring takes 34 ms. Step 6 decides about sharing.
   - **Verification:**
     - Edit mode 1530 of 1530, play mode 21 of 21.
     - The 416 `Start` hashes are identical to step 0.
     - Compiling gave no errors.
3. **Intersection-free polygons (medium).** The concave test, the choice of diagonal, four triangles around a 1D point, `splitQuads`. The crossing-triangles helper and the polygon tests. The count of crossings on cells with several vertices (Design, Vertices). **Done 2026-11-12:**
   - **`BurstOccupancy`:** `isConcave` (equations 3 and 4), `splitOf`, which chooses between the two diagonals and the fan, and `distanceToTriangle` for the choice when both diagonals fit.
   - **`SplitQuadsJob`** takes the place of step 1's job that found the polygons: in parallel, it also decides how each is split, in two bits for each of a cell's three polygons. It works out the 1D point of the edge for that.
   - **`ContourJob`** makes the triangles, and for a fan adds the 1D point as a vertex, with the sum of the normals of the four around it.
   - **Setting:** `splitQuads`, on unless set. Off gives two triangles along the diagonal the other algorithms take, and no vertices on edges.
   - **Planes between the vertices of a cell** (Design, Vertices): `DualMarchingCubes.lookupSeparation`, its copy in `BurstTables` written out by a probe, and `BurstOccupancy.intoCell` keeping to them.
   - **The triangles of a polygon** go around the way the polygon does, which makes them face out of the shape. With the split off the mesh has the triangles of step 2.
   - **New tests:**
     - **`MeshIntersections`,** the helper beside `MeshTopology`: the pairs of triangles of which a side of one passes through the inside of the other. Triangles that share a side are not looked at, of two that share a corner only the sides opposite it, and triangles that only touch or lie in one plane do not count.
     - **`OccupancyPolygonTest` (5),** on polygons made by hand around an edge:
       - a flat polygon is split along the second diagonal;
       - a vertex pushed in past the diagonal of its neighbours is concave, and the polygon is split along the diagonal that ends in it;
       - a vertex is also concave where that diagonal passes beyond the end of the edge;
       - with neither diagonal it becomes a fan;
       - of two diagonals that fit, the one nearer the 1D point.
     - **In the topology fixture,** 37 now:
       - no two triangles cross on any model at 16³ and 32³, nor on the shapes that cut into each other, the two spheres and the block with hollows;
       - the vertices of every cell with several are on their own sides of the planes between them;
       - on the sphere cutaway the polygons no diagonal fits are four triangles around a 1D point, and without the split triangles cross there;
       - triangles face out of the shape, and those of the cutaway enclose the volume it has.
     - **In the shape fixture,** 66 now: no two triangles cross on any of the nine shapes, upright or turned, and without the split they do on the turned cuboid, octahedron and cone. A vertex moved onto a plane between pieces counts as moved, like one moved onto a face of its cell.
     - **`DualMarchingCubesTest`:** the plane between two vertices has the edges of each on its side, and is the same plane turned round from the other. `BurstTablesTest` holds the Burst copy to it.
   - **Not built as planned:** the test that every triangle passes the envelope test it was chosen by. The mesh does not say which polygon a triangle is of, so the choice is tested on polygons made by hand, and its effect by counting crossings.
   - **The tests catch mistakes.**
     - Equation 3 the wrong way round, no planes between the vertices of a cell and a fan going around the wrong way were put in together. They failed all 5 polygon tests, 19 of the topology tests and 1 shape test.
     - Polygons going around their edges the other way and the farther of two diagonals failed 2 polygon tests and 4 topology tests, among them the triangles facing out.
   - **Measured:** finding 15, and the table of crossings under Measurements. The clamp was switched off for it by making `intoCell` return the vertex as it is, which the component cannot do.
   - **Speed:** a contour of the three balls at 128³ takes 80 ms, 6 ms more than after step 2. That is the split working out the 1D point of every polygon's edge once more, which sharing points between cells in step 6 would take away.
   - **Verification:**
     - Edit mode 1570 of 1570, play mode 21 of 21.
     - The 416 `Start` hashes are identical to step 0.
     - Compiling gave no errors.
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

### Burst timings after step 2 (2026-11-12)

`ContouringPerformanceTest` on the three balls, with Unity in front for all 20 cases. In ms, the two rounds:

| Cells | Model | Voxelize | SampleCorners | SolveVertices | Contour |
|---|---|---|---|---|---|
| 128³ | upright | 73.3, 73.4 | 6.6, 6.8 | 61.4, 61.2 | 4.2, 4.3 |
| 256³ | upright | 339.4, 338.9 | 52.3, 52.5 | 253.3, 252.9 | 25.9, 26.4 |
| 128³ | turned | 80.7, 81.1 | 8.0, 7.9 | 67.2, 67.7 | 4.2, 4.2 |

- The face searches and the QEF took the vertices from 18.7 ms in step 1 to 61 ms at 128³.
- Manifold Dual Contouring in the same session: 33.5 and 34.0 ms upright, 35.6 and 35.8 turned.
- At 128³ a contour is past the 50 ms up to which the interactive editor follows a drag live.

### How far vertices are from single shapes (step 2, 2026-11-12)

Each shape alone at 16³, SVD solver, as `ShapeContourTest` sizes and turns it, the ellipsoid here a ball 0.3 in radius. Every shape is an exact distance, so its density at a vertex is how far the vertex is from it. In cells, as average and farthest:

| Shape | | Manifold, 6 steps | Manifold, 12 steps | ODC | ODC without the clamp |
|---|---|---|---|---|---|
| Cuboid | upright | 0.0069, 0.014 | 0.0002, 0.0003 | 0.0000, 0.0001 | the same |
| Cuboid | turned | 0.0029, 0.016 | 0.0001, 0.0003 | 0.0050, 0.24 | 0.0002, 0.035 |
| Ball | either | 0.039, 0.076 | 0.039, 0.070 | 0.0016, 0.019 | the same |
| Cylinder | upright | 0.022, 0.058 | 0.018, 0.051 | 0.0008, 0.0040 | the same |
| Cylinder | turned | 0.016, 0.050 | 0.015, 0.050 | 0.0050, 0.20 | 0.0017, 0.064 |
| Cone | upright | 0.035, 0.19 | 0.032, 0.19 | 0.0056, 0.093 | 0.0031, 0.060 |
| Cone | turned | 0.027, 0.25 | 0.026, 0.26 | 0.014, 0.40 | 0.0054, 0.15 |
| Capsule | upright | 0.060, 0.16 | 0.060, 0.16 | 0.0041, 0.038 | 0.0039, 0.021 |
| Capsule | turned | 0.058, 0.14 | 0.058, 0.14 | 0.0034, 0.022 | 0.0034, 0.025 |
| Torus | upright | 0.067, 0.16 | 0.067, 0.16 | 0.0050, 0.043 | 0.0048, 0.034 |
| Torus | turned | 0.064, 0.15 | 0.064, 0.16 | 0.0054, 0.034 | 0.0054, 0.036 |
| Pyramid | upright | 0.0066, 0.013 | 0.0001, 0.0002 | 0.0083, 0.22 | 0.0000, 0.0004 |
| Pyramid | turned | 0.0032, 0.015 | 0.0001, 0.0003 | 0.011, 0.39 | 0.0008, 0.095 |
| Octahedron | upright | 0.0033, 0.010 | 0.0001, 0.0002 | 0.0065, 0.23 | 0.0006, 0.034 |
| Octahedron | turned | 0.0054, 0.10 | 0.0008, 0.093 | 0.0089, 0.28 | 0.0017, 0.071 |
| Hexagonal prism | upright | 0.0062, 0.016 | 0.0001, 0.0002 | 0.0001, 0.0089 | 0.0000, 0.0001 |
| Hexagonal prism | turned | 0.0031, 0.015 | 0.0000, 0.0003 | 0.0043, 0.25 | 0.0002, 0.017 |

- **Manifold with 6 steps** is the component's default. On flat faces its vertices are as far off as its intersections are, a sixty-fourth of a cell at most, and 12 steps take that away. On curved surfaces the steps make no difference: the sag of the tangent planes is all of it.
- **ODC without the clamp** was measured with `intoCell` made to return the vertex as it is. It is not something the component can do.

How well corners and edges are kept, from the measuring runs of `BurstOccupancyShapeContourTest`. The nearest vertex to the worst corner, and how far the mesh is from the points along the edges and rims, at worst and at nine points in ten, in cells:

| Shape | | Corner | Corner, no clamp | Edges | Edges, no clamp |
|---|---|---|---|---|---|
| Cuboid | upright | 0.0002 | 0.0002 | 0.0001, 0.0001 | the same |
| Cuboid | turned | 0.44 | 0.44 | 0.47, 0.22 | 0.42, 0.049 |
| Cylinder | upright | | | 0.29, 0.032 | 0.036, 0.032 |
| Cylinder | turned | | | 0.36, 0.20 | 0.081, 0.048 |
| Cone | upright | 0.23 | 0.23 | 0.33, 0.21 | 0.041, 0.037 |
| Cone | turned | 0.39 | 0.39 | 0.52, 0.34 | 0.25, 0.089 |
| Pyramid | upright | 0.41 | 0.41 | 0.39, 0.29 | 0.36, 0.0001 |
| Pyramid | turned | 0.44 | 0.44 | 0.56, 0.30 | 0.43, 0.0081 |
| Octahedron | upright | 0.29 | 0.083 | 0.39, 0.21 | 0.083, 0.041 |
| Octahedron | turned | 0.40 | 0.40 | 0.40, 0.15 | 0.39, 0.064 |
| Hexagonal prism | upright | 0.0001 | 0.0001 | 0.014, 0.0001 | 0.0001, 0.0001 |
| Hexagonal prism | turned | 0.37 | 0.17 | 0.34, 0.12 | 0.11, 0.0013 |

- One corner of the turned cuboid is in a cell none of whose corners is inside the cuboid, so no method on this grid gives it a vertex.
- Manifold Dual Contouring's own tests hold a vertex within 0.03 of a cell of the corners of the cone, the pyramid, the octahedron and the hexagonal prism, upright and turned, and the mesh within 0.03 of their edges.

### Burst timings after step 3 (2026-11-12)

The same test, with Unity in front for all 20 cases. In ms, the two rounds:

| Cells | Model | Voxelize | SolveVertices | Contour | Contour after step 2 |
|---|---|---|---|---|---|
| 128³ | upright | 79.6, 79.5 | 61.3, 61.4 | 10.4, 10.3 | 4.2, 4.3 |
| 256³ | upright | 367.3, 367.6 | 252.2, 252.8 | 54.1, 53.7 | 25.9, 26.4 |
| 128³ | turned | 87.8, 87.4 | 67.9, 67.5 | 10.7, 10.7 | 4.2, 4.2 |

The three balls have no polygon that becomes a fan: 52,252 vertices and 104,500 triangles, as before.

### Triangles that pass through each other (step 3, 2026-11-12)

Pairs of triangles, counted by `MeshIntersections`, added up over 27 models: the four of `TopologyTest`, the two spheres, the block with hollows, the turned shapes, the new shapes, a torus, and each of the nine shapes upright and turned. The models have 20,000 triangles between them at 16³, 84,000 at 32³ and 339,000 at 64³.

| Vertices kept in their cells | Polygons split | 16³ | 32³ | 64³ |
|---|---|---|---|---|
| yes | as the paper splits them | 0 | 0 | 4 |
| yes | always along the same diagonal | 99 | 151 | 436 |
| no | as the paper splits them | 436 | 645 | 1,849 |
| no | always along the same diagonal | 401 | 745 | 2,238 |

- **The 4** are on the new shapes at 64³, all between triangles around one cell that has two vertices.
- **Which models:** with the same diagonal always, 11 of the 27 have crossings at each size. They are the ones with edges at an angle to the grid, and the upright pyramid and octahedron. Curved shapes and the upright cuboid have none either way.
- **Four triangles around a 1D point** are rare: 22, 14 and 39 polygons over all the models.
- **Without the clamp** the paper's split makes more fans (92, 134 and 322) and no fewer crossings.

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

### Open after step 2

6. **Should keeping vertices in their cells be a setting?** Decision 3 has the clamp always on, as the reference code has it. Step 2 measured what it costs on turned shapes with edges (finding 14), and step 3 what it buys (finding 15): with it there are no crossings to speak of, and without it as many as Manifold Dual Contouring has, whatever the split. Recommended: a setting, on unless turned off. Curved models lose nothing to the clamp, and models of turned boxes lose their edges to it, so which is better depends on the model. Off, the component would also leave the split and the planes between vertices out, which do nothing for vertices outside their cells.
