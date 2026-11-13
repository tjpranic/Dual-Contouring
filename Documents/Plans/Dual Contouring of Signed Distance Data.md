# Dual Contouring of Signed Distance Data

Plan written 2026-11-13 for adding the algorithm of *Dual Contouring of Signed Distance Data* (Carrera, Wang, Batty, Stein and Sellán, 2026; `Documents/Papers/Dual Contouring of Signed Distance Data.md`) as a fifth algorithm, with a Burst implementation, a GPU implementation if the decision below goes that way, and no managed one.

**Status:** draft. Nothing is built. The decisions at the end are open.

## Goal

A new voxelizer component, `SignedDistanceDualContouring`, contours the same volumes as the other four on a uniform grid. It reads the density and material at the corners of the grid and nothing else: it never samples a volume between corners and never asks for a gradient. It can be picked in the interactive editor like the other algorithms and has example scenes of its own. The other four algorithms give the meshes they give today.

## What the paper does

The paper starts from Dual Contouring on a uniform grid, with one vertex in every cell the contour passes through and a quad around every edge it crosses. What it changes is where the vertices go, when all that is known is the distance at the corners of the grid.

- **Start (section 3.1).** The point where the contour crosses an edge is interpolated from the two distances. Its normal is the gradient of the trilinear interpolant there, averaged over the cells around the edge. Each cell's vertex starts at the average of the points on its edges.
- **Outer loop (section 3.2),** 100 times:
  - **Samples.** Each grid sample is a sphere around its corner with the distance as its radius, which the surface has to touch. Each sample is given to the cell that holds the point of the current mesh nearest to it.
  - **Local mesh.** In each cell, a fan of triangles around the cell's vertex, through the points on the cell's edges and the points where the mesh's edges cross the cell's faces.
  - **Inner loop (section 3.3, appendix C),** up to 100 times for each cell, all cells at once: move the vertex so that the local mesh touches the cell's spheres. Each step is a small least-squares problem of three unknowns, with the planes of the edges (weight `w_H` = 0.02) and a pull toward the last position (`μ` = 0.1) beside the spheres.
  - **Hermite update.** The point and normal of every crossed edge are moved toward the plane that best fits the four vertices around it (weight `w_u` = 0.2).
- **End.** The quads between the vertices are the mesh.
- **What it claims.** Sharp edges and corners from distances alone, approaching Dual Contouring with exact Hermite data (figure 6) and well ahead of Dual Contouring with estimated Hermite data. Vertices are free to leave their cells, which the paper found helps (figure 15).
- **What it leaves open.** One vertex in a cell, so the mesh is not always manifold. No octree. Sensitive to noise. The run time is dominated by the inner loop.

## What exists today

- **A voxelizer is a component with a switch over implementations.** `OccupancyDualContouring` is the model for one with Burst and GPU and no managed code: it starts as Burst, throws for Managed, and its inspector hides the settings of `Voxelizer` it does not use.
- **The corners are already sampled as this algorithm needs them.** `BurstSampleCornersJob` gives the density and the material at every corner, and `BurstSolidCornersJob` a byte of solid corners for every cell. On the GPU, `DensityFunctions.hlsl` has the shapes.
- **Uniform Dual Contouring in Burst** has the rest of the frame: one vertex per cell, `BurstQuads` for the four cells around an edge, `FindQuadsJob`, `ContourJob`, `BurstMesh`, `BurstDebugInformation`.
- **A symmetric 3 × 3 eigen-solver** (Jacobi sweeps) is in `BurstQEF` and in `QEF.hlsl`, private to the QEF. The plane through four vertices needs one.
- **GPU:** two compute shaders with their drivers, `SurfaceExtractor.readMesh` for the readback, and drawing straight from the GPU. `InterlockedAdd` is in use on this platform, and a kernel may write to eight buffers at most.
- **Interactive editor:** `Algorithm` is `{ Uniform, Adaptive, Manifold, Occupancy }`, nine variants. `RemeshScheduler` follows a drag live for contours up to 50 ms, and after a contour over a second waits for **Contour** to be pressed.

### Findings that shape the work

1. **On this project's volumes the algorithm cannot beat what is already here, and will be far slower.** The paper's ceiling is Dual Contouring with exact Hermite data, which is what Uniform Dual Contouring computes from the volumes. What the new algorithm adds is the paper's result itself beside the others, and a contour that needs nothing but a grid of distances: the input a sampled or stored distance field would give. Expect its vertices near Uniform's, not past them.

2. **The authors' code is public, MIT licensed, and differs from the paper.** I read both copies: the one kept up in gpytoolbox (`src/cpp/dual_contouring_of_signed_distance_data`, commit d6e6ae3 of 2026-11-09) and the frozen one the paper's figures were made with (github.com/xianacarrera/dual-contouring-of-signed-distance-data, commit 0edfd78). They agree on everything below. I did not run either.

   | | Paper | Code |
   |---|---|---|
   | Direction `d` of a sample's row | Normalized | Not normalized: its length is the sample's radius, so a row is weighted by the radius (finding 3) |
   | Samples inside the solid | As the others | Left out of a step when the mesh is farther from the corner than the radius: they push and never pull |
   | Points on faces | Where the mesh's edges cross the faces | The same, blended with the last outer iteration's by 0.2, and kept from before when the segment between two vertices misses the face's plane |
   | Normal update (equation 7) | `n + w_u n_old`, normalized | `( 1 - w ) n_old + w n`, normalized, with `w` = 0.2 |
   | Point update | Toward the plane's crossing of the edge | The same, skipped when the plane crosses the edge's line outside the edge or runs along it |
   | Which samples | All, or a random batch of 200,000 | All within three cell diagonals of the surface, plus random others only while that is fewer than 200,000 |
   | Solving a step | As Ju et al. | Truncated SVD of `AᵀA` with a threshold of 0.01, which `μ` = 0.1 keeps from ever truncating |
   | When the inner loop stops | A threshold `τ` | A step shorter than 1e-6, on a grid two units wide |
   | A cell with no samples | Not said | Keeps its vertex: not even the planes of its edges move it |
   | Equation 1 | `t = |s_a| / ( |s_a| - |s_b| )` | `s_a / ( s_a - s_b )`: the paper's minus is a slip |

   The published results came from the code, and the default values were tuned on it. **The plan follows the code** (decision 2). MIT allows this project to follow it closely; where a function is ported, the source is credited in a comment and in the README.

3. **The code's weights depend on the size and the resolution of the grid.** A sample's row is weighted by its radius in the units of the grid, and `w_H` and `μ` are plain numbers. Every grid in the paper spans -1 to 1.
   - **What that does at 128³:** a sample at the outside of the band has a squared weight of 0.0066, the plane of an edge 0.0004, and staying put 0.1. Each inner step moves a fraction of the way, which is why there are a hundred of them.
   - **At 16³** the same sample weighs 0.42, and the samples dominate.
   - **What the plan does:** the weights are worked out as if the grid were two units wide, whatever the size of the object, so that the paper's values mean here what they meant there. This project's grid is one unit wide in object space, so a radius is twice the density.
   - **gpytoolbox's own test** runs the function on a grid from 0 to 1 with the same defaults, so even the authors' two uses weigh differently.

4. **The project's density is an exact distance only for a single shape.** The algorithm treats every sample as a sphere the surface touches.
   - **Single shapes:** exact, except `Ellipsoid`, whose formula is an approximation unless it is a ball.
   - **Unions** (`min`): exact outside, and inside only a lower bound of the distance. The code's rule for inside samples (finding 2) is what that needs: a bound may push the surface out of its sphere, and must not pull the surface to it.
   - **Intersections and subtractions** (`max`): the other way round, exact inside and a bound outside. The code pulls the surface toward outside samples, so I expect the rims of cut-outs to be drawn toward the cut. This is the case the paper mentions for swept volumes, on the other side.
   - **A model that mixes them,** a union with a piece cut out, can be a bound on both sides.
   - **What the plan does:** builds it as the code has it, measures cut-outs in step 3, and asks then. One fix that stays within the grid's data: `calculateDensityAndMaterial` can say for each corner whether its density is known to be exact, by carrying two flags through the list of shapes, and samples that are not would only push.

5. **Inside and outside are decided by materials here, not by the sign of the density.** The two differ where a Subtraction passes through a shape whose material it does not name (finding 17 of the Occupancy plan). The plan takes solid from the material, as every other algorithm does, and the radius from the size of the density. In that one case the spheres describe a surface the materials do not have, and the mesh there will be wrong in its own way, as it is dented in the other algorithms.

6. **No tree is needed to find the nearest point of the mesh.** The code builds an AABB tree over the whole mesh every outer iteration, and one over each cell's fan every inner iteration.
   - **Samples are in a band** around the surface (finding 2), so the nearest point of the mesh is within the band's width plus a cell diagonal, or the sample is dropped anyway. A search outward from the sample's corner over the cells that own quads, stopping when nothing nearer can follow, finds exactly what the tree finds.
   - **A cell's fan** has at most 24 triangles, usually 8. Trying them all is cheaper than a tree.
   - **Vertices outside their cells** stretch triangles past the four cells of their quad. The search stays exact by widening its bound by the farthest any vertex is outside its cell, which is found once per outer iteration.

7. **Everything the loop keeps hangs off the cells that have a vertex.** The three balls at 128³ have 52,224 of them among 2.1 million.
   - **Hermite data:** a crossed edge inside the grid always has all four of its cells with a vertex, and belongs to the one that has it at its maximum, as `BurstQuads` already has it. Three edges a cell.
   - **Points on faces:** a face between two cells with vertices belongs to the lower one. Three faces a cell.
   - **Edges on the outside of the grid** have no quad, so their Hermite data is never updated and can be worked out from the corners whenever a cell needs it.
   - **In all** about 150 bytes for each such cell, beside the density and material of every corner that Uniform keeps too.

8. **It is slow by design.** At the paper's values a contour is up to 10,000 small solves for every cell with a vertex, each over about ten samples and eight triangles.
   - **My estimate for Burst,** from those counts and not from any measurement: the three balls at 128³ take between several seconds and a minute, depending on how early the inner loops stop, and 32³ around a second. It could be off by a factor of several either way. Step 2 measures it.
   - **Against the others:** Uniform Dual Contouring takes about 30 ms at 128³.
   - **The editor copes as it is:** a contour over a second already waits for **Contour**. But a contour blocks the editor while it runs, and Burst jobs cannot be interrupted.
   - **The iteration counts are settings,** so a scene can trade accuracy for time.

9. **The project's QEF solvers do not fit the inner step.** They drop singular values below 0.1, which is exactly `μ`, and solve around the average of the points they were given. The step is a full-rank 3 × 3 system once `μ` is in it, so it gets a direct solve of its own, for the move from the last position, which keeps the numbers small in float. The QEF solver and minimizer iterations settings have no meaning for this component.

10. **Five settings of `Voxelizer` have no meaning here:** QEF Solver and Minimizer Iterations (finding 9), and Binary Search Iterations, Intersection Approximation and Surface Correction Iterations, which sample the volumes between corners.

11. **The quads are split along the other diagonal than Uniform's.** The code measures samples against the mesh split from the lowest to the highest of the four cells around an edge. Uniform's Burst contour splits along the other one. This component splits the way the code does, in the loop and in the mesh it hands over, so that what was optimized is what is drawn.

12. **The code is not deterministic and this will be.** The code picks its extra samples at random. Here the samples are the band, the same every time.

13. **A good fit for the GPU.** See the section on it under Design: every stage is one pass over corners, cells, edges or faces that reads a fixed neighbourhood, and nothing in between has to come back to the CPU.

## Design

### The component

`SignedDistanceDualContouring : Voxelizer`, in `Assets/Scripts/SignedDistanceDualContouring.cs`, a partial class like the others.

- **Grid:** uniform, 2^resolution cells across, one vertex in every cell with both solid and empty corners.
- **Implementations:** Burst, and GPU from step 4. A new component starts as Burst. Managed throws `NotSupportedException` and the inspector turns it into Burst.
- **Settings it uses from `Voxelizer`:** resolution, vertex normals, the debug view.
- **Settings it hides:** the five of finding 10.
- **Its own settings,** with the paper's values:

  | Setting | Default | What it does |
  |---|---|---|
  | `outerIterations` | 100 | How many times samples are handed out, vertices moved and Hermite data updated. Zero gives the starting mesh. |
  | `innerIterations` | 100 | The most steps a vertex takes in one outer iteration. |
  | `hermiteWeight` | 0.02 | `w_H`: how much the planes of a cell's edges count beside its samples. |
  | `regularization` | 0.1 | `μ`: how strongly a step is held to where it started. |
  | `updateWeight` | 0.2 | `w_u`: how far Hermite data and face points move toward their new values each outer iteration. The code has three weights for this, all 0.2. |
  | `sampleBand` | 3 | How far from the surface a sample may be to count, in cell diagonals. |

- **Constants:** the inner loop's stopping distance, in cells, and the thresholds of the code for lines that run along planes. Chosen for float in step 2, by measurement.

### What it reads

The density and the material of every corner, once. A corner is solid when it has a material. Its sphere has the size of its density as radius, and is an inside sample when the corner is solid.

### Start

- **Crossed edges:** those between a solid and an empty corner.
- **Point:** `t = |s_a| / ( |s_a| + |s_b| )` along the edge, and the middle when both are zero.
- **Normal:** the gradient of the trilinear interpolant at the point, normalized in each cell around the edge, then averaged over those cells and normalized.
- **Vertex:** the average of the points on the cell's crossed edges.

### One outer iteration

In the code's order.

1. **Samples.** For every corner within the band: the nearest point of the current mesh (finding 6). The sample is dropped when that point is farther than its radius plus a cell diagonal, and otherwise belongs to the cell the point lies in, if that cell has a vertex.
2. **Face points.** For every face between two cells with vertices: where the segment between the two vertices crosses the face's plane. The first one is taken as it is, later ones are blended in by `updateWeight`, and when the segment misses the plane the last one stays.
3. **Hermite update,** from the second iteration on. For every crossed edge inside the grid: the plane of least spread through the four vertices around it, with its normal turned from the solid end to the empty one. The edge's normal is blended toward it and normalized. Where the plane crosses the edge within its length, the edge's point is blended toward the crossing.
4. **Vertices.** For every cell with samples and a fan, up to `innerIterations` times:
   - **The fan:** a triangle (vertex, edge point, face point) for every face of the cell that has a face point and every crossed edge of that face.
   - **For each sample:** the nearest point of the fan and its barycentric weights. A point on the side of its triangle opposite the vertex, with no weight on the vertex, is replaced by the vertex itself. An inside sample whose sphere the fan is clear of is left out. The others give one row: along the direction from the corner to that point, the fan should be at the radius.
   - **The step:** least squares over those rows weighted by radius (finding 3), the planes of the cell's edges weighted by `hermiteWeight`, and `regularization` times the squared length of the move.
   - **Stop** when a step is shorter than the stopping distance.

### Polygons

A quad around every crossed edge inside the grid, facing from its solid end to its empty one, in the sub-mesh of the solid end's highest material, as in Uniform. It is split from the lowest to the highest of its four cells (finding 11).

- **Normal of a vertex:** the normalized sum of the normals of its cell's crossed edges, as they are after the last update.
- **Boundaries between materials** are not looked for: each material has its sub-mesh and the boundary follows the grid, as in Occupancy-Based Dual Contouring.

### Burst

`Assets/Scripts/Burst/SignedDistanceDualContouring.Burst.cs`, with the geometry it needs in `Assets/Scripts/Burst/BurstSignedDistance.cs`: nearest point of a triangle with its weights, the 3 × 3 solve, the plane through four points, the trilinear gradient.

1. Sample the corners (`BurstSampleCornersJob`), solid corners per cell (`BurstSolidCornersJob`).
2. Lay out the cells with a vertex row by row (`BurstLayoutRowsJob`), with each one's place along its row. A search for nearby quads then goes through rows and not through every cell.
3. Start: Hermite data of each cell's three edges, then vertices (parallel).
4. List the corners in the band, once.
5. The outer loop, each stage a parallel job: samples, a counting sort of them into a list per cell, face points, Hermite update, vertices.
6. Contour and mesh, in a fixed order, so the same input gives the same mesh.
7. Debug information when wanted.

Profiler markers are named `SignedDistanceDualContouring.Burst.<Stage>`: `SampleCorners`, `Start`, `AssignSamples`, `FacePoints`, `UpdateHermite`, `SolveVertices`, `Contour`, `BuildMesh`, `DebugInformation`.

### GPU

`Assets/Scripts/GPU/SignedDistanceDualContouring.GPU.cs` and `Assets/Resources/Shaders/SignedDistanceDualContouring.compute`, kernel for job.

**Why it fits:**

- **Every stage is a map** over corners or cells that reads a fixed neighbourhood: the search for the nearest quad, the face points, the plane of four vertices, the inner loop of a cell. None of it needs a tree, a sort or a queue (finding 6).
- **The lists of samples need no list-building.** A cell can collect its samples by looking through the corners around it for those given to it: count, take room with `InterlockedAdd`, fill. The order within a list is the order of the look, so the arithmetic is the same on every run.
- **Nothing comes back in between.** One readback after the cells are counted, to size the buffers, as the Occupancy shader has. Then the whole loop runs as dispatches, several hundred for a hundred outer iterations.
- **What the shader keeps is small** (finding 7), beside a density and a material for every corner.
- **The work is what a GPU is fastest at,** arithmetic on a few floats for very many cells at once. Occupancy-Based Dual Contouring, whose searches are of the same kind, takes 32 ms on the GPU against about 55 ms in Burst. I expect a larger gain here, since nearly all the time is in the inner loop, but have nothing measured to put a number on it.

**What could go wrong:**

- **A long kernel.** A hundred inner steps for every cell in one dispatch may hold the GPU long enough to stall the editor's window. If it does, the inner loop is cut into several dispatches, with each cell's state kept in a buffer.
- **Burst and the GPU will not agree as closely as they do elsewhere.** Ten thousand steps of an optimization that is not convex can take a difference in the last bit of a float a long way. The two are compared tightly after one outer iteration and by their distance from the shapes after all of them.
- **Eight writable buffers** a kernel: the buffers of the grid are declared twice, as in the Occupancy shader.

**Recommendation:** build it, as step 4, after the Burst implementation is complete and measured. If finding 8's estimate holds, the GPU is what makes the paper's values usable past 64³.

### Debug view

- **Corners:** density and material.
- **Edges:** the point and normal as they are after the last update.
- **Voxels:** the cell and its vertex.
- **Face points, fans and samples** are not shown.

### Interactive editor

- `Algorithm.SignedDistance`, and the component on the editor object: eleven variants with the GPU, ten without.
- **Switching** as for Occupancy: resolution carries over as for Uniform, Managed becomes Burst, and the five settings it does not use are carried through it.
- **The signature** has its six settings and leaves out the five.
- **The scene** needs the component saved on its editor object, which needs the user's go-ahead at that step.

### Example scenes

In `Assets/Scenes/Signed Distance Dual Contouring/Burst` and `/GPU`, made from the Uniform ones as the Occupancy scenes were. The cube is turned off the grid's axes, which is the paper's own showcase (figure 16). Iteration counts are saved per scene so that each opens in a reasonable time, decided once step 3 has measured.

## Tests

There is no managed implementation to compare against. The tests hold the pieces to answers known in closed form, the whole to the shapes, Burst and the GPU to each other, and, if decision 7 goes that way, the whole to the authors' code. Each new test is shown to fail on a deliberate mistake, as in the earlier plans.

- **Geometry** (new `SignedDistanceGeometryTest`, through small Burst jobs):
  - the nearest point of a triangle and its weights, in each of the seven regions around it, and against a search over many points of the triangle;
  - the 3 × 3 solve against systems with known answers, and against `regularization` alone, which leaves the vertex where it was;
  - the plane through four points: exact for four in a plane, turned from solid to empty, and no update for four on a line;
  - the trilinear gradient of a linear field is that field's gradient.
- **Start:** on a plane at any angle, every edge point and normal is exact and every vertex is on the plane. The topology and triangle count of every model of `TopologyTest` are Uniform's.
- **Samples:** on random small grids with vertices moved out of their cells, the search gives the nearest point a search over every triangle gives. The drop rule. A sample goes to the cell its point lies in.
- **Face points and Hermite update:** by hand, each rule of finding 2: the first value, the blend, the kept value, the skipped update.
- **One step:** a fan and a sphere built by hand move the vertex to where the fan touches the sphere. An inside sample clear of the fan does nothing. A point with no weight on the vertex moves the vertex itself.
- **The whole:**
  - a plane stays exact through every iteration;
  - a ball's vertices are within a tolerance of it, measured in step 3;
  - a cube turned off the axes has a vertex within a tolerance of every corner and the mesh along every edge, and is nearer to both than the starting mesh is;
  - each shape of `ShapeContourTest`, upright and turned, has every vertex within a tolerance of the shape;
  - the same input twice gives the same mesh, bit for bit.
- **Materials:** `SubMeshTest`'s models give each material its sub-mesh.
- **Against the authors' code** (decision 7): a handful of small grids, sampled by this project and contoured once by gpytoolbox with every sample in use, kept as files beside the tests. The Burst contour of the same grid with the band over the whole grid is held to them within a tolerance.
- **Burst against GPU:** within a tight tolerance after zero and one outer iterations, and by distance from the shape after all.
- **Editor:** the tests over all variants take the new ones in; switching to and from it; the signature.
- **No change for the others:** the `Start` hashes of step 0 after every step.
- **Performance:** `ContouringPerformanceTest` gains the algorithm at 16³, 32³ and 64³.

## Steps

Each step compiles, passes every test, and is reviewed before the next one starts. Nothing is committed for you.

0. **Baseline, the oracle and the geometry (small).** Both suites green; the `Start` hashes and Burst timings recorded with Unity in front. `BurstSignedDistance` with the four pieces of geometry and their tests. If decision 7 is yes: gpytoolbox in a virtual environment outside the project, a probe that writes a grid of corner densities to a file, and the first reference meshes.
1. **Component and the starting mesh (medium).** The component, its settings and inspector, defaults and dispatch. Burst: corners, cells laid out by row, Hermite data from the grid, vertices at the averages, quads, mesh, debug information. This is the contour with `outerIterations` at zero. The start, topology and material tests.
2. **One outer iteration without the Hermite update (large).** The band, the search for the nearest quad, the lists, face points, fans, the inner loop. The sample, face point and step tests. **Measured here:** how long each stage takes at 16³ to 128³, how many inner steps cells take, and how many samples a cell gets. Finding 8's estimate is replaced by those.
3. **The Hermite update and the whole loop (medium).** The whole-contour tests. **Measured here,** on single shapes and on models with unions and cut-outs, against the starting mesh and against Uniform Dual Contouring:
   - distance of vertices from the shapes, and from their edges and corners;
   - how the error falls with outer iterations, which says what the scenes should save;
   - what cut-outs look like (finding 4), with the flags for exact densities tried if they are drawn in;
   - the paper's reading of the sample rows, normalized, run once to see what the difference is.

   The questions under "Asked after step 3" are put then.
4. **GPU (large).** The shader and its driver, the kernels in the order of the jobs, the Burst-against-GPU tests, drawing straight from the GPU, a timed remesh.
5. **Interactive editor and scenes (medium).** `Algorithm.SignedDistance`, switching, the signature, the editor tests. The component in `Interactive Editor.unity` and the example scenes, with the user's go-ahead for the scene.
6. **Tuning and documentation (medium).** Whatever step 2 and 4 show to cost most. Candidates, each kept only if the mesh stays within the tolerances: cells whose vertex, samples, face points and Hermite data did not move in the last outer iteration skip the next one; an end to the outer loop when nothing moves. README section, the table of implementations, `AGENTS.md`.

## Measurements

None yet.

## Out of scope

- **A managed implementation.**
- **More than one vertex in a cell.** The paper names Manifold Dual Contouring as a possible extension. Here the mesh has Uniform's topology, non-manifold edges included.
- **An octree or simplification.** The paper has none.
- **Keeping vertices in their cells.** The paper tried it and found it worse.
- **Random batches and samples beyond the band.** The band can be set wide enough to take a small grid whole.
- **Distances from anything but the project's volumes:** a stored grid, a texture, noise. The implementation reads only the corner arrays, so such a source would be a small addition later.
- **Choosing each quad's diagonal by the samples,** which the paper names as future work.
- **Contouring without blocking the editor,** or a way to cancel a contour.
- **Hermite data on the boundary between two materials.**

## Decisions

1. **A component of its own, with one vertex in a cell?** Recommended: yes, `SignedDistanceDualContouring`, as the paper has it. The other way is a mode of `UniformDualContouring`, which would hang six settings on it that mean nothing to its other mode.
2. **Follow the authors' code where it differs from the paper?** Recommended: yes, in everything finding 2 lists, with the weights of finding 3. The results in the paper are the code's. Step 3 runs the paper's normalized rows once for comparison.
3. **Solid by material, radius by density?** Recommended: yes (finding 5), so that sub-meshes and the quads' facing mean what they do in the other algorithms.
4. **A band of samples in place of the random batch?** Recommended: yes, three cell diagonals unless set otherwise, which is what the code uses on any grid large enough to matter. It makes the contour repeatable and the search local.
5. **Defaults.** Recommended: the paper's, 100 and 100, on the component, with lower values saved in scenes where step 3 shows they are enough. The other way is defaults chosen for speed, which would make a new component give a worse mesh than the paper's unless its values are raised.
6. **The GPU implementation.** Recommended: yes, as step 4. It can still be dropped after step 3 if Burst turns out fast enough to make it pointless, which I do not expect.
7. **The authors' code as an oracle.** Recommended: yes. It is the only independent check of the whole loop. It needs `pip install gpytoolbox` in a virtual environment in a scratch folder, which this session was not permitted to run, so it needs your go-ahead. The reference meshes would be small files beside the tests; Python would not be needed to run the tests.
8. **Editor and scenes in this plan?** Recommended: yes, as step 5.

### Asked after step 3

- **Cut-outs** (finding 4): whether samples that are only bounds should push and not pull, once it is measured what they do.
- **Scenes' iteration counts,** from the error curves.
- **Whether a contour of many seconds needs a warning in the inspector** before it is started.
