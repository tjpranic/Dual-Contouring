# Dual Contouring Unity Project
A Unity implementation of various **Dual Contouring** algorithms for mesh generation from implicit surfaces. This project includes multiple dual contouring variants with managed C#, Burst and GPU implementations.

## Overview
Dual Contouring is a technique for generating meshes from volumetric data represented by implicit functions. Unlike Marching Cubes, Dual Contouring preserves sharp features and produces better-quality meshes by solving a Quadratic Error Function (QEF) to find optimal vertex positions.

This Unity project implements four main variants of the dual contouring algorithm:

- **Uniform Dual Contouring** - Regular grid-based implementation
- **Adaptive Dual Contouring** - Octree-based implementation with level-of-detail support
- **Manifold Dual Contouring** - Specialized version that ensures manifold mesh topology
- **Occupancy-Based Dual Contouring** - Manifold version on a regular grid that only asks whether a point is inside or outside

## Implementations
Each voxelizer component has an **Implementation** setting that selects how it contours.

| Algorithm | Managed | Burst | GPU |
|---|---|---|---|
| Uniform Dual Contouring | Yes | Yes | Yes |
| Adaptive Dual Contouring | Yes | Yes | No |
| Manifold Dual Contouring | Yes | Yes | No |
| Occupancy-Based Dual Contouring | No | Yes | Yes |

- **Managed** is plain C#, written to follow the papers. It is the reference the other two are tested against, and it is slow: a grid of 128³ cells takes most of a minute.
- **Burst** runs the same algorithm as jobs compiled by the Burst compiler, on all cores. It contours 128³ cells in about 28 to 33 ms in the editor, which is over a thousand times faster than Managed, and has all three QEF solvers. Occupancy-Based Dual Contouring samples the volumes several times as often and takes about 52 ms.
- **GPU** runs Uniform or Occupancy-Based Dual Contouring in a compute shader, with the SVD and probabilistic QEF solvers. Contouring 128³ cells and reading the mesh back takes about 133 ms for Uniform and 32 ms for Occupancy-Based Dual Contouring, whose searches suit a GPU and whose shader keeps far less for each cell.

Burst gives the same mesh as Managed, triangle for triangle, with vertices that differ in the last bits of their coordinates. Where octree simplification or vertex clustering decides right at the error threshold, the two can collapse different nodes.

Occupancy-Based Dual Contouring has no managed implementation. Its tests hold it to answers known in closed form, and its two implementations to each other: the GPU's vertices are within 0.02 of a cell of Burst's, and every polygon is split the same way.

Volumes are placed by their transforms: position, rotation and scale. Every implementation contours turned shapes. A turned shape costs a little more to sample than an unturned one: Burst contours a model whose shapes are all turned 6 to 11% slower.

## Shapes
There are nine kinds of volume, each its own component. Each is made from the box its transform gives it: centred on the box, with its axis along the box's own y.

| Shape | Made from its box |
|---|---|
| `Cuboid` | The box itself. |
| `Ellipsoid` | The ellipsoid that touches the middle of each face of the box. |
| `Cylinder` | Standing along y, as high as the box. |
| `Cone` | Its base fills the bottom of the box, and its tip is in the middle of the top. |
| `Capsule` | A cylinder along y with a half ball on each end, as high as the box. A box lower than it is wide gives a ball. |
| `Torus` | Lying across x and z, with a tube as thick as the box is high. |
| `Pyramid` | Its square base fills the bottom of the box, and its tip is in the middle of the top. |
| `Octahedron` | Its six corners are in the middle of the box's six faces. |
| `HexagonalPrism` | Standing along y, the largest regular hexagon that fits across x and z, with two corners on the x axis. |

A cylinder, cone, capsule or torus is round across x and z, and a pyramid is square. They take the smaller of the box's x and z sizes for both.

The seven shapes after the ellipsoid come from Inigo Quilez's [distance functions](https://iquilezles.org/articles/distfunctions/). Each is an exact signed distance, checked against distances worked out another way. Two of them differ from the article:
- **Pyramid:** the article's pyramid is off below its base and inside near it; corrected here.
- **Octahedron:** the article's exact one needs all three sizes equal; this one is exact for any box.

Knowing the new shapes makes the Burst and GPU implementations slower over cuboids and ellipsoids: about 8 to 15% for Burst and 11% for the GPU.

Example scenes for each implementation are in `Assets/Scenes/<Algorithm>/Managed` and `Assets/Scenes/<Algorithm>/Burst`. For Uniform Dual Contouring those two folders are under `CPU`, next to `GPU`. Occupancy-Based Dual Contouring has `Burst` and `GPU`.

## QEF Solvers
The vertex of a cell goes where the QEF of its hermite samples is least: the sum of the squared distances to their planes. Every voxelizer has a **QEF Solver** setting that picks how that point is found:

| Setting | Default | What it does |
|---|---|---|
| QEF Solver | SVD | **Simple:** steps from the centre of the cell toward the planes. **SVD:** the point of least error nearest the mass point of the samples, as in *Dual Contouring: The Secret Sauce*. Directions the samples hardly pin down, with singular values under 0.1, stay at the mass point. **Probabilistic:** the point of least expected error when the normals are uncertain, as in *Fast and Robust QEF Minimization using Probabilistic Quadrics* (Trettner and Kobbelt, 2020). |
| Minimizer Iterations | 6 | The simple solver's steps and the SVD solver's sweeps. Not shown under Probabilistic, which has nothing to iterate. |
| Normal Deviation | 0.05 | From 0.02 to 1, and only shown under Probabilistic: how uncertain the normals are taken to be, as a standard deviation in each of their components. Lower keeps shallow creases; higher pulls vertices toward the average of their samples. |

**How the probabilistic solver works:** to the plane of each sample it adds a pull toward the sample's own point, weighted by the deviation squared. The QEF then has exactly one point of least error, found with a 3 × 3 linear solve, with nothing to truncate. The error that simplification and clustering compare with their thresholds is the same for all three solvers: the QEF at the vertex.

**The error is worked out in doubles.** Written as xᵀAᵀAx − 2xᵀAᵀb + bᵀb, it is a small difference of three large terms close to the surface. They grow with the number of samples and their distance from the origin, and in floats they round by more than the error itself.
- **Example:** an octant of a cube 0.6 across, at 128³ cells, has an error of exactly 0. Floats rounded it to 1.2e-4, twelve times the interactive editor's threshold. A cube 0.5 across rounds to 0 by luck, since all its numbers are powers of two.
- **How:** every QEF keeps its sums a second time in doubles (`QEFError`), and works its error out from those. It still solves with the float sums, so the vertices are as before, and the same as the GPU's. The one exception is the managed simple solver, which keeps its planes and adds up the squared distances to them one by one, with nothing to cancel.
- **The GPU** has no doubles and works the error out in floats, but none of its algorithms compares the error with a threshold.
- **Cost:** at 128³ cells on the three balls with Burst, Uniform contours as fast as before, also at 256³. Manifold Dual Contouring is about 5% slower, 37 ms against 35. Adaptive is about 14% slower, 40 ms against 35: it keeps a QEF for every node of its octree, and each is now twice the size.
- **What it changes:** clusters and nodes that rounding let through are refused where their real error is over the threshold. At 128³ cells and 2e-4 the three balls keep 2,428 vertices instead of 2,359 in Manifold Dual Contouring, and 2,652 instead of 2,586 in Adaptive. On the four test models at 8³ and 16³ cells, at 2e-4, the meshes of both are the same as before.

**Where it works:** every algorithm and implementation. The GPU's shaders have no simple solver and solve with SVD in its place, and the inspector turns a GPU voxelizer's Simple into SVD.

**How it compares with SVD,** at 0.05:
- **Shallow creases:** it keeps them where SVD flattens them. Where the faces of a crease are less than about 20° apart, a cell's few samples give singular values under 0.1, and SVD leaves the vertex at the mass point across the crease. On a ridge whose faces are 10° apart, at 64³ cells, its vertices are 0.10 of a cell from the ridge on average, against 0.32 with SVD.
- **The project's shapes:** about the same, since their normals are exact and their edges sharp. On the nine shapes at 32³ cells, upright and turned:
  - corners and edges are a few thousandths of a cell farther from the mesh than with SVD, at most 0.035 of a cell, at the pyramid's tip;
  - the worst vertex off a shape is nearer, 0.10 of a cell against 0.18.
- **Noise:** Normal Deviation should be about as large as the noise in the normals. Below it, vertices wander along curved surfaces and out of their cells. With the project's exact normals, 0.03 already does that: on the torus 78 to 108 vertices of about 1,200 are outside their cells, against 13 to 16 with SVD and 31 to 34 at 0.05.
- **Simplification:** Adaptive and Manifold Dual Contouring add up the QEFs of many cells, and the pull weighs every sample alike, so a feature that few of them see gives way to the many that do not. Only the error threshold holds such a cluster's vertex to its corner or edge, and fewer clusters pass it.
  - **Turned cube:** turned about a diagonal, it keeps 251 vertices in Manifold Dual Contouring at 32³ cells and 2e-4, against SVD's 104. At 16³ one of its corners was 0.14 of a cell from the nearest vertex.
  - **Curved models** simplify as far as with SVD.
  - Where simplification matters, SVD is the better choice.
- **Time:** the same within a few percent, in every algorithm and implementation: at 128³ cells Burst contours the three balls in 36.6 ms with it and 38.4 ms with SVD. The solve is a small part of a contour.
- **Precision:** the solve divides by the number of samples times the deviation squared, which magnifies the rounding of the QEF's sums. Below 0.02 the solve loses its precision in floats, which is why the setting stops there.

The `Probabilistic Solve` scenes sit beside each `SVD Solve` scene, for Uniform, Adaptive and Manifold Dual Contouring, Managed and Burst.

## Vertex Normals
Every voxelizer has a **Vertex Normals** setting, with two more after it:

| Setting | Default | What it does |
|---|---|---|
| Vertex Normals | Creased | **Creased:** smooth across curved surfaces and broken at sharp edges. **Split:** flat, every triangle with vertices of its own. |
| Crease Angle | 40 | With creased normals, triangles around a vertex that meet at more than this many degrees get vertices of their own. At 180 nothing is split. |
| Smooth Normals | Contour | With creased normals, the normal of a vertex whose triangles all meet within the angle. **Contour** keeps the normal the contour gave it, where that is within the angle of every one of its triangles and among their normals. **Faces** takes the normal of its triangles. |

**Why creased:** a vertex that the faces of a sharp edge share has one normal, halfway between them, and every triangle around it shades towards that normal. The edge comes out smeared across a cell, and where it crosses the grid at an angle, as a sawtooth. On a simplified mesh, or one an octree simplified, the large triangles smear it across whole faces. Split normals keep every edge sharp but facet every curved surface.

**How it creases:**
- **Grouping:** around every vertex its triangles go into groups by their normals, the largest corner first. Each triangle joins the group whose mean is nearest, if that is within the crease angle, and starts a group otherwise. A group's mean weighs its triangles' normals by their angles at the vertex.
- **Normals:** each group becomes a vertex of its own, whose normal weighs the group's triangles' normals by their angles at the vertex times their areas, so that a sliver with a wide corner has little say in it.
- **Why the group's mean:** comparing with it, and not with the triangle next to it, keeps a run of triangles that turn a little at a time, such as a cone's tip, from becoming one group.
- **Which contour normals stay:** under Contour, a normal that leans outside its triangles' normals gives way to theirs, even within the crease angle. Near an edge the contour's normals lean towards the surface across it, and on the faces beside it they made streaks.
- **No corner ends up further than the crease angle from its triangle.** Should a group's normal move too far from a triangle in it as others join, that triangle gets a group of its own.
- **Flat triangles:** a triangle too flat to have a normal has no say in any group. Its angles are all within 0.06° of 0° or 180°, as along the edge of a polygon Occupancy-Based Dual Contouring splits.

**Where it works:** every algorithm and implementation, after mesh simplification, in play mode and in the interactive editor.
- **The managed and Burst implementations:** a Burst pass creases their meshes, `BurstCreases`.
- **The GPU implementations:** they crease their own in a compute shader, drawn straight from the GPU or read back. The groups are Burst's, and the normals within 0.001° of Burst's on every contour tested.

**What it adds,** with Burst at 128³ cells:

| Model | Uncreased | Creased | Split |
|---|---|---|---|
| Three balls | 52,224 | 53,581 | 313,500 |
| The new shapes | 45,270 | 49,104 | 272,184 |
| Three balls, simplified | 2,268 | 2,734 | 13,764 |

**Cost:** about what splitting costs.
- **Burst:** 9 ms at 128³ and 35 ms at 256³ on the three balls under Contour, 8 and 31 ms under Faces, against Burst contours of 30 and 178 ms.
- **GPU, Occupancy-Based Dual Contouring:** about 1 ms at 128³ and 5 ms at 256³.
- **GPU, Uniform Dual Contouring:** too little to see beside its own contour of 100 to 250 ms at 128³. That implementation also reads back two numbers after its contour to size the crease, which waits for the GPU to finish.

**Shared normals:** before creased normals there were shared ones, which Creased replaced. At 180° under Contour a voxelizer shows what Shared showed, triangle for triangle. A scene saved with Shared vertex normals opens with creased ones.

**What it does not do:**
- **Slivers:** where a triangle of the contour straddles a sharp edge, the edge shows a small notch, in every mode, Split included. That comes from where the vertices are placed, not from the normals.
- **The GPU at 0°:** it can split triangles that lie in one plane, which Burst keeps together. The GPU's square root is not exact, and at 0° only exactly equal normals stay together.

The interactive editor's scene, three of the Mesh Simplification scenes (CSG Operations, Half Detail and Multiple Materials) and the Remeshing scenes use creased normals.

## Wireframe
Every voxelizer can draw the edges of its contour's triangles over the contour. Its inspector has a **Wireframe** section:

- **Look** is Off, **Over Surface** or **Lines Only**.
  - **Over Surface** draws the lines over the shaded surface.
  - **Lines Only** draws them over a flat fill, which hides the surface and the lines behind it. Each material's lines are in that material's colour.
- **Line Width** is in pixels, and stays the same at any distance.
- **Line Color** is the colour of the lines over the surface. Its alpha blends them.
- **Fill Color** is the fill under Lines Only.

**How the lines are drawn:**
- **Edges:** the lines are the edges of the triangles, so every quad of a contour shows its diagonal. A remeshed mesh is drawn with the edges of the faces the remesh made, so its quads and pentagons show none (see Remeshing).
- **The contour:** it is drawn as before, by its renderer or straight from the GPU, and the lines go over it. Picking, materials and shadows stay as they were; under Lines Only the hidden surface still casts its shadows.
- **Where it works:** every implementation, in play mode, and in the interactive editor in edit mode as well.
- **No contouring:** the settings change only what is drawn, so changing them never contours again.

**Cost:** on the three balls at 1920 × 1080, drawing the lines adds about 0.1 ms to a frame of 418,000 triangles from Burst, and about 0.2 ms from the GPU.

**Dense meshes:** seen whole, they fill up with lines. With the grid 600 pixels across, a cell at 256³ is two or three pixels wide.

## Mesh Simplification
Every voxelizer can simplify the mesh of its contour before it is shown. It follows *Surface Simplification Using Quadric Error Metrics* (Garland and Heckbert, 1997): edges are collapsed one at a time, the cheapest first, where what an edge costs is how far its new vertex would be from the planes of the triangles it stands for. It works on the finished mesh, so it serves every algorithm and implementation alike. The simplifier itself is Burst code.

Its settings are in the **Mesh Simplification** section of the inspector:

| Setting | Default | What it does |
|---|---|---|
| Enabled | Off | Simplifies the mesh of every contour. Off, every mesh is what it was without this feature. |
| Error | 0.05 | How far a collapse may leave a vertex from the planes of the surface it stands for, as a root mean square distance in cells. Edges are collapsed for as long as they stay within it. |
| Line Quadric Weight | 1e-5 | How much every vertex is held to the line through it along its normal, after *Controlling Quadric Error Simplification with Line Quadrics* (Liu, Rahimzadeh and Zordan, 2025). A little settles where the vertices of flat regions go and in which order. More evens out the triangles and leaves more of them. Zero is Garland and Heckbert's algorithm as written. |
| Detail | 0 | Which mesh between the simplest and the contour is shown: 0 is the simplest the error allows, 1 is the contour itself. |

**What it takes away,** from Burst contours at 128³ cells at the defaults:

| Model | Triangles | Simplified |
|---|---|---|
| Three balls | 104,500 | 4,588 |
| Cubes and a sphere | 68,812 | 1,576 |
| A slab with a band of a second material | 63,036 | 28 |

- **Flat shapes** come down to their corners: an upright cuboid to 8 vertices and 12 triangles, with any algorithm.
- **After an octree has simplified,** in Adaptive and Manifold Dual Contouring, it takes about another quarter at the default error.

**What it keeps:**
- **Topology:** no collapse joins, splits, opens or closes anything, after *Topology Preserving Edge Contraction* (Dey, Edelsbrunner, Guha and Nekhayev, 1999). That holds for meshes that are not manifolds too, as Uniform and Adaptive Dual Contouring give them. A small ball never goes below a tetrahedron.
- **Materials:** every sub-mesh, and the lines between them, by the rules of *Progressive Meshes* (Hoppe, 1996). A line between two materials is held to the error by itself, so a round one stays round on a flat face.
- **Open edges:** where the surface leaves the grid, the rim stays a rim.
- **Sharp edges and corners:** as far as the error allows. A corner can slide along its edge by about three times the error.
- **Distance:** vertices and the middles of triangles end up within about three times the error of the shape.

**What it does not:**
- **Triangles that pass through each other** are not prevented. What Occupancy-Based Dual Contouring promises holds with simplification off: on a model of turned shapes at 64³ cells, simplifying its contour leaves 9 such pairs where it had none.
- **GPU contours** are read back to be simplified, and are not drawn straight from the GPU. Their vertices come in a different order every time, so two contours of the same model simplify alike and not the same.
- **A higher line weight is no free lunch here.** The paper stops at a number of triangles, and this stops at an error. Its default weight of 1e-3 moves vertices off the planes far enough to leave vertices on large flat faces that are not needed, which is why the default here is a hundredth of it.

**Detail:** the collapses are kept in the order they were made, with the contour as it came, which makes a progressive mesh (Hoppe, 1996). The mesh after any number of them is had without simplifying again, so the detail can be changed at once. In between is where the line quadrics show: a flat face part-way down has its vertices spread evenly with them, and two fifths thin triangles without.

**Changing a setting without contouring:** `Voxelizer.simplify( )` shows the last contour again under the simplification settings as they are now. Another detail is a walk through the collapses; another error or line weight simplifies the kept contour again. The interactive editor does this by itself.

**Cost,** on the three balls with Burst, in the editor:

| Cells | Triangles | Simplified | Simplifying | Another detail |
|---|---|---|---|---|
| 32³ | 6,428 | 1,330 | 9 ms | under 1 ms |
| 64³ | 26,036 | 2,510 | 39 ms | 1 ms |
| 128³ | 104,500 | 4,588 | 206 ms | 3 to 6 ms |
| 256³ | 417,876 | 8,848 | 1.2 s | 11 to 24 ms |

- **Against the contour:** simplifying takes about seven times as long as the Burst contour it follows. Most of it is the loop that collapses one edge after another, which runs on one core.
- **After an octree** there is little left to do: 3 to 4 ms at 128 leaves across.
- **Memory:** the contour and its collapses are kept until the next contour, 4 MB at 128³ and 17 MB at 256³.

Example scenes are in `Assets/Scenes/Mesh Simplification`, each a copy of a Burst scene on 64³ cells with simplification on and the wireframe drawn, so the triangles show.

## Remeshing
Every voxelizer can rebuild the mesh of its contour before it is shown, after mesh simplification if that is on and before its normals are creased or split. It works on the finished mesh, so it serves every algorithm and implementation alike. Two techniques are built, both Burst code:

- **Isotropic** follows section 4 of *A Remeshing Approach to Multiresolution Modeling* (Botsch and Kobbelt, 2004): a loop that splits long edges, collapses short ones, flips edges towards vertices of six and smooths, about five times over. Its triangles come out close to equilateral and all about one size, and it keeps the mesh's sharp edges, open edges and lines between materials as lines, and its topology as it was.
- **Instant Meshes** follows *Instant Field-Aligned Meshes* (Jakob, Tarini, Panozzo and Sorkine-Hornung, 2015): a field of directions and a field of lattice points are smoothed over the surface, and a mesh is read off them. Its faces are triangles, mostly quads, or only quads, in rows that follow the shape's edges and curvature.

A third technique, *QuadriFlow* (Huang, Zhou, Niessner, Shewchuk and Guibas, 2018), quads with fewer irregular vertices, is planned and not built.

Its settings are in the **Remeshing** section of the inspector:

| Setting | Default | What it does |
|---|---|---|
| Technique | Off | **Off**, **Isotropic** or **Instant Meshes**. Off, every mesh is what it was without this feature. |
| Edge Length | 1 | How long the new edges are, against the mesh the pass is given. At 1 the result has about as many vertices as that mesh, at 0.5 about four times as many and at 2 about a quarter. |
| Feature Angle | 40 | Two triangles that meet at more than this many degrees have a sharp edge between them, which is kept. Open edges, edges of more than two triangles and edges between materials are kept at any angle. |
| Iterations | 5 | Isotropic only. How many times its loop runs: the paper's "about 5". |
| Area Iterations | 5 | Isotropic only. How many rounds of smoothing by the vertices' areas follow the loop, which even out the sizes of the triangles. Zero leaves them out. |
| Faces | Quad Dominant | Instant Meshes only. **Triangles**; **Quad Dominant**, quads with triangles and pentagons where rows of faces begin and end; or **Quads**, quads only. |
| Normals | Input | The normals the remeshed mesh starts from, before they are creased or split. **Input:** those the mesh it was given would show creased at the Crease Angle, where each vertex lies on it. **Faces:** those of each vertex's own triangles. Split vertex normals take Faces whatever this says. |

**The edge length goes by the mesh it is given.** At 1 it is the length at which equilateral triangles, or squares, of that mesh's area would have its number of vertices.
- **On a contour** that is about a cell.
- **After simplification** it is several cells, more on flat shapes: the cube the simplifier leaves at 8 vertices is remeshed with edges of about 15 cells at 64³. So the simplifier's Error and Detail decide how many vertices there are, and the remesh how they are spread.

**What it gives,** on 15 test models by every algorithm with Burst at 64³ cells, at the defaults, medians over the meshes:

| Technique | Mesh | Vertices against the mesh given | Faces | Regular inner vertices | Smallest angle of a face, mean |
|---|---|---|---|---|---|
| Isotropic | contour | 1.01 | triangles | 85% | 53° |
| Isotropic | simplified | 1.23 | triangles | 66% | 45° |
| Instant Meshes, Quad Dominant | contour | 1.00 | 98.5% quads, 1% triangles, 0.5% pentagons | 98% | 87° |
| Instant Meshes, Quad Dominant | simplified | 1.02 | 94% quads, 4% triangles, 2% pentagons | 92% | 86° |
| Instant Meshes, Triangles | contour | 1.01 | triangles | 94% | 57° |
| Instant Meshes, Quads | contour | 1.01 | quads | 99% | 87° |

- **Regular** is six edges at a vertex for triangles and four for quads. A contour has half its inner vertices of six edges, and a simplified mesh a third to two fifths.
- **The best angles** are 60° for a triangle and 90° for a quad.
- **Triangles are Instant Meshes' weakest faces:** a triangular lattice meets these models' right angles badly, and where it does not fit, larger faces are cut into triangles of whatever shape is left, down to slivers.

**What both keep:**
- **The surface:** every vertex is on the mesh they were given, to within rounding. With simplification on that is the simplified mesh, so a curved shape keeps the simplifier's facets, with new vertices spread over them.
- **Features:** edges by the Feature Angle, and open edges, edges of more than two triangles and lines between materials at any angle. Features by angle are found at the scale of the edge length, but never more than a cell, which clears the noise a contour makes where an edge crosses the grid at an angle.

**Isotropic** keeps more:
- **Lines and corners:** a vertex on a line moves only along it, and a corner does not move. A line between two materials stays exactly where it was.
- **Topology:** nothing joins, splits, opens or closes, as with the simplifier. Its collapses go through the same check.
- **What it does not keep:** details smaller than the edge length, such as small hollows on a simplified flat face. Corners found by angle that are closer than four fifths of it can become one. Folds in the mesh it is given stay folded, and no move turns a triangle over.
- **Simplified flat shapes** can get an edge length longer than their own edges. Two overlapping cubes, simplified to 18 vertices at 64³, get edges of 28 cells, and lose their shape. At an Edge Length of 0.25 their edges are 7 cells long, and they keep it exactly, the line between their materials too.

**Instant Meshes** keeps less:
- **Lines:** rows of faces run along open edges and lines between materials, which hold both fields. On contours a line between materials stays within 0.004 of a cell of where it was on the median model. On a simplified mesh, whose edges are many cells long, it moves by about a quarter of a cell on the mean, and by up to a cell.
- **Sharp edges** by angle steer the fields, so rows of faces run along them. Corners are not held, and end up near their place, not at it.
- **Topology is not kept,** as its paper says. Faces are walked round the edges read off the fields, and holes of up to six sides are filled. On 11 of the 60 contours above it opened lines of edges the mesh did not have, all on the turned shapes, the new shapes and the three balls. The inspector warns when this happens.
- **Edges of four triangles,** which Uniform and Adaptive Dual Contouring give where two sheets of surface meet, are split apart first, as the fields need. Such a contour comes out as a manifold whose sheets touch along a line.

**When it gives up,** it shows the mesh as it came, and the statistics and the editor's inspector say why: Isotropic where its edge length would make more than 2²⁰ vertices, Instant Meshes where the mesh its fields are on would have more than 2²¹, and where no faces come out of its fields. Quads alone give up on the smallest simplified meshes, such as a slab of 16 vertices, whose fields at twice the edge length have no faces in them.

**The wireframe** draws the faces the remesh made: a quad without its diagonal, a pentagon without its two.

**Changing a setting without contouring:** the voxelizer keeps the mesh it last gave the pass, whether remeshing is on or not, and `Voxelizer.remesh( )` shows it again under the remeshing settings as they are now. `Voxelizer.simplify( )` remeshes what it makes. The interactive editor does both by itself.

**GPU contours** are read back to be remeshed, and are not drawn straight from the GPU. Their vertices come in a different order every time, so two contours of the same model remesh alike and not the same.

**Cost,** the remesh alone with Burst, in the editor, the median and the slowest of 15 test models by Uniform Dual Contouring, in ms:

| Cells | Mesh | Isotropic | Instant Meshes, Quad Dominant | Instant Meshes, Triangles | Instant Meshes, Quads |
|---|---|---|---|---|---|
| 32³ | contour | 57 (140) | 170 (306) | 231 (419) | 58 (97) |
| 32³ | simplified | 8 (57) | 26 (81) | 34 (106) | 12 (33) |
| 64³ | contour | 209 (505) | 630 (1,207) | 842 (1,624) | 189 (340) |
| 64³ | simplified | 17 (117) | 44 (146) | 59 (186) | 19 (56) |
| 128³ | contour | 834 (2,113) | 2,608 (5,474) | 3,366 (6,828) | 741 (1,521) |
| 128³ | simplified | 25 (192) | 77 (248) | 94 (310) | 29 (88) |

- **After simplification,** which is what the pass is for, a remesh on most models takes less than the simplifying before it, which is 206 ms for the three balls at 128³.
- **Without it,** a remesh takes many times the Burst contour, which is about 30 ms at 128³.
- **Instant Meshes' fields** are most of its time. They live on the vertices of the mesh it is given, split until no edge is longer than half the edge length, which is about twelve times as many vertices. A looser split would be quicker, and was measured to leave more holes.
- **Quads alone** are solved at twice the edge length, on a quarter of the vertices, and are the quickest.

Example scenes are in `Assets/Scenes/Remeshing`, each a copy of a Uniform Burst scene on 64³ cells with creased normals and the wireframe drawn, so the faces show:
- **Isotropic Sphere** and **Isotropic Multiple Materials**, the second simplified first and remeshed at an Edge Length of 0.25;
- **Instant Meshes Union**, a cube and a ball as quads with triangles and pentagons, which leaves a small hole where the ball meets an edge of the cube;
- **Instant Meshes Multiple Materials**, the same with its line between materials held;
- **Instant Meshes Quads Sphere**, quads only.

## Manifold Dual Contouring
`ManifoldDualContouring` follows *Manifold Dual Contouring* (Schaefer, Ju and Warren, 2007). Every piece of surface in a leaf of the octree has a vertex. On the way up the octree, the vertices that the surface connects inside a cell are clustered into one. A cluster stands in for the vertices below it when its error is within the threshold and, with Enforce Manifold, its piece of surface is a disk that crosses the edges of each face of its cell 0 or 2 times. However far that goes, the mesh stays closed and manifold.

| Setting | Default | What it does |
|---|---|---|
| Error Measure | Sum | What a cluster's error is measured as. **Sum:** the QEF at its vertex, the sum of the squared distances to the planes of its samples, as in the paper. **Distance:** the root mean square of those distances in cells, and never less than that of the clusters below it. |
| Error Threshold | 6e-12 | Only shown under Sum: the largest sum at which a cluster may stand in for its vertices. |
| Error Distance | 0.05 | Only shown under Distance: the largest distance, in cells. |
| Enforce Manifold | On | Lets a cluster stand in only for a disk, as above. Off, the error decides alone, and tubes and sheets can collapse into each other. |

**Why there are two measures:** the sum grows with the number of samples, and so with the size of a cluster. A threshold that the small clusters on a face pass refuses the large ones above them, and one that lets the large ones through lets small ones stray from the surface. The distance does not grow with size, so 0.05 of a cell means about the same at every resolution, as the Error of mesh simplification does. Distance is not in the paper.
- **The largest of the clusters below** keeps a feature that few samples see from being averaged away by the many around it. A dent a cell deep that four samples see is 0.4 of a cell in a cluster of 25.
- **Leaves count for nothing in it.** A leaf is never judged, and among its handful of samples one normal taken across a sharp edge is most of its mean. On a turned cube at 128³ cells, two leaves came to 0.1 of a cell that way, and every cluster above them would have been refused with them.

**A cube,** at 128³ cells with Burst, in vertices:

| Cube | Sum, 1e-5, error in floats | Sum, 1e-5 | Distance, 0.05 |
|---|---|---|---|
| 0.6 across, at the centre | 14 | 8 | 8 |
| 0.5 across, off the centre | 24 | 8 | 8 |
| 0.6 across, turned 45° about y | 54 | 48 | 48 |
| 0.5 across, turned about all three axes | 357 | 378 | 16 |

- **The error in floats** is how it was worked out before (see QEF Solvers). The fully turned cube's real error is not 0: its samples are found by a binary search and its normals by differences.
- **The fully turned cube under Sum** comes to 16 vertices only at 1e-3, a hundred times the threshold. At 64³ cells, Distance takes it to its 8 corners at 0.1 of a cell, each within a tenth of a cell of where it should be.

**Where clustering stops:** a cluster is a cell of the octree, so clusters on either side of a plane that divides the octree never merge.
- **The cube turned 45° about y** has its four upright edges in the planes that divide the root. All 48 cells of the level below keep a vertex, including 8 in the middle of faces.
- **A cube that spans three cells** along an axis keeps 12.
- **Mesh simplification** takes every one of these cubes to its 8 corners.

**On the models,** Distance leaves about as many vertices as Sum for the same accuracy. Its accuracy in cells changes less with the resolution. On the sphere at Distance 0.05, the middles of triangles are within 0.06 of a cell of the surface at 32³ cells, and 0.10 at 128³. At Sum 2e-4 the figures are 0.09 and 0.26.

The example scenes in `Assets/Scenes/Manifold Dual Contouring` use Sum, so their thresholds mean what they did. In the interactive editor, the Error Measure is among the Manifold voxelizer's settings below the editor.

## Occupancy-Based Dual Contouring
`OccupancyDualContouring` follows *Occupancy-Based Dual Contouring* (Hwang and Sung, 2024). It is Manifold Dual Contouring on a uniform grid, without simplification, and it only ever asks the volumes whether a point is inside or outside, never how far away the surface is. A point is inside where the volumes give it a material.

- **Edges:** where the contour crosses an edge of the grid is found by halving the edge.
- **Faces:** on every face of a cell that the contour crosses, three searches find the point where the surface's two straight pieces would meet on that face.
- **Vertices:** the plane through a point on an edge and the two points on the faces next to it takes the place of the gradient. The vertex of each piece of surface in a cell is the minimizer of the QEF of those planes.
- **Polygons:** each is split along the diagonal that keeps its triangles from passing through those of other polygons, or into four triangles around the point on its edge.
- **Manifold:** where two cells meet at a face that both would join their surface across, both take the other way of joining it, so every edge of the mesh has two triangles.

Its settings, beside those every voxelizer has:

| Setting | Default | What it does |
|---|---|---|
| Edge Search Iterations | 15 | How many times an edge is halved. |
| Face Search Samples | 4 | How many samples a search across a face takes to find where it first crosses the contour. |
| Face Search Iterations | 11 | How many times it then halves the step it found the contour in. |
| Keep Vertices In Cells | On | Moves a vertex that the QEF places outside its cell back into it. |
| Split Quads | On | Splits polygons so that triangles do not pass through each other. Off, always along the same diagonal, as the other algorithms do. |

Binary Search Iterations, Intersection Approximation and Surface Correction Iterations place points by distance. This algorithm does not use them, and its inspector does not show them.

How it compares with Manifold Dual Contouring on the same grid, measured on single shapes at 16³ cells:

- **Curved surfaces:** its vertices are ten to twenty times closer to the shape. On a ball they are 0.0016 of a cell off on average, against 0.039.
- **Flat faces, and edges that every face of the grid sees bend once:** exact, as on an upright cuboid.
- **Corners:** where a face of the grid sees three faces of a shape, the search on that face goes wrong, and the corner can be missed by up to 0.44 of a cell. Manifold Dual Contouring has a vertex within 0.03 of a cell of the corners its tests look at.
- **Keep Vertices In Cells** decides between two faults:
  - **On,** no triangles passed through each other on 27 test models at 16³ and 32³ cells, and four pairs did at 64³. Edges of turned shapes are blunted where they pass just outside a cell: one vertex in six to ten is moved, by up to 0.4 of a cell.
  - **Off,** edges are as sharp as the planes give them, and triangles pass through each other as they do in Manifold Dual Contouring: 436 pairs on the same models at 16³ and 1,849 at 64³. Split Quads then does nothing.
- **Materials:** each material has its sub-mesh, but vertices are not drawn to where two materials meet on the surface, so that boundary follows the grid.
- **Subtractions and materials:** a Subtraction takes away only the materials it names. Where one passes through a shape of another material, this algorithm leaves that shape whole. The other algorithms dent it there: they keep its triangles, but place their vertices by a density that every subtraction is taken out of. To cut a shape with any algorithm, give the Subtraction that shape's material as well.

## Interactive editor
`Assets/Scenes/Interactive Editor.unity` lets you build a model out of shapes and see it contoured as you edit, in edit mode, without pressing Play.

The **Interactive Editor** object holds the editor and a voxelizer for each algorithm. In its inspector:

- **Algorithm** picks Uniform, Adaptive, Manifold or Occupancy. The settings all voxelizers share, such as resolution, implementation, solver, vertex normals, mesh simplification and wireframe, carry over, so the same model is compared like for like. Resolution carries over as the number of cells across. An implementation the new algorithm does not have becomes Burst. The selected voxelizer's own settings, such as the Implementation, appear below the editor.
- **Shapes** lists every shape below the object, in the order their operations apply. Each row has an enabled box, the shape's name, which selects it, its operation (Union, Intersection or Subtraction) and material, and a remove button. Drag rows to reorder them. The **Add** buttons, one for each kind of shape, add a new one at the centre of the grid. Everything can be undone.
- **Contour** contours at once. The line below it says whether the mesh is up to date, and how long the last contour took.
- **Warnings** point out likely mistakes: a first shape that is not a Union, a shape without a material or outside the grid, a material the renderer has no material for, a shape a group would shear, a round or square shape whose x and z sizes differ, a debug view that draws so much it slows the editor down.

Shapes are read off their transforms the way Unity shows them: where they are, how they are turned and how large they are. A direct child of scale 1 fills the grid, whatever the scale of the editor object, and the box around the editor in the scene view is the grid. Groups, empty objects between the editor and its shapes, move, turn and scale the shapes in them.

One arrangement cannot always be contoured as Unity shows it. A shape turned inside a group that is scaled unevenly can come out with slanted sides, and a shape is always made from a box with square sides. Such a shape is contoured with square sides instead, and the inspector warns about it. A group scaled the same along every axis never slants anything, and neither does an uneven scale along the axes a shape is turned about or by quarter turns.

In the scene view, an outline shows each shape where and as it is contoured, so subtracted shapes can be seen and clicked. Outlines are green for Union, yellow for Intersection and red for Subtraction, and dimmer for shapes that are not selected.
- **Thickness:** **Outline Thickness** sets how thick they are, in pixels.
- **Depth:** outlines are hidden where the mesh is in front of them. They are drawn half a cell nearer the camera than they are, so that the mesh, which is only near the shapes, does not cut through them.
- **Outlines On Top:** draws them over everything instead. A click on an outline where the mesh is in front of it then still selects the mesh.

How the mesh follows changes depends on the implementation:

| Implementation | Contours by itself |
|---|---|
| Burst, GPU | After every change, from the inspector, a handle (moving, turning or scaling), the hierarchy or an undo. Contours that take up to 50 ms follow a drag live; slower ones wait until the changes settle for the **Debounce** interval. After a contour over a second, it waits for **Contour** to be pressed until a contour is fast again. |
| Managed | Never. Managed contours take from a tenth of a second at 16³ cells to most of a minute at 128³, so it contours when **Contour** is pressed. |

With **Mesh Simplification** on, the mesh follows its settings without contouring, with any implementation: another detail shows at once, and another error or line weight simplifies the last contour again. Turning simplification on or off is a contour, and waits for **Contour** with a managed implementation. The line under **Contour** then also has the size of the mesh before it was simplified.

With **Remeshing** on, a change to its settings remeshes the last mesh again without contouring, with any implementation, and so do turning it on and off and changing the technique. Another detail or error of the simplifier remeshes what it gives. The lines under **Contour** then also have the size of the mesh the remesh was given, the faces it made and how long it took, with a warning where it gave the mesh back as it came, or left lines of open edges or of edges of more than two triangles that the mesh it was given did not have.

Nothing contours by itself while Unity is not the active application, where everything runs many times slower. The mesh is not saved with the scene, so a Burst or GPU variant contours when the scene opens, and a Managed one waits for **Contour**.

## Code layout
Each algorithm is one component, such as `AdaptiveDualContouring`. Its class is split into partial class files: the component file, plus one file for each implementation.

| Folder | Contents |
|---|---|
| `Assets/Scripts` | The components, with their settings, the switch between implementations and their inspectors. Also everything the implementations share: the voxelizer, the volumes, the density function and surface extractor interfaces, the dual marching cubes table, and the sums in doubles that every QEF works its error out from (`QEFError.cs`). `DensityFunction.Frame` works out once how a shape is turned, and all three implementations take positions into the shape's frame with the numbers it gives. The nine volume components (`Cuboid.cs` to `HexagonalPrism.cs`), each with its distance function, are here, and `DensityFunction.sample` chooses between them for the managed code. The interactive editor (`InteractiveEditor.cs`, with its inspector and shape outlines) and the scheduler that decides when it contours (`RemeshScheduler.cs`) are here too, as is the wireframe (`Wireframe.cs`), whose shader is `Assets/Resources/Shaders/RenderContourWireframe.shader`. `MeshSimplification.cs` is what a voxelizer keeps of a simplified contour: the contour as it came and the collapses that simplify it. `Remeshing.cs` is what it keeps for remeshing, the mesh the pass was last given, with the switch between the techniques. |
| `Assets/Scripts/Managed` | The managed implementations (`<Algorithm>.Managed.cs`) and what only they use: the QEF solvers, the octree with its tables and topology checks, and the density, intersection and normal functions. |
| `Assets/Scripts/Burst` | The Burst implementations (`<Algorithm>.Burst.cs`) and the Burst code they share: ports of the QEF solvers, density functions, tables and topology checks, and the native grid, octree and mesh data. The searches of Occupancy-Based Dual Contouring are in `BurstOccupancy.cs`. The mesh simplifier is here as well, for every implementation: `BurstMeshSimplifier.cs`, with the error quadrics it adds up in `BurstQuadric.cs`. So is the pass that creases the normals of the managed and Burst implementations' meshes, `BurstCreases.cs`. So are the remeshing techniques: `BurstIsotropic.cs` and `BurstInstantMeshes.cs`, which reads its mesh off the fields of `BurstFields.cs`, and what they share: the feature skeleton (`BurstFeatures.cs`), the search for the nearest point of the mesh given (`BurstSurfaceGrid.cs`), the split into sheets (`BurstSheets.cs`), and the lengths, faces and normals out (`BurstRemeshing.cs`). |
| `Assets/Scripts/GPU` | The GPU implementations of Uniform and Occupancy-Based Dual Contouring (`<Algorithm>.GPU.cs`), the mesh readback they share, the layout they upload density functions in and the number the shaders know each QEF solver by (`QEFSolver.GPU.cs`). The compute shaders are in `Assets/Resources/Shaders`, one for each algorithm, with the shape functions both use in `DensityFunctions.hlsl` and the QEF solves in `QEF.hlsl`. Both crease their normals with `GPUCreases.cs` and its shader, `CreaseNormals.compute`. |

Code that only one implementation uses goes in that implementation's folder; code that several of them use stays in `Assets/Scripts`. `SurfaceExtractor`, `QEFSolver` and `DensityFunction` are partial interfaces for the same reason: their managed and GPU members sit in those folders, and their shared members sit in `Assets/Scripts`.

## Requirements
- **Unity 6000.4.5f1** or later
- **Unity Mathematics** (1.3.3)
- **Burst** (1.8.29) and **Unity Collections** (6.4.0) for the Burst implementations
- Compute Shader support for the GPU implementation
- **Shapes** library by Freya Holmer
- **Unity Test Framework** (1.6.0) and **Performance Testing** (3.4.0) to run the tests

## License
The MIT License (MIT)

Copyright (c) 2025 Thomas Pranic

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

---

This project implements academic algorithms for dual contouring. Refer to the original papers for algorithmic details:

- *Dual Contouring of Hermite Data* by Ju et al.
- *Dual Contouring: The Secret Sauce* by Schaefer and Warren.
- *Manifold Dual Contouring* by Schaefer et al.
- *Surface Simplification Using Quadric Error Metrics* by Garland and Heckbert.
- *Controlling Quadric Error Simplification with Line Quadrics* by Liu, Rahimzadeh and Zordan.
- *Progressive Meshes* by Hoppe.
- *Topology Preserving Edge Contraction* by Dey, Edelsbrunner, Guha and Nekhayev.
- *Occupancy-Based Dual Contouring* by Hwang and Sung.
- *Fast and Robust QEF Minimization using Probabilistic Quadrics* by Trettner and Kobbelt.
- *A Remeshing Approach to Multiresolution Modeling* by Botsch and Kobbelt.
- *Instant Field-Aligned Meshes* by Jakob, Tarini, Panozzo and Sorkine-Hornung.

The linear solve of the probabilistic QEF solver (`ProbabilisticQEF.minimize`, `BurstQEF.minimizeProbabilistic` and `minimizeProbabilistic` in `QEF.hlsl`) is ported from `minimizer()` in the authors' [probabilistic-quadrics](https://github.com/Philip-Trettner/probabilistic-quadrics), under this licence:

> MIT License
>
> Copyright (c) 2020 Philip Trettner
>
> Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
>
> The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
>
> THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
Where *Instant Field-Aligned Meshes* is silent, Instant Meshes remeshing follows its authors' code, [Instant Meshes](https://github.com/wjakob/instant-meshes) (copyright 2015 Wenzel Jakob, Daniele Panozzo, Marco Tarini and Olga Sorkine-Hornung, under a BSD 3-clause licence). None of it is ported as a whole. What is taken from it is listed at the top of `BurstFields.cs` and `BurstInstantMeshes.cs`, and in `Documents/Plans/Remeshing.md`.
