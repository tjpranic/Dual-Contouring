# Mesh Simplification

Plan written 2026-11-13 for a mesh simplifier that runs on the finished mesh of any algorithm: quadric error edge collapse, with line quadrics and a progressive record of the collapses.

**Status:** complete. All eight steps are done. Step 0 (2026-11-13) took the baseline and counted what the contours are like. Step 1 (2026-11-14) built `BurstQuadric` and measured floats against doubles. Step 2 (2026-11-14) built the mesh's lists, the tests of which collapses are allowed, and the collapse itself. Step 3 (2026-11-14) built the loop and the hook: every voxelizer has **Mesh Simplification** and **Simplification Error**, at a line weight of zero. Step 4 (2026-11-14) added the line quadrics and **Line Quadric Weight**, which starts at 1e-5 and not at the paper's 1e-3 (finding 24). Step 5 (2026-11-14) added the record of collapses, **Simplification Detail** and `Voxelizer.simplify`. Step 6 (2026-11-14) taught the interactive editor the settings and added four example scenes. Step 7 (2026-11-14) tuned the loop, took a fifth to a quarter off its time, and wrote the README section. All seven decisions were answered as recommended on 2026-11-14.

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

4. **Not every contour is a closed manifold.** Step 0 counted this (Measurements).
   - **Uniform and Adaptive** give edges with more than two triangles where a cell holds two sheets of surface. Every one of them has exactly four triangles, and such meshes also have vertices whose triangles form more than one fan.
   - **Manifold and Occupancy** give a manifold when nothing is clustered. Manifold with clustering at 2e-4 gave eight edges of four triangles, in 4 of 39 meshes, on the two models of turned shapes (finding 15).
   - **Every algorithm** leaves the mesh open where the surface leaves the grid, and nowhere else.
   - **What it means:** the mesh is kept as vertices with lists of their triangles and edges, as Garland and Heckbert have it, not as half-edges. Legality is theorem B of Dey et al.

5. **In the ranking I said the quadrics could be seeded from the contourer's own planes. I no longer recommend that.**
   - **They are not there to take.** Burst Uniform keeps only a vertex and a normal per cell. The GPU implementations do not read their QEFs back. Occupancy's planes are fitted through searched points. Only Adaptive and Manifold keep a QEF for each vertex.
   - **The line quadric's weight is defined against area-weighted triangle quadrics** (equations 8 and 16). Hermite QEFs count samples, not area, so the published weights would not carry over.
   - **Triangle planes are what both papers use,** and they need nothing but the mesh. See decision 2.

6. **Floats are not enough for a quadric kept about the origin.** The constant term of a vertex 100 cells from the origin is near 10⁴ times its area, where a float resolves about 10⁻³. The costs to be ordered on a flat region are zero or close to it. The Burst plan met the same limit in the octree's error test. Doubles cost Burst nothing in speed here, and the simplifier has no managed or GPU version to match. See decision 3.
   - **Step 1 measured it** (Measurements). On a ball of 128³ cells the cost of an edge is a distance of 0.009 cells, and a float quadric is off by 0.011. 86% of its costs are more than half wrong and a fifth are below zero. A double quadric is off by about a millionth of a cell at most.
   - **The speed of doubles was not measured against floats:** there is no float simplifier to time. Step 3's times are of doubles.

7. **With line quadrics the cost of a collapse is no longer a distance to the surface.** On a flat face of a cube it is the line term alone, and it grows as the face is merged down to two triangles. A bound on the whole cost would stop a cube far short of twelve triangles.
   - **So the two sums are kept apart,** twenty numbers a vertex: the heap orders by their sum, as the paper has it, and the bound is tested against the plane part alone.
   - **This is my addition.** The paper stops at a triangle count and does not need it.
   - **Step 3 already keeps two sums apart for another reason** (finding 21), so with line quadrics a vertex has three.
   - **Step 4 built it so.** A flat face comes down to two triangles at any error bound, with its vertices at the centre of what they stand for on the way. But the lines do one thing more that the bound does not let pass (finding 24).

8. **The bound can be a length.** A vertex's plane quadric, divided by the area it stands for, is the mean squared distance to its planes weighted by area. Its root is a distance, which the setting gives in cells, as the project's tolerances are. The division is also my addition.
   - **It is a distance to planes and not to the shape.** Step 3 measured the second: a vertex ends up at most about three times the error from the shape, and so does the middle of a triangle (finding 22).

9. **Occupancy's triangles may cross after simplification.** Its split of every polygon is chosen so that triangles do not pass through each other, and a collapse undoes that choice. The inversion test catches a fold around the collapsed edge and nothing further away. Step 3 counts crossing pairs before and after with `MeshIntersections`. See decision 6.
   - **Step 3 counted them** (Measurements). Of seven models at 64³, one gets crossing pairs it did not have: the turned shapes, 9 pairs among 1,680 triangles at the default error. The other six have none before or after, but for the new shapes, whose contour has 4 and keeps them up to an error of 0.05.
   - **Uniform and Manifold contours cross themselves to begin with,** by hundreds of pairs on models with creases, and simplifying takes most of those away.

10. **The collapse loop is serial.** Each collapse changes the costs around it, so the heap is worked one edge at a time. Building the quadrics and the first costs is parallel.
    - **An estimate and nothing more:** taking 52,000 vertices to their floor is about 50,000 collapses, each recomputing about ten edges with a solve, an inversion test and a heap move. That is perhaps 50 to 150 ms at 128³ and a quarter of it at 64³.
    - **The estimate was too low.** Step 3 measured 248 ms for the three balls at 128³ and 48 ms at 64³ (Measurements): 4.4 microseconds a collapse, and eight times what the contour itself takes. Nothing was tuned; that is step 7.
    - **What it means for the editor:** with simplification on, a drag is followed live up to 32³, where it takes 11 ms. At 64³ the contour and the simplification together are just over the 50 ms limit, and the editor waits for changes to settle. The scheduler does that by itself.

11. **A changed simplification setting needs no new contour.** If the voxelizer keeps the unsimplified mesh, a new error bound is a new run of the simplifier, and a new detail is a walk through the record.
    - **Step 5 built and measured it** (Measurements). At 128³ a new detail takes 3 to 6 ms where a simplified contour takes 286. A new error takes 259, which saves the contour and nothing else.
    - **Turning simplification on is not such a change:** nothing is kept of a contour that was not simplified, and a GPU contour drawn straight from its buffers never reached the CPU. `simplify` then gives nothing and the caller contours.

12. **The order must not depend on chance.** Equal costs are common on flat regions when the line weight is zero. Ties go to the lower edge index, so the same mesh always simplifies the same way and hashes can be compared between runs.
    - **Step 3 puts the area before the index:** of edges that cost the same, the one whose two ends stand for less area goes first. Every edge of a flat face along the grid costs exactly nothing, and going by the index alone one vertex takes its neighbours one after another, with every edge of it worked out again at each collapse. A cube at 128³ took 2.9 seconds that way and takes 0.17 with the area first. On a cube at 64³ a collapse works out 7 edges again, not 59.
    - **The GPU's contours do not simplify the same way twice.** Its vertices come in a different order every time, and the order still decides between edges that cost the same and stand for the same area. The meshes are alike and not equal. A CPU contour simplifies the same way every time, which a test holds it to.

13. **Garland and Heckbert do not say how large the "large penalty" on boundary planes is.** Step 2 measures what keeps a straight boundary between two materials straight and a round one round, and sets it.
    - **What the weight multiplies is my choice:** the plane through a sharp edge that stands on one of its triangles counts as that triangle's own plane does, a third of its area, times the weight. A weight of 1 then means that a line may leave its place sideways as far as the surface may leave its planes.
    - **Step 2 measured single collapses** (Measurements). Any weight above zero keeps the new vertex on the line. At zero a collapse into a line moves it by up to 1.4 cells where it is straight and 2 where it is round. Above zero the weight only sets what rounding a curved line costs, which grows with its root.
    - **It is 1 for now.** Whether a whole simplification keeps the lines needs the loop, so step 3 measures the lines after simplifying at 0.1, 1, 10 and 100 and settles it.
    - **Step 3 settled it at 1,** and found that the weight was the wrong thing to ask of (finding 21). With the lines held to the bound by themselves, weights from 0.1 to 100 give the same lines to within a few edges.
    - **Occupancy's lines between materials are less exact:** 0.38 cells from the straight line and up to 0.67 from the round one, where Uniform's are within 0.03. At a weight of 1 a tenth of the collapses along its round line cost 0.29 cells, so a small error bound will refuse them. Step 3 shows what that does to its triangle counts.

14. **Degenerate triangles have no plane.** A triangle without area adds nothing to a quadric, and a vertex whose faces cancel has no normal for its line. Liu et al. give such a vertex a point quadric of weight 1e-6 (section 4.2), which is used here too.
    - **Step 0 found no triangle without area** in any contour, and no triangle that uses a vertex twice. The smallest triangle is 4e-7 of a cell's face, so areas and planes are worked out in doubles.
    - **Vertices without a normal do occur.** Occupancy gives a zero normal to up to 162 vertices of the cut-away sphere, all on its flat cut, which lies in a plane of the grid. Their triangles do not cancel. Step 4 decides whether the line of such a vertex runs along the normal of its triangles, as the paper has it, or the vertex gets the point quadric.
    - **Step 4 decided: the line of every vertex runs along the normal of its triangles,** the sum of their normals weighted by area, which is equation 12 of the paper. The normals the contour came with are for shading and play no part. Occupancy's 162 vertices then have a line like any other.
    - **The point quadric** is for a vertex whose triangles' normals cancel exactly, at a millionth of its area. Two triangles back to back are such a case, and their vertices are locked anyway (finding 15).

15. **A contour can hold two triangles on the same three vertices.** Step 0 found this in Manifold with clustering at 2e-4, on the two models of turned shapes: 15 such pairs in 4 of 39 meshes, each pair back to back. Fourteen of them make seven quads with two sides and nothing inside, each a piece of its own, and one hangs on the main surface. Each quad's diagonal, and one edge of the last pair, has four triangles. Those models' meshes also hold vertices that no triangle uses, 46 in all.
    - **Why it matters:** the link conditions are stated for a complex, where three vertices have at most one triangle. They say nothing about such a pair.
    - **What is done:** no edge that touches a vertex of such a pair is collapsed, so the pair leaves the simplifier as it came. Vertices without triangles are passed over, and the mesh out holds only vertices that a triangle uses.
    - **No collapse makes such a pair:** the link conditions refuse the collapse that would.

16. **The lines between materials end and meet.** Hoppe's tests are written for curves that may end at a vertex or meet there in threes, and both happen:
    - **Ends at open edges:** where a band of a second material leaves the grid with the surface.
    - **Ends and meetings away from them:** in Uniform and Adaptive, on the two models of turned shapes: 37 ends and 46 meetings of three or more over 39 meshes. 55 of their edges of four triangles also lie between materials.
    - **Manifold and Occupancy without clustering** have neither away from open edges. Manifold at 2e-4 has three meetings.

17. **A vertex can have many triangles.** Up to 15 in a plain contour and 28 after Adaptive or Manifold have simplified. The lists of a vertex have no fixed size.

18. **Thin features come as islands.** Where Uniform joins two sheets along an edge of four triangles, Manifold and Occupancy keep them apart, and on the turned shapes at 128³ that leaves 26 closed pieces of 12 to 20 triangles beside the main one. With decision 5 none of them goes below a tetrahedron.

19. **At a line weight of zero, most edges of a flat model have no best point, and some best points are far from their edge.** Step 1 solved the quadric of every edge of five contours at 128³ (Measurements).
    - **A cube on the grid's axes:** 106,674 of its 106,722 edges have none. The fallbacks of Garland and Heckbert are then the usual case and not the exception.
    - **A turned cube:** every edge has one, because its faces are flat only to within rounding. All of them are within two edge lengths of their edge.
    - **Curved models:** a few best points are far away. On a flat ellipsoid 56 are more than ten edge lengths from their edge and one is 111 away. On the three balls two are.
    - **What it means:** the paper has no rule against such a point, and its cost is low by definition. Step 3 counts how many of them the inversion test refuses and says what happens to the rest. With line quadrics every system is well-conditioned, which step 4 measures.
    - **Step 4 measured it** (Measurements). At any weight above zero, down to 1e-6, no edge is without a best point in any model, where a cube at 128³ has 247,000 such edges at zero. The best points far from their edges go with the weight: 25 on the three balls at 128³ at zero, 22 at 1e-5, 4 at 1e-4 and none from 1e-3.

20. **The two papers leave a gap between them, and step 2 closes it with a rule of its own.** Hoppe's tests are for manifolds and those of Dey et al. know no materials. Where a mesh is not a manifold and has lines between materials, both allow collapses that change where the lines are on the surface:
    - **A pinched vertex moved onto a meeting of three materials, or onto a line it was beside.** The mesh keeps its topology and the lines keep theirs, and still two pieces of one material that were apart now touch.
    - **How it was found:** greedy collapsing of the Uniform and Adaptive contours of `newShapes` took the pieces of a sub-mesh from four to two. A box with a pyramid standing on its tip on a line between materials shows the same by hand.
    - **The rule:** a vertex is a point of its own, which no collapse may move, when it is of order 2 or when sharp lines end or meet at it. Refused are an edge between two such points, and an edge that is not sharp from a vertex of order 2 to a vertex on a sharp line. It is checked after the papers' tests, and reported as its own reason.
    - **What it costs:** on those two contours at 32³ it refuses an edge 21 and 24 times, beside about 2,600 and 1,600 collapses made.
    - **It has no proof.** The tests hold every mesh they collapse to its pieces, its Euler characteristic, the same two for each sub-mesh, and the counts of its special vertices and lines.

21. **A sharp line needs an error bound of its own.** This is a flaw in my design that step 3's measurement found, and the largest change to it.
    - **What went wrong:** the bound divided a vertex's whole error by all the area it stands for. On a flat face every collapse costs nothing, so a vertex of a line across the face comes to stand for a great deal of face and very little line, and no error of the line was ever over the bound. A circle between two materials on a flat face, of 12.8 cells' radius, came out with 12 sides and 1.3 cells off at an error of 0.05. Raising the weight to 100 only made it 0.14.
    - **What is done:** the planes of the sharp edges are a second quadric with an area of its own, the thirds of the triangles they stand on. A collapse has to keep the surface within the error over the surface's area, and the lines within the error over theirs. The heap still orders by the sum of both, and the new vertex is still the best point of both.
    - **What it gives:** the same circle keeps 62 of its 204 edges, with its vertices within 0.09 cells and the middles of its edges within 0.14, which is the three errors a curved surface gets.
    - **It is thirty numbers a vertex with line quadrics,** not twenty (finding 7).

22. **The error is not how far the mesh is from the shape.** Step 3 measured that on nine shapes whose distances are known, upright and turned, at 64³ (Measurements).
    - **Vertices:** at most 2.1 to 3.2 times the error from the shape, at every error from 0.02 to 0.25 and for every algorithm.
    - **Middles of triangles:** at most 3.3 times.
    - **Corners:** a corner can slide along an edge it belongs to. Its vertex stands for little of the third face while the edge next to it has been collapsed into a vertex that stands for much of the other two, and the two can then collapse to a point a little way down the edge. A corner of the hexagonal prism ended 0.29 cells from its place at an error of 0.1, still on the shape.
    - **The tests hold vertices to four times the error,** or to the distance the contour's own vertices already have.

23. **A job works on a copy of what it is given.** The connectivity counted its stamps in a field of the struct. The build job counted in its copy, the collapse job started again from zero in another, among the stamps the first had left, and took old notes for new: on a capsule at 32³ the loop then read past the end of a list. Step 2's tests ran both in one job and could not see it. The counter is now in memory that every copy shares.

24. **The lines pull vertices off the planes, and a simplifier that stops at an error pays for that in triangles.** This is what step 4's measurement found, and why the default weight is not the paper's.
    - **What the lines do:** they hold a vertex to where the vertices it stands for were. Inside a flat face that costs nothing. At an edge or a corner it moves the vertex off the planes it should be on, by about the weight times the distance to the centre of what it stands for. The corners of Occupancy's upright cuboid at 64³, which the contour has to within 0.00004 cells, end 0.0001 cells off its faces at a weight of 1e-5, 0.001 at 1e-4 and 0.011 at 1e-3.
    - **What the bound makes of it:** that distance counts against the error, as it should. The last collapses of a large flat face are the ones with the longest pull, and they are refused. A cube then keeps a vertex in the middle of every edge and every face, 26 in all, and the slab with a band up to 58 where 16 are enough.
    - **How large:** a flat face comes down to its corners while the weight times its size in cells is under a quarter to a half of the error for a thin slab, and under the error for a cube (Measurements). At the paper's 1e-3 and the default error that is 12 to 50 cells: the slab at 64³ already keeps 22 vertices.
    - **The paper does not meet this.** It stops at a number of triangles, and its weight trades the shape of the triangles against the distance to the surface. Here every bit of distance is a collapse not made.
    - **So the default is 1e-5.** By that rule it is good for faces of 1,250 cells and more at the default error. The largest measured is a slab of 205 cells at 256³: it comes down fully at errors of 0.02 and 0.05, and keeps 20 vertices at 0.005. At 1e-4 it keeps 20 at the default error. It still does what a small weight is for: every edge has a best point, and flat models take a fifth to a half less time.
    - **A way around it that I did not take:** place the vertex by the planes alone, with the lines only choosing among points that are equally good, and let the weight decide nothing but the order. Large flat faces would then come down at any weight. It would no longer be equation 16, and the setting would no longer be the paper's.

25. **On a mesh simplified until nothing is left within the error, the lines hardly change the triangles.** The even triangles of the paper's figures are of meshes stopped at a count, with vertices left inside the flat regions. Here a flat face keeps none at any small weight, since no collapse on it costs any distance.
    - **Measured** (Measurements): the mean smallest angle and the share of thin triangles are the same from 0 to 1e-4 to within what one run differs from another. From 1e-2 the triangles get more even and more numerous: the slab with a disc goes from a mean smallest angle of 8° to 17° and from 136 triangles to 202.
    - **Where the order will show** is in the meshes between the contour and the simplest one, which step 5's Detail setting picks: there the flat regions are part-way down.
    - **Step 5 measured that, and there the lines do what the paper shows** (Measurements). A cube at 64³ with 2% of its collapses left to make has a mean smallest angle of 18° and 38% thin triangles without lines, and 43° and none at the default weight. The slab with a disc has 5° and 89% against 35° and 8%. Round models are the same either way. The default weight is enough for it; 1e-3 gives the same.

26. **What a voxelizer keeps is all in managed arrays, and the mesh is always made from the record.** Two things step 5 built otherwise than this plan had them.
    - **No `Mesh` is kept.** The plan had the unsimplified contour as a `Mesh`. A script reload forgets the voxelizer's hold on it, and nothing else holds a mesh that is not shown, so it would have been left behind. The contour is kept as arrays of positions, normals and triangles beside the collapses: 4.3 MB at 128³ and 17 MB at 256³.
    - **One way to the mesh.** The loop no longer writes its mesh out. The mesh after all collapses is made from the record like any other level, and a test holds the record to the connectivity: after k collapses, for several k, both give the same vertices in the same order and the same triangles.
    - **Not parallel.** The plan had the mesh at a level as parallel passes. It is one job on one core, 3 to 4 ms at 128³ with the mesh built, and I left it at that.

27. **Where the time went, and what tuning took off it.** Step 7 put profiler markers into the loop for one run. Of the 240 ms it took on the three balls at 128³: taking the edges of both ends out of the heap 47, the collapse itself 58, checking that it is allowed 39, working the edges out again and putting them back 41, taking the cheapest 23, looking for refused edges 13. Of the 26 ms that building the lists took, 19 were the order of each vertex.
    - **The heap keeps what it orders by.** An edge in it has its cost and the area of its ends beside it, where the loop used to look both up for every comparison, in three arrays. An edge of the kept vertex that is in the heap moves to its new place and is no longer taken out and put back. The loop went from 230 ms to 191.
    - **The order of a vertex among edges of two triangles** is found by going round its triangles once, each edge keeping its two triangles as one number. The general way, which sorts them into sheets, is left for vertices of a boundary. Building the lists went from 26 ms to 19.5, and the loop to 180.
    - **Every mesh is the same.** 236 simplifications were hashed collapse by collapse before and after: four algorithms on nine models at two errors and three weights at 32³, and ten at 128³.
    - **A test for what the heap is for** came with it: at every collapse of five meshes, the edge collapsed is the cheapest one that is within the error and allowed, found by looking at all of them.
    - **What is left** is the collapse, the check and the evaluation, about 50 ms each, which follow lists through memory. I see no change there that is as contained as these two.
    - **Not done:** placing the vertex by the planes alone (finding 24), which is a change to the meshes and not to the time, and which was not asked for.

28. **The editor paused simplifying after a slow managed contour.** Step 6 gave the simplifier's scheduler the time of the whole contour to start from, on the argument that simplifying cannot take longer. For a managed contour that is seconds of contouring around milliseconds of simplifying, and over a second it paused simplifying by itself. A test of step 6 failed on it once, in step 7's last run, when a managed contour at 32³ took just over a second.
    - **What is done:** a contour times its simplification by itself (`ContourStatistics.simplificationMilliseconds`), and the scheduler starts from that.
    - **A test that does not depend on chance:** a managed contour at 64³, which takes seconds, is simplified again by itself.

## Design

### Settings

On `Voxelizer`, so every algorithm has them, carried over by the editor's `select` and part of its signature:

| Setting | Default | What it does |
|---|---|---|
| Mesh Simplification | Off | Simplifies the mesh of every contour. |
| Simplification Error | 0.05 | How far a collapse may leave a vertex from the planes of the surface it stands for, as a root mean square distance in cells. Set in step 3: it takes the three balls at 128³ from 104,500 triangles to 4,644, with every vertex within about 0.15 cells of the shape. |
| Line Quadric Weight | 1e-5 | The weight of the line quadrics. Zero gives Garland and Heckbert's algorithm as written. Set in step 4: the paper's 1e-3 leaves vertices on large flat faces that are not needed (finding 24). |
| Simplification Detail | 0 | Which mesh of the record is shown: 0 is the simplest the error allows, 1 is the contour itself. In between it is a share of the collapses, and so of the vertices taken away. Built in step 5. |

### Code

- **`Assets/Scripts/Burst/BurstQuadric.cs`:** a quadric in doubles. From a plane, from a triangle with its area, from a boundary edge and one of its triangles, and from a vertex and its normal as a line. Add, evaluate, and the point of least error. Built in step 1:
  - **The line quadric** is written as I − n nᵀ. Equations 13 to 15 add two planes through the line with normals x and y, and x xᵀ + y yᵀ is that matrix for any such pair, so no x has to be chosen. A test holds it to the two planes.
  - **The point of least error** is solved by elimination, as a Cholesky decomposition does it, starting with the coordinate that has the largest entry on the diagonal. A quadric has no such point when a pivot is under 1e-10 of the largest. At that limit rounding moves the solution by about a millionth of the grid.
  - **Not by cofactors:** my first version divided cofactors by the determinant. On an exactly flat patch both are rounding noise, and their ratio said the system was well-conditioned. I saw it before the tests ran. Put back in afterwards, it fails four of them.
- **`Assets/Scripts/Burst/BurstMeshSimplifier.cs`:** the connectivity, the legality tests, the heap, the loop and the record, as Burst jobs. It takes positions, normals and `BurstMesh.Triangle`s and gives the same back. Built in step 2, as `BurstMeshSimplifier.Connectivity`:
  - **The lists:** every vertex has a linked list of its triangle corners and one of its edge ends, kept in arrays. Edges are records of their own, made once at the lower of their two vertices, and their index is what ties will be broken by (finding 12).
  - **Kept per edge:** how many triangles it has and why it is sharp. **Per vertex:** its order (Dey et al., section 3), how many sharp edges it has, whether it is locked (finding 15), its area and its quadric.
  - **`check`** gives the reason an edge may not collapse: locked, one of the five ways to break the link conditions, one of Hoppe's tests, the rule of finding 20, or a triangle that would turn over or lose its area.
  - **`collapse`** keeps the lower of the two vertices. It works out again the triangle counts and sharpness of the kept vertex's edges, its order, and the sharp counts of the vertices around it. The orders of the vertices around it are left alone: a collapse that the link conditions allow does not change them, and the tests compare with a mesh built from nothing.
  - **Hoppe's tests run on all sharp edges together:** open, of more than two triangles, and between materials.
  
  Built in step 3:
  - **`simplify( mesh, error )`** reads a mesh of any implementation and gives a new one, through six stages with a profiler marker each: read the mesh, build the lists, quadrics (parallel), first costs (parallel), the loop, write the mesh.
  - **Two quadrics and two areas per vertex** (finding 21).
  - **`targetOf`** is Garland and Heckbert's choice of the new vertex: the best point, else the best on the edge (`BurstQuadric.minimizeBetween`), else the middle or an end. The middle is taken unless an end is better by more than rounding, so that a flat face gives the middle whether it lies along the grid or not.
  - **The heap** holds only edges that are within the error bound and that `check` has not refused, ordered by cost, then area, then index (finding 12). The edges of both ends leave it before a collapse and the edges of the kept vertex go back in after.
  - **Refused edges come back** when a collapse leaves a vertex next to one of their ends, since that is all that can change what `check` says. The draft had them come back only with a new cost, which would have left edges behind that could collapse. A test counts what the loop leaves.
  - **`Statistics`:** collapses, how many of them put a vertex far from its edge (finding 19), and how many edges were worked out again.
  
  Built in step 4:
  - **Three quadrics per vertex:** the lines of the vertices it stands for are the third (findings 7 and 21). `QuadricsJob` takes the weight and gives each vertex its line along the normal of its triangles, or a point (finding 14). `collapse` adds them up like the other two.
  - **`evaluate`** places the vertex and orders the heap by all three, and holds the first two to the bound.
  - **`BurstQuadric.point`:** the squared distance to a point.
  - **`simplify( mesh, error, lineWeight )`.**
  - **`Statistics`** also counts the edges worked out again that had no best point. `targetOf` says whether it found one.
- **`Assets/Scripts/MeshSimplification.cs`:** the record as managed arrays, and the mesh at a level of detail. Built in step 5, as a class that holds one simplified contour:
  - **`of( mesh, error, lineWeight )`** simplifies a contour and keeps it with its collapses. **`under( error, lineWeight )`** simplifies the kept contour again.
  - **`levelOf( detail )`** is how many collapses a detail stands for, and **`meshAt( level )`** the mesh after that many.
  - **In `BurstMeshSimplifier`:** `Collapse`, which `CollapseJob` writes for every collapse it makes; `LevelJob`, the mesh after a number of collapses; `record` and `meshAt`, which `simplify` is now made of. `CollapseJob` can be told to stop after a number of collapses, which the tests use.
  - **The settings** are fields of `Voxelizer`, as step 3 put them.
- **`Voxelizer.contour`:** the hook, between the mesh arriving and vertices being split. Built in step 3:
  - **`lineQuadricWeight`** is the third setting, built in step 4.
  - **`cellSize`:** the size of a cell, which the error is given in. An octree has half as many leaves across as a grid of its resolution has cells.
  - **`contourNormals`:** what an implementation is asked for, shared normals when the mesh is going to be simplified. The GPU is the only one it changes anything for.
  - **The unsimplified mesh is destroyed** once it is simplified. Step 5 keeps it, as arrays (finding 26).
  - **`simplify( )`,** built in step 5 (What the voxelizer keeps).

### The algorithm

1. **Connectivity.** For each vertex its triangles and its edges. For each edge how many triangles it has, and whether its triangles are of different sub-meshes. Edges that are open, non-manifold or between materials are sharp.
2. **Quadrics,** in parallel. Each triangle adds its plane quadric times a third of its area to its three vertices, and each sharp edge its boundary planes to its two ends, in a sum of their own. The line quadric of each vertex, times the weight and its area, is a third sum.
3. **Costs,** in parallel. For every edge, the point of least error of the three sums of both ends, and the error there.
   - **Position:** with a line weight above zero the system is always solvable. At zero the fallbacks are the paper's: the best point on the edge, then the better of its ends and middle.
4. **The loop.** Take the cheapest edge. It is collapsed unless one of these refuses it:
   - **Error:** the plane part at the new position is over the bound (findings 7 and 8), or the part of the sharp edges is over theirs (finding 21).
   - **Topology:** the link conditions of theorem B fail.
   - **Materials:** one of Hoppe's six tests says the curves between materials would change.
   - **Inversion:** a triangle around either end would turn over.
   
   A refused edge leaves the heap and returns when a collapse leaves a vertex next to one of its ends. A collapse moves one end to the new position, gives it both ends' triangles, edges, sums and area, removes the triangles that lost a side, and recomputes the edges around it. It ends when the heap is empty.
5. **Normals.** The new vertex takes the sum of the two normals, each weighted by the area its vertex stands for, made unit length. Manifold's clusters add their members' normals the same way, without the weights.
6. **The record.** Each collapse writes the vertex kept, the vertex removed, and the position and normal the kept one got.

### The mesh at a level of detail

A level is a number of collapses. The mesh at that level is the unsimplified mesh with every vertex replaced by its ancestor after that many collapses (Hoppe, section 3.2), at the position its ancestor last got, without the triangles that then use a vertex twice. That is one pass over the record and one over the triangles, with no connectivity, and it is parallel.

- **Detail 0** is every collapse, and the mesh the loop ended with.
- **Detail 1** is none, and the contour itself.

As step 5 built it, in `LevelJob`: the record is gone through first to last for the positions and normals, and last to first for the ancestors, since the ancestor of a removed vertex is that of the vertex it went into, which the later collapses have settled by then. The vertices and triangles come out in the order `Connectivity.write` gives them. At detail 1 the mesh is the contour less the vertices it does not use and the triangles that use a vertex twice.

### What the voxelizer keeps

The unsimplified mesh and the record of its last contour, as a `Mesh` and managed arrays, so a script reload leaks nothing. They are dropped by `clear` and replaced by the next contour. As built, all of it is managed arrays (finding 26), and a contour that is not simplified or is empty leaves nothing kept.

- **`Voxelizer.simplify( )`** shows the last contour again under the settings as they are now: a new walk through the record when only the detail changed, a new run of the simplifier when the error or the weight did.
  - **It gives nothing** when there is no simplified contour to go back to: simplification is off or was off for the last contour, the last contour was empty, `clear` was called, or the scripts were loaded again. What is shown then stays as it is, and the caller contours.
  - **The error is in the cells of the contour that is kept,** whatever the resolution was set to since.
  - **It splits vertices for flat shading** as `contour` does, and leaves the debug view's data alone.
- **`ContourStatistics`** gains the sizes before simplification: `unsimplifiedVertexCount` and `unsimplifiedTriangleCount`, which a contour that was not simplified does not have.

### Interactive editor

- The signature is split in two: what a contour depends on, and the four simplification settings. A change to the second alone calls `simplify`, not `contour`. Where `simplify` gives nothing, as when simplification was just turned on, the editor contours (finding 11).
- The line under **Contour** shows the sizes before and after.
- Nothing changes in `Interactive Editor.unity`: the voxelizers in it take the defaults, which are off.

As step 6 built it:

- **Three signatures, not two.** Whether the mesh is simplified is part of what a contour depends on, since turning it on or off takes a contour (finding 11). The error and the line weight are what the collapses depend on, and the detail is what the mesh shown depends on beside those. With simplification off the last two are empty, and the settings do nothing.
- **A scheduler for each,** of the same kind as the one that decides when to contour, because each kind of work takes its own time: at 128³ a simplified contour takes 290 ms, simplifying it again 260 and another detail 4. With one time for all three, a drag of the detail would wait as a drag of a shape does, or be paused after a slow contour.
  - **The contour comes first.** While a contour is owed nothing else is looked at, and the contour brings the simplification settings with it.
  - **Simplifying again starts from the time simplifying took within the contour,** until it has a time of its own. Step 6 had it start from the time of the whole contour, which step 7 found wrong (finding 28).
  - **The detail waits for its first change to settle,** and follows a drag after that.
- **The managed implementations too.** They contour only when asked, because that takes seconds. Simplifying is Burst code whatever made the contour, so a new error or detail shows by itself for them as well. Turning simplification on is a contour, and waits for the button.
- **A mesh whose contour is forgotten** is contoured again when a simplification setting changes: after a script recompile the mesh is still shown and `simplify` has nothing to make another from. A contour of nothing has nothing to simplify and counts as up to date.
- **The inspector** says "Simplifying" for "Contouring" where that is what is due, and has a second line under the last contour: "Simplified from 52,224 vertices, 104,500 triangles". The first line keeps the time of the contour and has the sizes of what is shown now.
- **`select`** carries the four settings over to the other algorithm's voxelizer.

### Example scenes

New scenes in `Assets/Scenes/Mesh Simplification`, copies of Burst scenes with the setting on. Which ones is decided at that step, with a Uniform and an Occupancy scene, one with several materials and one with flat shading among them.

As step 6 made them, each a copy of a Burst scene with simplification on at the defaults, on 64³ cells where the scene it is copied from has 8³ or 16³, and with the wireframe drawn over the surface so that the triangles show:

| Scene | Copied from | Shading | Triangles |
|---|---|---|---|
| Uniform Sphere | Uniform, Burst, Sphere | Smooth | 38,460 to 1,888 |
| Uniform Multiple Materials | Uniform, Burst, Multiple Materials | Flat | 24,964 to 32, sixteen of each material |
| Occupancy CSG Operations | Occupancy, Burst, CSG Operations | Flat | 33,932 to 2,024 |
| Uniform Half Detail | Uniform, Burst, CSG Operations | Flat | 33,932 to 17,846, at a detail of 0.5 |

- **Nothing else differs** from the scenes they are copied from: the resolution, the simplification settings and the wireframe.
- **A fifth was made and taken out again:** Manifold's Simplification scene, whose octree clusters at 0.014 and leaves 396 triangles, of which the simplifier takes 12.
- **No existing scene was changed,** and none is in the build settings, which list no scenes.

## Tests

There is no second implementation to compare with, so the tests hold the simplifier to answers known in closed form and to invariants, as Occupancy's do. Each new test is shown to fail on a deliberate mistake, as in the earlier plans.

- **Quadrics:**
  - The error of a point is the sum of squared distances to the planes, worked out one plane at a time.
  - A line quadric is the squared distance to its line.
  - The best point of three planes is where they meet.
  - On a flat patch with line quadrics, the best point of a set of vertices is their area-weighted centre (lemma 4.1).
  - A boundary plane holds its edge and is square to its triangle.
  - **As step 4 built them:**
    - The quadric of a point is the squared distance to it.
    - Every vertex of a box, and of a tetrahedron with four faces of different areas, has the line along the normal of its triangles weighted by area, times the weight and its area. At a weight of zero no vertex has a line. Where two triangles lie back to back their vertices have a point.
    - On a flat patch the vertex of a collapsed edge goes to the centre of the vertices it stands for, collapse after collapse (lemma 4.1 with the simplifier's own quadrics). The collapse costs the lines alone, and is within an error bound of nothing (finding 7).
    - The first costs are those of every edge under its three quadrics.
- **Legality,** on meshes built by hand:
  - The two cases of figure 2 of Dey et al., one legal and one not.
  - No edge of a tetrahedron collapses.
  - An edge with four triangles, a pinched vertex and an open edge, each against theorem B.
  - Two triangles on the same three vertices: no edge at them collapses, and vertices without triangles are passed over (finding 15).
  - Each of Hoppe's six cases around a boundary between materials, with lines that end and lines that meet (finding 16).
  - A fan that the best position would fold over.
- **Shapes with known answers,** for every algorithm that contours them exactly: an upright cuboid becomes 8 vertices and 12 triangles at its corners, an octahedron 6 and 8, a pyramid 5 and 6, a hexagonal prism 12 and 20. A cuboid of two materials keeps a straight boundary between them, with 12 vertices and 20 triangles. Turned shapes within a tolerance measured in the step. As step 3 built them:
  - **The cuboid** is exact for every algorithm and implementation at any error, Occupancy and the GPU among them.
  - **The other three** reach their counts at an error of 0.1 with Uniform, Adaptive and Manifold, on 32³ and 64³. Their corners are held to four times the error (finding 22). Occupancy does not reach them: its tilted faces are fitted and not exact.
  - **The cuboid of two materials** is the slab with a band, which has two lines around it: 16 vertices and 28 triangles, with the band's vertices where the band begins and ends.
  - **A round line on a flat face** stays within four errors of its circle (finding 21).
  - **Turned shapes** do not reach the counts: their edges wobble by 0.3 cells in the contour itself. They are in the test of distances.
  - **Since step 4** all of these run at a line weight of zero and at the default.
- **Invariants,** over the models of `TopologyTest`, every algorithm, Burst and GPU, and Managed on small grids:
  - As many components and the same Euler characteristic as the contour, for the mesh and for each sub-mesh.
  - As many vertices of order 2, as many ends and meetings of sharp lines, and as many separate lines of open edges, of non-manifold edges and of edges between materials. The edges themselves get fewer: a line of 144 open edges is still one line when it has 20. The draft of this plan asked for as many open and non-manifold edges as the contour, which was wrong.
  - No degenerate triangles, no winding conflicts, and no triangle turned over. As built: no triangle without area, and every vertex used. A turned triangle is what `check` refuses, tested in step 2.
  - Every vertex within the bound of the shape, by the exact distances. As built: within four times the bound (finding 22).
  - A lower bound never gives fewer triangles. It holds in all 121 contours of the test and in every row of the measurements.
    - **It is no law, and step 5 met an exception:** in one run of the suite, the GPU's plate that leaves the grid had 48 triangles at an error of 0.02 and 50 at 0.1, at a line weight of 1e-2. A larger error allows other collapses, which can stand in the way of more. The same test passed in twelve runs after and in the four full runs before. The GPU's contours are now allowed a tenth more, or four triangles; the others come out the same every time and are held to it strictly.
  - The torus keeps its handle.
  - Since step 4 the invariants are held at line weights of zero, the default and 1e-2, and the distances at zero and the default.
- **Line quadrics** (step 4):
  - With them no edge worked out in the loop is without a best point, on twelve contours; without them most edges of a slab are.
  - A weight of 1e-2 leaves more triangles than the default on a slab with a disc, and their mean smallest angle is more than 1.2 times as large.
  - The voxelizer's weight is the one the simplifier is given.
- **The same every time:** two runs give the same mesh bit for bit.
- **The record:**
  - Detail 1 is the contour, triangle for triangle.
  - Detail 0 is the mesh the loop ended with.
  - The mesh at level k is the mesh of a run stopped after k collapses.
  - Every level passes the invariants.
  - **As step 5 built them:**
    - The mesh the record gives after k collapses is the mesh the connectivity holds when the loop is stopped after k, bit for bit, for seven values of k on three meshes built by hand and four contours, with and without lines. The loop stopped after k has made the first k collapses of the whole one.
    - On 55 contours, at details of 0, 0.25, 0.5, 0.75 and 1: the shape of the contour, no triangle without area, every vertex used, and one vertex fewer than the contour uses for every collapse. Detail 0 is the mesh `simplify` gives, and detail 1 the contour's used vertices and its triangles.
    - A detail is a share of the collapses, and levels past either end of the record are its ends.
    - The kept contour simplified under another error and weight has the collapses of a fresh simplification under those.
- **The frame:**
  - With the setting off, the `Start` hashes are those of step 0.
  - Flat shading after simplification, and past 65,535 split vertices.
  - A GPU contour is read back and simplified.
  - `simplify` after a change of detail or error gives the mesh of a fresh contour under those settings, without calling `voxelize`. As built: the volumes are taken away before `simplify` is called, so a contour would show nothing; for every CPU implementation, shared and flat shaded, through a change of detail, of error and of line weight, and back to the first mesh. The GPU's contours are shown again at detail 1 and 0.
  - `simplify` gives nothing in the six cases it has no simplified contour in, and leaves the mesh that is shown.
  - `contour` shows the mesh at the detail, and reports the sizes before.
  - The error stays in the cells of the kept contour after the resolution is changed.
- **Editor:** the settings carry over in `select`; a change of detail does not contour; undo. As step 6 built them:
  - Whether the voxelizers contoured or simplified is read off their profiler markers, and a test holds that reading to a contour with and without simplification.
  - A change of detail shows another mesh without a contour, for all nine variants, the managed ones among them. Its first change waits to settle and says that the simplification is what is owed. Changed back, as by an undo, it shows the first mesh again. Every material keeps its sub-mesh, and the inspector's sizes follow.
  - A change of error or line weight simplifies again without a contour, and shows what a contour under those settings shows.
  - Turning simplification on or off contours, and waits for the button with a managed implementation.
  - With simplification off the other three settings do nothing.
  - A mesh whose contour was forgotten is contoured again for another detail, and a contour of nothing is up to date at any.
  - The signature of a contour changes with simplification on or off, and not with the error, the weight or the detail.
- **Performance:** `ContouringPerformanceTest` gains the simplifier, by stage.

## Steps

Each step compiles, passes every test, and is reviewed before the next one starts. Nothing is committed for you.

0. **Baseline and what the contours are like (small).** Both suites green; the `Start` hashes; Burst timings with Unity in front. A probe over the test models and example scenes, for every algorithm and implementation: vertices, triangles, open edges, non-manifold edges, pinched vertices, edges between materials, degenerate and repeated triangles, and vertices that share a position. That settles finding 4 and says what the connectivity has to cope with. **Done 2026-11-13:**
   - **Suites:** edit mode 1808 of 1808, play mode 21 of 21.
   - **`Start` hashes:** 480 configurations, none failed. They are the baseline for the later steps.
   - **Timings and the probe:** under Measurements. Findings 4 and 14 are updated from the probe, and findings 15 to 18 are new.
   - **Nothing in the repository changed.** The probes are scripts outside it.
1. **Quadrics (small).** `BurstQuadric` and its tests. **Measured here:** the error of a float quadric against a double one on a flat face and on a ball at 128³ (finding 6, decision 3). **Done 2026-11-14:**
   - **Built:** `Assets/Scripts/Burst/BurstQuadric.cs` and `Assets/Tests/Edit Mode/BurstQuadricTest.cs`, 15 tests that run the quadrics in a Burst job.
   - **Tests beyond the plan's list:** a triangle's quadric and area; triangles, lines and edges that have no quadric; planes that almost do not meet, on both sides of the pivot limit; a small error far from the origin, which a float `c` fails.
   - **Deliberate mistakes:** 26 were tried, and every test failed on at least one. Three of the first twenty went unnoticed, and each pointed at code that could go:
     - **A second choice of pivot** changed no result a test could pin, and was removed.
     - **A check for an edge without length** was covered by the check after it. The plane is now worked out without a division, and one check is left.
     - **The check of the second pivot** went unnoticed only while the second pivot was chosen. It is caught now.
   - **Measured:** floats against doubles, and the best point of every edge (Measurements). Decision 3 stands, and finding 19 is new.
   - **Baselines after step 1:** edit mode 1823, play mode 21, 480 `Start` hashes unchanged.
2. **Connectivity and legality (medium).** The lists, sharp edges, the link conditions, Hoppe's tests, the inversion test, and one collapse with everything it updates, on meshes built by hand, with what step 0 found in the contours (findings 15 to 17). No heap yet. **Measured here:** the weight of boundary planes (finding 13). **Done 2026-11-14:**
   - **Built:** `Assets/Scripts/Burst/BurstMeshSimplifier.cs` with `Connectivity` (Design, Code), and `Assets/Tests/Edit Mode/BurstMeshSimplifierTest.cs`, 91 tests that run it in a Burst job.
   - **Meshes built by hand:** boxes on a grid of whole numbers, alone, sharing an edge, touching in a vertex, with a fin and with two triangles back to back; flat patches and strips; an octahedron, whole, with a vertex put into a face (figure 2 of Dey et al.) and with a face taken out; a tetrahedron; a box with lines between up to four materials on its top.
   - **Every reason for refusing** has a mesh that gives exactly that reason, and a mesh next to it that collapses.
   - **Many collapses:** every edge that may collapse is collapsed into its midpoint, four times over, on 13 meshes built by hand and on 30 contours at 32³ (five kinds of contour on six models). Each result is held to the invariants under Tests and to a connectivity built from nothing. The three balls go down to 4 vertices, a tetrahedron, from every algorithm's contour.
   - **What the tests found:** finding 20, and two of my own expectations that were wrong about which condition a mesh breaks first.
   - **Deliberate mistakes:** 38 were tried and every one failed at least one test.
   - **Measured:** the boundary weight on single collapses (Measurements, finding 13). It is 1 for now and step 3 settles it.
   - **Not timed.** Step 3 times the simplifier.
   - **Baselines after step 2:** edit mode 1914, play mode 21, 480 `Start` hashes unchanged.
3. **The simplifier (large).** Heap, loop, bound, normals, the mesh out; the hook, the first two settings and the inspector; shared normals asked of the GPU. Line weight zero, so this step is Garland and Heckbert's algorithm with its own fallbacks. The shape, invariant and frame tests. **Measured here:** time by stage from 16³ to 128³ and Uniform at 256³, triangles against the error bound for each algorithm, distance to the shapes, and crossing triangle pairs before and after (finding 9). The default error comes out of this. Also left to this step by the earlier ones: the lines between materials after simplifying at boundary weights of 0.1, 1, 10 and 100 (finding 13), and the best points far from their edges (finding 19). **Done 2026-11-14:**
   - **Built:** the jobs and `simplify` in `BurstMeshSimplifier.cs`, `minimizeBetween` in `BurstQuadric.cs`, and the settings, `cellSize`, `contourNormals` and the hook in `Voxelizer.cs` (Design, Code). The two settings are fields under a **Mesh Simplification** heading, after Vertex Normals, which the inspector draws by itself. I checked that they are among the properties it draws, and did not look at the inspector.
   - **Tests:** `MeshSimplificationTest.cs` is new, with 232 tests. `VoxelizerContourTest` has 71 more, `BurstQuadricTest` 4 and `BurstMeshSimplifierTest` 3, less the 4 of step 2 that went through the boundary weight. `ContouringPerformanceTest` has the simplifier by stage, run by hand like the rest of it.
   - **Three things the measurements changed:**
     - **Sharp lines have a bound of their own** (finding 21).
     - **Ties go by area before index** (finding 12).
     - **The default error is 0.05 cells.**
   - **One bug the tests of step 2 could not see** (finding 23).
   - **What I expected and did not find:** corners of polyhedra exactly in place. They are within three errors (finding 22).
   - **Best points far from their edges** (finding 19): at most 25 collapses of 50,000 put a vertex more than two edge lengths from the middle of its edge, on the three balls at 128³, and no model has more. `check` refused up to 16 such edges on a contour. They do not show in the distances to the shapes, so nothing is done about them.
   - **Deliberate mistakes:** 25 were tried and 24 failed a test. The one that passed was the tie by area, which only changes how long the loop takes. `Statistics` now counts the edges worked out again and a test holds a cube to 12 for each collapse, which the mistake fails.
   - **Not done here:** the interactive editor does not know the two settings yet. It carries neither over in `select` and does not contour when one changes. That is step 6.
   - **Baselines after step 3:** edit mode 2220, play mode 21, 480 `Start` hashes unchanged.
4. **Line quadrics (small).** The second sum, the weight, the point quadric for degenerate vertices, what a vertex without a normal gets (finding 14), their tests. **Measured here:** weights 0, 1e-4, 1e-3, 1e-2 and 1e-1 as in the paper's figure 4: triangles, distance to the shapes, the smallest angle of the triangles, and time. The default weight is set from it. **Done 2026-11-14:**
   - **Built:** the third quadric of every vertex in `BurstMeshSimplifier.cs`, `BurstQuadric.point`, and the **Line Quadric Weight** setting on `Voxelizer` (Design, Code). The line of a vertex runs along the normal of its triangles (finding 14).
   - **The default weight is 1e-5,** a hundredth of the paper's. The measurement also took in 1e-6 and 1e-5, and they are what it came down to (finding 24).
   - **What I expected and did not find:** more even triangles at a small weight. On a mesh simplified to its error bound the weight has to be 1e-2 for that, and it costs triangles (finding 25).
   - **What it does give at the default:** every edge has a best point, so the fallbacks of step 3 are only used at a weight of zero. A cube at 128³ takes 147 ms and not 180.
   - **What it costs:** the third sum makes the three balls at 128³ take 258 to 261 ms where step 3's code takes 250, run one after the other.
   - **Tests:** 394 more. `MeshSimplificationTest` has 607, most of them step 3's at two or three weights, and takes 63 seconds where it took 22; the whole edit mode suite takes 3 minutes.
   - **Deliberate mistakes:** 22 were tried and every one failed a test. Four tests were added or tightened to get there before the run: the tetrahedron for normals weighted by area, the first costs, and what `targetOf` reports.
   - **One expectation of my own was wrong:** that the vertices of a box stand for four different areas. They stand for two.
   - **Not done here:** as in step 3, the interactive editor does not know the setting. That is step 6.
   - **Baselines after step 4:** edit mode 2614, play mode 21, 480 `Start` hashes unchanged.
5. **The record (medium).** The record, the mesh at a level, the Detail setting, what the voxelizer keeps, `simplify`, their tests. **Measured here:** a change of detail and a change of error at 128³, against a whole contour. **Done 2026-11-14:**
   - **Built:** `Collapse`, `LevelJob`, `record` and `meshAt` in `BurstMeshSimplifier.cs`; `Assets/Scripts/MeshSimplification.cs`; on `Voxelizer` the **Simplification Detail** setting, `simplify( )`, and the sizes before simplification in `ContourStatistics` (Design).
   - **Two things built otherwise than planned** (finding 26): what is kept is arrays and no `Mesh`, and the mesh is always made from the record.
   - **The meshes are step 4's:** seven models at seven weights give the same counts, angles and crossing pairs as before the change.
   - **A change of detail takes 3 to 6 ms at 128³,** against 286 for a simplified contour. A change of error takes 259 (Measurements).
   - **The lines show in the meshes in between** (finding 25), which step 4 could not see.
   - **Tests:** 88 more. The edit mode suite takes 3½ minutes.
   - **A test that failed once:** that a larger error leaves no more triangles is no law, and a GPU contour broke it in one run of the suite (Tests). It came in with step 4, which runs that test at three weights, and is not from this step's code.
   - **One expectation of my own was wrong:** that every contour has a collapse to make. An octree can leave none.
   - **Deliberate mistakes:** 31 were tried and every one failed a test.
   - **Not done here:** the interactive editor does not call `simplify` or know the setting. That is step 6.
   - **Baselines after step 5:** edit mode 2702, play mode 21, 480 `Start` hashes unchanged.
6. **Interactive editor and scenes (small).** The split signature, the statistics line, `select`, the editor tests, the scenes. **Done 2026-11-14:**
   - **Built:** in `InteractiveEditor.cs` the three signatures, a scheduler for each, `select`, and the inspector's lines; `RemeshScheduler.invalidate`; four scenes in `Assets/Scenes/Mesh Simplification` (Design).
   - **Otherwise than planned:** three signatures and three schedulers where the plan had two signatures, and the managed implementations simplify by themselves (Design, Interactive editor).
   - **Tests:** 35 more, in `InteractiveEditorTest` and `RemeshSchedulerTest`.
   - **Deliberate mistakes:** 23 were tried and 21 failed a test. The two that passed were lines of the editor that did what the same update does a few lines on, and both are gone.
   - **Not looked at:** the inspector, and the scenes on screen. Each scene was opened beside what was open, contoured, and closed again, which is where the triangle counts above are from. `Interactive Editor.unity` was open throughout and is unchanged.
   - **Baselines after step 6:** edit mode 2737, play mode 21, 480 `Start` hashes unchanged.
7. **Tuning and documentation (medium).** Whatever steps 3 to 5 show to cost most. From step 4: whether the vertex should be placed by the planes alone (finding 24), if large flat faces at small errors turn out to matter. Candidates: Burst implementations handing their arrays to the simplifier without building a mesh first; the heap; stopping at the detail asked for when the record is not wanted. From step 3's times: the loop is 88% of the time at 4.4 microseconds a collapse, and building the lists is 11% and serial, 26 ms at 128³, as long as the contour itself. README (a section, the settings, the code layout), `AGENTS.md` (the papers) and this plan marked complete. **Done 2026-11-14:**
   - **Tuned:** the heap and the order of a vertex (finding 27). The three balls at 128³ take 206 ms to simplify where they took 263, a cube 108 where it took 149, and the three balls at 64³ 39 where they took 48, which puts a simplified contour at 64³ under the 50 ms the editor follows a drag in.
   - **Left as it is:** building a mesh before simplifying, which is half a millisecond; stopping at the detail asked for, since the record is always kept now; and where a vertex is placed (finding 24).
   - **A flaw of step 6 found and mended** (finding 28).
   - **Documented:** a section in `README.md` with the settings, what is kept and what is not, the cost and the scenes; the code layout and the interactive editor there; the papers in `AGENTS.md`.
   - **Tests:** 4 more, among them the cheapest edge at every collapse and the slow managed contour. The edit mode suite takes about 4½ minutes.
   - **Deliberate mistakes:** 14 were tried and every one failed a test, after the cheapest-edge test was given a model on which edges get cheaper. One run of the hashes afterwards did not match, and that was the last mistake still loaded in the editor.
   - **The editor stopped compiling once,** for twenty minutes, and results in that time were of the code before the change. Taking another application to the front and Unity again let it go on.
   - **Baselines at the end:** edit mode 2741, play mode 21, 480 `Start` hashes unchanged.

## Measurements

### Baseline (step 0, 2026-11-13)

**A whole contour of the three balls,** as `Voxelizer.contour` gives it with shared normals, the mesh read back from the GPU and the vertex buffer reordered. The simplifier will run inside this call. Median of 7 runs after a warm-up, in ms, with Unity in front. The octrees simplify at 2e-4.

| Cells | Uniform Burst | Adaptive Burst | Manifold Burst | Occupancy Burst | Uniform GPU | Occupancy GPU |
|---|---|---|---|---|---|---|
| 16³ | 0.4 | 0.8 | 1.0 | 1.1 | | |
| 32³ | 1.4 | 1.8 | 2.4 | 3.3 | | |
| 64³ | 5.7 | 6.4 | 7.4 | 12.5 | 19.6 | 6.9 |
| 128³ | 29.4 | 32.4 | 34.1 | 54.9 | 122.5 | 20.5 |
| 256³ | 179.4 | | | 283.4 | 854.5 | 95.6 |

- **The runs agree:** from 64³ up, the least and the most of the 7 are within 4% of the median on Burst. On the GPU the most is up to 55% above it.
- **Triangles at 128³:** 104,500 from Uniform and Occupancy, 5,224 from Adaptive and 4,714 from Manifold.
- **Against finding 10:** the estimate of 50 to 150 ms for the collapse loop at 128³ is two to five times a Burst Uniform contour.

### The simplifier (step 3, 2026-11-14)

All at a line weight of zero, with Burst contours, the octrees simplifying at 2e-4 first as in their scenes. Errors are in cells.

**Time by stage,** the three balls at the default error of 0.05, median of 5 runs in ms with Unity in front:

| Cells | Vertices | Left | Simplify | Build | Quadrics | Costs | Loop | The contour itself |
|---|---|---|---|---|---|---|---|---|
| Uniform 16³ | 760 | 414 | 1.6 | 0.4 | 0.1 | 0.1 | 1.0 | 0.4 |
| Uniform 32³ | 3,216 | 655 | 10.6 | 1.7 | 0.2 | 0.1 | 8.4 | 1.4 |
| Uniform 64³ | 13,000 | 1,241 | 47.7 | 6.7 | 0.6 | 0.3 | 39.9 | 5.7 |
| Uniform 128³ | 52,224 | 2,296 | 247.7 | 26.4 | 2.0 | 0.9 | 217.6 | 29.4 |
| Uniform 256³ | 208,900 | 4,405 | 1,521.7 | 104.0 | 7.7 | 3.6 | 1,404.0 | 179.4 |
| Occupancy 128³ | 52,252 | 2,229 | 228.7 | 27.0 | 2.0 | 0.9 | 198.1 | 54.9 |
| Adaptive 128³ | 2,586 | 1,964 | 4.1 | 1.7 | 0.2 | 0.1 | 2.0 | 32.4 |
| Manifold 128³ | 2,359 | 1,829 | 3.6 | 1.5 | 0.2 | 0.1 | 1.6 | 34.1 |

- **Reading and writing the mesh** are 2 ms together at 256³, less below, and left out.
- **Other models at Uniform 128³:** a cube 179 ms, the cubes and sphere 189, the turned shapes 217, the cut-away sphere, which has 67,936 vertices, 366.
- **The loop** takes 4.4 microseconds a collapse at 128³ and 6.9 at 256³.
- **After an octree has simplified** there is little left to do, and it takes 4 ms.

**Triangles left** at 128³:

| Contour | Model | Contour | 0.005 | 0.01 | 0.02 | 0.05 | 0.1 | 0.25 | 0.5 |
|---|---|---|---|---|---|---|---|---|---|
| Uniform | Sphere | 55,600 | 30,084 | 13,216 | 6,178 | 2,234 | 1,056 | 398 | 196 |
| Uniform | Three balls | 104,500 | 56,804 | 25,478 | 12,028 | 4,644 | 2,270 | 984 | 582 |
| Uniform | Cubes and sphere | 68,812 | 20,152 | 8,768 | 4,054 | 1,558 | 724 | 314 | 170 |
| Uniform | Cut-away sphere | 135,868 | 43,198 | 18,942 | 8,920 | 3,352 | 1,638 | 646 | 336 |
| Uniform | Slab with band | 63,036 | 28 | 28 | 28 | 28 | 28 | 28 | 28 |
| Uniform | Turned shapes | 90,832 | 16,238 | 9,328 | 5,836 | 3,316 | 2,182 | 1,338 | 1,160 |
| Uniform | New shapes | 90,728 | 42,564 | 25,496 | 14,484 | 5,488 | 2,746 | 1,304 | 926 |
| Occupancy | Sphere | 55,600 | 26,960 | 12,270 | 5,982 | 2,134 | 1,014 | 400 | 198 |
| Occupancy | Three balls | 104,500 | 51,674 | 24,298 | 12,156 | 4,454 | 2,102 | 790 | 392 |
| Occupancy | Cubes and sphere | 68,812 | 17,892 | 8,210 | 4,002 | 1,530 | 740 | 312 | 172 |
| Occupancy | Cut-away sphere | 135,868 | 39,074 | 19,742 | 9,464 | 3,656 | 1,692 | 672 | 336 |
| Occupancy | Slab with band | 63,036 | 28 | 28 | 28 | 28 | 28 | 28 | 28 |
| Occupancy | Turned shapes | 90,876 | 11,712 | 9,030 | 6,392 | 3,326 | 1,822 | 768 | 414 |
| Occupancy | New shapes | 90,748 | 40,474 | 24,388 | 13,422 | 5,416 | 2,574 | 1,062 | 554 |
| Adaptive | Three balls | 5,224 | 5,138 | 5,072 | 4,936 | 3,980 | 2,664 | 1,184 | 660 |
| Adaptive | Cut-away sphere | 3,036 | 2,906 | 2,870 | 2,746 | 2,296 | 1,818 | 940 | 442 |
| Adaptive | Turned shapes | 9,662 | 4,014 | 3,602 | 3,272 | 2,768 | 2,124 | 1,408 | 1,212 |
| Adaptive | New shapes | 11,416 | 7,460 | 6,574 | 5,804 | 4,498 | 3,202 | 1,728 | 1,096 |
| Manifold | Three balls | 4,714 | 4,682 | 4,664 | 4,558 | 3,654 | 2,428 | 964 | 464 |
| Manifold | Cut-away sphere | 2,988 | 2,882 | 2,842 | 2,748 | 2,290 | 1,816 | 946 | 434 |
| Manifold | Turned shapes | 3,230 | 1,490 | 1,338 | 1,156 | 882 | 592 | 330 | 204 |
| Manifold | New shapes | 5,536 | 4,628 | 4,360 | 4,074 | 3,356 | 2,400 | 1,198 | 592 |

- **At the default** Uniform's three balls have 4,644 triangles, as many as Manifold's clustering leaves them at 2e-4, which is 4,714.
- **A flat model** comes down to its corners at any error.
- **Uniform does not get below 1,160 on the turned shapes,** where Occupancy gets to 414. I put that down to its edges of four triangles, which are sharp lines and held in place, and did not check.
- **After an octree** the simplifier takes another quarter off at the default, and a half or more from 0.1.

**Distance to the shape,** on nine shapes whose density is their distance (cuboid, ball, cylinder, cone, capsule, torus, pyramid, octahedron, hexagonal prism), upright and turned, at 64³. The worst vertex and the worst middle of a triangle over the shapes whose contour is not already further off than twice the error, in cells and as multiples of the error:

| Contour | Error | Worst vertex | Worst middle |
|---|---|---|---|
| Uniform | 0.02 | 0.060 (3.0) | 0.028 (1.4) |
| Uniform | 0.05 | 0.116 (2.3) | 0.121 (2.4) |
| Uniform | 0.1 | 0.243 (2.4) | 0.291 (2.9) |
| Uniform | 0.25 | 0.562 (2.2) | 0.800 (3.2) |
| Manifold | 0.02 | 0.060 (3.0) | 0.028 (1.4) |
| Manifold | 0.05 | 0.162 (3.2) | 0.121 (2.4) |
| Manifold | 0.1 | 0.250 (2.5) | 0.291 (2.9) |
| Manifold | 0.25 | 0.543 (2.2) | 0.800 (3.2) |
| Occupancy | 0.05 | 0.109 (2.2) | 0.165 (3.3) |
| Occupancy | 0.1 | 0.224 (2.2) | 0.273 (2.7) |
| Occupancy | 0.25 | 0.513 (2.1) | 0.549 (2.2) |

- **The worst vertex** is on the upright capsule nearly every time, and the worst middle on the turned cylinder or the upright cone.
- **The contours themselves** have vertices up to 0.54 cells from a turned cone with Uniform and 0.44 from a turned pyramid with Occupancy. Simplifying does not move those further.
- **Manifold here is without clustering.**
- **Occupancy at 0.02 is left out:** the middle of a triangle of its own contour of the cone is 0.16 cells from the shape.

**Pairs of triangles that pass through each other,** at 64³, in the contour and after simplifying:

| Contour | Model | Contour | 0.02 | 0.05 | 0.1 | 0.25 | 0.5 |
|---|---|---|---|---|---|---|---|
| Occupancy | Turned shapes | 0 | 0 | 9 | 6 | 7 | 18 |
| Occupancy | New shapes | 4 | 4 | 4 | 0 | 0 | 0 |
| Occupancy | The other five | 0 | 0 | 0 | 0 | 0 | 0 |
| Manifold | Three balls | 502 | 31 | 0 | 0 | 0 | 0 |
| Manifold | Turned shapes | 1,015 | 481 | 215 | 42 | 10 | 10 |
| Manifold | New shapes | 602 | 111 | 43 | 16 | 0 | 0 |
| Uniform | Three balls | 498 | 49 | 20 | 7 | 4 | 4 |
| Uniform | Turned shapes | 836 | 517 | 349 | 236 | 159 | 126 |
| Uniform | New shapes | 607 | 206 | 132 | 91 | 33 | 35 |

- **Manifold is without clustering** here too. It and Uniform also have 156 pairs on the cubes and sphere, and none after simplifying at any error.
- **Occupancy's own contour of the new shapes has 4 pairs,** which its paper's rule should not leave. I did not look into why.

**The lines between materials,** at 64³ with Uniform, after a whole simplification. For each: how many of the contour's edges the lines keep, and how far their vertices and the middles of their edges are from the true line.

Before the lines had a bound of their own (finding 21), at an error of 0.05:

| Line | Contour | Weight 0.1 | 1 | 10 | 100 |
|---|---|---|---|---|---|
| Round, on a flat face | 204 edges | 12, middles 3.64 | 23, middles 1.29 | 47, middles 0.35 | 95, middles 0.14 |

With it:

| Line | Error | Contour | Weight 0.1 | 1 | 10 | 100 |
|---|---|---|---|---|---|---|
| Straight, flat | 0.05 | 256 edges | 8, exact | 8, exact | 8, exact | 8, exact |
| Round, flat | 0.05 | 204 edges | 62, middles 0.14 | 62, middles 0.14 | 62, middles 0.14 | 62, middles 0.14 |
| Round, flat | 0.25 | 204 edges | 27, middles 0.70 | 28, middles 0.70 | 28, middles 0.70 | 28, middles 0.70 |
| Round, crease | 0.05 | 204 edges | 38, middles 0.08 | 37, middles 0.07 | 42, middles 0.07 | 39, middles 0.14 |
| Round, crease | 0.25 | 204 edges | 15, middles 0.67 | 15, middles 0.74 | 16, middles 0.71 | 16, middles 0.68 |

- **Exact** means as near the true line as the contour's own vertices, 0.014 cells.
- **At a weight of zero** the bound alone still holds the lines, with more edges: 89 on the round flat line at 0.05.
- **Occupancy's round line on the flat face keeps 120 of its 204 edges** at either error. Its line is up to 0.67 cells from the circle in the contour and jumps from side to side of it, which no collapse within these errors can take out.

### Tuning (step 7, 2026-11-14)

Simplifying alone, at the defaults, median of 5 runs in ms with Unity in front. Before is step 6's code, measured the same day.

| Contour | Vertices | Before | After | Build | Quadrics | Costs | Loop | Level |
|---|---|---|---|---|---|---|---|---|
| Uniform, three balls 32³ | 3,216 | 10.6 | 8.7 | 1.3 | 0.3 | 0.1 | 6.8 | 0.2 |
| Uniform, three balls 64³ | 13,000 | 47.8 | 39.3 | 5.0 | 0.7 | 0.3 | 32.4 | 0.7 |
| Uniform, three balls 128³ | 52,224 | 262.8 | 205.8 | 19.6 | 2.3 | 1.0 | 179.5 | 2.5 |
| Uniform, three balls 256³ | 208,900 | 1,527 | 1,214 | 77.0 | 8.9 | 4.0 | 1,111 | 9.8 |
| Uniform, cube 128³ | 35,576 | 148.5 | 108.0 | 11.4 | 1.5 | 0.7 | 92.1 | 1.6 |
| Uniform, cube 128³, weight 0 | 35,576 | 180.4 | 103.1 | 11.5 | 1.5 | 0.6 | 87.3 | 1.6 |
| Occupancy, three balls 128³ | 52,252 | 233 | 186.8 | 20.9 | 2.4 | 1.1 | 159.4 | 2.5 |
| Adaptive, three balls 128³ | 2,586 | 4.1 | 3.7 | 1.4 | 0.2 | 0.1 | 1.6 | 0.2 |
| Manifold, three balls 128³ | 2,359 | 3.6 | 3.2 | 1.2 | 0.2 | 0.1 | 1.3 | 0.2 |

- **The loop** takes 3.6 microseconds a collapse at 128³, where it took 4.4.
- **Through the voxelizer** a simplified contour of the three balls takes 44 ms at 64³, 240 at 128³ and 1,400 at 256³, and simplifying again under another error 41, 210 and 1,280. Another detail takes what it took before.
- **A cube without lines** gained most: every edge of a flat face costs the same there, and every comparison went on to the areas.

### The record (step 5, 2026-11-14)

The three balls at the default error and line weight, with Unity in front, in ms.

**A change of a setting against a whole contour,** through `Voxelizer.contour` and `Voxelizer.simplify`, median of 5 runs (3 at 256³):

| Contour | Triangles | Contour, not simplified | Contour, simplified | New detail, 0.25 or 0.5 | Detail 1 | Detail 0 | New error |
|---|---|---|---|---|---|---|---|
| Uniform Burst 64³ | 26,036 | 5.6 | 54.5 | 1.1 | 1.5 | 0.8 | 50.3 |
| Uniform Burst 128³ | 104,500 | 29.2 | 286.0 | 4.2 | 6.0 | 2.8 | 259.0 |
| Uniform Burst 128³, flat shaded | 104,500 | 37.4 | 290.1 | 6.6 | 13.9 | 3.1 | 261.6 |
| Occupancy Burst 128³ | 104,500 | 54.5 | 287.9 | 4.3 | 6.1 | 2.7 | 235.8 |
| Occupancy GPU 128³ | 104,500 | 21.6 | 288.4 | 4.3 | 6.3 | 2.7 | 257.8 |
| Uniform Burst 256³ | 417,876 | 181.6 | 1,717.1 | 16.3 | 24.0 | 10.9 | 1,537.8 |

- **A new detail** is 50 to 100 times quicker than contouring and simplifying again, and quicker than the contour alone. It grows with the triangles it shows.
- **A new error** saves the contour and nothing else, which is a tenth at 128³.

**The parts,** median of 7 runs:

| Cells | Kept | Simplifying and keeping | Simplifying alone | Mesh after all collapses | After half | After none |
|---|---|---|---|---|---|---|
| 32³ | 0.2 MB | 10.5 | 10.6 | 0.25 | 0.32 | 0.33 |
| 64³ | 1.1 MB | 47.6 | 47.8 | 0.77 | 1.10 | 1.14 |
| 128³ | 4.3 MB | 261.4 | 259.1 | 2.73 | 4.35 | 4.47 |
| 256³ | 17.4 MB | 1,526.9 | 1,556.2 | 10.58 | 17.40 | 18.11 |

- **Keeping the contour and the record costs no time that shows:** the two columns differ by less than one run does from the next.
- **The mesh at a level** includes copying the kept arrays to where a job can read them and building the `Mesh`.

**The meshes in between, by line weight,** Uniform at 64³. Detail, triangles, the mean smallest angle, and the share of triangles where it is under 10°:

| Model | Weight | Detail 0 | 0.02 | 0.1 | 0.25 | 0.5 |
|---|---|---|---|---|---|---|
| Cube | 0 | 12, 45°, 0% | 376, 18°, 38% | 1,836, 32°, 19% | 4,572, 39°, 8% | 9,132, 42°, 4% |
| Cube | 1e-5 | 12, 45°, 0% | 376, 43°, 0% | 1,836, 42°, 0% | 4,572, 43°, 0% | 9,132, 38°, 0% |
| Cube | 1e-3 | 12, 45°, 0% | 376, 42°, 0% | 1,836, 42°, 0% | 4,572, 43°, 0% | 9,132, 36°, 0% |
| Slab with disc | 0 | 136, 8°, 79% | 448, 5°, 89% | 1,694, 19°, 42% | 4,028, 24°, 35% | 7,922, 33°, 16% |
| Slab with disc | 1e-5 | 144, 8°, 75% | 456, 35°, 8% | 1,700, 40°, 1% | 4,036, 41°, 1% | 7,926, 39°, 0% |
| Slab with disc | 1e-3 | 152, 11°, 53% | 464, 37°, 2% | 1,708, 42°, 0% | 4,040, 43°, 0% | 7,930, 38°, 0% |
| Turned cube | 0 | 226, 20°, 27% | 614, 33°, 6% | 2,164, 36°, 1% | 5,070, 36°, 1% | 9,912, 36°, 1% |
| Turned cube | 1e-5 | 230, 21°, 27% | 618, 34°, 5% | 2,168, 39°, 1% | 5,072, 41°, 0% | 9,916, 39°, 0% |
| Three balls | 0 | 2,518, 42°, 2% | 2,988, 42°, 2% | 4,870, 43°, 1% | 8,398, 41°, 1% | 14,276, 43°, 1% |
| Three balls | 1e-5 | 2,510, 42°, 2% | 2,980, 42°, 2% | 4,862, 43°, 1% | 8,392, 41°, 1% | 14,272, 43°, 1% |
| New shapes | 0 | 3,224, 30°, 6% | 3,606, 30°, 5% | 5,136, 33°, 4% | 8,002, 36°, 3% | 12,780, 37°, 2% |
| New shapes | 1e-3 | 3,284, 32°, 3% | 3,666, 33°, 2% | 5,190, 36°, 1% | 8,048, 39°, 1% | 12,810, 40°, 1% |

- **Flat models part-way down** are where the lines show (finding 25): without them a cube with 2% of its collapses left is two fifths thin triangles, even with ties going by area (finding 12).
- **At detail 0** the weight makes no such difference, as step 4 found.

### Line quadrics (step 4, 2026-11-14)

Burst contours at the default error of 0.05 cells unless a table says otherwise. Manifold clusters at 2e-4 first. The slab with a disc is a slab with a cylinder of a second material set into it, flush on both faces. The turned cube is a cube of half the grid turned about all three axes.

**Triangles left** at 64³, by weight:

| Contour | Model | Contour | 0 | 1e-6 | 1e-5 | 1e-4 | 1e-3 | 1e-2 | 1e-1 |
|---|---|---|---|---|---|---|---|---|---|
| Uniform | Three balls | 26,036 | 2,518 | 2,502 | 2,510 | 2,574 | 2,494 | 2,578 | 2,700 |
| Uniform | Cubes and sphere | 16,972 | 868 | 860 | 840 | 792 | 830 | 916 | 1,004 |
| Uniform | Cut-away sphere | 33,932 | 1,792 | 1,794 | 1,780 | 1,748 | 1,756 | 1,866 | 1,936 |
| Uniform | Turned cube | 19,600 | 226 | 232 | 230 | 230 | 240 | 278 | 316 |
| Uniform | Slab with disc | 15,708 | 136 | 148 | 144 | 138 | 152 | 202 | 212 |
| Uniform | Turned shapes | 22,376 | 1,522 | 1,528 | 1,538 | 1,524 | 1,554 | 1,632 | 1,836 |
| Uniform | New shapes | 22,336 | 3,224 | 3,218 | 3,226 | 3,232 | 3,284 | 3,584 | 3,920 |
| Occupancy | Three balls | 26,036 | 2,360 | 2,356 | 2,362 | 2,454 | 2,326 | 2,362 | 2,658 |
| Occupancy | Cubes and sphere | 16,972 | 824 | 814 | 812 | 830 | 842 | 938 | 1,018 |
| Occupancy | Turned cube | 19,608 | 404 | 414 | 348 | 310 | 452 | 810 | 954 |
| Occupancy | Slab with disc | 15,708 | 258 | 258 | 258 | 264 | 268 | 332 | 384 |
| Occupancy | Turned shapes | 22,412 | 1,680 | 1,678 | 1,648 | 1,632 | 1,766 | 2,130 | 2,436 |
| Occupancy | New shapes | 22,342 | 3,038 | 3,010 | 3,016 | 3,050 | 3,138 | 3,644 | 4,096 |
| Manifold | Three balls | 1,884 | 1,598 | 1,598 | 1,598 | 1,598 | 1,598 | 1,600 | 1,602 |
| Manifold | Cubes and sphere | 876 | 584 | 584 | 588 | 584 | 604 | 652 | 740 |
| Manifold | Turned cube | 184 | 12 | 12 | 12 | 12 | 12 | 36 | 46 |
| Manifold | Turned shapes | 1,812 | 560 | 556 | 558 | 548 | 584 | 662 | 748 |
| Manifold | New shapes | 4,236 | 2,232 | 2,236 | 2,226 | 2,228 | 2,244 | 2,352 | 2,554 |

- **Up to 1e-4** the counts move by a few percent either way, which is the order of the collapses and not the weight. Occupancy's turned cube moves most, from 404 to 310.
- **From 1e-3** they rise with the weight, by a tenth to a half at 1e-1 and by more on flat models.

**The smallest angle of each triangle,** at 64³: its mean over the mesh, and the share of triangles where it is under 10°.

| Contour | Model | 0 | 1e-5 | 1e-4 | 1e-3 | 1e-2 | 1e-1 |
|---|---|---|---|---|---|---|---|
| Uniform | Three balls | 42° / 2% | 42° / 2% | 42° / 2% | 42° / 2% | 43° / 2% | 42° / 2% |
| Uniform | Cubes and sphere | 40° / 3% | 40° / 2% | 40° / 3% | 40° / 3% | 41° / 0% | 41° / 2% |
| Uniform | Turned cube | 20° / 27% | 21° / 27% | 21° / 34% | 22° / 30% | 24° / 29% | 22° / 31% |
| Uniform | Slab with disc | 8° / 79% | 8° / 75% | 11° / 65% | 11° / 53% | 16° / 22% | 15° / 26% |
| Uniform | New shapes | 30° / 6% | 30° / 5% | 30° / 4% | 32° / 3% | 35° / 3% | 35° / 3% |
| Occupancy | Three balls | 42° / 0% | 43° / 0% | 43° / 0% | 44° / 0% | 43° / 0% | 43° / 0% |
| Occupancy | Turned cube | 19° / 28% | 18° / 33% | 19° / 29% | 20° / 26% | 24° / 16% | 23° / 21% |
| Occupancy | Slab with disc | 19° / 46% | 20° / 38% | 22° / 36% | 20° / 35% | 18° / 38% | 14° / 58% |
| Occupancy | New shapes | 29° / 7% | 29° / 8% | 29° / 6% | 31° / 4% | 35° / 2% | 35° / 3% |
| Manifold | Slab with disc | 14° / 45% | 16° / 38% | 16° / 38% | 22° / 29% | 20° / 20% | 22° / 18% |
| Manifold | New shapes | 35° / 4% | 35° / 4% | 35° / 4% | 35° / 3% | 36° / 2% | 36° / 3% |

- **Round models** have good triangles at any weight.
- **Flat faces with something on them** are where the triangles are thin, and a weight of 1e-2 is what it takes to change that. Occupancy's slab with a disc gets worse at 1e-1.
- **The contours themselves** have a mean of 36° to 44° on these models.

**Vertices left on flat models,** Uniform, by error and weight. A cube has 8 corners, the slab with a band 16 vertices that are needed:

| Model | Cells across | Error | 0 | 1e-6 | 1e-5 | 1e-4 | 1e-3 | 1e-2 |
|---|---|---|---|---|---|---|---|---|
| Cube at 64³ | 38 | 0.002 | 8 | 8 | 8 | 21 | 26 | 26 |
| Cube at 64³ | 38 | 0.02 | 8 | 8 | 8 | 8 | 20 | 26 |
| Cube at 64³ | 38 | 0.05 | 8 | 8 | 8 | 8 | 8 | 26 |
| Cube at 128³ | 77 | 0.005 | 8 | 8 | 8 | 17 | 26 | 26 |
| Cube at 128³ | 77 | 0.05 | 8 | 8 | 8 | 8 | 17 | 26 |
| Cube at 256³ | 154 | 0.005 | 8 | | 8 | 25 | 26 | |
| Cube at 256³ | 154 | 0.05 | 8 | | 8 | 8 | 26 | |
| Slab at 64³ | 51 | 0.002 | 16 | 16 | 16 | 30 | 58 | 58 |
| Slab at 64³ | 51 | 0.02 | 16 | 16 | 16 | 16 | 31 | 58 |
| Slab at 64³ | 51 | 0.05 | 16 | 16 | 16 | 16 | 22 | 56 |
| Slab at 128³ | 102 | 0.002 | 16 | 16 | 19 | 40 | 58 | 58 |
| Slab at 128³ | 102 | 0.005 | 16 | 16 | 16 | 30 | 58 | 58 |
| Slab at 128³ | 102 | 0.05 | 16 | 16 | 16 | 16 | 31 | 58 |
| Slab at 256³ | 205 | 0.005 | 16 | | 20 | 36 | 58 | |
| Slab at 256³ | 205 | 0.02 | 16 | | 16 | 24 | 56 | |
| Slab at 256³ | 205 | 0.05 | 16 | | 16 | 20 | 36 | |

- **26 vertices of a cube** are its corners and the middles of its edges and faces.
- **The slab is 13, 26 and 51 cells thick,** a quarter of its width, and gives way sooner than the cube.

**Distance to the shape,** on the nine shapes of step 3 at 64³. The worst vertex over all of them is the contour's own at every weight: 0.27 cells upright and 0.54 turned for Uniform. Where the weight shows is on shapes with flat faces, here the upright cuboid, in cells:

| Contour | | 0 | 1e-5 | 1e-4 | 1e-3 | 1e-2 |
|---|---|---|---|---|---|---|
| Occupancy | Worst vertex | 0.00002 | 0.0001 | 0.001 | 0.011 | 0.033 |
| Occupancy | Worst middle of a triangle | 0.00002 | 0.0007 | 0.005 | 0.042 | 0.060 |
| Uniform | Worst middle of a triangle | 0.010 | 0.011 | 0.020 | 0.078 | 0.058 |

- **A middle further off than any vertex** is a corner that has slid along its edge (finding 22): it is still on the shape, and the faces next to it are not.

**Time, edges without a best point and best points far from their edge,** at 128³ with Unity in front, median of 5 runs in ms:

| Contour | Model | | 0 | 1e-6 | 1e-5 | 1e-4 | 1e-3 | 1e-2 |
|---|---|---|---|---|---|---|---|---|
| Uniform | Three balls | ms | 268 | 264 | 261 | 256 | 248 | 246 |
| Uniform | Three balls | far | 25 | 24 | 22 | 4 | 0 | 0 |
| Uniform | Cubes and sphere | ms | 192 | 152 | 147 | 146 | 145 | 143 |
| Uniform | Cubes and sphere | no best point | 118,313 | 0 | 0 | 0 | 0 | 0 |
| Uniform | Cube | ms | 181 | 146 | 147 | 148 | 149 | 146 |
| Uniform | Cube | no best point | 247,330 | 0 | 0 | 0 | 0 | 0 |
| Uniform | Turned shapes | ms | 229 | 223 | 225 | 214 | 213 | 205 |
| Uniform | Turned shapes | no best point | 12,972 | 0 | 0 | 0 | 0 | 0 |
| Occupancy | Three balls | ms | 236 | 236 | 233 | 235 | 232 | 230 |
| Occupancy | Slab with band | ms | 266 | 129 | 130 | 129 | 129 | 128 |
| Occupancy | Slab with band | no best point | 298,471 | 0 | 0 | 0 | 0 | 0 |
| Occupancy | Turned shapes | ms | 249 | 214 | 210 | 210 | 210 | 205 |

- **No best point** counts the edges worked out again in the loop, about seven for every collapse. The three balls have none at any weight.
- **Against step 3's code,** run one after the other with Unity in front, median of 7: the three balls at 128³ take 250 ms with step 3's code, 264 with step 4's at a weight of zero and 258 to 261 at the default. The cube takes 177, 181 and 147. At 64³ the three balls take 47.7, 48.4 and 48.2.
- **By stage at the default,** the three balls at 128³: build 26.2, quadrics 2.3, costs 1.0, loop 227 to 230.

**Pairs of triangles that pass through each other** at 64³ stay as step 3 found them up to 1e-3, give or take a few. Two things stand out: Occupancy's turned cube has none at zero and from 1e-4, and 14 and 11 at 1e-6 and 1e-5; Manifold's turned shapes go from 113 to 237 at 1e-1.

**Vertices without a normal,** in 18 contours at 64³ (three algorithms on six models): the normals of a vertex's triangles cancel exactly at one vertex, in Manifold's turned shapes, which is a pair of triangles back to back (finding 15). Occupancy's cut-away sphere has 84 vertices that the contour gave no normal, all of which have one from their triangles.

### The weight of boundary planes (step 2, 2026-11-14)

**Setup:**
- **Models,** as Burst Uniform contours at 64³, each with a line between two materials whose true place is known:
  - **Straight on a flat face:** the slab with a band of `TopologyTest`.
  - **Round on a flat face:** a cylinder of a second material set into the slab, flush with it. The line is a circle of 12.8 cells' radius.
  - **Round on a crease:** the cut-away sphere, where the inner ball meets the flat cut.
- **One collapse at a time,** without the loop: for every edge with an end on the line, the quadric of the edge with the boundary planes at the given weight, and the point Garland and Heckbert take for it: the best point, else the best on the edge, else the better of its ends and middle.
- **Two kinds of edge:** along the line, with both ends on it, and into the line, with one.
- **The contours' own line vertices** are within 0.014, 0.026 and 0.021 cells of the true line.

How far the new vertex of a collapse into the line is from the true line, in cells, for 99% of the edges and for the worst:

| Line | Weight 0 | 0.1 | 1 | 10 | 100 |
|---|---|---|---|---|---|
| Straight, flat | 1.38 / 1.38 | 0.014 / 0.014 | 0.014 / 0.014 | 0.014 / 0.014 | 0.014 / 0.014 |
| Round, flat | 1.77 / 2.03 | 0.025 / 0.026 | 0.025 / 0.026 | 0.025 / 0.026 | 0.025 / 0.026 |
| Round, crease | 1.44 / 1.48 | 0.071 / 0.41 | 0.061 / 0.43 | 0.060 / 0.48 | 0.055 / 0.23 |

The cost of a collapse along the line, as the distance the error bound tests, median in cells:

| Line | Weight 0 | 0.1 | 1 | 10 | 100 |
|---|---|---|---|---|---|
| Round, flat | 0 | 0.0014 | 0.0043 | 0.014 | 0.043 |
| Round, crease | 0.0033 | 0.0033 | 0.0039 | 0.0053 | 0.011 |

**What they show:**
- **At a weight of zero nothing holds the line.** On a flat face no quadric has a best point, the fallback takes an end or the middle of the edge, and the line moves into the face.
- **Any weight above zero puts the vertex on the line,** on a flat face exactly as far from the true line as the contour's own vertices are. On a flat face a collapse into a straight line costs nothing at any weight.
- **On the crease a few vertices land 0.4 cells off** at every weight up to 10. The loop's error bound decides whether those collapses are made, which this does not measure.
- **The weight sets what rounding a curved line costs.** On the flat face that cost is the weight's root times 0.0043 cells.
- **Occupancy** on the same models has its line vertices 0.38 cells from the straight line and up to 0.67 from the round one, and a collapse along its round line costs up to 0.29 cells at a weight of 1.
- **A model that did not work:** two balls a little apart, for a round line on a curved face. Where both are solid the later one sets the material, and near their seam the first ball's own shell is thinner than a cell, so the line has no single place.

### Floats against doubles (step 1, 2026-11-14)

**Setup:**
- **Models:** Burst Uniform contours at 128³: a cube on the grid's axes, a cube turned about its diagonal, the ball of `TopologyTest`, an ellipsoid a quarter as thick as it is wide, and the three balls.
- **Quadrics:** every vertex gets the planes of its triangles, each weighted by a third of its area. An edge's quadric is the sum of its ends'.
- **What is compared:** the error of the midpoint of each edge, from the quadric, against the same error worked out one plane at a time from a point of each plane, which cancels nothing. Errors are given as the root of the error over the area, a distance in cells.
- **Floats:** every operation rounded to a float, as Burst does in strict mode. The arithmetic is emulated in a script outside the repository. The doubles are `BurstQuadric` itself.

| Model | True distance, median | Double off by, median / most | Float off by, median / most | Float costs below zero | Float costs more than half wrong |
|---|---|---|---|---|---|
| Cube | 0 | 0 / 0 | 0 / 0.018 | 24 of 106,722 | 0 of 3,672 |
| Turned cube | 0.0017 | 4e-7 / 1.2e-6 | 0.0097 / 0.030 | 33% | 96% |
| Ball | 0.0086 | 5e-7 / 1.4e-6 | 0.011 / 0.031 | 20% | 86% |
| Flat ellipsoid | 0.0037 | 1e-7 / 1.3e-6 | 0.0029 / 0.031 | 11% | 47% |
| Three balls | 0.0087 | 6e-7 / 2.0e-6 | 0.013 / 0.050 | 23% | 88% |

The last column counts the edges whose true distance is over a thousandth of a cell.

The ball on other grids, float quadrics only:

| Cells | True distance, median | Float off by, median | Float costs more than half wrong |
|---|---|---|---|
| 32³ | 0.035 | 0.0028 | 0 of 5,190 |
| 64³ | 0.017 | 0.0056 | 7% |
| 128³ | 0.0086 | 0.011 | 86% |
| 256³ | 0.0045 | 0.023 | 100% |

**What they show:**
- **A float quadric is wrong by a fixed part of the grid,** about a ten-thousandth of its side, while the costs to be ordered shrink with the cells. The two cross between 64³ and 128³.
- **A double quadric is four orders finer** than the costs at 128³ and three at 256³.
- **Positions suffer less than costs.** Where the system is well-conditioned, the best point of a float quadric is within 0.06 cells of the double one's.

**The best point of every edge,** from `BurstQuadric.minimize` at a line weight of zero:

| Model | Edges | Without a best point | Condition of A, median | Best point over 1 / 2 / 10 edge lengths from its edge | Furthest |
|---|---|---|---|---|---|
| Cube | 106,722 | 106,674 | not finite | 0 / 0 / 0 | 0.5 |
| Turned cube | 118,600 | 0 | 1.6e5 | 219 / 0 / 0 | 1.8 |
| Ball | 83,400 | 0 | 7.9e3 | 14 / 2 / 0 | 3.4 |
| Flat ellipsoid | 61,644 | 3,284 | 7.0e4 | 1,055 / 343 / 56 | 111 |
| Three balls | 156,722 | 0 | 7.8e3 | 83 / 27 / 2 | 11.4 |

- **The condition** is the largest eigenvalue of A over the smallest. It reaches 5.7e6 on the turned cube and 2.4e12 on the flat ellipsoid.
- **A flat face and a gently curved one cannot be told apart by it:** the median is 1.6e5 on the turned cube, whose faces are flat, and 7.0e4 on the flat ellipsoid, which has none.

### What the contours are like (step 0, 2026-11-13)

**Setup:**
- **Models:** the ten of `TopologyTest`, and three that leave the grid: a ball through one side, a slab and a band of a second material that leave through different sides, and a plate whose band leaves with it.
- **Grids:** 32³, 64³ and 128³ cells for Burst and the GPU, 32³ and 64³ for Managed. That is 442 meshes.
- **Settings:** the SVD solver, shared normals, no surface correction. The octrees plain and at 2e-4.
- **Scenes:** the voxelizer of each of the 71 example scenes, with its own settings and shared normals.
- **Implementations agree:** Managed, Burst and the GPU give the same counts on every model and grid they share.

Totals over the 39 Burst meshes of each column:

| | Uniform | Adaptive | Adaptive 2e-4 | Manifold | Manifold 2e-4 | Occupancy |
|---|---|---|---|---|---|---|
| Open edges | 3,444 | 3,444 | 174 | 3,444 | 174 | 3,444 |
| Edges of four triangles | 494 | 494 | 494 | 0 | 8 | 0 |
| Edges of three, or of five or more | 0 | 0 | 0 | 0 | 0 | 0 |
| Vertices with more than one fan | 69 | 69 | 69 | 0 | 0 | 0 |
| Edges between materials | 4,735 | 4,735 | 1,410 | 4,682 | 944 | 4,682 |
| of those, with four triangles | 55 | 55 | 55 | 0 | 0 | 0 |
| Ends of lines between materials | 61 | 61 | 37 | 24 | 0 | 24 |
| of those, at an open edge | 24 | 24 | 0 | 24 | 0 | 24 |
| Vertices where three or more such lines meet | 46 | 46 | 46 | 0 | 3 | 0 |
| Triangles on the vertices of another | 0 | 0 | 0 | 0 | 15 | 0 |
| Vertices without triangles | 0 | 0 | 0 | 0 | 46 | 0 |
| Vertices without a normal | 0 | 0 | 0 | 0 | 0 | 288 |
| Most triangles at a vertex | 15 | 15 | 26 | 12 | 28 | 13 |
| Smallest triangle, in faces of a cell | 7e-6 | 7e-6 | 4e-7 | 6e-6 | 4e-7 | 4e-7 |

**The same in every mesh:**
- No triangle uses a vertex twice, and none has no area.
- No two vertices share a position.
- No normal is of a length other than one or zero.

**What they show:**
- **Open edges** are on the three models that leave the grid and on no other. In a plain contour both ends of each are within a cell and a half of the grid's bounds. After the octrees simplify, they are as far in as the collapsed nodes reach.
- **Edges of four triangles** are on the three balls and the two models of turned shapes in Uniform and Adaptive, and on the turned shapes alone in Manifold at 2e-4.
- **Vertices without a normal** are on the cut-away sphere and on no other model: 42, 84 and 162 at the three grids.
- **The example scenes** are closed manifolds, every one, with none of the rows above but edges between materials and, in three Occupancy scenes, vertices without a normal. The tests of the hard cases have to use the models.

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

All seven were answered as recommended on 2026-11-14.

1. **One Burst simplifier for every implementation?** A managed contour would be simplified by Burst code, and nothing would be simplified on the GPU. The other way is a managed simplifier beside it as a reference, which doubles the algorithm for a mesh the Burst one would give anyway. Recommended: Burst only, with tests against known answers, as Occupancy has.
2. **Quadrics from triangle planes?** Finding 5. The other way seeds them from Hermite QEFs where they exist, which is Burst Adaptive and Manifold only, and needs its own weights. Recommended: triangle planes, with Hermite seeding out of scope.
3. **Doubles for the quadrics?** Finding 6. You kept the QEF solvers in floats on 2026-11-11, since precision was not a must-have. This is different code with nothing to stay in step with, and its ordering depends on it. The other way is floats about a moving origin, which is more code. Recommended: doubles, confirmed by step 1's measurement before step 2 builds on them. Step 1 confirmed it (finding 6).
4. **An error bound in cells and a detail slider?** Findings 7 and 8. The other way is one target, a triangle count or a share of the triangles, as both papers have it. That is simpler, but a cube and a ball then need different numbers. Recommended: both settings as designed.
5. **Topology never changes?** No collapse joins, splits, opens or closes anything, on the surface or on the boundaries between materials. A small ball then never goes below a tetrahedron and never vanishes. The other way allows such collapses at a penalty (Hoppe, section 4.6), and gives fewer triangles on models with small features. Recommended: never, for a project whose algorithms are about keeping the surface manifold.
6. **Crossing triangles: counted, not prevented?** Finding 9. Preventing them means testing every collapse against the triangles near it, from a paper that is not in `Documents/Papers/` (Gumhold, Borodin and Klein, 2003). Recommended: step 3 counts them and reports, and the README says Occupancy's guarantee holds with simplification off.
7. **The record as a full mesh and a list of collapses?** The paper's own form is the simplest mesh and a list of vertex splits. That suits sending a mesh piece by piece, and needs connectivity to refine. Recommended: the full mesh and the collapses, read through the ancestor map. Every contour then runs to its floor, whatever the detail, and step 7 can stop early when the record is not wanted.
