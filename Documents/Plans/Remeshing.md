# Remeshing

Plan written 2026-11-17 for a pass that rebuilds the mesh of a contour after mesh simplification, with three techniques to choose between: evenly shaped triangles from a loop of edge splits, collapses, flips and smoothing; triangles or quads that follow an orientation field; and quads with few irregular vertices.

**Status:** step 0 done on 2026-11-17, on the user's "Alright, begin on step 0." No code is written in the repository. All eight decisions were answered on 2026-11-17 (Decisions). Step 0 raised a ninth, about what the Edge Length is measured against (finding 16), which the user answered on 2026-11-17: against the mesh the pass is given. Step 1 waits for the go-ahead.

**Papers,** all in `Documents/Papers/`:
- *A Remeshing Approach to Multiresolution Modeling* (Botsch and Kobbelt, 2004): the technique called **Isotropic** below. It is its section 4.
- *Instant Field-Aligned Meshes* (Jakob, Tarini, Panozzo and Sorkine-Hornung, 2015): **Field Aligned**.
- *QuadriFlow: A Scalable and Robust Method for Quadrangulation* (Huang, Zhou, Niessner, Shewchuk and Guibas, 2018): **QuadriFlow**. It is built on the second paper and shares its first half.

*Isotropic Surface Remeshing* (Alliez et al., 2003) is also in that folder. The first draft of this plan was written for it. On 2026-11-17 the user added Botsch and Kobbelt's paper in its place: "I've added in the proper isotropic surface remeshing paper. We'll use that one." Nothing in this plan uses the earlier paper any more.

## Goal

Every voxelizer has a **Remeshing** setting, off by default. On, the mesh a contour gives is rebuilt before it is shown, after mesh simplification if that is on, and before its normals are creased or split:

- **Isotropic** gives triangles that are close to equilateral and all about one size, with the mesh's sharp edges, open edges and lines between materials kept as lines, and its topology kept as it was.
- **Field Aligned** gives triangles, or mostly quads, or only quads, laid out in rows that follow the shape's edges and curvature.
- **QuadriFlow** gives quads, with far fewer irregular vertices than Field Aligned at the price of less even faces.

One **Edge Length** setting sets the size of the faces for all three, so they can be compared like for like. It goes by the mesh the pass is given: at 1 the result has as many vertices as that mesh, so the simplifier decides how many vertices there are and the remesh how they are spread (decision 9). With the setting off, every mesh is the one it is today, bit for bit.

It applies to all four algorithms and all three implementations, as mesh simplification does, since it works on the mesh. The wireframe shows the faces the technique made: a quad without its diagonal.

## What the papers do, and what of them is used

- **Isotropic (Botsch and Kobbelt, section 4).** The paper is about editing shapes through a smooth base surface, and remeshes that base surface so that the linear systems solved on it behave well. The remeshing is one section of it.
  - **The loop:** for a target edge length *l*, about five times over:
    1. split every edge longer than 4/3 *l* at its midpoint;
    2. collapse every edge shorter than 4/5 *l* into its midpoint;
    3. flip edges so that vertices come as near as they can to six edges each, or four on a boundary;
    4. move every vertex towards the middle of its neighbours, within its tangent plane.
  - **Equal areas:** after the loop, fewer than 20 rounds of a second smoothing. Each vertex moves towards the middle of its neighbours weighted by their Voronoi areas, again within its tangent plane and damped by a factor λ. Vertices with much area pull others in and so lose some.
  - **What the paper claims:** edges close to *l*, vertices of close to six edges, and vertex areas that differ by under 5% on average.
  - **What the paper says it leaves out, and why it can:** its base surface is smooth, so "there is also no need to handle sharp features". It accepts "slight deviations from the original surface", so vertices are only kept in their tangent planes and never put back on the surface. It gives no rule for which collapses and flips are allowed, and no value for λ.
  - **Used:** the loop and the equalizing of areas, as written.
  - **Added, and my own:** everything this project's meshes need that a smooth base surface does not (finding 9): features that are kept, collapses and flips that are refused, and vertices put back on the input.
  - **Left out:** the rest of the paper, which is the shape editing, the detail encoding, and the comparison of linear solvers (sections 3 and 5).
- **Field Aligned (Jakob et al.).** Two fields are smoothed over the input's vertices, and the mesh is read off them.
  - **Orientation field (section 3.2):** a direction in every vertex's tangent plane, up to quarter turns for quads or sixth turns for triangles. It is smoothed by visiting every vertex and averaging its neighbours' directions, each turned by whichever whole number of steps brings it nearest (equation 3). Measuring the difference in 3D and not in the tangent plane (equation 4, "extrinsic") is what lines the field up with edges and curvature without being told to.
  - **Position field (section 3.3):** a point of a lattice of spacing ρ in every vertex's tangent plane, lined up with its direction. It is smoothed the same way, each neighbour's point shifted by whichever whole number of lattice steps brings it nearest (equations 6 and 7, and the appendix). Lattice lines snap to sharp edges for the same reason.
  - **Hierarchy (section 3.5):** the fields are solved on a coarse graph first, made by merging neighbouring vertices in pairs, level by level, and handed down. Six iterations a level. Without it the iteration sticks in poor local minima.
  - **Extraction (section 3.8):** input vertices whose lattice points agree are merged into one output vertex, and vertices one lattice step apart are joined by an edge. Faces are found by walking round the edges.
  - **Quads only (figure 1):** the fields are solved at twice the edge length, and every face is then split once.
  - **Used:** the extrinsic energies, quarter and sixth turns, the hierarchy, parallel iteration by graph colouring (section 4), the extraction, the split into quads, and constraints that hold the fields to open edges (section 3.7), used here for lines between materials too.
  - **Left out:** the intrinsic energies, half turns, point clouds, the randomized iteration of section 3.4, the brushes and the singularity attractor, and edge lengths that vary over the mesh.
- **QuadriFlow (Huang et al.).** The fields are Field Aligned's, for quads. Field Aligned's position field has irregular points of its own, which the output pays for with triangles and pentagons among the quads. QuadriFlow removes every one of them.
  - **Integer offsets (section 3.2):** for every input edge, how many lattice steps lie between its two ends. Round a triangle they must add up to nothing (equation 3), and a triangle must not be turned over (equation 4).
  - **Minimum cost flow (section 3.3):** the offsets are changed as little as possible so that every triangle adds up. With the triangles as nodes and the offsets as flows between the two triangles of an edge, that is a minimum cost flow problem, which has a polynomial solution where the general problem is NP-hard. Offsets left longer than one step have their edge split.
  - **Turned triangles (section 3.4):** a greedy pass shrinks them away, and a satisfiability solver is tried on the small regions that are left.
  - **Positions (section 3.5):** with the offsets fixed, the lattice points are solved for again as linear least squares (equation 9). A vertex on a sharp edge may only move along it.
  - **Extraction (section 3.6):** edges of offset zero are collapsed. What is left are right-angled triangles, and two that share their long side are a quad.
  - **Used:** every stage.
  - **Changed:** the flow problem is solved exactly by a solver written here. The paper's shortcuts, a hierarchy of networks and maximum flow in place of minimum cost, come in only if the time demands them (decision 6).

## What exists today

- **One place every contour passes through:** `Voxelizer.contour`. The mesh arrives with vertices its triangles share. If Mesh Simplification is on it is simplified there, and the contour is kept with its collapses (`MeshSimplification`). `finish` then creases or splits its normals and shows it. `Voxelizer.simplify` shows the kept contour again under other simplification settings and goes through `finish` too.
- **The GPU** draws its contour straight from its buffers unless the mesh is simplified. A mesh to be simplified is asked for with shared vertices and read back (`Voxelizer.contourVertices`).
- **The mesh:** triangles only. Sub-mesh n holds the triangles of material bit n, and `BurstMesh.WriteMeshJob` writes them in the order they come, sub-mesh by sub-mesh.
- **The simplifier's lists** (`BurstMeshSimplifier.Connectivity`): every vertex with its triangles and edges, for a mesh that changes under them and need not be a manifold.
  - **Sharp edges** of three kinds: open, of more than two triangles, and between materials. Not by angle.
  - **`check`** says why an edge may not collapse: the link conditions of Dey et al., Hoppe's tests on sharp lines, the simplifier's own rule for pinched vertices, and a triangle that would turn over.
  - **`collapse`** collapses an edge into a given position. There is no split and no flip.
- **The wireframe** draws the three edges of every triangle, from barycentric coordinates the vertex shader makes up from the corner number (`Wireframe.cs`, `RenderContourWireframe.shader`).
- **The interactive editor** has a signature and a scheduler for each kind of work: contouring, simplifying again, and another detail. `select` carries the shared settings over to another algorithm's voxelizer. Its scheduler class is called `RemeshScheduler`, where "remesh" means "contour again".
- **Test helpers:** `MeshTopology`, `MeshComparison`, `MeshIntersections`, exact distances to every shape in `ShapeDistanceTest`, offscreen rendering in `OffscreenRender`, and the `Start` hash probe.
- **Not in the project,** and not in its packages: a search for the nearest point of a mesh, a sparse linear solver, a network flow solver, a satisfiability solver.

### Findings that shape the work

From reading the papers and the code. Nothing was measured; step 0 measures.

1. **Isotropic's paper describes the loop in half a page, for a surface that is smooth and may drift.** That is all it needs: its base surface has no sharp features and no error bound. A contour has sharp edges, lines between materials, open edges, sometimes edges of four triangles, and a shape it must stay on. What the paper leaves out is most of the work, and all of it is my addition (finding 9).
   - **What it shares with the simplifier:** collapses. The simplifier's lists and its tests of which collapses keep the topology and the lines are what this technique needs, already built and tested.
   - **What is new:** splits, flips, smoothing, and putting vertices back on the input.

2. **One hook serves every algorithm and implementation, as for simplification.** The pass takes the mesh in `Voxelizer.contour` after the simplifier and before `finish`. With remeshing on, a GPU contour is read back with shared vertices and is not drawn straight from the GPU, as with simplification.

3. **Two of the three techniques undo the simplifier's work before they start.**
   - **Field Aligned and QuadriFlow keep their fields on the input's vertices,** so the input has to be finer than the output (section 3.8 of the second paper). Both reference codes split, longest first and at midpoints, every edge longer than the smaller of half the output's edge length and twice the input's mean edge (finding 11).
   - **A simplified mesh is the opposite:** a cube is 12 triangles. It is split back to triangles of half an edge length, which is about as many vertices as the contour had. What the simplifier leaves these two is a cleaner surface: flat faces that are exactly flat and straight edges without the contour's wobble.
   - **Isotropic splits only down to the edge length asked for,** and works as well from 12 triangles as from 12,000. It gains the same cleaner surface, and a smaller input to search for nearest points.

4. **Not every input is a closed manifold,** and the three techniques cope differently.
   - **What the contours are like** was counted for the simplification plan (its Measurements). Uniform and Adaptive give edges of four triangles and vertices with more than one fan on models where two sheets meet. Every algorithm leaves the mesh open where the surface leaves the grid. Thin features come as closed islands of a dozen triangles. The simplifier keeps all of this.
   - **Isotropic** works on the simplifier's lists, which hold such meshes. An edge of four triangles is a feature: it can be split, is never flipped, and is collapsed only where `check` allows.
   - **Field Aligned** takes any graph. Around an edge of four triangles its vertex normals mean nothing and the paper shows the output breaking down there (its figure 3).
   - **QuadriFlow** needs a manifold: its flow network has an arc for every edge between that edge's two triangles. The authors' README says so and points to another tool for anything else.

   See decision 4.

5. **None of the papers knows materials.** A contour has a sub-mesh for each material, and the lines between them often lie on flat faces where the geometry shows nothing.
   - **Isotropic:** a line between materials is a feature like any other. Its vertices move only along it, and every triangle keeps its sub-mesh through every split, collapse and flip. The line stays where it is, exactly.
   - **Field Aligned and QuadriFlow:** the output's faces do not follow input triangles. The fields can be held to a line by the constraints the papers have for open edges, which makes rows of faces run along it. Each output face then takes the material most of the input under it has. The line between two materials comes out as edges of the new mesh that follow the old line to within about an edge length, and not exactly. How closely is measured.

6. **One definition of a feature serves all three, and it is mine.** Isotropic's paper has no features. The second gives vertices on creases a face's normal and constrains open edges, and the third holds vertices on sharp edges to their lines; neither says what a corner is.
   - **A feature edge** is open, has more than two triangles, lies between two materials, or has its two triangles meet at more than the **Feature Angle**. The first three are the simplifier's sharp edges.
   - **A corner** is a vertex where the number of feature edges is not two; or where a feature line turns by more than the angle, such as the corner of a square of a second material on a flat face; or a vertex with no feature edge whose triangles' normals spread by more than the angle, such as a cone's tip.
   - **Corners are kept in place** by Isotropic and QuadriFlow. Field Aligned has no way to hold a vertex; its corners end up near and not at their place.
   - **Step 0 measured the skeleton this gives** (finding 17). It is exact on shapes whose edges run along the grid. Where an edge crosses the grid at an angle, or where round shapes meet, it breaks up into short lines and tiny patches, on contours and simplified meshes alike. Features found by angle therefore hold only at the scale of the edge length (finding 9).

7. **Only Isotropic keeps the topology.**
   - **Isotropic:** a split changes no topology. A collapse is made only where the simplifier's `check` allows it, which is where the topology of the surface and of its sharp lines stays what it was. A flip is made only where the new edge does not exist yet. The tests hold the output to the input's pieces, its Euler characteristic and each sub-mesh's, as they hold the simplifier.
   - **Field Aligned:** the paper says its extraction "can sometimes produce non-manifold output" at coarse settings, and its code fills holes of up to six sides and leaves larger ones.
   - **QuadriFlow:** "nearly always manifold". On the authors' large test a fifth of the outputs were not watertight, because the satisfiability step does not always succeed.
   - **What it means:** the simplifier's promise that nothing joins, splits, opens or closes holds for Isotropic and cannot be given for the other two. Their output is counted with `MeshTopology` and the result is shown (decision 5).

8. **Quads have to pass through a pipeline of triangles.** A Unity mesh for rendering, `BurstCreases`, the statistics and the wireframe all work on triangles.
   - **A polygon becomes triangles,** and a byte for each triangle says which of its three edges are edges of its polygon.
   - **The wireframe leaves out the others.** Its shader already works per triangle and per corner, so it needs one more buffer and a mask.
   - **The triangle order has to survive `finish`.** `WriteMeshJob` keeps the order within a sub-mesh, `BurstCreases` rewrites triangles in place, and `splitVertices` goes through them in order. `Mesh.OptimizeReorderVertexBuffer` should only move vertices; step 1 checks it.

   See decision 3.

9. **What Isotropic adds to its paper.** Each of these is common practice for this loop in later writing, such as Botsch et al.'s book *Polygon Mesh Processing*, which is not in `Documents/Papers/`. Here they are worked out from what this project already has.
   - **Features are kept.**
     - **Split:** the two halves of a feature edge are feature edges of the same kind, and the new vertex is on the line.
     - **Collapse:** an edge along a feature line collapses into its midpoint; an edge from a line into the surface collapses into its end on the line; an edge at a corner collapses into the corner. An edge that joins two lines, or two corners, without being a feature itself is not collapsed. Most of this is what `check` already refuses under Hoppe's tests; the position is what is new.
     - **Flip:** never a feature edge, and never where the new edge's two triangles would meet at more than the Feature Angle, which would make a feature where there was none.
     - **Smoothing:** a vertex on a line moves along the line, and a corner does not move.
   - **Two strengths of feature,** since step 0 (finding 17):
     - **Hard:** open edges, edges of more than two triangles and lines between materials. They are the simplifier's sharp edges, `check` holds their topology by Hoppe's tests, and the rules above hold them at every length.
     - **By angle:** the rules above hold them, but they are not sharp edges to `check`. A feature edge by angle shorter than 4/5 *l* collapses like any other edge, and two corners by angle closer than that may become one, where `check` allows it. Edges and corners of the shape that are further apart than that keep their place, and the zigzags and slivers of finding 17, which are shorter, go. A detail of the shape smaller than the edge length goes too, as any detail under a remesh's resolution does.
     - **What this changes:** the first draft had the angle kind as a fourth kind of sharp edge in the simplifier's lists. That would have put the noise under Hoppe's tests, which never let two such corners meet.
   - **Collapses and flips are refused** where they would change the topology or turn a triangle over (finding 7), and a collapse also where it would leave an edge longer than 4/3 *l*, which the next round would only split again.
   - **Vertices are put back on the input.** Moving within a tangent plane leaves a curved surface a little each time, and over five rounds and the area smoothing after them the mesh would shrink. After every move a vertex goes to the nearest point of the input:
     - **a vertex on a line:** of its own line of the input;
     - **any other:** of the input's triangles of its own material whose normals are within 90° of its own, near where it was.
     - **Why not the patches:** the first draft had the patch between feature lines. Step 0 found patches by angle of under a cell² by the hundred on some models (finding 17), and a vertex would be held to a sliver. The normal keeps a vertex from going to the far side of a thin wall.
     - **The input is the surface.** With simplification on that is the simplified mesh, so a curved shape keeps the simplifier's flat facets, with new vertices spread over them. The distance to the true shape is the simplifier's and no worse.
   - **A move that would turn a triangle over is not made.**
   - **The damping factor λ** of the area smoothing has no value in the paper. Step 3 sets it by measurement.
   - **Edges are worked longest first when splitting and shortest first when collapsing,** with ties by index, so the result is the same every time. The paper gives no order.

10. **What has to be written from nothing.**

    | | Isotropic | Field Aligned | QuadriFlow |
    |---|---|---|---|
    | A mesh that changes | Split and flip, added to the simplifier's lists | — | — |
    | Nearest point of the input | A grid over its triangles | — | — |
    | Sparse least squares | — | — | The positions (equation 9), by conjugate gradients in doubles |
    | Combinatorial solvers | — | — | Minimum cost flow, and perhaps satisfiability |
    | Shared with another | The feature skeleton; its split is how the other two prepare their input | Fields, hierarchy | All of Field Aligned's first half |

    - **Isotropic is now the smallest** of the three, and the only one with nothing to solve.
    - **QuadriFlow cannot be built without Field Aligned's fields.**

11. **Both field papers have public reference code, and it differs from the papers in many places.** Step 0 read it: Instant Meshes at commit 7b31608 (2019-11-03) and QuadriFlow at commit 810b7a0 (2019-12-07), the files that hold the algorithm, read by two agents with line references, and the claims below that matter most checked against the source by me. Nothing was run. Isotropic's paper has no reference code.
    - **Licences:** Instant Meshes (github.com/wjakob/instant-meshes) is BSD 3-clause with one clause added: changes that are published are licensed back to the authors. QuadriFlow (github.com/hjwdzh/QuadriFlow) is MIT. Both allow this project to follow them, with credit.
    - **The rule this plan follows** (decision 2): where a paper speaks, the paper; where it is silent, the code. Each item below says which applies.
    - **The draft's claims, checked:**
      - **Splitting the input:** both split every edge longer than min(ρ/2, 2 × the mean edge), longest first, at midpoints, until none is. Instant Meshes only does it when the longest edge is over ρ/2 or twice the mean. The draft said "half the output's edge length", which was the half of it. *Silent paper: the code.*
      - **Constraints along open edges:** weight 1 on both fields, but off by default in both (`-b`, `-boundary`). The weights are clamped to 1 on coarse levels, so the constraints are hard everywhere. The direction held is the edge's own, the position the vertex's own. *Silent paper: the code, and this plan turns them on and adds the lines between materials.*
      - **Snapping:** a vertex within 0.3 ρ of an edge's line goes to that edge's **midpoint**, and to a vertex within 0.3 ρ of it, repeated until nothing moves; for quads, then diagonals are removed by how far they are from √2 times their sides, with no threshold. *Silent paper: the code.*
      - **Faces:** walked from three to eight sides, triangles first, a larger face cut at its best-angled corner, holes of up to six sides filled. Confirmed.
      - **Quads only:** the default in the code. Every quad is split into four at the length asked for, so its edges come out at half of it. *The paper (figure 1) solves at twice the length and splits once, and this plan keeps the paper.*
      - **QuadriFlow's flow:** confirmed: a maximum flow by default (Boykov and Kolmogorov, or unit Edmonds and Karp under 20 units of supply), network simplex with `-mcf`. **There is no hierarchy of networks in the code at all:** it is built with one level. The paper describes one, and admits the maximum flow.
      - **Satisfiability:** off by default, through an outside program. Its clauses forbid only a negative determinant where the paper writes "≤ 0".
      - **The varying scale:** in the code and not in the paper. Without `-adaptive` it is all ones and changes nothing.
    - **Field Aligned, more that the paper leaves out or says otherwise:**
      - **Weights and areas:** uniform weights. The areas are barycentric thirds of triangles; *the paper says Voronoi areas for a mesh, and this plan keeps the paper.*
      - **The hierarchy's score** is the normals' dot product times max(Aᵢ/Aⱼ, Aⱼ/Aᵢ); *the paper writes min.* The paper decides, and step 4 measures both, since they differ by one word and the paper may have a slip.
      - **Normals at creases:** the code takes the mean of one sector of triangles, the paper a single face's normal, and by default the code has no creases at all. *The paper decides,* and step 4 compares it with no creases (finding 17).
      - **Iterations:** six per level from the coarsest down, the vertices of one colour at a time. After the finest level the coarse orientations are made again from the finest before the positions are solved. *Silent paper: the code.*
      - **The random start:** positions are drawn near each vertex, not in the bounding box. Modulo the lattice it is the same thing.
      - **The search for lattice shifts:** 16 pairs for quads as the paper says, and also 16 for triangles, where *the paper says 36, and decides.*
      - **Extraction:** the integer shifts are always found the extrinsic way. A vertex is left out when it gathered no more than a tenth of the mean number of members (*the paper: fewer than a tenth*). Output positions are the members' lattice points weighted by exp(−9 |p − v|² / ρ²).
      - **Quad Dominant:** the code makes every triangle and pentagon a fan of triangles round a new middle vertex. *The paper keeps them as they are, and so does this plan,* whose polygons can be of any size (finding 8).
      - **Smoothing afterwards:** two rounds, each vertex not on a crease to the middle of its neighbours and back onto the input along its normal. *Silent paper: the code.*
      - **Non-manifold input:** the vertices of an edge of more than two triangles get no neighbours and no area, and are left out of the fields. The sheets of decision 4 come first here.
      - **Floats,** and not the same output twice unless run with `-d`.
      - **Not to copy:** a constraint bug in the hierarchy, where the position constraint is moved by the direction; triangles accepting a diagonal shift; a 16-bit counter of members that overflows.
    - **QuadriFlow, more that the paper leaves out or says otherwise:**
      - **The edge length** is √(area / faces). An edge whose two faces meet at more than 60° is sharp, with `-sharp`, which is off by default.
      - **Balancing** goes by a spanning tree and evens out supply and demand for each connected piece of the network, by moving offsets at random. Nothing checks that they come out even, and the capacity is raised at most ten times. **So the code does not guarantee what the paper claims,** that no position singularity is left; triangles that still do not add up are left as they are. This plan's test that every triangle adds up exactly needs what the code leaves out: balancing to zero in every piece, and raising the capacity until the flow is full, which theorem 3.1 says is then always reached.
      - **The greedy pass** accepts a shrink when the number of turned triangles round it drops, where *the paper says their area, and decides.* It never moves a vertex of an open edge. Turned triangles it leaves are made into quads like any others.
      - **Positions:** a direct sparse Cholesky solve, once for each group of vertices joined by edges of offset zero, with the target the offset times the mean of both ends' frames. Vertices along sharp lines are first spaced evenly along them, and corners held at their lattice point. *The paper says linear least squares and nothing about how; this plan solves it by conjugate gradients, for want of a sparse Cholesky, on the same groups, and holds corners at the input's corner, which is mine.*
      - **Extraction:** quads only. A triangle without a partner is dropped, and holes of under 25 edges are filled with quads. *The paper does not say; this plan keeps such a triangle, which leaves the mesh closed.*
      - **Afterwards, not in the paper:** vertices of 2 edges merged away and of more than 5 split, and the positions solved again on the quad mesh ten times with a little smoothing. *Left out;* step 7 measures what the output looks like without them, and asks if they seem needed.
      - **Non-manifold input:** an edge of more than two triangles becomes open edges, and vertices are split by fan until each has one. Two sheets that touch along an edge are left with a slit there. The sheets of decision 4 close both instead (finding 18).

12. **The output has to be the same every time.** The second paper starts its fields from random directions and positions. A fixed seed, colours visited in a fixed order and stable sorts make the same mesh give the same output, bit for bit, which the tests and the `Start` hashes rely on. Isotropic's loop is in a fixed order (finding 9). A GPU contour has its vertices in another order every run, so it will remesh alike and not the same, as it simplifies today.

13. **How long they will take is not known.** These are the papers' own figures, not measurements of this project:
    - **Isotropic:** 100,000 triangles in under 5 seconds on a Pentium 4 of 2004, on one core. Its splits, collapses and flips are one after another, as the simplifier's loop is; its smoothing and the search for nearest points are parallel. The simplifier takes 206 ms for the three balls at 128³, and this is work of the same kind and order.
    - **Field Aligned:** under a second for inputs under 100,000 triangles on twelve cores.
    - **QuadriFlow:** five seconds for a million faces, with the fields solved on a GPU and the rest on one core.

    The interactive editor's scheduler already copes with slow work: it waits for changes to settle and pauses over a second.

14. **Two pieces of solver code are QuadriFlow's alone,** and each is a known algorithm with answers in closed form to test against: conjugate gradients and minimum cost flow. They get their own files and their own tests, apart from the technique.

15. **What the pass gets** (step 0, Measurements). Over the 13 models, the four algorithms and Manifold clustered at 2e-4, at 32³, 64³ and 128³:
    - **Contours have edges of about one cell:** the median is 0.95 to 1.0 cells and 90% are between 0.57 and 1.47, at every grid. The longest is about 5 cells, and the shortest a ten-thousandth.
    - **Simplified meshes have edges of 2 to 5 cells at the median,** growing with the grid, and their longest edge spans the model: 36, 72 and 145 cells at the three grids.
    - **Triangles:** a contour's smallest angle is 38° on the mean, 42° from Occupancy; a simplified mesh's 40° to 43°. On the median model under one triangle in a hundred has an angle under 10°. A remesh is to bring these towards 60°.
    - **Vertices of six edges:** half of a contour's inner vertices, 0.69 of Occupancy's, and a third to two fifths of a simplified mesh's.
    - **Manifold clustered at 2e-4** has the longest edges and the worst angles of the contours, as its octree merges cells.

16. **An edge length in cells cannot follow the simplifier.** The edge length at which an isotropic remesh keeps the vertex count of the mesh it gets (from its area, a vertex for every (√3/2) *l*² of it):
    - **of a contour,** 0.87 to 1.22 cells, on every model and grid;
    - **of a simplified mesh,** 1.2 to 6.5 cells on the round and mixed models and 7 to 48 on the flat ones, growing with each grid, and doubling with it for the flat ones.

    So at any one length in cells, a remesh after simplifying gives back many times the vertices the simplifier left. At 2 cells on the 13 models at 64³ it gives 27,000 vertices where the simplifier left 6,600 and the contour had 118,000. The cube the simplifier brings down to 8 vertices gets 440, and the slab with a band 2,300 where it had 16. At 128³ the same length gives a quarter of the contour.
    - **The three papers' own defaults all go by the mesh they are given:** Botsch and Kobbelt take "slightly less than the average edge length of the original surface", Instant Meshes' code aims at a sixteenth of the input's vertices, and QuadriFlow's at as many faces as the input has vertices.
    - **What that would mean here:** the simplifier's Error and Detail would set how many vertices there are, and the remesh how they are spread. It is still a uniform spread: a round part comes out coarser than the simplifier left it and a flat part finer.

    Decision 9 settled it on 2026-11-17: the length goes by the mesh the pass is given (Design, Settings).

17. **Features found by angle are noisy where an edge crosses the grid.** Step 0 counted the skeleton of finding 6 at 20° to 60° (Measurements).
    - **Exact** wherever edges run along the grid: the cube, both slabs, the plate, the hollows and the cubes with a ball give their corners, lines and faces and nothing more, contoured and simplified, at every angle tried.
    - **Broken up** where an edge crosses the grid at an angle and where round shapes meet. Simplified at 128³ at 40°, the turned shapes have 1,626 lines, 632 of them under a cell long, and 732 patches, 590 of them under a cell². The new shapes and the three balls are alike, and so is the contour before simplifying. Raising the angle to 60° takes away a fifth to a half of the lines, by algorithm, and leaves the rest.
    - **The cause** is the slivers of the creased normals plan's finding 9. Triangles straddle a sharp edge, so the edge by angle zigzags through vertices beside it, and the sliver between is a patch of its own.
    - **The open rim** of a ball that leaves the grid is a staircase: 153 of its vertices turn by more than 40°, which would make each a corner.
    - **What it changes** in Isotropic: features found by angle hold only at the scale of the edge length (finding 9). In Field Aligned, normals at creases would follow the noise, so step 4 compares them with no creases at all, which is the reference code's default.
    - **Corners where triangles spread** (the third kind of finding 6, measured from their mean normal) are rare: at most 19 on any model at any angle. Measured between the triangles themselves they are far more common at 20°, so the mean stays.

18. **Splitting into sheets edge by edge is not enough.** Step 0 built the split of decision 4 in the probe (Measurements).
    - **Where it is needed:** 40 meshes have edges of more than two triangles, from Uniform and Adaptive on the turned shapes, the new shapes and the three balls, and from Manifold clustered on the first two. Every one of those edges has exactly four triangles.
    - **The pairing:** round each such edge the four triangles are sorted by angle, and each is paired with the next across a wedge inside the solid.
    - **Where that has no answer:** up to nine edges in a mesh, where the windings do not alternate round the edge (→, ←, ←, →). Two sheets pass through each other there. Of those, the probe paired each triangle with its neighbour of opposite winding.
    - **What came out:** no open edge and no pinched vertex anywhere. The three balls stay one surface of a ball's topology, and the turned and new shapes come apart into their pieces. But 16 meshes at 64³ and 128³ keep one to three edges of four triangles. Five of the six looked at were paired by the fallback. Their two pairs still meet at both ends, since the sheets go on through each other there.
    - **What the build has to do instead:** choose, at such an edge, between the two pairings that suit the windings by what the fans round its two ends become, so that each is a disk. Step 4 does that and counts what is left.

## Design

### Settings

On `Voxelizer`, under a **Remeshing** header after Mesh Simplification, carried over by the editor's `select`:

| Setting | Default | What it does |
|---|---|---|
| Technique | Off | Off, Isotropic, Field Aligned or QuadriFlow. Off, every mesh is what it is without this feature. |
| Edge Length | 1 | How long the edges of the new faces are, against the mesh the pass is given: 1 is the length at which the result has as many vertices as that mesh, 0.5 gives about four times as many and 2 about a quarter. On an unsimplified contour 1 is about one cell (finding 16, decision 9). |
| Feature Angle | 40 | Two triangles that meet at more than this many degrees have a sharp edge between them, which is kept. The same number as the Crease Angle's default, and a setting of its own: one decides where the mesh is held, the other how it is shaded. |
| Iterations | 5 | Isotropic only. How many times the loop of splits, collapses, flips and smoothing is gone through. The paper's "about 5". |
| Area Iterations | 10 | Isotropic only. How many rounds of smoothing by vertex area follow the loop. Zero leaves them out. The paper's "fewer than 20"; provisional until step 3 measures. |
| Faces | Quad Dominant | Field Aligned only. **Triangles**; **Quad Dominant**, quads with triangles and pentagons where rows of faces begin and end; or **Quads**, quads only. |

- **How the length is had** (decision 9). From the mesh the pass is given: its area *A*, the sum of its triangles', and its vertex count *V*, of the vertices its triangles use. A mesh of equilateral triangles of side *l* has a vertex for every (√3/2) *l*² of its area, and one of squares a vertex for every *l*². So the length at 1 is √(*A* / ((√3/2) *V*)) for Isotropic and Field Aligned's triangles, and √(*A* / *V*) for quads, Field Aligned's or QuadriFlow's; the setting multiplies it. Field Aligned's Quads are solved at twice that and split, so they come out at it too.
- **What follows from it:** the simplifier's Error and Detail now change how many vertices a remesh has, and the interactive editor already remeshes after each. A mesh with few vertices gets long edges: the cube the simplifier leaves at 8 vertices is remeshed at about 15 cells at 64³.
- **Edge length and not a vertex count.** All three papers take a length, or a count that is turned into one. A share of the mesh's own length reads the same on a cube and a ball, and on a contour and a simplified one.
- **Not settings:** the split and collapse limits of 4/3 and 4/5, the damping factor, the random seed, the six iterations a level, and Field Aligned's extrinsic energy.

### Where it sits

- **`Voxelizer.contour`:** after the simplifier's block and before `finish`. The pass takes a mesh with shared vertices and gives one back.
- **What the voxelizer keeps:** the pass's input, as managed arrays, in a class of its own beside `MeshSimplification`. With simplification on that input is small; without, it is the contour, 4 MB at 128³.
- **`Voxelizer.remesh( )`** shows the kept input again under the remeshing settings as they are now, without contouring or simplifying. It gives nothing when there is nothing kept, as `simplify` does.
- **`Voxelizer.simplify( )`** remeshes what it makes, so another detail or error is shown remeshed.
- **`contourVertices`** asks for shared vertices when remeshing is on, and such a contour is not drawn straight from the GPU.
- **`ContourStatistics`** gains the time the pass took, the size of the mesh that went in, how many faces of each number of sides came out, and what `MeshTopology` counts of the output that the input did not have (finding 7).
- **When a technique cannot do its work** it gives the input back unchanged with the reason, which the statistics carry. Nothing throws and nothing half-made is shown.
- **Burst only** (decision 8). Doubles, except Field Aligned's fields, which the paper says floats do for.

### What the three share

In `Assets/Scripts/Burst`, as jobs over native arrays of positions, normals and `BurstMesh.Triangle`:

- **The feature skeleton** (finding 6): the kind of every edge, the corners, and the lines as runs of edges between corners or closed loops, each marked hard or by angle (finding 9). The simplifier's lists keep their three kinds of sharp edge: the angle kind is the remesher's own, kept beside them, and `check` never sees it (finding 17).
- **Splitting long edges:** the first operation of Isotropic's loop. Field Aligned and QuadriFlow prepare their input with it (finding 3).
- **Sheets** (decision 4): the mesh split along its edges of four triangles and at its vertices of several fans, so that what is left is a manifold. Field Aligned and QuadriFlow work on the sheets. Isotropic does not need them.
  - **The pairing round an edge of four triangles,** as step 0's probe found it (finding 18): across the wedges inside the solid where the windings alternate round the edge; else each triangle with its neighbour of opposite winding.
  - **Then the ends,** which step 0 found the pairing alone leaves wrong where sheets pass through each other: where the two pairs of an edge still meet at both its ends, the other pairing that suits the windings is tried, and kept if it leaves each fan a disk.
  - **What still cannot be paired** is left as open edges, counted, and shown as finding 7 has it.
- **Polygons out,** for the two field techniques: each polygon as triangles with the flags of finding 8. A quad is split along the diagonal that leaves its two triangles flatter.
- **Materials out,** for the two field techniques: an output face takes the sub-mesh that most of the input's area mapped into it has.

### Isotropic

1. **The lists:** the simplifier's `Connectivity`, built from the input, with the feature skeleton marked in it. Two operations are added to it, each updating everything `collapse` updates:
   - **`split`** of an edge at a point, for an edge of any number of triangles. Every triangle of the edge becomes two of its sub-mesh.
   - **`flip`** of an edge of two triangles.
2. **The input, kept to go back to:** its triangles in a grid for the search of the nearest point, each with its sub-mesh and normal, and its feature lines (finding 9).
3. **The loop,** for the set number of iterations, with *l* the edge length:
   1. **Split** every edge longer than 4/3 *l* at its midpoint, longest first, the new edges among them, until none is longer.
   2. **Collapse** every edge shorter than 4/5 *l*, shortest first, into the position finding 9 gives it, where `check` allows it and no edge longer than 4/3 *l* comes of it. A feature by angle does not stop it (finding 9).
   3. **Flip** every edge of two triangles that is not a feature where that brings its four vertices nearer their number of edges: six, or four on an open edge, as the paper has it. A flip is refused where the new edge exists, where a triangle would turn over, and where it would make a feature (finding 9).
   4. **Smooth:** every vertex towards the middle of its neighbours, within its tangent plane, then back onto the input. All moves of a round are worked out from the positions the round began with, in parallel.
4. **Equal areas,** for the set number of rounds: the same move towards the middle of the neighbours weighted by their Voronoi areas, damped, and back onto the input. The Voronoi area is the one the paper's section 3 describes: from the circumcentres of a vertex's triangles, or the midpoints of edges where an angle is obtuse.
5. **Out:** the positions; for each vertex the input's normal at the point it was put back on; the triangles with their sub-meshes.

### Field Aligned

1. **Preparation:** the sheets, and every edge longer than min(ρ/2, 2 × the mean edge) split, longest first, at its midpoint (findings 3 and 11).
2. **The graph:** the vertices with the mesh's edges between them, uniform weights, and a normal and a Voronoi area for each. A vertex on a feature edge takes the normal of one of its faces, as section 3.5 has it; step 4 compares that with no creases (finding 17).
3. **The hierarchy** of section 3.5, scored by the paper's min of the two areas' ratios, with the code's max measured beside it at step 4 (finding 11), and a colouring of every level so that vertices of one colour share no edge and can be visited in parallel.
4. **The orientation field,** then **the position field,** six iterations a level from the coarsest level down, from a fixed random start. Before the positions, the coarse orientations are made again from the finest, as the code does. Along open edges and lines between materials both fields are held, hard: the direction to the line's, and the lattice to pass through it (section 3.7). The search for lattice shifts takes the paper's 16 pairs for quads and 36 for triangles.
5. **Extraction** as section 3.8 has it, with what the code adds where the paper is silent (finding 11): merging in order of distance, leaving out vertices too few inputs agree on, snapping within 0.3 ρ, taking out the diagonals of quads, walking the faces of three to eight sides, filling holes of up to six.
6. **Faces:**
   - **Triangles:** sixth turns and a triangular lattice.
   - **Quad Dominant:** quarter turns and a square lattice. Triangles and pentagons stay as they are, as polygons, where the code makes them fans round a new vertex.
   - **Quads:** the same at twice the edge length, then every face split into quads round its middle, so the edges come out at the length that was asked for. The code splits at the length asked for, which halves it.
7. **Smoothing,** two rounds as the code has them: each vertex not on a crease or a held line to the middle of its neighbours, and back onto the input.
8. **Out:** positions and normals as the weighted means of section 3.8, materials by area (finding 5), polygons as triangles with their flags.

### QuadriFlow

1. **Fields:** Field Aligned's steps 1 to 4 for quads. The input must be a manifold here, so the sheets are not optional.
2. **Offsets:** for every edge, from the fields' own whole numbers (definition 3.1).
3. **The flow network** of section 3.3: a breadth-first walk over the triangles turns each triangle's equation so that an edge's offset enters its two triangles with opposite signs; offsets it cannot balance, and those of open edges, are fixed; the fixed ones are adjusted until supply equals demand, in every connected piece of the network, and that they do is checked. The code does the last for each piece too, but at random and without the check (finding 11).
4. **The flow:** successive shortest paths, which gives a minimum cost flow exactly. Offsets start within two steps, and the limit is raised until the flow is full, which theorem 3.1 guarantees once supply equals demand. The code stops after ten raises.
5. **Long offsets:** an edge whose offset is over one step is split in the middle (figure 6).
6. **Turned triangles:** the greedy pass of section 3.4, which accepts a shrink when the turned area round it drops, as the paper says (the code counts triangles). What it leaves is counted (decision 6).
7. **Positions:** equation 9 by conjugate gradients, with one unknown for each group of vertices joined by offsets of zero, as the code has it; each in its tangent plane, a vertex on a feature edge along that edge, and a corner at the input's corner.
8. **Extraction** (section 3.6): edges of offset zero collapsed, right-angled triangles paired along their long sides. A triangle left without a partner, as at an open edge, stays a triangle, where the code drops it and fills the hole. The code's work after extraction, which the paper does not have, is left out until step 7 shows whether it is wanted (finding 11).
9. **Out:** as Field Aligned's.

### Wireframe

`Wireframe.draw` takes the flags of finding 8 as an optional buffer, one byte a triangle in the mesh's index order, and the fragment shader leaves out the distance to an edge that is not flagged. Without the buffer it draws as today. The voxelizer keeps the flags beside the mesh it shows and lets go of them with it.

### Interactive editor

- **A fourth signature and scheduler,** for the remeshing settings. A change to them alone calls `remesh`. It comes after the other three: a contour brings everything with it, simplifying again and another detail bring the remeshing with them.
- **Whether remeshing is on** is not part of the contour's signature, unlike simplification: the input is kept either way, so turning it on or changing the technique takes no contour. The exception is a contour drawn straight from the GPU, which the editor never has.
- **`select`** carries the settings over.
- **The inspector** has a line for the pass: what went in, what came out by number of sides, how long it took, and a warning when the output has open or non-manifold edges the input did not have, or when the pass gave the input back.

### Example scenes

New scenes in `Assets/Scenes/Remeshing`, copies of existing Burst scenes with the wireframe on, at least one for each technique and one with several materials. Which ones is settled at the last step. No existing scene is changed without asking.

## Tests

There is no second implementation, so the tests hold each piece to answers known in closed form and each technique to invariants, as the simplifier's do. Each new test is shown to fail on a deliberate mistake. What a technique does not promise (finding 7) is measured and reported, not asserted.

- **Shared pieces,** on meshes built by hand:
  - **Skeleton:** a box has 12 feature edges and 8 corners; a prism's sides are features exactly when they turn more than the angle; a square of a second material on a flat face has four corners; a cone has a tip; a flat disc of a second material is one closed line without a corner.
  - **Sheets:** two boxes that share an edge come apart into two closed manifolds; a mesh that is a manifold comes back as it was; two sheets built to pass through each other at an edge come apart into two; the contours of finding 18 come apart with no edge of more than two triangles and no pinched vertex left, or with what is left counted.
  - **Polygons:** the flags of a quad, a pentagon and a triangle; the welded triangles give back the polygons.
- **Isotropic:**
  - **Split and flip,** as the simplifier's collapse is tested: after any run of splits, flips and collapses the lists are those of a mesh built from nothing; a split keeps the surface point for point, every sub-mesh's area, and the kind of every feature; a flip of a flat pair gives the other diagonal; each reason for refusing a flip has a mesh that gives it and one next to it that flips. Edges of one, two and four triangles are split.
  - **Nearest point:** against a search over every triangle, on points near faces, edges and vertices, and never onto another patch.
  - **Positions of collapses:** each case of finding 9 on a box with a line between two materials.
  - **A flat square:** after the loop, the share of edges between 4/5 and 4/3 of the length, the share of inner vertices with six edges, and the mean smallest angle, each over a bar step 3 measures and sets.
  - **Equal areas:** the spread of vertex areas is smaller after the rounds than before, by a factor step 3 measures; the paper claims about five.
  - **An upright box** stays the box: every vertex on a face, every edge vertex on its edge, every corner at its corner, the volume unchanged.
  - **A turned box,** whose contour has the zigzags of finding 17, comes out with its own 12 edges and 8 corners as the only features at the Feature Angle, within a tolerance of the true ones that step 3 measures.
  - **Whole,** over the models of `TopologyTest`, every algorithm, with and without simplification:
    - every vertex is on the input's surface, every feature vertex on its line, every corner where it was;
    - lines between materials are the input's as sets of points, and every sub-mesh has the input's area to within rounding where it is flat;
    - the simplifier's invariants: the pieces, the Euler characteristic and each sub-mesh's, the special vertices and the lines;
    - no triangle turned over or without area;
    - an edge length larger than a small island leaves the island a closed piece;
    - the same output twice.
- **Field Aligned:**
  - **The operators:** the best turns and the best lattice shifts of two vertices against a search over all of them; rounding to the lattice against the same; the point between two tangent planes satisfies its two equations and is the nearest that does.
  - **Hierarchy and colouring:** each level about half the one below, areas add up, no two neighbours of one colour.
  - **A flat grid:** the fields come out constant, and the extraction gives a square or triangular lattice of the right spacing with every inner vertex regular.
  - **A box along the axes:** rows of faces run along its edges, and its vertices near edges are within a measured tolerance of them.
  - **Whole:** vertices within a tolerance of the input's surface; every sub-mesh's area within a tolerance of the input's; the same output twice.
- **QuadriFlow:**
  - **Conjugate gradients:** against systems with known solutions, a singular one among them.
  - **The flow solver:** the paper's own example (figure 4), small networks against a search over every flow, an infeasible one, and on every solution: conservation, capacities, and no cheaper cycle in what is left.
  - **After the flow:** every triangle's offsets add up to nothing, on every model. That is the paper's claim and it is exact.
  - **After the positions:** the normal equations hold; feature vertices are on their lines.
  - **Whole:** only quads on closed manifolds, but for triangles the statistics count; on a flat grid a square lattice; the same output twice.
- **The frame:**
  - With the technique off, the `Start` hashes are step 0's.
  - With `split` and `flip` added to its lists, the simplifier's meshes are what they were, collapse for collapse.
  - `remesh` after a change of edge length or technique shows what a fresh contour under those settings shows, without contouring.
  - **The length** (decision 9): on a flat square and a fine sphere, by hand, the length at 1 is the formula's for triangles and for quads. On the 13 models, contoured and simplified, each technique at 1 gives a vertex count within a tolerance of the input's that its own step measures, and at 0.5 about four times as many.
  - A GPU contour is read back and remeshed; creased and split normals after remeshing; past 65,535 vertices.
  - A technique that gives up shows the input and reports why.
  - The wireframe of a quad mesh has no diagonals, by offscreen render.
- **Editor:** the settings carry over; a change of remeshing settings does not contour; a change of detail remeshes.
- **Measured and reported for every technique,** on the 13 models of the simplification probe and all four algorithms, with and without simplification: time by stage; faces by number of sides and vertices by number of edges; the spread of angles, areas and edge lengths, as table 1 of the second paper and table 2 of the third have them; distance to the shapes; open and non-manifold edges and pieces against the input's; how far lines between materials moved.

## Steps

Each step compiles, passes every test, and is reviewed before the next one starts. Nothing is committed for you. The order is by what depends on what: the frame, then Isotropic, which is the smallest and whose split the others use, then Field Aligned, then QuadriFlow, which needs Field Aligned's fields.

0. **Baseline and what the pass will meet (small).** Both suites green, the `Start` hashes, timings with Unity in front. A probe over the 13 models, every algorithm, simplified and not: edge lengths in cells; feature edges, corners and lines at angles from 20° to 60°; patches between feature lines and their sizes; edges of four triangles and what the sheets make of them. The parts of the reference code that finding 11 lists, read properly. The default edge length comes out of this.
   - **Done 2026-11-17:**
     - **Suites:** edit mode 3326 of 3326 in 321 s, play mode 21 of 21, Unity in front throughout.
     - **`Start` hashes:** 480 configurations, identical byte for byte to the creased normals plan's step 3 baseline. They are the baseline from here on.
     - **Timings:** whole contours with simplification off and on (Measurements), the budget the pass adds to.
     - **The probe:** 390 meshes, every algorithm and Manifold clustered at 2e-4, at 32³ to 128³, as contoured and simplified at the defaults (Measurements). It gave findings 15 to 18.
     - **The reference code:** read (finding 11). Several of the draft's claims were half right. The code also does much the papers do not describe, and it does not guarantee what the QuadriFlow paper claims. The Design of both field techniques is updated from it.
     - **Changed in the plan by what step 0 found:**
       - features by angle hold only at the scale of the edge length, and are not sharp edges to the simplifier's `check` (findings 9 and 17);
       - a vertex is put back on the input by material and normal, not by patch (finding 9);
       - sheets are paired with a look at the ends of each edge (finding 18);
       - QuadriFlow balances every piece of its network and runs its flow until it is full (finding 11).
     - **The default edge length:** a length in cells cannot follow the simplifier (finding 16), so decision 9 asked what it should be measured against. The user's answer, the mesh the pass is given, makes the default 1.
     - **Where the probe ran:** `RemeshProbe.cs` in the probes folder outside the repository, on hidden objects. No file in the repository changed but this plan. The open scene was not touched.
1. **The frame (medium).** The settings and inspector, the hook, what is kept, `remesh`, the statistics, the request for shared vertices, the flags through `finish`, the wireframe's mask, the editor's signature, scheduler and `select`. A stand-in technique that turns pairs of triangles into quads where they are flat, so that the flags and the wireframe can be tested before a technique gives quads; it is taken out again at step 5.
2. **Isotropic: skeleton and operations (medium).** The feature skeleton with its two strengths, `split` and `flip` on the simplifier's lists, the positions of collapses, the nearest-point search, with their tests. The simplifier's meshes hashed before and after.
3. **Isotropic: the loop (medium).** The loop, the smoothing, the equalizing of areas. Wired in as a technique. **Measured here:** edge lengths, edges per vertex, angles and vertex areas after each iteration and each area round, which sets the damping factor and the two defaults; how far vertices are from the shapes; time by stage from 32³ to 128³, simplified and not.
4. **Field Aligned: the fields (large).** Sheets, the preparation, graph, normals, areas, hierarchy, colouring, both fields, for quads and triangles. **Measured here:** the energies by level, the irregular points of both fields against the edge length, time by stage from 32³ to 128³; what is left of finding 18 after the sheets; the hierarchy scored by min against max, and normals at creases against none (findings 11 and 17).
5. **Field Aligned: the mesh (large).** Extraction, faces, holes, the split into quads, polygons, materials and normals out. Wired in. **Measured here:** everything under Tests' last point, and what the constraints do for lines between materials.
6. **QuadriFlow: offsets and flow (large).** Offsets, the network, the flow solver with its tests, long offsets. **Measured here:** the size of the flow and the time it takes against the edge length; whether the paper's hierarchy of networks is needed.
7. **QuadriFlow: the mesh (large).** Conjugate gradients, the greedy pass, the positions, extraction. Wired in. **Measured here:** turned triangles left after the greedy pass, which is what decision 6 waits for; vertices of other than four edges and holes, which say whether the code's work after extraction is wanted (finding 11); the rest as for step 5.
8. **Scenes, tuning and documentation (medium).** The scenes, whatever the measurements show to cost most, a README section for each technique with its settings, what it keeps and what it does not, `AGENTS.md`, and this plan marked complete.

Four of the nine are large, all of them the two field techniques. For scale: the simplifier was one large step among eight.

**Changed from the first draft:** Isotropic was three large steps after QuadriFlow, for the earlier paper. Before the new paper was read, the draft's decision 1 put the loop at one medium step. It is two here, because the paper turned out to leave features and staying on the surface to its reader (finding 9).

## Measurements

### What the pass gets (step 0, 2026-11-17)

**Setup:**
- **Models:** the 13 of the simplification plan's probe, the ten of `TopologyTest` and three that leave the grid.
- **Contours:** Burst, every algorithm plain, and Manifold clustered at 2e-4, at 32³, 64³ and 128³: 195 contours. Each is also simplified at the voxelizer's defaults (0.05 cells, line weight 1e-5), as `Voxelizer.contour` does before it creases, which makes 390 meshes.
- **Units:** lengths in cells and areas in cells².
- **The probe:** `RemeshProbe.cs` in the probes folder, with `run_remesh_models.sh` and `summarize_remesh.py`. Its counts were checked by hand on the cube (8 corners, 12 lines, 6 disks; simplified, 8 vertices and 12 triangles) and the two balls (2 closed patches).

**Edges and triangles,** Uniform, the median over the models of each figure but the longest edge, which is the largest:

| Grid | Mesh | Median | 5% to 95% | Longest | Smallest angle, mean | Vertices of six edges |
|---|---|---|---|---|---|---|
| 32³ | contour | 1.00 | 0.58 to 1.47 | 5 | 38° | 0.51 |
| 32³ | simplified | 2.09 | 1.45 to 7.18 | 36 | 43° | 0.38 |
| 64³ | contour | 1.00 | 0.57 to 1.42 | 5 | 38° | 0.51 |
| 64³ | simplified | 3.17 | 2.18 to 9.33 | 72 | 42° | 0.40 |
| 128³ | contour | 1.00 | 0.57 to 1.42 | 6 | 38° | 0.52 |
| 128³ | simplified | 4.74 | 3.19 to 9.79 | 145 | 42° | 0.40 |

Adaptive and Manifold are within a few hundredths of Uniform. Occupancy's contours have edges of 0.95 at the median, angles of 42° and 0.69 of their vertices of six edges.

**Vertices,** Uniform, all 13 models together: the contour, the simplified mesh, and an isotropic remesh of the simplified mesh at three edge lengths, from its area:

| Grid | Contour | Simplified | Remesh at 1 cell | at 2 | at 4 |
|---|---|---|---|---|---|
| 32³ | 28,945 | 3,466 | 27,011 | 6,753 | 1,688 |
| 64³ | 117,621 | 6,613 | 108,574 | 27,145 | 6,787 |
| 128³ | 473,922 | 12,125 | 435,009 | 108,754 | 27,189 |

**The edge length that keeps the vertex count** of the mesh the pass gets, Uniform, in cells (finding 16):

| Model | Contour, 64³ | Simplified, 32³ | 64³ | 128³ |
|---|---|---|---|---|
| Sphere | 0.88 | 2.0 | 3.0 | 4.4 |
| ThreeBalls | 0.88 | 1.9 | 2.9 | 4.2 |
| CubesAndSphere | 0.98 | 2.9 | 4.4 | 6.5 |
| SphereCutaway | 0.89 | 2.6 | 3.9 | 5.7 |
| CubeAtOrigin | 1.15 | 7.4 | 14.9 | 29.8 |
| TwoSpheres | 0.89 | 1.2 | 1.6 | 2.4 |
| SlabWithBand | 1.08 | 11.9 | 23.8 | 47.6 |
| TurnedShapes | 0.88 | 2.8 | 3.6 | 4.9 |
| NewShapes | 0.89 | 1.7 | 2.4 | 3.6 |
| HollowsAcrossFaces | 1.08 | 9.1 | 18.1 | 36.3 |
| BallAcrossBounds | 0.88 | 2.0 | 3.0 | 4.3 |
| SlabAcrossBounds | 1.06 | 9.4 | 19.0 | 38.3 |
| PlateAcrossBounds | 1.06 | 11.8 | 23.9 | 44.3 |

**The skeleton at 40°,** simplified at 128³: lines, lines under a cell long, patches, patches under a cell² (finding 17):

| Model | Uniform | Occupancy |
|---|---|---|
| Sphere | 0, 0, 1, 0 | 0, 0, 1, 0 |
| ThreeBalls | 293, 164, 139, 137 | 181, 115, 57, 51 |
| CubesAndSphere | 24, 0, 11, 0 | 24, 0, 9, 0 |
| SphereCutaway | 0, 0, 3, 0 | 117, 40, 30, 26 |
| CubeAtOrigin | 12, 0, 6, 0 | 12, 0, 6, 0 |
| TwoSpheres | 0, 0, 2, 0 | 0, 0, 2, 0 |
| SlabWithBand | 28, 0, 14, 0 | 28, 0, 14, 0 |
| TurnedShapes | 1,626, 632, 732, 590 | 1,195, 581, 398, 362 |
| NewShapes | 850, 288, 378, 294 | 893, 366, 267, 226 |
| HollowsAcrossFaces | 84, 0, 42, 0 | 84, 0, 42, 0 |
| BallAcrossBounds | 170, 114, 9, 8 | 163, 112, 3, 2 |
| SlabAcrossBounds | 39, 0, 13, 0 | 40, 0, 14, 0 |
| PlateAcrossBounds | 20, 0, 6, 0 | 20, 0, 6, 0 |

- **From 40° to 60°** the lines fall by a fifth (Manifold) to a half (Occupancy) on the 13 models together, and the patches by a fifth to a half.
- **Rule (c) corners,** from the mean normal: at most 19 on the 13 models together, for any algorithm, grid and angle.

**Sheets** (finding 18): 40 meshes have edges of more than two triangles, every one of four. After the split by the probe's rule none has an open edge or a pinched vertex it did not have. 24 have no edge of four triangles left, and 16, at 64³ and 128³, keep one to three. The three balls, which overlap, stay one surface of a ball's topology, without its 20 to 28 edges of four triangles. The turned shapes come apart into 6 pieces at 32³ and 25 at 128³, and the new shapes into a ball and a torus at 32³ and 9 pieces at 128³.

**Timings,** a whole `Voxelizer.contour` of the three balls and of the new shapes, as the editor and `Start` make one, with simplification off and on; the median of five after one to warm up, Unity in front, in ms. Adaptive here has its octree unsimplified, as in the probe, which is why it is slow:

| Model | Algorithm | Grid | Contour | Simplified | of which simplifying |
|---|---|---|---|---|---|
| ThreeBalls | Uniform | 64³ | 7.8 | 45.2 | 39.4 |
| ThreeBalls | Uniform | 128³ | 36.7 | 239.0 | 210.3 |
| ThreeBalls | Adaptive | 64³ | 46.0 | 85.2 | 41.3 |
| ThreeBalls | Adaptive | 128³ | 345.7 | 566.1 | 228.0 |
| ThreeBalls | Manifold | 64³ | 14.7 | 53.7 | 41.0 |
| ThreeBalls | Manifold | 128³ | 64.8 | 263.3 | 206.7 |
| ThreeBalls | Occupancy | 64³ | 14.5 | 51.2 | 38.6 |
| ThreeBalls | Occupancy | 128³ | 62.1 | 243.4 | 190.1 |
| NewShapes | Uniform | 64³ | 12.0 | 44.6 | 34.4 |
| NewShapes | Uniform | 128³ | 57.0 | 256.6 | 204.1 |
| NewShapes | Occupancy | 64³ | 19.4 | 52.5 | 34.8 |
| NewShapes | Occupancy | 128³ | 87.4 | 259.5 | 180.3 |
| ThreeBalls | Uniform, GPU | 128³ | 125.1 | 367.2 | 240.3 |
| ThreeBalls | Occupancy, GPU | 128³ | 22.0 | 241.2 | 213.0 |

## Out of scope

- **A managed or a GPU remesher** (decision 8).
- **Meshes of quads as such:** export, subdivision surfaces, `MeshTopology.Quads` (decision 3).
- **Repairing what Field Aligned and QuadriFlow leave open or non-manifold** (decision 5).
- **Isotropic:** the shape editing, detail encoding and linear solvers its paper is about; edge lengths that vary with curvature; putting vertices back on the true shape and not on the input mesh.
- **Field Aligned:** the intrinsic energies, half turns, point clouds, brushes, moving irregular points by hand, edge lengths that vary over the mesh.
- **QuadriFlow:** the scale that varies over the mesh, which is in its code and not in its paper.
- **The technique of *Isotropic Surface Remeshing*** (Alliez et al.), which the first draft planned: sampling by error diffusion and a centroidal Voronoi tessellation in a conformal map.
- **A view of the fields** in the scene. The probes draw them where a step needs to see them.
- **Remeshing a part of the mesh,** or keeping the remeshed mesh while shapes move.

## Decisions

The first eight were answered on 2026-11-17, before step 0. The first: "I've added in the proper isotropic surface remeshing paper. We'll use that one." The rest: "For the other decisions, I'll go with your recommendations."

1. **Which isotropic technique?** The first draft was written for *Isotropic Surface Remeshing* (Alliez et al., 2003), which samples the surface, flattens it and relaxes the samples there. It recommended the loop of splits, collapses, flips and smoothing instead. **Answered: the loop,** from *A Remeshing Approach to Multiresolution Modeling*, which the user added. This plan is rewritten for it.
2. **The reference code: read it where the papers are silent?** Finding 11. **Answered: yes, as recommended.** The papers decide wherever they speak, their code fills the gaps, nothing is ported as a whole, and whatever follows the code closely is credited in a comment and in the README. Both repositories are read, Instant Meshes' with its added licence clause among them.
3. **Quads as two triangles and a flag?** Finding 8. The other way was a Unity mesh of quads, which creasing, the statistics and the wireframe would each have to learn, and which has no place for a pentagon. **Answered: triangles and flags, as recommended.**
4. **Edges of four triangles: split into sheets?** Finding 4. QuadriFlow cannot take them and Field Aligned breaks down around them. Split, a Uniform or Adaptive contour that has them comes out of these two techniques as a manifold whose sheets touch along a line, which is not the topology it went in with. **Answered: split, and say so in the README, as recommended.** Isotropic keeps such edges as they are.
5. **Topology: counted and shown, not enforced?** Finding 7. Field Aligned and QuadriFlow can leave holes and non-manifold edges by their own papers' account. **Answered: counted with `MeshTopology`, shown in the statistics and the editor, and stated in the README, as recommended.**
6. **QuadriFlow's two solvers.**
   - **Flow: answered, as recommended:** written here, exact, by successive shortest paths, with the paper's shortcuts only if step 6 shows the time is needed.
   - **Satisfiability: answered, as recommended, which leaves it open until step 7.** The greedy pass is built, step 7 measures what it leaves on this project's models, and the question is asked again then with those numbers. If it is wanted it is a small solver of its own for regions of a few hundred variables.
7. **Isotropic's density by curvature?** This was a question about the earlier paper, whose samples follow a density. **It no longer applies.** Botsch and Kobbelt's loop has one edge length for the whole mesh, and edge lengths that vary with curvature are out of scope.
8. **Burst only?** **Answered: yes, as recommended.** A managed contour is remeshed by Burst code, and a GPU contour is read back, as for simplification.

### Asked after step 0

9. **What is the Edge Length measured against?** Finding 16. **Answered on 2026-11-17: the mesh the pass is given,** as recommended, in the user's words "For decision 9, option 2." The Design's Settings have it.
   - **Cells, as the plan had it,** with a default of 2. Every mesh comes out at the same density whatever the simplifier did. At 64³ that is four times as many vertices as the simplifier leaves on round models, and 50 to 150 times as many on flat ones. With remeshing on, the simplifier would then only clean the surface, and its Error and Detail would change little of what is shown.
   - **The mesh the pass is given,** as all three papers' defaults do: 1 is the length that keeps its vertex count, worked out from its area for each technique's faces, and 0.5 gives four times the vertices. The simplifier's Error and Detail then set how many vertices there are, and the remesh how they are spread. On an unsimplified contour, 1 is about one cell. The spread is still uniform, so a round part comes out coarser than the simplifier left it and a flat part finer.
   - **Both,** as a mode beside the length: more to explain, and the two would only meet on unsimplified contours.

   Recommended: the mesh the pass is given, at 1. It is what "a pass that goes after mesh simplification" asks for, as far as I can read it, and it is what the papers do.

