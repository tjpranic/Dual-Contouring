# Remeshing

Plan written 2026-11-17 for a pass that rebuilds the mesh of a contour after mesh simplification, with three techniques to choose between: evenly shaped triangles from a loop of edge splits, collapses, flips and smoothing; triangles or quads that follow an orientation field; and quads with few irregular vertices.

**Status:** steps 0 to 2 done on 2026-11-17. Step 3, the isotropic loop, done; its turned-box acceptance (finding 19) holds since step 3b, where the test now asks it to. Step 4, the fields of Field Aligned, done on 2026-11-18. Step 3b, features by angle at the scale of the edge length, done on 2026-11-18 after step 4. Step 5, Field Aligned's mesh, done on 2026-11-19. **Steps 6 and 7, QuadriFlow, deferred** on 2026-11-19, and step 8, the scenes, tuning and documentation, done the same day without them. **The plan is complete but for QuadriFlow.**
- **Step 0,** on the user's "Alright, begin on step 0.": the baseline, and a measure of what the pass will meet.
- **Step 1,** on "Start step 1.": the frame, with a stand-in technique.
- **Step 2,** on "Looks good so far, continue.": Isotropic's skeleton, operations and search, committed as 7caab82.
- **Step 3,** on "Alright, continue with step 3.", and after a pause on "Just do whatever is easiest to fix the long test, then proceed with step 3 of the plan.": the loop, with Isotropic wired in as a technique. Committed as 0454939.
  - On 2026-11-18, "Add those fixes in.": preserve hard-polyline breakpoints and validate face orientation against the immutable input. Scale-aware angle-feature cleanup remains separate.
- **Step 4,** on "Let's continue implementing the resmeshing plan. We are on step 4 now.": the sheets, and the two fields with their hierarchy. Nothing is wired in yet, which is step 5's. Committed as e7e5769.
  - **Step 3b was passed over** by that go-ahead, and done after it. The fields are held to open edges and lines between materials, which 3b leaves alone; what it changes for them is which vertices count as on a crease. Step 4's measurements were made before it.
- **Step 3b,** on "Let's do step 3b first.": the skeleton at the scale of the edge length, in `BurstFeatures`, for Isotropic now and the fields at step 5. Committed as c24286d.
- **Step 5,** on "Alright, start on step 5.": Field Aligned's mesh read off the fields, wired in as a technique, with the stand-in taken out. Committed as ae0a7af.
- **Steps 6 and 7 deferred,** on the user's "I don't want to implement QuadriFlow right now. Is it possible to skip steps 6 and 7 and go straight to 8?" and, after the answer that nothing built depends on them, "Alright, mark 6 and 7 as deferred and start on step 8.". QuadriFlow is not built: none of its code exists and it is not one of the techniques to choose. Its design, its tests and the half of decision 6 left open stay in this plan as they are, for when it is taken up.
- **Step 8,** on the same message: the scenes, tuning and documentation, for Isotropic and Field Aligned. The scenes were chosen by the user from three sets I proposed, on 2026-11-19: the five recommended (Example scenes). Uncommitted.

**Decisions:** the first eight were answered on 2026-11-17 (Decisions). Step 0 raised a ninth, about what the Edge Length is measured against (finding 16). The user answered it the same day: the mesh the pass is given. Step 3 raised a tenth: whether to find features by angle at the scale of the edge length (finding 19). The user answered it on 2026-11-18, as recommended: yes, in a step of its own, 3b, before Field Aligned. After step 3 the user found smooth shading broken under Isotropic (finding 21), and answered the eleventh decision, how to put it right, on 2026-11-18: both ways, as a setting. Step 4 raised none, and made two choices: the code's score for the hierarchy in place of the paper's (finding 22), and two departures from the code in how the fields are held to lines (finding 23). The user was told of them on 2026-11-19, before saying to start step 5. Step 3b raised a twelfth: the scale of the features of a simplified mesh, which 3b builds at no more than a cell, short of decision 10's *l* (finding 26). The user answered it on 2026-11-18, as recommended: no more than a cell. Step 5 raised none, and made five choices of its own where the code does otherwise or the paper is silent (step 5, findings 27 and 28). Step 8 raised none: the split of finding 31 was kept as it was, and its two speed-ups change no output (findings 32 and 33).

**Next:** nothing, unless the user takes up QuadriFlow, steps 6 and 7, which are deferred.

**Papers,** all in `Documents/Papers/`:
- *A Remeshing Approach to Multiresolution Modeling* (Botsch and Kobbelt, 2004): the technique called **Isotropic** below. It is its section 4.
- *Instant Field-Aligned Meshes* (Jakob, Tarini, Panozzo and Sorkine-Hornung, 2015): **Field Aligned**.
- *QuadriFlow: A Scalable and Robust Method for Quadrangulation* (Huang, Zhou, Niessner, Shewchuk and Guibas, 2018): **QuadriFlow**. It is built on the second paper and shares its first half.

*Isotropic Surface Remeshing* (Alliez et al., 2003) is also in that folder. The first draft of this plan was written for it. On 2026-11-17 the user added Botsch and Kobbelt's paper in its place: "I've added in the proper isotropic surface remeshing paper. We'll use that one." Nothing in this plan uses the earlier paper any more.

## Goal

Every voxelizer has a **Remeshing** setting, off by default. On, the mesh a contour gives is rebuilt before it is shown, after mesh simplification if that is on, and before its normals are creased or split:

- **Isotropic** gives triangles that are close to equilateral and all about one size, with the mesh's sharp edges, open edges and lines between materials kept as lines, and its topology kept as it was.
- **Field Aligned** gives triangles, or mostly quads, or only quads, laid out in rows that follow the shape's edges and curvature.
- **QuadriFlow** gives quads, with far fewer irregular vertices than Field Aligned at the price of less even faces. Deferred with steps 6 and 7, and not built.

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
       - Every non-collinear bend of their input polylines is a fixed geometric breakpoint, even below the Feature Angle. This does not make it a feature-angle corner. Only collinear samples may be removed or redistributed, and new samples stay on the segment between breakpoints. Exact boundary geometry takes precedence over the target vertex count.
     - **By angle:** the rules above hold them, but they are not sharp edges to `check`. A feature edge by angle shorter than 4/5 *l* collapses like any other edge, and two corners by angle closer than that may become one, where `check` allows it. Edges and corners of the shape that are further apart than that keep their place, and the zigzags and slivers of finding 17, which are shorter, go. A detail of the shape smaller than the edge length goes too, as any detail under a remesh's resolution does.
     - **What this changes:** the first draft had the angle kind as a fourth kind of sharp edge in the simplifier's lists. That would have put the noise under Hoppe's tests, which never let two such corners meet.
   - **Collapses and flips are refused** where they would change the topology or turn a triangle over (finding 7), and a collapse also where it would leave an edge longer than 4/3 *l*, which the next round would only split again.
   - **Vertices are put back on the input.** Moving within a tangent plane leaves a curved surface a little each time, and over five rounds and the area smoothing after them the mesh would shrink. After every move a vertex goes to the nearest point of the input:
     - **a vertex on a line:** of its own line of the input;
     - **any other:** of the input's triangles of its own material whose normals are within 90° of its own, near where it was.
     - **Why not the patches:** the first draft had the patch between feature lines. Step 0 found patches by angle of under a cell² by the hundred on some models (finding 17), and a vertex would be held to a sliver. The normal keeps a vertex from going to the far side of a thin wall.
     - **The input is the surface.** With simplification on that is the simplified mesh, so a curved shape keeps the simplifier's flat facets, with new vertices spread over them. The distance to the true shape is the simplifier's and no worse.
   - **A move that would turn a triangle over is not made.** Every proposed face of a split, collapse, flip or smoothing move must also face the nearest input triangle of its material at its centre. This reference search does not filter by the proposed normal. Faces carry an input-facing anchor, inherited on split and retained through collapse and flip; the candidate and the nearest reference must agree with that anchor. A flip must agree with both replaced faces' anchors. No reference within reach, or an opposing reference, refuses the edit. These conservative checks prevent the accumulated drift that a comparison with only the preceding face permits; they are not a global self-intersection test.
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
      - **Afterwards, not in the paper:** vertices of 2 edges merged away and of more than 5 split, and the positions solved again on the quad mesh ten times with a little smoothing. *Left out;* step 7 measures what the output looks like without them, and asks if they seem needed. Step 7 is deferred.
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
    - **Step 4 built it, and nothing is left** (Measurements): on 360 meshes, 44 of them with such edges, 28 edges in 16 meshes took their other pairing, and no mesh keeps an edge of more than two triangles, a pinched vertex or an open edge it did not have.

19. **Isotropic keeps the noise of finding 17 on contours** (step 3, Measurements). A cube turned about an axis or a diagonal, remeshed at the defaults:
    - **Clean** where the input is: simplified, the cube turned about an axis comes out with its own 12 lines and 8 corners from every algorithm at 32³ and 64³, which a test now holds. So does Occupancy's contour at 32³, but not at 64³, where its 32 lines become 26.
    - **Not clean** on contours whose edges cross the grid. At 64³ Uniform's contour of the cube turned about a diagonal has 359 lines and Occupancy's 259, and the remesh leaves 242 and 109. Simplified, Occupancy's comes out with 15 lines and Uniform's with 131; Manifold's came out with 12 in a run before the last changes to the loop.
    - **Why:** where an edge crosses the grid, the contour's triangles bridge the crease, and each side of the bridge meets its face at about half the crease's angle. At a Feature Angle of 40° both sides of a right angle's bridge are features, which makes the crease two lines with a strip between them. Their corners are further apart than 4/5 *l*, so the rule of finding 9 that merges corners keeps them.
    - **What would clear it** is a skeleton found at the scale of *l*, which all three techniques share. Decision 10 chose that, and step 3b builds it.
    - **Since step 3b** (Measurements): read at the scale the remesh finds its features at, the cube turned about an axis comes out as its 12 lines and 8 corners from every algorithm at every grid, contoured and simplified, and so does the cube turned about a diagonal by Manifold and by Occupancy simplified, within a cell of the true ones. A test holds them at 32³ and 64³. Uniform's and Adaptive's cube turned about a diagonal keep the points where their sheets touch, which are hard. Occupancy's contours of it keep strips that bridge the creases wider than half a cell, and come out with 29, 34 and 89 lines at the three grids, where they had 62, 109 and 228. The cause turned out to be more than finding 19 says (finding 25).

20. **Contours and simplified meshes fold in places, and a remesh keeps their folds.** Uniform's contour of the turned shapes has a sliver lying in a flat face the other way up, and its simplified mesh has places crumpled within a cell. Every operation of the loop keeps a triangle facing as it did, so what is folded stays folded. The tests hold the remesh to adding no fold where the input faces one way.

21. **A remesh that takes over the input's normals as they are breaks smooth shading.** The user saw it after step 3, and the step's tests did not: they run at a crease angle of 180°, where creasing changes nothing (Measurements).
    - **Why:** the input's vertex normals lean between its faces at sharp edges. That is the creased normals plan's finding: a shared vertex on an edge has a normal halfway between its faces, and the simplifier carries such averages across whole faces.
    - **With remeshing off,** those normals sit only on the vertices at the edges. The creasing replaces them there, because those vertices have triangles on both sides.
    - **Isotropic as first built** gave each new vertex the input's normal mixed where it lies. That put the leaning normals inside the faces, where a vertex's triangles all lie within the crease angle. **Smooth Normals: Contour** kept them, and flat faces shaded in blotches.
    - **What is built now** (decision 11): a setting, **Normals**, with both ways out:
      - **Input:** the normals the input would show creased at the Crease Angle, at the corners of the input triangle each vertex lies in, mixed by where it lies;
      - **Faces:** each vertex's own triangles' normal.
      - The two measure within a point of each other. On contours they are as close to the true normals as remeshing off is, and on simplified meshes of flat faces they are closer.
    - **Field Aligned and QuadriFlow** take their normals out the same way (What the three share).

22. **The paper's score for the hierarchy stops it short; the code's does not** (step 4, Measurements). The paper scores two neighbours by the dot product of their normals times the *smaller* of the two ratios of their areas, and its code by the *larger* (finding 11).
    - **With the smaller ratio,** vertices of equal area merge first. A vertex that is passed over once stays small while its neighbours double, has the worst score at every level after, and is passed over again. The levels stop halving after four or five and then lose a few vertices each: the three balls, simplified at 64³, go 15,839, 9,298, 5,888, 4,129, 3,197 and are still at 1,907 at the limit of 32 levels.
    - **With the larger ratio,** a small vertex is the first to be merged into a large neighbour: 15,839, 9,026, 4,882, 2,653 and so on down to one vertex at level 16.
    - **The paper's own account is the code's:** "each iteration reduces the number of vertices approximately by half", down to one vertex for every piece. Its formula does not do that, so the formula is taken for a slip.
    - **What is built:** the larger ratio. The smaller is kept as a switch of the fields' settings, for the measurement only (`Settings.smallerRatio`).

23. **Holding the fields to lines needed two departures from the code** (step 4). The paper has one sentence on it (section 3.7), so the code decided (finding 11), and on flat shapes whose fields are known in closed form the code's way did not give them.
    - **A triangular lattice was moved across its line by the wrong amount.** The code moves a held vertex's lattice by however far its lattice point is from the line. For squares any such move gives the same lattice. For triangles it does not: the rows of a triangular lattice are half a step along from each other, so a move of an odd number of rows shifts the lattice half a step along the line. The vertices of a line then never agree. A flat triangle of nine edges a side at a length of three came out with 14 irregular points of the lattice where it should have none, at six iterations a level and at six hundred.
      - **Built:** the lattice is moved by the least that puts one of its lines on the held line (`BurstFields.ontoHeldLine`). The same triangle comes out exact, as do a rhombus and larger triangles.
    - **Two held vertices that merge were given a line neither has.** The code mixes their two directions and their two points. For two vertices of one straight line that is the line. For two vertices of different lines, as at every corner of a rim, the mix is a third line. For sixth turns it is not even one of the two directions.
      - **Built:** a merged vertex keeps the line of the one of its two with the larger area. That is always a line the mesh really has.
      - **Measured** on flat squares and triangles with the lattice move already mended: with the code's mix the lattice's energy at level 0 is 4 × 10⁻⁷ to 3 × 10⁻³ and one triangle keeps two irregular points; keeping one line, it is under 10⁻¹² on every one and none has an irregular point.
    - **Which edge a vertex is held along** is not in the paper, and in the code it is whichever open edge comes last. Here it is the vertex's hard edge that lies flattest in its tangent plane, and of two as flat the lower: where a line between materials goes over a crease, the vertex has the normal of one side, and the line's edge on the other side stands up out of its plane.

24. **At an edge length of 1 the fields need a mesh of twelve times the input's vertices** (step 4, Measurements). The fields live on the input's vertices, which have to be closer than half the edge length (finding 3). At 1, the edge length is about the input's own, so every edge is split twice over.
    - **How many:** 11.9 times the input's vertices on a contour and 12.9 times on a simplified mesh, at the median, for quads, and 47 and 51 times at an edge length of 0.5. At 2 it is 3 times.
    - **What it costs** is in the Measurements. A simplified mesh has few vertices to begin with, so twelve times as many is still fewer than the contour had. A contour at 128³ is given fields on a mesh of over a million vertices.
    - **The reference code never meets this:** its default asks for a sixteenth of the input's vertices, at which nothing is split.
    - **Not changed here.** Whether a looser limit than half the edge length gives fields as good is step 8's to measure. Step 8 measured it: the fields are as good and the holes more, and the limit stays (finding 31).

25. **The noise of finding 17 is of four kinds, and lies on the crease more often than beside it** (step 3b, Measurements). Step 3b measured every feature edge of the two turned cubes against their true edges, which are known exactly.
    - **Slivers along the crease.** On the cube turned about an axis every one of the contour's feature edges lies on a true edge, within a fiftieth of a cell, and the contour's vertices on each edge are joined one to the next, yet Uniform's contour at 32³ has 66 lines and 38 corners. Its slivers are 10⁻⁵ cells high, their three corners on the crease, many folded flat over it: the long edge and the two short ones both draw the crease, and both ends get three feature edges.
    - **Triangles without area.** Some slivers are exactly flat. Their normal is nothing, they bend against no triangle, and found by angle they join the two sides of a crease into one patch.
    - **Strips that bridge the crease,** as finding 19 says. Occupancy's at 40.7° to 43° against each face, just over the Feature Angle.
    - **Creases spread over two or three edges** of 30° to 45° each, of which only some are over the angle, which breaks the crease into pieces with loose ends between them. Manifold's contours and Occupancy's.
    - **Folds:** Uniform's contour has triangles turned back by 150° to 165° beside the crease, and edges of four triangles on it (finding 18).
    - **What clears them** is the rule of step 3b (Design): thin patches merged into their neighbours, and creases cut by the band across them.

26. **At the scale of *l*, a simplified mesh of flat faces loses features the simplifier kept** (step 3b, Measurements). The edge length goes by the mesh the pass is given (decision 9), and on simplified flat models it is 15 to 48 cells (finding 16). At that scale the sides of the slab with a band, 6 cells wide at 32³, are thin patches, and merge into its top and bottom.
    - **What is lost,** at 64³: the slab with a band's feature edges by angle go from 461 cells long to 218 by Uniform and 123 by Occupancy, the hollows across faces' lines from 84 to 12, and the slab across the bounds' from 274 cells to 137. A remesh would round off what the simplifier kept sharp.
    - **The noise is of the contour's cells,** whatever the edge length: a sliver is no wider at 128³ than at 32³, and a bridge is never wider than a cell.
    - **What is built** (decision 12, answered as recommended): features by angle found at the edge length, but at no more than a cell (`Remeshing.featureLengthOf`). Every simplified flat model then keeps its skeleton exactly, and the noise is still more than halved. At the edge length uncapped, the simplified round and turned shapes would have somewhat fewer lines still (Measurements).

27. **The walk round a face stops at its first vertex, so the rim of an open mesh is walked as a face** (step 5). The code's walk goes along an edge, turns at its end to the next neighbour round, and is done when it comes back to the vertex it started from, not to the edge.
    - **What follows:** the rim of an open mesh, of up to eight sides, is walked like any other cycle, and comes out as a face turned the other way, laid over the faces inside it. Of up to six sides, the hole step fills it the same way. In the code's own use its rims are long; here a small opening of the surface would be capped upside down. The rim of a square split by its diagonal was taken for a third face.
    - **Built:** a face, or a hole filled, must face the way its corners' normals do: its area's normal against their sum. A rim walked from outside faces the other way.
    - **A walk that comes round to a vertex twice** cannot happen on a flat graph, as a result of the same rule: the shorter cycle through that vertex is always walked first. The check is kept as a guard; it and the rule that a walk stops at a vertex of one neighbour stand for each other, and taking out either changes no output of the 15 models (step 5's deliberate mistakes).

28. **A vertex of the graph is where its lattice points are, which on a crease can be off the surface** (step 5). The paper puts each vertex at the mean of its group's lattice points, each in its own vertex's tangent plane, and the code puts vertices back on the input only when it smooths them, and never those on creases or held lines.
    - **What follows:** on a curved surface the mean is a little inside or outside it. At a crease, a group of one vertex next to a corner has its lattice point past the corner: on the box of four as triangles, one vertex came out 0.32 lengths outside the box, and folded the faces round it over.
    - **Built:** every vertex goes to the nearest point of the input that faces its way, within an edge length, once the graph is tidied and again for the vertices a split into quads adds. A vertex already on the surface stays where it is, so a flat square and a box along the axes are exact. With it, no vertex of the 480 meshes measured was out of reach of the input, and on the test's models every vertex out is on it to within the rounding of floats.

29. **Holding the fields to lines between materials keeps those lines where they were, on contours** (step 5, Measurements). Section 3.7 holds the fields to open edges, and this plan holds them to lines between materials too (finding 5). Step 5 measured both ways, the 15 models by every algorithm at 64³, as quads with triangles and pentagons, from the middle of every edge between two materials out to the nearest of the input's, and back:
    - **Held:** over the 24 contours that have such lines, the median of their means is 0.004 of a cell, and of their furthest 0.05. The turned and the new shapes, whose lines cross the grid, keep theirs within a third of a cell to a cell at their furthest.
    - **Free:** 0.28 of a cell, and 0.5 at the furthest, the same way: wherever the lattice happens to put its rows, and a face to whichever material has more of it. Free, the simplified slab with a band lost its band altogether.
    - **On simplified meshes,** whose edges are several cells long, held lines move a quarter of a cell on the median and up to a cell, and free ones a cell and up to two.
    - **Quads alone** are read off a lattice of twice the length, and on a small simplified mesh can lose a narrow material: the slab across the bounds by Uniform at 32³ simplified kept none of its band's area, and the slab with a band simplified gives up, no faces coming out of its fields at all.
    - **The cost of holding** is small: by model the irregular vertices differ by up to a fifth, either way, and the quads and their angles are the same.

30. **What Field Aligned leaves of the topology** (finding 7, step 5, Measurements). Counted on the 60 meshes of each kind at 64³, against the mesh each was given, as lines of open edges and of edges of more than two triangles:
    - **Quads with triangles and pentagons, on contours:** 11 of the 60 gain lines of open edges, 74 between them: the three balls by three algorithms, and the turned and the new shapes by all four, where sheets meet and thin parts are. Manifold's and Occupancy's turned and new shapes gain lines of more than two triangles too, 4 meshes. Uniform's and Adaptive's lose theirs, as the sheets split them (decision 4).
    - **Pieces** change both ways: Uniform's and Adaptive's turned and new shapes come apart along their sheets into more, and Manifold's and Occupancy's lose islands smaller than an edge length, as any detail under the length goes (finding 9).
    - **Triangles** leave the most: 24 of 60 contours gain open lines and 31 gain lines of more than two triangles, the box and the slabs among them, where a triangular lattice meets a right angle and the faces walked round it overlap.
    - **Quads alone** the fewest: 8 gain open lines and 4 lines of more than two triangles.
    - **Simplified meshes** gain few: 8 of 60 gain open lines, 4 between them, as quads with triangles and pentagons.

31. **A looser split than half the edge length is quicker, and leaves more holes** (step 8, Measurements; finding 24 left it to this step). The 15 models by every algorithm at 64³, with the edges of the mesh the fields are on no longer than 0.5, 0.625 or 0.75 of the lattice's length, and 1 on Uniform alone:
    - **The fields' mesh** shrinks with the square of it: a contour's from 119,000 vertices on the median to 64,000 and 41,000, and every stage with it. As quads with triangles and pentagons a contour takes 674, 407 and 314 ms; a simplified mesh 68, 62 and 49, little less, as most of its time was the smoothing of levels by one worker (finding 32).
    - **The faces do not change:** regular inner vertices, the spread of areas and angles, edges against the length and the distance to the shapes are the same to within noise, for each kind of face.
    - **Holes and open lines grow:** on contours as quads with triangles and pentagons, the holes left unfilled go from 105 to 145 and 156, and the lines of open edges the remesh adds from 74 to 101 and 107; at 1, on Uniform, from 36 to 72 holes. Triangles and quads alone go the same way, and quads alone give up on one more simplified mesh at 0.625.
    - **Why:** the graph has an edge between two vertices of the mesh whose lattice points are a step apart. Further apart than half a step, two neighbours more often lie two steps apart, or across a diagonal, and the edge between them is missing from the graph.
    - **Kept at half,** the code's, as topology is where Field Aligned is weakest (finding 30), and a simplified mesh, which is what the pass is for, gains little from it. Changing it is a constant in `BurstFields.longestEdge`.

32. **Small levels of the hierarchy are worth smoothing in parallel** (step 8, Measurements). Levels of under 4,096 vertices were smoothed by one worker, and the rest colour by colour, 64 vertices to a batch. A simplified mesh's fields are on a few thousand vertices, so most of its smoothing was by one worker.
    - **Measured** on the 15 models by Uniform at 64³, contoured and simplified, at 4,096 to 0 vertices and 256 to 1 a batch: from 256 vertices and 4 to a batch a simplified mesh's remesh takes 45 ms on the median where it took 71, its fields 25 where they took 50, and a contour's 638 where it took 666. Smaller levels or batches gain under 2 ms more, and no mesh is more than 1% slower, which is within the noise.
    - **The same output:** the vertices of one colour share no edge, so the order they are smoothed in changes nothing. All 30 remeshes hashed the same at every setting.
    - **Built:** 256 and 4 (`BurstFields.smallLevel`, `vertexBatch`).

33. **Isotropic's guard against turning a face over had become most of its time** (step 8, Measurements). The correctness review after step 3 (step 3's entry) checks every face a move makes against the nearest triangle of the input at its middle, and its measurements were left as from before it. Step 8 timed it: on Uniform's 15 models at 64³ a contour took 504 ms on the median where step 3 measured 110, and a simplified mesh 40 where it was 8. Of the 504, the smoothing took 188 and the area rounds 175, where each had taken 11.
    - **Why there:** the moves of a round are made one after another, each where it turns none of its triangles over with the moves before it made (step 3), so every check, one search of the input for each triangle of the vertex, was by one worker.
    - **What is built** (`BurstIsotropic.GuessJob`): each vertex's check is made first in parallel, on the guess that every move before it in a triangle of its was made. The moves are then made one after another as before, each taking its guess unless a move before it in one of its triangles was refused, where it is checked again. Moves are seldom refused, at most 2,374 of about 100,000 in a contour's rounds, so nearly every check is made in parallel.
    - **The same output:** the guess is what the check one after another finds wherever it held, and it is not used where it did not. The 15 models by every algorithm at 32³ and 64³, contoured and simplified, each remeshed by Isotropic and by Field Aligned's three kinds of face, 960 remeshes, hash the same as from the code before step 8, normals and flags included.
    - **What it gains,** on the median of those 60 meshes by every algorithm at 64³: a contour's remesh takes 216 ms where it took 519, and a simplified mesh's 17 where it took 35. Uniform's alone are in the Measurements.
    - **What is left by one worker** is the loop's splits, collapses and flips, in the order of their lengths, which the paper's loop has to keep.


## Design

### Settings

On `Voxelizer`, under a **Remeshing** header after Mesh Simplification, carried over by the editor's `select`. The inspector shows Iterations and Area Iterations only when Technique is Isotropic.

| Setting | Default | What it does |
|---|---|---|
| Technique | Off | Off, Isotropic or Field Aligned; QuadriFlow is a fourth when steps 6 and 7 are taken up. Off, every mesh is what it is without this feature. |
| Edge Length | 1 | How long the edges of the new faces are, against the mesh the pass is given: 1 is the length at which the result has as many vertices as that mesh, 0.5 gives about four times as many and 2 about a quarter. On an unsimplified contour 1 is about one cell (finding 16, decision 9). |
| Feature Angle | 40 | Two triangles that meet at more than this many degrees have a sharp edge between them, which is kept. The same number as the Crease Angle's default, and a setting of its own: one decides where the mesh is held, the other how it is shaded. |
| Iterations | 5 | Isotropic only. How many times the loop of splits, collapses, flips and smoothing is gone through. The paper's "about 5". |
| Normals | Input | The normals the remeshed mesh starts from, before it is creased or split. **Input:** those the mesh it was given would show creased at the Crease Angle, where each vertex lies on it. **Faces:** those of each vertex's own triangles. Split vertex normals ask for Faces, which costs nothing (finding 21, decision 11). |
| Area Iterations | 5 | Isotropic only. How many rounds of smoothing by vertex area follow the loop. Zero leaves them out. The paper's "fewer than 20". Step 3 measured that ten rounds narrow a contour's spread of areas by 5% more than five do, for 8 to 16% more time, and leave a simplified mesh's as it is (Measurements). |
| Faces | Quad Dominant | Field Aligned only. **Triangles**; **Quad Dominant**, quads with triangles and pentagons where rows of faces begin and end; or **Quads**, quads only. |

- **How the length is had** (decision 9). From the mesh the pass is given: its area *A*, the sum of its triangles', and its vertex count *V*, of the vertices its triangles use. A mesh of equilateral triangles of side *l* has a vertex for every (√3/2) *l*² of its area, and one of squares a vertex for every *l*². So the length at 1 is √(*A* / ((√3/2) *V*)) for Isotropic and Field Aligned's triangles, and √(*A* / *V*) for quads, Field Aligned's or QuadriFlow's; the setting multiplies it. Field Aligned's Quads are solved at twice that and split, so they come out at it too.
- **What follows from it:** the simplifier's Error and Detail now change how many vertices a remesh has, and the interactive editor already remeshes after each. A mesh with few vertices gets long edges: the cube the simplifier leaves at 8 vertices is remeshed at about 15 cells at 64³.
- **Edge length and not a vertex count.** All three papers take a length, or a count that is turned into one. A share of the mesh's own length reads the same on a cube and a ball, and on a contour and a simplified one.
- **Not settings:** the split and collapse limits of 4/3 and 4/5, the damping factor, the random seed, the six iterations a level, and Field Aligned's extrinsic energy.

### Where it sits

- **`Voxelizer.contour`:** after the simplifier's block and before `finish`. The pass takes a mesh with shared vertices and gives one back.
- **What the voxelizer keeps:** the pass's input, as managed arrays, in a class of its own beside `MeshSimplification`. With simplification on that input is small; without, it is the contour, 4 MB at 128³.
  - **As step 1 built it** (`Remeshing`): kept whenever the mesh has shared vertices, remeshing on or off. That is every CPU contour, and a GPU contour when it is asked for shared vertices, which is when simplification or remeshing is on. A GPU contour with both off splits or creases its own vertices as it did, bit for bit, and keeps nothing: turning remeshing on there takes a contour.
  - **What keeping costs** with remeshing off: about 1 ms on a Burst contour of 38.5 ms at 128³, for reading the mesh into arrays. Its area, vertex count and edges are only worked out when it is first remeshed.
- **`Voxelizer.remesh( )`** shows the kept input again under the remeshing settings as they are now, without contouring or simplifying. It gives nothing when there is nothing kept, as `simplify` does.
- **`Voxelizer.simplify( )`** remeshes what it makes, so another detail or error is shown remeshed.
- **`contourVertices`** asks for shared vertices when remeshing is on, and such a contour is not drawn straight from the GPU.
- **`ContourStatistics`** gains the time the pass took, the size of the mesh that went in, how many faces of each number of sides came out, and what `MeshTopology` counts of the output that the input did not have (finding 7).
- **When a technique cannot do its work** it gives the input back unchanged with the reason, which the statistics carry. Nothing throws and nothing half-made is shown. Isotropic gives up before it starts where the length would make more than 2²⁰ vertices (step 3). Field Aligned gives up there too, where the mesh its fields are on would have more than 2²¹, which it finds as it splits, and where no faces come out of its fields (step 5).
- **Burst only** (decision 8). Doubles, except Field Aligned's fields, which the paper says floats do for.

### What the three share

In `Assets/Scripts/Burst`, as jobs over native arrays of positions, normals and `BurstMesh.Triangle`:

- **The feature skeleton** (finding 6): the kind of every edge, the corners, and the lines as runs of edges between corners or closed loops (`BurstFeatures`). An edge is hard where the simplifier's lists count it sharp, and by angle otherwise (finding 9). The simplifier's lists keep their three kinds of sharp edge: the angle kind is the remesher's own, kept beside them, and `check` never sees it (finding 17). What is a corner goes by the feature edges a vertex has at the time, with the turns and tips found on the input carried along (step 2).
  - **Features by angle at a scale** (step 3b, findings 25 and 26), the edge length but no more than a cell:
    - **Patches:** the triangles are joined across every edge that is not hard and does not bend at that scale. An edge of up to half the Feature Angle does not. One of more than the angle does. One in between does where the band of triangles within the length of it, on either side of it, bends by more than the angle, or where one side of the band weighs less than a tenth of the other, as beside a sliver. In the band a triangle's normal is weighted by its area, but by no more than the band's along the edge. A triangle without area joins nothing.
    - **Thin patches,** of twice their area over their perimeter under half the length, are merged into the neighbour across edges that are not hard that they lie best against, by the length between them and how near their normals are, thinnest first, again where a merge leaves one thin.
    - **Feature edges by angle** are the edges between two patches, and the edges that bend at that scale inside a patch as it was found that was not merged away, which is a crease that fades out.
    - **Loose lines** by angle shorter than the length are taken out.
    - **Turns:** a line by angle turns where it turns by more than the angle over the length on either side. A **tip** is where its triangles' normals, weighted by area, add up to less than the cosine of the angle times their sum.
    - **At a length of 0,** each edge's own angle, as before. Hard features are as they were at every length.
- **Splitting long edges:** the first operation of Isotropic's loop. Field Aligned and QuadriFlow prepare their input with it (finding 3).
- **Sheets** (decision 4; `BurstSheets`, step 4): the mesh split along its edges of four triangles and at its vertices of several fans, so that what is left is a manifold. Field Aligned and QuadriFlow work on the sheets. Isotropic does not need them.
  - **The pairing round an edge of four triangles,** as step 0's probe found it (finding 18): across the wedges inside the solid where the windings alternate round the edge; else each triangle with its neighbour of opposite winding.
  - **Then the ends,** which step 0 found the pairing alone leaves wrong where sheets pass through each other: where the two pairs of an edge still meet at both its ends, the other pairing that suits the windings is tried, and kept if it leaves each fan a disk.
  - **What still cannot be paired** is left as open edges, counted, and shown as finding 7 has it.
  - **A vertex keeps its index** for the first of its fans, and the others are added after the mesh's vertices, so a manifold comes back as it was.
- **Polygons out,** for the two field techniques: each polygon as triangles with the flags of finding 8. A quad is split along the diagonal that leaves its two triangles flatter.
- **Normals out** (finding 21): the normals at the corners of the input's triangles as the input would show them creased (`BurstRemeshing.cornerNormalsOf`), which a new vertex takes mixed where it lies; or, under Faces, each vertex's own triangles'.
- **Materials out,** for the two field techniques: an output face takes the sub-mesh that most of the input's area mapped into it has.

### Isotropic

1. **The lists:** the simplifier's `Connectivity`, built from the input, with the feature skeleton marked in it. Two operations are added to it, each updating everything `collapse` updates:
   - **`split`** of an edge at a point, for an edge of any number of triangles. Every triangle of the edge becomes two of its sub-mesh.
   - **`flip`** of an edge of two triangles.
2. **The input, kept to go back to:** its triangles in a grid for the search of the nearest point, each with its sub-mesh and normal, and its feature lines (finding 9).
3. **The loop,** for the set number of iterations, with *l* the edge length:
   1. **Split** every edge longer than 4/3 *l* at its midpoint, longest first, the new edges among them, until none is longer. The new vertex goes on to the nearest point of the input (step 3).
   2. **Collapse** every edge shorter than 4/5 *l*, shortest first, into the position finding 9 gives it, where `check` allows it and no edge longer than 4/3 *l* comes of it. A feature by angle does not stop it (finding 9). A collapse into the middle of an edge goes to the nearest point of the input.
   3. **Flip** every edge of two triangles that is not a feature where that brings its four vertices nearer their number of edges: six, or four on an open edge, as the paper has it. A flip is refused where the new edge exists, where a triangle would turn over, and where it would make a feature (finding 9).
   4. **Smooth:** every vertex towards the middle of its neighbours, within its tangent plane, then back onto the input. All moves of a round are worked out from the positions the round began with, in parallel, and made one after another where they turn no triangle over (step 3). Whether each does is found first in parallel, on the guess that the moves before it were made, and found again one after another only where one of those was refused (finding 33).
4. **Equal areas,** for the set number of rounds: the same move towards the middle of the neighbours weighted by their Voronoi areas, damped, and back onto the input. The Voronoi area is the one the paper's section 3 describes: from the circumcentres of a vertex's triangles, or the midpoints of edges where an angle is obtuse. Only vertices of the surface move: equal areas are a matter of the surface, and the loop has spread the lines' vertices along them.
5. **Out:** the positions; for each vertex a normal as the Normals setting says (finding 21): the input's as it would show them creased, at the point the vertex lies on, or the vertex's own triangles'; the triangles with their sub-meshes.

### Field Aligned

Steps 1 to 4 are built (`BurstFields`, step 4), and are what QuadriFlow starts from too. Steps 5 to 8 are built (`BurstFieldAligned`, step 5).

1. **Preparation:** the sheets, and every edge longer than min(ρ/2, 2 × the mean edge) split, longest first, at its midpoint (findings 3 and 11). The features are found on the sheets before the splits, and the splits carry them. At an edge length of 1 this makes about twelve times the input's vertices (finding 24).
2. **The graph:** the vertices with the mesh's edges between them, uniform weights, and a normal and a Voronoi area for each.
   - **Normals:** the mean of a vertex's triangles' normals, weighted by their angles at it. A vertex on a feature edge by angle takes the normal of one of its faces, the largest, as section 3.5 has it. Step 4 compared that with no creases and kept it (Measurements).
   - **Left out:** a vertex of an edge of more than two triangles that the sheets left, or without triangles or area. The sheets leave none on the models measured.
3. **The hierarchy** of section 3.5, scored by the larger of the two areas' ratios, as the code has it: with the paper's smaller ratio the levels stop halving (finding 22). Every level is coloured so that vertices of one colour share no edge and can be visited in parallel. Levels of under 256 vertices are smoothed by one worker, which gives the same fields; until step 8 it was 4,096 (finding 32).
4. **The orientation field,** then **the position field,** six iterations a level from the top level down, from a random start at the top level with a fixed seed. Before the positions, the coarse orientations are made again from the finest, as the code does.
   - **Held:** along open edges and lines between materials both fields are held, hard: the direction to the line's, and the lattice to have a line on it (section 3.7). A vertex is held along the one of its hard edges that lies flattest in its tangent plane. The lattice is moved onto the line by the least that does it, and a merged vertex keeps the line of the larger of its two, both of which depart from the code (finding 23).
   - **The searches:** the paper's 16 pairs for quads and 36 for triangles. The paper's "immediate neighbors" of the point between two vertices are taken to be, for triangles, the six lattice points nearest it.
   - **The sum of the directions** keeps its own length from one neighbour to the next, as equation 3 has it. The code keeps it at the number of neighbours added.
   - **Rounding** goes to the nearest lattice point, which is what the paper says it does. Its appendix's formula for triangles rounds the two steps one by one, which is not always the nearest.
5. **Extraction** as section 3.8 has it, with what the code adds where the paper is silent (finding 11): merging in order of distance, leaving out vertices too few inputs agree on, snapping within 0.3 ρ, taking out the diagonals of quads, walking the faces of three to eight sides, filling holes of up to six.
   - **A step** between two lattice points is to one of the four neighbours of a square, or the six of a triangle. The code takes a step along both axes of a triangular lattice too, which is two triangles' height away (finding 11).
   - **Spurious:** a vertex fewer than a tenth of the mean number of vertices went into, as the paper has it.
   - **Back on the input:** every vertex at the nearest point of it that faces its way, before the faces are walked (finding 28).
   - **A face** must face the way its corners' normals do, which a rim walked from outside does not (finding 27); a walk that comes round to a vertex twice is none.
6. **Faces:**
   - **Triangles:** sixth turns and a triangular lattice.
   - **Quad Dominant:** quarter turns and a square lattice. Triangles and pentagons stay as they are, as polygons, where the code makes them fans round a new vertex.
   - **Quads:** the same at twice the edge length, then every face split into quads round its middle, so the edges come out at the length that was asked for. The code splits at the length asked for, which halves it.
   - **A face cut down** loses the run of corners whose own face has the angles nearest a triangle's or a square's. The code scores a run by the angles of the face before it is cut, one corner along.
7. **Smoothing,** two rounds as the code has them: each vertex not on a crease or a held line to the middle of its neighbours, and back onto the input, at its nearest point within an edge length. The code casts a ray from the middle along the neighbours' plane's normal, and keeps the middle where it hits nothing; a vertex with no point of the input in reach stays where it was.
8. **Out:** positions; normals as Isotropic's, the input's or the faces' (finding 21, decision 11), the weighted means of section 3.8 serving only for the tangent planes of the extraction; materials by area (finding 5); polygons as triangles with their flags.
   - **Materials by area:** every triangle of the mesh the fields are on maps to the vertex of the graph nearest its middle of those its corners went into, and there to the face whose corner it lies in, by its angle round the vertex. A face takes the sub-mesh most of the area that maps into it has, and one nothing maps into, the input's at its middle. A face cut down, or split into quads, gives its pieces its own.

### QuadriFlow

**Deferred** with steps 6 and 7 on 2026-11-19, and not built. The design below is as it was planned.

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

**Chosen at step 8** by the user, from three sets I proposed: five, each a copy of a Uniform Burst scene at 64³ cells with creased normals at 40° and the wireframe over the surface, at the remeshing defaults otherwise:

| Scene | Copied from | Technique | Faces | Simplified |
|---|---|---|---|---|
| Isotropic Sphere | Sphere | Isotropic | | no |
| Isotropic Multiple Materials | Multiple Materials | Isotropic | | yes |
| Field Aligned Union | Union | Field Aligned | Quad Dominant | no |
| Field Aligned Multiple Materials | Multiple Materials | Field Aligned | Quad Dominant | no |
| Field Aligned Quads Sphere | Sphere | Field Aligned | Quads | no |

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
- **QuadriFlow** (deferred with steps 6 and 7, not built):
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
   - **Done 2026-11-17:**
     - **Built:**
       - `Assets/Scripts/Burst/BurstRemeshing.cs`: the edge length of decision 9 (`lengthOf`); a job for a mesh's area and the vertices its triangles use; the sides of a mesh's edges, sorted so that each edge's triangles come together; a job that counts open and non-manifold edges and the faces the flags make, by their sides; and the stand-in, `FlatPairsJob`.
       - `Assets/Scripts/Remeshing.cs`: the `Technique` setting's values, what a remesh did (`Remeshing.Statistics`), the kept mesh, and the pass, which gives a new mesh and its flags in the order of its index buffer.
       - On `Voxelizer`: a **Remeshing** section with **Technique** and **Edge Length**; `contourVertices` asks for shared vertices while remeshing; the hook between simplifying and `finish`; `remesh( )`; `simplify( )` remeshes what it shows; `ContourStatistics.remeshing`; `shownEdgeFlags`; profiler markers `Voxelizer.KeepForRemeshing` and `Voxelizer.Remesh`.
       - `Wireframe.draw` takes the flags, packed four triangles to a word in a raw buffer, and the shader leaves out the edges they do. Flags that are not one for every triangle are taken for none.
       - In `InteractiveEditor`: a fourth signature and scheduler, `stateIsOfRemeshing`, the two settings carried over by `select`, and the inspector's lines: what went in, the edge length in cells, the faces by sides, the time, and warnings when the pass gave up or left new open or non-manifold edges.
     - **Settled while building:**
       - **Only Technique and Edge Length so far.** Feature Angle comes with the skeleton at step 2, Iterations and Area Iterations with the loop at step 3, Faces with step 5. A setting that does nothing yet would only mislead.
       - **The setting's values:** Off is 0, the stand-in 99, shown as "Flat Pairs (Stand-In)". The techniques of the plan get 1 to 3 at their steps, so the stand-in's value never comes back as one of them.
       - **The stand-in** pairs triangles across an edge of exactly two, of one sub-mesh, wound the same way, within a degree of each other and making a convex quad, longest edge first. It moves nothing. It gives up, and shows the mesh as it came, when it pairs nothing.
       - **The order of the flags survives `finish`:** creasing, splitting and `Mesh.OptimizeReorderVertexBuffer` keep the triangles' order in the index buffer, which the tests hold with reordering on and off.
       - **The editor's remeshing scheduler** starts from the time the remesh within the last contour took, as the simplifier's does. With remeshing off that is nothing, so the first remesh after turning it on is done at once.
       - **The project's first sort in a Burst job:** `NativeArray.Sort` works there.
     - **Tests:** 190 more.
       - **`BurstRemeshingTest`, 23:** the lengths, the area, the edge and face counts with and without flags, the stand-in on patches, boxes, an octahedron, bent and dart-shaped pairs and two materials, and the order of the flags.
       - **`RemeshingTest`, 145:** through the voxelizer for every algorithm and implementation, three models, both kinds of normals, reordered or not. What is kept, `remesh` against a fresh contour, simplification and remeshing together, the cells of the kept contour, and 32-bit indices.
       - **`WireframeTest`, 5 in three tests:** edges left out are not drawn, other flags are, and the voxelizer draws its remeshed faces.
       - **`InteractiveEditorTest`, 17:** the carry-over, a change of remeshing remeshes without contouring for all nine variants, a change of detail remeshes, a forgotten input takes a contour, a contour of nothing is up to date, the signature.
       - **`VoxelizerDefaultsTest`:** remeshing off at an edge length of 1.
     - **Deliberate mistakes:** 43 were tried, two of them in the shader, and 42 failed a test.
       - **Four tests were added to get there** before the run: the edges a remesh adds, the cells after a change of resolution, new flags on the same wireframe, and nothing owed after a contour that remeshed.
       - **The one that passed** is the line by which `simplify` tells the remeshing scheduler that what it shows is remeshed. Without it, a change of detail and of the remeshing at once remeshes twice, with the same mesh shown. It is kept: no test can see it without a counter in production code.
     - **Found and not changed:** a voxelizer that draws straight from the GPU throws in `contour` when a material slot of its renderer is empty, since it reads the colour of every material. It is older than this plan, and the editor's voxelizers and the scenes always have materials. The test that met it gives its voxelizer one.
     - **Baselines after step 1:** edit mode 3516 of 3516, play mode 21 of 21, 480 `Start` hashes unchanged from step 0.
2. **Isotropic: skeleton and operations (medium).** The feature skeleton with its two strengths, `split` and `flip` on the simplifier's lists, the positions of collapses, the nearest-point search, with their tests. The simplifier's meshes hashed before and after.
   - **Done 2026-11-17,** on the user's "Looks good so far, continue." after committing step 1 as ebcac5e:
     - **Built:**
       - `BurstMeshSimplifier.Connectivity`: `split`, `checkFlip` and `flip`, the reasons a flip is refused (`FlipRefusal`), and `isPoint`, which `check` now asks too. Its arrays of vertices and triangles are native lists.
       - `Assets/Scripts/Burst/BurstFeatures.cs`, shared by the three techniques: which edges are features, the corners found by angle, the lines numbered from corner to corner, and the rule for what is a corner (`isCorner`).
       - `Assets/Scripts/Burst/BurstIsotropic.cs`: `FeatureMesh`, the simplifier's lists with the features kept beside them, which split, flip and collapse carry along; what a vertex is to the remesh (`Kind`); where a collapse goes (`placeCollapse`); the longest edge a collapse would leave.
       - `Assets/Scripts/Burst/BurstSurfaceGrid.cs`: `SurfaceGrid`, the input's triangles and feature lines in a grid, and the search for the nearest point.
     - **Settled while building:**
       - **The simplifier's arrays became lists,** so that a split can add to them from inside a job: a job works on a copy of the struct, and a list is the same list in every copy. Simplifying never adds to them. Its meshes are the same, and it is 7 to 10% faster: 198, 168 and 178 ms for the three balls by Uniform, the new shapes by Uniform and the three balls by Occupancy at 128³, against 213 to 221, 185 to 187 and 193 to 198 ms at ebcac5e. That is the median of nine after one to warm up, twice over, measured A, B, A, B with Unity in front. Why lists would be faster than arrays was not looked into.
       - **`check` writes out `isPoint`'s rule** rather than asking it: asking it made simplifying 5 to 9% slower, measured the same way.
       - **A split** makes every triangle of the edge two of its sub-mesh, wound as it was, for an edge of any number of triangles. The edge keeps its index and runs to the new vertex. The second half is the first edge added, the edges across the triangles follow, and the triangle at the second end is added. The new vertex has the mean of its ends' normals and stands for no area and no quadric. It is locked where the edge had two triangles on the same three vertices. Only it and the third vertices are worked out again: the two ends keep their triangles, sharp edges and fans one for one.
       - **A flip is refused** where the edge does not have two triangles, is sharp, has triangles that face apart, or has the edge across already; where a triangle it makes would face away from the sum of the two it replaces, or have no area; where the two it makes would meet at more than the Feature Angle; and, in `FeatureMesh`, where the edge is a feature by angle. Nothing is worked out again after a flip, since nothing that follows from the triangles changes. The tests show it on a box of two materials, on two boxes that share an edge of four triangles, and on two that touch in a vertex.
       - **What a corner is goes by the feature edges a vertex has now:** a point the simplifier keeps, whatever the angle; one feature edge, or three and more; two where the line turned on the input (rule b of finding 6); none where the triangles spread on the input (rule c). Rules b and c are found once, on the input, and the vertex carries them. So a corner whose lines have collapsed away is no corner any more, which is how the noise of finding 17 can go. A corner whose lines are longer than 4/5 *l* stays where it is. Step 0 counted a third as many corners by rule b as by rule a on the turned shapes, nearly all of them noise, so whether this is enough is step 3's to measure on the turned box.
       - **Two corners along a line become one:** into a point the simplifier keeps, else into the one where more lines meet, else into the lower. Two points never do.
       - **The grid** is of cubes whose size the caller gives. A triangle goes into the cells of its bounding box that its plane passes through, a segment into those whose centres are within half a cell's diagonal of it. A search goes out ring by ring and stops when the rings left are further than the nearest point found, or than a reach the caller gives. The grid keeps the input's triangles in their order, those the lists left out among them, so that step 3 can take the input's normals from the triangle found. The cell size and the reach are step 3's to set.
       - **Feature Angle comes at step 3,** with the technique: nothing uses it before.
     - **Tests:** 30 more, on meshes built by hand (`HandBuiltMesh`), each run in a job compiled by Burst (`FeatureMeshRun` for the features).
       - **`BurstMeshSimplifierTest`, 10:** a split of an open edge, an edge between materials, an edge inside a face, an edge of four triangles and an edge of two triangles on the same three vertices: the lists are those of the split mesh built from nothing, the surface is the same point for point, and every sub-mesh has its area and faces as it did. A flip of a flat pair gives the other diagonal. Each reason a flip is refused has a mesh that gives it and one next to it that flips. A run of 27 to 54 splits, 41 to 83 flips and 48 to 104 collapses keeps the lists and the shape, on a box of two materials, two boxes that share an edge of four triangles, and two that touch in a vertex.
       - **`BurstIsotropicTest`, 12:** the skeleton of a box of one and of two cells, an octagonal prism at 40° and 50°, a square of a second material at 40° and 100°, a cone's tip, and a disc of a second material. A split of a feature edge leaves its halves on its line. A feature by angle is not flipped. Every edge of two boxes with a line between materials goes where finding 9 says, eight cases between them. Two corners become one unless both are points. A collapse carries the line and the corner of the end it goes into. The longest edge a collapse would leave.
       - **`BurstSurfaceGridTest`, 8:** the nearest point of a triangle over its face, beyond its edges and corners, by hand and against 80,000 samples of it; the grid against a search of every triangle and every segment, in cells of 0.3, 1 and 3; a nearer triangle of another material, or facing away, passed over; nothing beyond the reach; the input's triangles kept in their order.
     - **Deliberate mistakes:** 45 were tried, 14 in split and flip, 8 in the skeleton, 13 in the feature mesh and 10 in the grid. Every one failed a test. Two of the three runs of operations, on the boxes that share an edge and on those that touch, were added after I took out what a flip worked out again, to show that it needs none of it.
     - **Baselines after step 2:** edit mode 3546 of 3546 in 359 s, play mode 21 of 21, Unity in front; the 480 `Start` hashes are step 0's, byte for byte.
3. **Isotropic: the loop (medium).** The loop, the smoothing, the equalizing of areas. Wired in as a technique. **Measured here:** edge lengths, edges per vertex, angles and vertex areas after each iteration and each area round, which sets the damping factor and the two defaults; how far vertices are from the shapes; time by stage from 32³ to 128³, simplified and not.
   - **Implemented 2026-11-17; turned-box acceptance outstanding,** on the user's "Alright, continue with step 3." after committing step 2 as 7caab82. It was paused once at the user's word, and resumed on "Just do whatever is easiest to fix the long test, then proceed with step 3 of the plan."
     - **Built:**
       - `BurstIsotropic.cs`: the loop, as one job for each operation, with profiler markers `Remeshing.Isotropic.*`:
         - `SplitJob`, `CollapseJob` and `FlipJob`;
         - the smoothing in three jobs: `AreasJob` for the Voronoi areas, `SmoothJob` for where each vertex goes, worked out in parallel, and `ApplyJob` for the moves;
         - `NormalsJob` and `WriteJob` for the output.
         - Also its `Settings` and `Report`; `placeOnInput`, which finds where a new vertex goes on the input; and `turnsOver`.
         - `FeatureMesh` learned which corners are held.
       - `SurfaceGrid`: a search over every sub-mesh, and a stop as soon as the nearest point found is nearer than the walls of the rings left. That stop is one of several changes made for speed while building, with the cached counts of edges, the flips' worklist, normals worked out in parallel, and a search only for collapses the lists allow at the middle. Their shares were not measured one by one.
       - `BurstRemeshing.Topology`: lines of open and of non-manifold edges, joined end to end.
       - `Remeshing`: the technique Isotropic (1); settings as a struct of their own; a give-up before it starts where the length would make more than 2²⁰ vertices; new open and non-manifold edges counted as lines; and what Isotropic did, in the statistics.
       - On `Voxelizer`: **Feature Angle** (40), **Iterations** (5), **Area Iterations** (5) and **Normals** (Input). The editor's `select` carries them over, and its signature and warnings follow them.
       - `BurstRemeshing.cornerNormalsOf`: the normals at the corners of the input's triangles as it would show them creased, which Isotropic's normals out are mixed from under Normals: Input (finding 21).
       - `Assets/Tests/Edit Mode/TestProgress.cs`: writes `Temp/test_progress.txt` as each test starts and finishes, for the watchdog below.
     - **Settled while building:**
       - **Every vertex is on the input from when it is made.** A split puts its vertex on the nearest point of the input to the edge's middle: on the edge's line for a feature edge, else on the surface of its sub-mesh facing within 90° of its triangles. A collapse into the middle goes there too. Where there is no such point within 2 *l*, or a half of the split would turn over, the edge is left as it is. Before those refusals, vertices were left up to 0.29 cells off the input. Now the furthest is under 10⁻⁵ cells.
       - **The moves of a smoothing round** are worked out together and made one after another. A move is held back where it would turn a triangle over, or face it away from the fan's normal. Without that second test, a sliver turned a little in each round and was over after a few.
       - **The area rounds move only surface vertices, damped by λ.** The loop's own smoothing is undamped, as the paper has it.
       - **Corners on a hard line are held.** Two corners along a line merge only where at most one is held. Otherwise a thin face between two materials lost its corners at long edge lengths.
       - **A flip** goes by the sum of the squared differences from six edges, or four on a rim of two open edges. The paper gives no measure. The flips go from a worklist, with the counts of edges kept as they change.
       - **The statistics count lines, not edges.** A remesh samples a rim again, which changes how many open edges it has but makes no new line.
       - **The output normals** were at first the input's own, mixed at the point each vertex was put on. The user found smooth shading broken by that (finding 21). They now come, by the Normals setting (decision 11), from the input's corners as the input would show them creased at the Crease Angle, or from each vertex's own triangles. The input is creased with Smooth Normals at Contour whatever that setting is: under Faces, the creasing after the remesh gives every vertex its triangles' normal whatever it came with.
       - **Tried and taken out:** a search that took only triangles facing within the Feature Angle. It changed nothing.
       - **The defaults** are 5 iterations, 5 area rounds and λ = 0.75, from the sweep (Measurements).
       - **A watchdog on Unity.** A deliberate mistake, L24, let splits turn a triangle over, and its test run went on for 45 minutes until the runner's timeout. Unity answers nothing while a Burst job runs, and it had to be restarted.
         - **Every call that drives Unity now goes through a guard script** kept with the probes. It kills Unity when no test has started or finished for two minutes, or when a probe has written no row for two minutes. Unity is then opened again.
         - **L24 is now caught that way.** The guard stopped it at 126 s, and the run went on.
         - **A limit on vertices was tried and taken out.** The split pass stopped at eight times the vertices the mesh had or the length made, and 1024 more. It did not stop L24 within two minutes, most likely because each split got slower rather than the vertices growing without end. The user had it removed: "Before anything, since the vertex limit didn't fix the L24 mutant, remove the vertex limit." The sweep and timing ran with it in place. It was never reached, and it cost one more sum in a loop the build already makes.
         - **The relaunched editor** opens without a Unity account signed in. The tests are unaffected.
     - **Measured** (Measurements): the loop by iteration, the area rounds against λ, three edge lengths, how far the vertices are from the input, and the time by stage from 32³ to 128³. It gave findings 19 and 20, and decision 10 (Decisions).
     - **Tests:** 115 more.
       - **`BurstIsotropicTest`, 19 more:**
         - on meshes built by hand, the stages one at a time: Voronoi areas, flips towards six edges, a round of smoothing as the paper describes it, a split onto the input, a line between materials round a disc;
         - the whole loop: a flat square meshed evenly, the normals, areas made more alike, an upright box at two lengths, a line between materials, the same mesh twice, an island shorter than the length;
         - the normals out: a vertex inside a face of a box creased at 40° has the face's normal, at two lengths; normals that turn over a patch are mixed where a vertex lies, against the mix worked out in the test; under Faces, each vertex has its own triangles' normal.
       - **`BurstRemeshingTest`, 2 more:** a rim sampled again adds no line; the normals at a box's corners as creased at 40° and at 180°.
       - **`RemeshingTest`, 94 more:** through the voxelizer, for every algorithm and implementation, three models, contoured and simplified. Each holds:
         - the topology of the surface and of each material, and its lines and pinched vertices;
         - every vertex within 10⁻⁵ cells of the input, and every triangle facing the input's way except where the input itself folds;
         - about as many vertices as the input, and the same remesh twice.
         - Besides those: every vertex on the input at 64³; the edge length against the vertex count; `remesh` against a fresh contour; each setting changing the remesh, Normals among them; the give-up at 2²⁰ vertices; the simplified box turned about an axis, which keeps 12 lines and 8 corners by every algorithm.
         - And the normals out: a box along the grid shades flat on its faces at 40°, contoured and simplified, by Uniform and Occupancy, under both kinds of normals; at 180°, Input shows the input's normals leaning into the faces and Faces each vertex's own triangles'; split vertex normals ask the remesh for Faces.
       - **Changed:** the carry-over and signature in `InteractiveEditorTest`, the defaults in `VoxelizerDefaultsTest`, and the settings struct in `WireframeTest`.
     - **Deliberate mistakes:** 43 were tried, and 40 were caught. L24 was caught by the guard stopping Unity. Four more, on the limit on vertices, were caught and went out with it.
       - **Ten were on the normals out,** and all ten were caught. One, the voxelizer's crease angle not passed on, first passed. The test of what Input shows at 180° then looked only at vertices whose triangles lie flat, which an input creased at any other angle does not lean into.
       - **The three that pass** are guards that no test model reaches. Splitting, or collapsing into the middle, where no point of the input is within reach (L22, L23): every vertex starts on the input and stays on it, so an edge's middle always has a point of the input near it on these models. Asking the lists again at the point on the input after they allowed the middle (L4b): the two points are a sagitta apart, and no test mesh has a collapse allowed at one and refused at the other. They are kept.
     - **Baselines after step 3:** edit mode 3665 of 3665 in 369 s with the normals out, Unity in front. Four of those tests are the user's own, added to `BurstIsotropicTest` and `BurstSurfaceGridTest` after the step was first reported. Play mode passed 21 of 21, and the 480 `Start` hashes are step 0's, byte for byte. Those two ran before the limit was taken out and the normals changed, neither of which they reach with remeshing off.
     - **Correctness review, 2026-11-18:**
       - The disc test kept boundary vertices on the input but allowed edges to cut across its bends. Its material area changed from 12.245869 to 12.208417 at a target length of 0.5. Hard-polyline breakpoints are now fixed independently of the feature classification, and projection along a hard line stays on its straight segment. The test checks complete segments, input bends and both materials' areas at lengths 0.5 and 2.
       - A fold-free height field, `z = 2 sin(1.3x) cos(1.7y)` on a 6 by 6 patch, acquired a backward-facing triangle in its first collapse pass at length 2. An inherited input normal alone also allowed it: after moving, the face was over a different part of the input. Every stage now checks the local input orientation as well as the inherited anchor, without using the candidate normal to choose a favourable reference. A regression checks the result against an independent search of the input faces after one and five iterations.
       - Validation: `BurstIsotropicTest` 31 of 31, `BurstSurfaceGridTest` 8 of 8, and the remeshing suites 266 of 266. The tests also cover a bent non-manifold line and an unoriented search that still respects material and reach. Removing the local-reference orientation check makes both height-field cases fail again.
       - The measurements below predate these guards and pinned breakpoints; they are historical, not measurements of the corrected implementation. Finding 19 remains outstanding; decision 10 was answered on 2026-11-18, and step 3b takes it up.

3b. **Features by angle at the scale of the edge length (medium).** Decision 10, finding 19. In `BurstFeatures`, which all three techniques share:
   - an edge is a feature by angle where the surface bends by more than the Feature Angle across a band about *l* wide, not across the one edge, so that a strip of triangles bridging a crease counts as one crease;
   - lines by angle shorter than *l* that end loose are taken out.

   Hard features (open edges, edges of more than two triangles, edges between materials) are not touched.
   - **Measured here:** the lines and corners of the two turned cubes against their own 12 and 8, contoured and simplified, by every algorithm from 32³ to 128³ (finding 19); the skeletons of the 15 models before and after; what it changes in Isotropic's measures and time.
   - **Tested here:** the test of the simplified box turned about an axis widens to the contours and the box turned about a diagonal, wherever they then hold.
   - **Done 2026-11-18,** on the user's "Let's do step 3b first.", after step 4.
     - **Built:**
       - `BurstFeatures.SkeletonJob` takes a length (`length`), and at more than 0 finds the features by angle at its scale (Design): patches, thin patches merged, the band, loose lines taken out, turns and tips at the scale. At 0 it is the edge's own angle, as before, which the step checked on all 48 meshes of the turned cubes: identical.
       - `BurstIsotropic.Settings.featureLength` and `BurstFields.Settings.featureLength`: the length each technique finds its features at, or 0. `Remeshing.featureLengthOf`, the edge length but no more than a cell, which `Remeshing` gives Isotropic. The fields are given it at step 5, when they are wired in; until then they find features by each edge's own angle.
       - The test helper `FeatureMeshRun` takes a length.
     - **Settled while building**, each from the probe against the turned cubes' true edges (finding 25):
       - **Patches before bands.** A rule by the band alone cannot draw a crease once: two lines either side of a bridge bend alike across the band. Patches of the edges that do not bend, with the thin ones merged away, can only meet along one line.
       - **A triangle without area joins nothing.** Through such triangles the faces either side of every crease of the cube turned about an axis were one patch, and the creases were left only as creases inside it.
       - **An edge over the angle is always cut.** A first rule joined it where its band was flat, for folds on flat faces, which a merge of thin patches takes care of anyway. It also joined slivers to the face either side of them, which joined the faces of a crease.
       - **The band is round the edge,** taking in triangles within the length of it along the line too. A band kept to the edge's own stretch found spread creases less often on contours: Occupancy's diagonal cube remeshed with 56 lines at 64³ where the round band gives 34. The cost is that a crease is carried up to a length past where it stops bending by the angle, through edges that still bend by more than half of it.
       - **A triangle's weight in the band** is capped at the band's area along the edge. Uncapped, a cube face's triangle 14 cells high outweighed a ball's 2 cells high, and the crease between them was lost.
       - **The thin limit is half the length.** At one length the faces of the simplified cubes were thin, since *l* there is about a face's size.
       - **Tips by area.** By any one triangle, a vertex whose sliver was merged away kept a tip, as the sliver faced the other way.
       - **Turns over the length.** By each edge, the zigzag of a merged bridge turns at every vertex: the cubes with a ball at 64³ have 72 lines read so, and 24 at scale.
       - **The scale is no more than a cell** (finding 26, decision 12).
     - **Measured** (Measurements): the turned cubes' skeletons and their remeshes against their true edges, by every algorithm from 32³ to 128³; the skeletons of the 15 models before and after; the scale at *l* against at a cell; what it changes in Isotropic's measures and time. It gave findings 25 and 26, and decision 12.
     - **Tests:** 47 more, where one test of 4 cases was replaced: 43 more in all.
       - **`BurstFeaturesTest`, 23,** on meshes built by hand with the noise of finding 25, each at a length of 0 and at a length: each edge's own angle at 0, on six meshes; a sliver without area, and a sliver folded over, on a crease; a strip that bridges a crease, thinner and wider than half the length; a crease spread over three edges of 30°, and a round whose edges of 30° bend by less across the band; a bend of 30° beside a triangle 40 high; a crease that fades out, shorter and longer than the length; a cone's tip; hard features at every length; a box at every length; a strip merged into the face it lies nearest, never across a line between materials, and again where it was merged into another thin strip; a crease inside a strip that is merged away; a sliver among triangles of 10; a crease that zigzags.
       - **`RemeshingTest`, 23:** the turned boxes, 21 cases, replace the test of the simplified box turned about an axis; and the simplified slab with a band keeps its features at the remesh's scale, by Uniform and Occupancy.
       - **`BurstFieldsTest`, 1:** the mesh the fields are on finds its features at their feature length.
     - **Deliberate mistakes:** 31 were tried, 27 in the skeleton and 4 where the scale is passed on. 30 failed a test.
       - **13 passed the new unit tests at first.** Six of them failed the turned boxes, and six more tests were added for the rest, each on a mesh built to need what it tests.
       - **The one that passes** is the guard that keeps a hard line from being taken out as a loose one. A vertex never has exactly one hard feature edge: open edges come in pairs at a vertex, and so do lines between materials. It is kept.
       - **Two tests needed a second try at their meshes,** and are worth knowing about: within a length of a crease, an edge that bends by more than half the angle is cut by the band, so a mesh whose strip twists by more than that a column, or whose faces twist, has features of its own.
     - **Baselines after step 3b:** edit mode 3781 of 3781 in 416 s, play mode 21 of 21, Unity in front; the 480 `Start` hashes are step 0's, byte for byte.

4. **Field Aligned: the fields (large).** Sheets, the preparation, graph, normals, areas, hierarchy, colouring, both fields, for quads and triangles. **Measured here:** the energies by level, the irregular points of both fields against the edge length, time by stage from 32³ to 128³; what is left of finding 18 after the sheets; the hierarchy scored by min against max, and normals at creases against none (findings 11 and 17).
   - **Done 2026-11-18,** on the user's "Let's continue implementing the resmeshing plan. We are on step 4 now.", with step 3b not built.
     - **Built:**
       - `Assets/Scripts/Burst/BurstSheets.cs`: a mesh split into sheets (`SplitJob`, `split`), and what the split did (`Report`).
       - `Assets/Scripts/Burst/BurstFields.cs`, which Field Aligned and QuadriFlow share:
         - the operators: a turn (`turned`), the turns and the lattice steps that bring two vertices nearest (`matchOrientations`, `matchPositions`), the point between two tangent planes (`middle`), rounding to a lattice (`roundSteps`), and the move of a held lattice onto its line (`ontoHeldLine`);
         - the mesh the fields are on (`prepare`, `PrepareJob`) and level 0 of its graph (`graph`, `VerticesJob`, `LinksJob`);
         - the hierarchy with its colours (`Hierarchy`, `HierarchyJob`);
         - the smoothing: `StartJob` for the random start, `ColourJob` for one colour by every worker, `LevelsJob` for the small levels by one, `HandDownJob`, and `HandUpJob` for the directions made again from level 0;
         - what the fields are like: `EnergyJob` for the paper's two energies and `SingularitiesJob` for the irregular points;
         - `solve`, which does all of it and reports the time by stage, with profiler markers `Remeshing.Fields.*`.
       - `BurstIsotropic.EdgeHeap` is now internal, so that the split before the fields takes edges in Isotropic's order.
       - **Not wired in:** no technique, no setting, and nothing in `Voxelizer` or the editor has changed. A mesh comes out at step 5.
     - **Settled while building:**
       - **The sheets** keep, for every side of every triangle, the side across the edge that it is paired with. The triangles of a vertex then fall into rows and rings, and each is a vertex of the sheets. An edge keeps its four triangles only where one ring passes through it twice at both its ends, which is what the look at the ends tests for and mends.
       - **Creases** are the feature edges by angle of the shared skeleton, at the Feature Angle. Open edges and lines between materials are not creases: on a flat face a line between materials has one normal on both sides.
       - **Held** are the vertices of open edges and of lines between materials, and no others. Field Aligned snaps to creases by itself, which is the paper's point.
       - **The paper against its code,** where they differ and step 4 had to choose: the score of the hierarchy (finding 22), the lines the fields are held to (finding 23), and three smaller ones that follow the paper: the sum of the directions, the search of 36 pairs for triangles, and rounding to the nearest lattice point (Design).
       - **Irregular points of the directions** are every triangle whose turns do not add up to a whole turn, as the paper has it. The code counts only those a step off.
       - **A direction handed down** is made unit length again in the tangent plane it lands in. The code leaves it short.
       - **The hierarchy stops at 32 levels.** With the larger ratio the largest mesh measured, of a million vertices, has 24, and none has more than 30.
       - **Floats** for the fields, as the plan has it, and doubles for the mesh they are on.
     - **Measured** (Measurements): the sheets on 360 meshes; the fields of 15 models by two algorithms at 64³, contoured and simplified, at three edge lengths, with and without creases, and with the paper's score; the levels of four meshes; the time by stage for every algorithm from 32³ to 128³. It gave findings 22 to 24.
       - **Creases stay on.** They give a fifth to a quarter fewer irregular points of the directions for quads, and leave the lattice's as they are. The noise of finding 17 does not show in these counts: the cube turned about an axis has its eight and no more.
       - **The larger ratio** gives fields as regular as the paper's score in half the time.
     - **Tests:** 73 more. The operators are run in a job compiled by Burst, and everything else through the jobs themselves.
       - **`BurstSheetsTest`, 18:**
         - a manifold comes back as it was; a triangle that uses a vertex twice is left out;
         - two boxes that share an edge, and two that touch in a vertex, come apart into two closed boxes;
         - two flat sheets that cross along a line come apart into two;
         - two that cross at one edge are taken straight through it, which needs the look at the ends: with both sheets going on past the edge, and with either one ending at it, so that the other pair is found from a triangle's partner;
         - two that meet round one end of an edge only are left as they were paired;
         - an edge of three triangles is counted and its odd triangle left open;
         - Uniform's contours of the turned and the new shapes at 32³ and 64³, contoured and simplified, come apart with nothing left over.
       - **`BurstFieldsTest`, 55:**
         - the operators on 2,000 pairs of vertices each, against a search over everything they choose from: turns, the point between two planes, rounding, lattice steps; and the move of a held lattice onto its line;
         - the mesh the fields are on: every edge short enough, the same surface, the same areas for every sub-mesh;
         - level 0: normals, Voronoi areas, neighbours, what is held and along what, what is left out, creases on and off;
         - the hierarchy: every level about half the one below, a merged vertex the sum of its two, its neighbours those of its two, the line it is held to, and no two neighbours of one colour; and that the paper's score stops short;
         - the fields in closed form: a flat square and a flat triangle with their lattices exact, a box with its eight irregular points and its lattice on its edges, held vertices on their lines;
         - one vertex smoothed once against six neighbours set by hand, for the directions and for the lattice; a field handed down to both of a vertex's two; the directions made again from below;
         - the energies and the irregular points on a triangle set by hand;
         - the same fields twice, measured or not, other fields from another seed, and the random start at the top level; an empty mesh; and the contours of the turned shapes by Uniform and Occupancy.
     - **Deliberate mistakes:** 83 were tried: 17 in the sheets, 16 in the operators, 14 in the mesh and level 0, 13 in the hierarchy, 16 in the smoothing and 7 in the measures. 81 failed a test.
       - **Seven passed at first,** and five tests were added and two sharpened for five of them: the sheet that ends at the crossed edge, both ways round; the edge met round one end only; the field handed down; the start at the top level; and a mean that its last neighbour no longer happens to equal.
       - **One of the seven was nearly no mistake.** Going round a vertex by the wrong side of each triangle goes round the edge's other end instead, and both ends are asked. It differs only where a triangle's own way round comes to a rim, which no test had.
       - **The two that pass** are lines no mesh reaches. Putting an edge's first pairing back when its other one mends nothing: on every mesh tried the other one mends it. And the count of edges the sheets leave with more than two triangles: they leave none. They are kept.
       - **A mistake that never ends was not tried:** an edge split off its middle stays a third as long as it is far from the origin, and is split for ever. An edge split off the surface stands in for it.
     - **Seen and recorded, not changed:**
       - **Triangles** have more irregular points than quads on every model with flat faces at right angles, as sixth turns must.
       - **A handful of vertices are left out** on one model at an edge length of 0.5: 4 to 6 of 405,000 on the cube turned about an axis, whose triangles there have no area.
       - **The fields of a contour at 128³ take over two seconds** (finding 24).
       - **The plan's test of a box along the axes** asks for rows of faces, which is step 5's. Step 4 holds the box's directions to its edges and its lattice to whole lengths from them.
     - **Baselines after step 4:** edit mode 3738 of 3738 in 405 s, play mode 21 of 21, Unity in front; the 480 `Start` hashes are step 0's, byte for byte.

5. **Field Aligned: the mesh (large).** Extraction, faces, holes, the split into quads, polygons, materials and normals out. Wired in. **Measured here:** everything under Tests' last point, and what the constraints do for lines between materials.
   - **Done 2026-11-19,** on the user's "Alright, start on step 5." after committing step 3b as c24286d.
     - **Built:**
       - `Assets/Scripts/Burst/BurstFieldAligned.cs`: the faces (`Faces`: Triangles, Quad Dominant, Quads), its settings and what it did (`Report`); the graph read off the fields (`Graph`), with the links classified (`ClassifyJob`), merged into vertices (`ClusterJob`), tidied (`SnapJob`) and put back on the input (`PlaceJob`); the faces walked, holes filled, cut and split (`FacesJob`); materials by area (`MaterialsJob`); the smoothing (`NeighboursJob`, `SmoothJob`); the triangles with their flags (`WriteJob`, `fanOf`, `flagsOf`) and the normals out (`NormalsJob`); `remesh`, with profiler markers `Remeshing.FieldAligned.*`.
       - `Remeshing.Technique.FieldAligned`, which replaces the stand-in: `FlatPairs` and `BurstRemeshing.FlatPairsJob` are gone. `Remeshing.Settings.faces`, the length of its faces (`facesOf`, `lengthOf`), the limit on the mesh the fields are on (`mostFieldVertices`, 2²¹), and its two ways of giving up. `Remeshing.Statistics.fieldAligned`.
       - On `Voxelizer`, **Faces** (`remeshingFaces`, Quad Dominant), which the editor carries over and has in its remeshing signature.
       - `BurstFields`: the split stops at a number of vertices (`Settings.mostVertices`, `Report.tooLarge`), and lines between materials can be left free, for the measurement only (`Settings.freeMaterials`).
       - `BurstRemeshing.inputNormalAt`, the input's normal at a point of it, which Isotropic's `NormalsJob` had to itself.
     - **Settled while building:**
       - **Five choices of my own,** where the code does otherwise or the paper is silent: a step of a triangular lattice to one of its six neighbours; every vertex back on the input before the faces (finding 28); a face to face its corners' way (finding 27); a face cut down by its own angles; materials by area through the corners round each vertex, and from the input at a face's middle where nothing maps into it (Design). Smoothing back onto the input by the nearest point and not by a ray is a sixth, smaller one.
       - **Spurious vertices** by the paper's rule, fewer than a tenth of the mean number of vertices of the mesh the fields are on that went into them; the code counts merges and leaves out a vertex of no more than a tenth.
       - **The limit on the fields' mesh** is found as it is split: an estimate before splitting would have to guess what the split makes of the mesh. Two million vertices take about a gigabyte. At an edge length of 1 the largest contour at 128³ needs about a million (step 4).
       - **The snapping stops after 64 rounds,** which no mesh measured comes near, so that a graph that never settled could not hang the editor (`Report.unsettled`).
       - **A trap, not of the code:** twice a job compiled by Burst ran its code from before an edit, after a compile that had failed on a test and then one that went through. Its results were plausible and wrong: the split stopped after one edge, and a check was not made. A job run as plain C# beside it showed it. A trivial edit and a recompile cured it; the memory has how to tell.
     - **Measured** (Measurements): the 15 models by every algorithm at 64³, contoured and simplified, as quads with triangles and pentagons, triangles, and quads alone, and with the lines between materials held and free: 480 meshes. The time by stage of Uniform from 32³ to 128³. It gave findings 27 to 30.
     - **Tests:** 129 more in all: 135 new, and the stand-in's 6 gone; one test of 162 cases replaced.
       - **`BurstFieldAlignedTest`, 49,** on graphs and meshes built by hand, every job compiled by Burst:
         - the parts: steps of the lattice; the fan of a quad and of a pentagon, the flags of a fan, and the polygons its triangles give back; merges nearest first and a step that keeps two apart; a vertex at the weighted mean of its lattice points; a spurious group; a vertex fixed by a held vertex or a crease, and not by a hard line; snapping onto an edge and into its end; the diagonal of a quad; the faces of a grid turning the way its normals face; a square with its diagonal as two triangles; holes of four, six and eight sides; faces of five to eight sides cut into quads or triangles, and the run cut off first by its angles; quads alone round the middle of each face; a face by the area under it; a face nothing maps into; a triangle mapped through the nearest of its corners' vertices;
         - the whole: a flat square as quads and as quads alone, a flat triangle as triangles, each its lattice exactly with every inner vertex regular; a box along the axes as 96 quads, either way, and closed as triangles; a band of a second material keeping its lines and its area; the normals out, the input's or the vertex's own; the limit on the fields' mesh; the same remesh twice.
       - **`RemeshingTest`:** the stand-in's test of the frame becomes Field Aligned's, 162 cases: the flags of the mesh shown make the faces the remesh says it made, creased or split, reordered or not, read back from the GPU, with every edge left out by its one other triangle where the remesh is a manifold. New: what Field Aligned keeps of the mesh it is given, for every algorithm and implementation, contoured and simplified, 54; each kind of face on the mesh, 8; the edge length against the vertex count, for each kind of face, 21; each of its settings, 1.
       - **`BurstRemeshingTest`:** the stand-in's six tests are gone, and a remesh of no faces gives the mesh back. **`BurstFieldsTest`, 1:** lines between materials left free.
       - **Changed:** the other tests that used the stand-in now use Field Aligned (`WireframeTest`, `InteractiveEditorTest`, `BurstFieldsTest`), and the carry-over, the signature and the defaults have Faces.
     - **Deliberate mistakes:** 58 were tried: 13 in the graph, 9 in the snapping, 2 in the placing, 14 in the faces, 4 in the materials, 3 in the smoothing, 4 in the output, 2 in the fields and 7 where it is wired in. 56 failed a test.
       - **Seven passed at first,** and four tests were added and one sharpened for five of them: quads alone on a curved mesh, a face by the area under it, one nothing maps into, a triangle through the nearest of its corners' vertices, and the two rounds of smoothing. Three more had been added before the run, for what it was clear no test held: the run a cut takes off, a remesh of no faces, and lines between materials left free.
       - **Checked by hand before calling any a gap:** the one that maps a triangle through its first corner changed 9 of the 15 models' 90 remeshes at 32³, by under a hundredth of an area on most, which is what the test added for it shows.
       - **The two that pass** change nothing. A walk let on through a vertex of one neighbour comes straight back to the vertex before it, which the check against a vertex twice refuses (finding 27); and a walk let close at fewer sides than its pass asks for has always been walked at its own number first. Applied by hand, neither changes any of the 90 remeshes.
       - **One did not compile** as first written, and was rewritten and run again with the one after it, since a failed compile is what left Burst stale.
     - **Baselines after step 5:** edit mode 4295 of 4295 in 489 s, play mode 26 of 26, Unity in front. The `Start` hashes are now 768, the probabilistic solver's among them since its plan: step 0's 480 are among them byte for byte, and all 768 are those of the last baseline, taken after the QEF errors in doubles.
6. **QuadriFlow: offsets and flow (large). Deferred** on 2026-11-19 (Status), and not built. Offsets, the network, the flow solver with its tests, long offsets. **Measured here:** the size of the flow and the time it takes against the edge length; whether the paper's hierarchy of networks is needed.
7. **QuadriFlow: the mesh (large). Deferred** with step 6. Conjugate gradients, the greedy pass, the positions, extraction. Wired in. **Measured here:** turned triangles left after the greedy pass, which is what decision 6 waits for; vertices of other than four edges and holes, which say whether the code's work after extraction is wanted (finding 11); the rest as for step 5.
8. **Scenes, tuning and documentation (medium).** The scenes, whatever the measurements show to cost most, a README section for each technique with its settings, what it keeps and what it does not, `AGENTS.md`, and this plan marked complete. With steps 6 and 7 deferred, it covers Isotropic and Field Aligned, the README says QuadriFlow is not built, and the plan is complete but for QuadriFlow.
   - **Done 2026-11-19,** on the user's "Alright, mark 6 and 7 as deferred and start on step 8.", after committing step 5 as ae0a7af.
     - **Scenes:** five in `Assets/Scenes/Remeshing`, the set the user chose of three I proposed (Example scenes), made by `RemeshScenes.cs` in the probes folder as the simplification plan's were: copied, opened beside what is open, set, saved and closed. No existing scene was changed. Each was contoured in a preview scene and drawn from its own camera with its wireframe, and two things came of it:
       - **Isotropic Multiple Materials** is remeshed at an Edge Length of 0.25. Its two cubes simplify to 18 vertices, which at 1 gives edges of 28 cells, longer than the cubes' own: corners found by angle within four fifths of that merge, and the cubes lose their shape, as finding 9 says details under the length do. At 0.25 the edges are 7 cells, and the cubes and their line between materials come out exactly.
       - **Field Aligned Union** leaves a hole about two cells across where the ball cuts the cube's top edge, and one edge of four triangles where they meet at the back: one more line of open edges and one of more than two triangles, which its inspector warns of (finding 30). It is kept, as an honest example.
     - **Tuning, what the measurements showed to cost most:**
       - **The split that finding 24 left to this step** (finding 31): a looser one is quicker on contours and leaves more holes; it stays at half the edge length.
       - **The fields' small levels by one worker** (finding 32): now smoothed colour by colour down to 256 vertices, 4 to a batch. A simplified mesh's Field Aligned remesh takes over a third less time, 45 ms where it took 71 at 64³ on the median, a contour's about 4% less, and the output is the same.
       - **Isotropic's guard against turning faces over** (finding 33), which the correctness review after step 3 added and had made Isotropic four to five times slower than step 3 measured: its checks are now guessed in parallel, and a remesh takes less than half the time, with the same output.
       - **What I tried and took out:** a share of the split as a setting, for measuring finding 31. The constants of finding 32 were made settable for its measurement only and are constants again.
     - **Documentation:** a Remeshing section in the README, with its settings, what each technique keeps and does not, what they give, their cost and the scenes, beside changes to its sections on the wireframe, creased normals, the interactive editor, the code layout and the papers, and the credit to Instant Meshes' code. `AGENTS.md` has a Remeshing section and, under Unity, the trap of finding Burst running stale code.
     - **Tests:** 6 more, in `BurstIsotropicTest`: a round's moves with the checks guessed are those made one after another without guesses, over five rounds on a flat and a bumpy patch at three sizes of move, with moves refused and moves after refused ones in every case.
     - **Deliberate mistakes:** 11, all in the guesses, and all caught. Two passed at first: the guess taking its own vertex's move as made, and the fan's normal from where the vertices were. The second took the bumpy patch to be caught, the first moves of 1.5 lengths. A face that has not moved yet faces its reference, and then the check against itself refuses what the check against its reference refuses: only steep turns of faces moved before in the round tell the two apart. The fields' constants change no output by design, so no test can tell them apart, and none were tried.
     - **Baselines after step 8:** edit mode 4344 of 4344 in 470 s, play mode 26 of 26, Unity in front. The 769 `Start` hashes are byte for byte those of the last baseline, taken after the creased normals' follow-up at 40° (`crease-anomalies-2026-11-19/hashes_40.txt`), which the user committed as 6d6e300; that follow-up changed the shading, so step 5's and step 0's hashes no longer all match, and step 8 changes none of them.

Four of the ten are large, all of them the two field techniques. For scale: the simplifier was one large step among eight.

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

### The isotropic loop (step 3, 2026-11-17)

**Setup:**
- **Models:** step 0's 13 and two cubes, one turned 45° about an axis and one about a diagonal.
- **Contours:** at 64³, by Uniform and Occupancy (Burst), contoured and simplified at the voxelizer's defaults: 60 meshes, 30 of each kind.
- **Measures:**
  - **edges within the paper's limits:** the share of edges between 4/5 *l* and 4/3 *l*;
  - **six edges:** the share of inner vertices with six edges;
  - **area spread:** the standard deviation of the inner vertices' Voronoi areas over their mean.
  - Each figure is the median over the 30 meshes of a kind.
- **The probe:** `IsotropicProbe.cs` in the probes folder, with `run_isotropic_sweep.sh`, `run_isotropic_times.sh` and `summarize_isotropic.py`. Unity was in front throughout, and the run was on the final code.

**By iteration,** with no area rounds, at an edge length of 1 (contour / simplified). The time is of one remesh:

| Iterations | Within the limits | Six edges | Smallest angle, mean | Triangles under 30° | Area spread | Vertices against the input | ms |
|---|---|---|---|---|---|---|---|
| 0 | 0.65 / 0.46 | 0.68 / 0.33 | 41.7° / 41.3° | 3.8% / 11.5% | 0.23 / 0.27 | 1 / 1 | 12 / 0.9 |
| 1 | 0.84 / 0.68 | 0.83 / 0.66 | 48.3° / 45.0° | 1.0% / 5.0% | 0.16 / 0.17 | 1.18 / 1.30 | 50 / 3.0 |
| 2 | 0.94 / 0.79 | 0.85 / 0.67 | 51.1° / 45.6° | 0.24% / 4.2% | 0.13 / 0.16 | 1.07 / 1.17 | 60 / 3.8 |
| 3 | 0.96 / 0.82 | 0.86 / 0.71 | 52.4° / 45.9° | 0.32% / 4.7% | 0.12 / 0.15 | 1.03 / 1.14 | 71 / 4.4 |
| 5 | 0.98 / 0.84 | 0.87 / 0.69 | 53.0° / 46.4° | 0.28% / 5.0% | 0.12 / 0.15 | 1.01 / 1.13 | 90 / 5.4 |
| 7 | 0.98 / 0.84 | 0.88 / 0.72 | 53.2° / 47.0° | 0.28% / 4.3% | 0.13 / 0.15 | 1.00 / 1.13 | 110 / 6.1 |
| 10 | 0.98 / 0.85 | 0.88 / 0.71 | 53.5° / 47.1° | 0.23% / 4.1% | 0.13 / 0.15 | 0.99 / 1.12 | 130 / 7.3 |

Past five iterations, little changes but the time, so the default is the paper's 5. Simplified meshes stay behind contours: they come in with long thin triangles, and their features hold vertices in place.

**The area rounds against λ,** after five iterations: the area spread, contour / simplified. With no rounds it is 0.123 / 0.147:

| Rounds | λ = 0.25 | 0.5 | 0.75 | 1 |
|---|---|---|---|---|
| 2 | 0.106 / 0.141 | 0.095 / 0.138 | 0.091 / 0.138 | 0.092 / 0.138 |
| 5 | 0.095 / 0.138 | 0.088 / 0.138 | 0.085 / 0.137 | 0.083 / 0.137 |
| 10 | 0.089 / 0.138 | 0.084 / 0.137 | 0.081 / 0.137 | 0.080 / 0.137 |
| 20 | 0.084 / 0.137 | 0.080 / 0.137 | 0.079 / 0.137 | 0.079 / 0.137 |

- **The other measures** move by less than a percentage point across these runs. Triangles under 30° at five rounds go from 0.23% of a contour's at λ = 0.25 to 0.15% at 1, and from 5.4% to 5.7% of a simplified mesh's.
- **A larger λ more often lets more rounds widen the spread again.** On contours, of the 120 steps from 0 to 2, 5, 10 and 20 rounds at each value, that happens in none at 0.25, 4 at 0.5, 9 at 0.75 and 13 at 1. The most is the cube's contour, by 13% from 10 rounds to 20.
- **The defaults:** five rounds, at λ = 0.75, between the two. Ten rounds take 8 to 16% more time than five, and twenty 35 to 48% more.

**Edge lengths,** five iterations and ten rounds at λ = 0.5 (contour / simplified):

| Edge length | Vertices against the input | Within the limits | Six edges |
|---|---|---|---|
| 0.5 | 3.93 / 4.09 | 0.99 / 0.94 | 0.82 / 0.76 |
| 1 | 1.01 / 1.13 | 0.99 / 0.85 | 0.87 / 0.69 |
| 2 | 0.26 / 0.37 | 0.98 / 0.63 | 0.82 / 0.54 |

**At the defaults** (5 iterations, 5 rounds, λ = 0.75, edge length 1):
- **On the input:** every vertex of the 60 meshes is within 1.7 × 10⁻⁶ cells of the input mesh. No vertex was left without a point of the input within reach.
- **The true shapes:** a contour's remesh has its vertices as far from the true shapes as the contour's, on average. A simplified mesh's remesh has them at 0.75 of the simplified mesh's distance.
- **Vertices:** 0.95 to 1.07 times the input's on contours, and 0.94 to 1.28 on simplified meshes of 300 vertices or more. The smallest simplified meshes gain the most: the cube goes from 8 to 14, and the plate across the bounds from 16 to 36.
- **Features smaller than *l* go,** as finding 9 has it. That is most visible on simplified meshes, where *l* is 9 to 48 cells (finding 16):
  - **The hollows across faces** go from 84 lines and 56 corners to 48 to 50 lines and 32 to 33 corners, at every edge length tried.
  - **The slab with a band** goes from 16 corners to 14, and at a length of 2 its slab is cut back to the band: 8 vertices and 12 lines.
  - **The cube** and **the cube turned about an axis** keep their 12 lines and 8 corners.
  - **The turned cubes' contours:** finding 19.

**Time,** the remesh alone at its defaults, every algorithm (Burst) and the 15 models, the median of five after one to warm up, Unity in front, in ms:

| Grid | Mesh | Median | Most | Build | Split | Collapse | Flip | Smooth | Areas | Out |
|---|---|---|---|---|---|---|---|---|---|---|
| 32³ | contour | 30 | 68 | 10% | 11% | 38% | 18% | 11% | 11% | 1% |
| 32³ | simplified | 3.7 | 20 | 9% | 11% | 33% | 15% | 15% | 14% | 2% |
| 64³ | contour | 114 | 285 | 10% | 10% | 38% | 19% | 11% | 11% | 1% |
| 64³ | simplified | 7.8 | 41 | 9% | 13% | 38% | 15% | 12% | 12% | 2% |
| 128³ | contour | 435 | 1,267 | 11% | 9% | 39% | 19% | 11% | 11% | 1% |
| 128³ | simplified | 11.6 | 80 | 9% | 13% | 40% | 16% | 10% | 11% | 1% |

- **Against the contour,** the three balls and the new shapes at 128³ beside step 0's timings:
  - **After simplification,** which is how the pass is meant to be used, the remesh adds 6% (Adaptive) to 31% (Occupancy, new shapes) to the contour. The three balls by Uniform take 33 ms, where contouring and simplifying took 239.
  - **Without it,** the remesh takes 3 (Adaptive, whose contour is slow) to 26 times as long as the contour. The three balls by Uniform take 938 ms, where contouring took 37.
- **The collapses** take the largest share everywhere. How that splits between `check`, the search for a point on the input and the rest was not measured.

### The remesh's normals (step 3, 2026-11-18)

**Setup:** Uniform (Burst), creased at 40° with Smooth Normals at Contour, the voxelizer's defaults. The measure is the share of triangle corners whose normal is more than 10° off the true normal of the shapes a quarter of the way into the triangle, which keeps clear of the edges where the true normal is not defined. The probe is `NormalsProbe.cs` in the probes folder, which also draws each mesh. Unity was in front.

| Model | Grid | Mesh | Remeshing off | Isotropic as first built | Input | Faces |
|---|---|---|---|---|---|---|
| NewShapes | 64³ | contour | 4.5% | 10.0% | 5.4% | 5.5% |
| NewShapes | 64³ | simplified | 13.3% | 27.9% | 13.3% | 13.2% |
| Cube, turned | 32³ | contour | 2.8% | 7.3% | 1.6% | 1.6% |
| Cube, turned | 32³ | simplified | 38.3% | 36.9% | 22.0% | 22.0% |
| Cube, turned | 64³ | simplified | 34.9% | | 23.5% | 23.5% |
| TurnedShapes | 64³ | simplified | 38.5% | | 28.6% | 29.0% |
| Sphere | 64³ | both | 0% | 0% | 0% | 0% |

- **The first build's** blotches show in the pictures as dark and bright patches on flat faces next to edges, and over whole faces of simplified meshes. **Smooth Normals: Faces** hid them, since it ignores the normals a vertex comes with.
- **Input and Faces** draw alike on these models. They differ only where the input's own normals are better than its faces', which is on curved surfaces at lengths long enough for the difference to show. The sphere was not one.
- **The cost:**
  - **The measurement:** at 128³, the 15 models by Uniform and Occupancy, the median of five after one to warm up, Unity in front.
  - **The result:** Input takes 1.4% longer than Faces on a contour at the median (−2.8% to 1.9%), and 4.8% longer on a simplified mesh (0.6% to 17%). It creases the input before the remesh and searches the input for every vertex's normal, where Faces only sums each vertex's triangles. A remesh's own spread from run to run was under 2%.
  - **Not comparable with the step's timings above:** both ran four to four and a half times slower than those, measured the night before. The reason was not found. That morning the editor had been restarted, its Profiler was recording, and it used about one core while idle.

### The fields (step 4, 2026-11-18)

**Setup:**
- **Models:** step 3's 15.
- **The sheets:** every algorithm (Burst) at 32³, 64³ and 128³, contoured and simplified at the voxelizer's defaults: 360 meshes.
- **The fields:** at 64³, by Uniform and Occupancy, contoured and simplified: 60 meshes, 30 of each kind, each solved nine ways. The Feature Angle is 40°, and the edge length goes by the mesh, as decision 9 has it.
- **Measures:**
  - **irregular points:** the triangles of the mesh the fields are on round which the directions, or the lattice, do not come back to where they started (section 3.6). Each becomes an irregular vertex or face of the mesh read off the fields;
  - **energies:** the paper's two (equations 4 and 6), as the mean over the edges of a level: the angle between two directions squared, in degrees, and the distance between two lattice points squared, in edge lengths.
- **The probe:** `FieldsProbe.cs` in the probes folder, with `run_fields_sweep.sh`, `run_fields_times.sh` and `summarize_fields.py`. Unity was in front throughout, and the runs were on the final code.

**The sheets** (finding 18). Of the 360 meshes, 44 have edges of more than two triangles, 2,196 edges between them, every one of four: Uniform's and Adaptive's of the three balls, the turned shapes, the new shapes and the cube turned about a diagonal.
- **Paired:** 2,037 across the solid and 159 by winding. 28 edges in 16 meshes took their other pairing after the look at their ends.
- **Left:** no edge of more than two triangles, no pinched vertex where the meshes had 276, no new open edge, and no edge whose two triangles run the same way where they had 4,392.
- **What comes apart:** the three balls and the cube turned about a diagonal stay one piece. The turned shapes come apart into 6, 9 and 27 pieces at the three grids, and the new shapes into 2, 3 and 10.
- **Time:** 60 ms at the most, at 128³.

**The fields nine ways,** the 30 meshes of each kind together (contour / simplified). The energies are medians:

| Faces | Edge length | Vertices against the input's | Levels | Irregular points of the directions | of the lattice | Energy of the directions | of the lattice |
|---|---|---|---|---|---|---|---|
| Quads | 1 | 11.9 / 12.9 | 21 / 14 | 1,957 / 926 | 5,233 / 1,170 | 0.71 / 5.8 | 0.0006 / 0.0015 |
| Quads, no creases | 1 | 11.9 / 12.9 | 20 / 14 | 2,351 / 1,238 | 5,233 / 1,139 | 1.02 / 19.4 | 0.0006 / 0.0016 |
| Quads, the paper's score | 1 | 11.9 / 12.9 | 32 / 32 | 1,974 / 933 | 5,590 / 1,142 | 0.69 / 5.6 | 0.0007 / 0.0016 |
| Quads | 0.5 | 47 / 51 | 23 / 16.5 | 2,346 / 1,322 | 12,088 / 2,881 | 0.34 / 2.1 | 0.0004 / 0.0012 |
| Quads | 2 | 3.0 / 3.3 | 18 / 12 | 1,438 / 727 | 2,103 / 481 | 2.0 / 15.5 | 0.0010 / 0.0037 |
| Triangles | 1 | 10.8 / 12.1 | 20 / 14 | 3,090 / 1,310 | 7,050 / 806 | 1.14 / 14.1 | 0.0009 / 0.0020 |
| Triangles, no creases | 1 | 10.8 / 12.1 | 20 / 14 | 3,784 / 1,823 | 6,122 / 717 | 1.20 / 16.6 | 0.0008 / 0.0022 |
| Triangles | 0.5 | 43 / 45 | 22.5 / 16 | 3,621 / 1,939 | 16,254 / 2,177 | 0.36 / 6.4 | 0.0005 / 0.0016 |
| Triangles | 2 | 2.7 / 3.3 | 18 / 12 | 2,269 / 1,057 | 3,194 / 346 | 3.5 / 32 | 0.0014 / 0.0036 |

- **Against the vertices the edge length asks for,** which are the input's 270,637 and 13,568 at 1: quads have an irregular point for every 38 vertices of a contour's remesh and every 6.5 of a simplified mesh's. The simplified meshes have 8 to 1,553 vertices, mostly of flat faces and sharp corners, and every corner of a box is an irregular point.
- **By edge length:** half the length gives 2.3 to 2.7 times the irregular points of the lattice, and a fifth to a half more of the directions. The paper says the lattice's grow with the resolution and the directions' do not (section 3.6). It also gives four times the vertices, so the mesh is the more regular.
- **Creases:** with them, quads have 17% fewer irregular points of the directions on contours and 25% fewer on simplified meshes, and as many of the lattice. Triangles have 18% and 28% fewer of the directions, and 15% and 12% more of the lattice.
- **The paper's score** (finding 22): within 1% on the directions and 7% on the lattice, at 32 levels where the larger ratio has 21 and 14, and twice the time: 2.1 s against 1.1 s on a contour at the median, measured with the energies.

**By model,** quads at an edge length of 1 with creases: irregular points of the directions / of the lattice:

| Model | Uniform, contour | Uniform, simplified | Occupancy, contour |
|---|---|---|---|
| Sphere | 8 / 173 | 8 / 38 | 8 / 160 |
| ThreeBalls | 91 / 362 | 14 / 82 | 8 / 278 |
| CubesAndSphere | 30 / 205 | 8 / 10 | 8 / 148 |
| SphereCutaway | 8 / 327 | 8 / 65 | 8 / 340 |
| CubeAtOrigin | 8 / 4 | 8 / 0 | 8 / 0 |
| TwoSpheres | 16 / 87 | 16 / 38 | 16 / 88 |
| SlabWithBand | 8 / 2 | 8 / 2 | 8 / 0 |
| TurnedShapes | 487 / 576 | 136 / 93 | 310 / 449 |
| NewShapes | 378 / 809 | 222 / 219 | 207 / 694 |
| HollowsAcrossFaces | 56 / 16 | 17 / 9 | 56 / 6 |
| BallAcrossBounds | 21 / 215 | 19 / 43 | 23 / 222 |
| SlabAcrossBounds | 8 / 9 | 8 / 0 | 8 / 14 |
| PlateAcrossBounds | 0 / 0 | 0 / 0 | 0 / 0 |
| TurnedCubeAxis | 8 / 10 | 8 / 0 | 8 / 2 |
| TurnedCubeDiagonal | 101 / 22 | 12 / 0 | 53 / 15 |

- **Eight is the least a closed surface without handles can have:** each is a quarter turn, and they have to add up to two whole turns. A ball, a box and the cube turned about an axis have their eight. Two balls have sixteen, and the plate, which is open, none.
- **The shapes that cross the grid at angles** have the most, and Uniform more than Occupancy, as its contours have the edges of four triangles and the folds of finding 20.

**By level,** the three balls by Uniform, simplified, quads at 1: 17 levels of 15,839, 9,026, 4,882, 2,653, 1,450, 786, 425, 229, 127, 69, 38, 20, 11, 6, 3, 2 and 1 vertices, in 5 to 6 colours each.
- **The energy of the directions** when a level is handed down falls from 600 at 11 vertices to 270 at 127, 43 at 1,450 and 6.6 at level 0.
- **The energy of the lattice** falls from 0.08 at 20 vertices to 0.05 at 127, 0.011 at 1,450 and 0.0018 at level 0.
- **With the paper's score** the same mesh has 32 levels of 15,839, 9,298, 5,888, 4,129, 3,197, 2,719 and then 2,464 down to 1,907: from the sixth level on, each loses under a tenth of its vertices (finding 22).

**Time,** the fields alone at an edge length of 1, with no mesh read off them yet. Every algorithm (Burst) and the 15 models for quads, and Uniform for triangles; the median of three after one to warm up, and of one at 128³ contoured; Unity in front; in ms:

| Grid | Mesh | Faces | Vertices of the fields | Median | Most | Sheets | Lists and split | Level 0 | Hierarchy | Directions | Lattice |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 32³ | contour | Quads | 29,200 | 181 | 276 | 1% | 20% | 2% | 14% | 24% | 40% |
| 32³ | simplified | Quads | 2,610 | 43 | 100 | 0% | 9% | 2% | 5% | 30% | 54% |
| 64³ | contour | Quads | 119,000 | 565 | 1,030 | 1% | 26% | 2% | 18% | 20% | 33% |
| 64³ | simplified | Quads | 5,170 | 62 | 157 | 0% | 11% | 2% | 8% | 29% | 51% |
| 128³ | contour | Quads | 483,000 | 2,260 | 4,660 | 1% | 30% | 3% | 19% | 17% | 30% |
| 128³ | simplified | Quads | 10,500 | 94 | 262 | 1% | 15% | 2% | 10% | 27% | 45% |
| 64³ | contour | Triangles | 101,000 | 807 | 1,420 | 1% | 15% | 2% | 11% | 14% | 58% |
| 64³ | simplified | Triangles | 4,790 | 111 | 234 | 0% | 6% | 1% | 4% | 17% | 73% |
| 128³ | contour | Triangles | 408,000 | 2,970 | 5,830 | 1% | 18% | 2% | 12% | 13% | 55% |
| 128³ | simplified | Triangles | 8,520 | 141 | 344 | 0% | 9% | 1% | 6% | 16% | 68% |

- **The four algorithms** are within 8% of each other at every grid.
- **Against Isotropic** (step 3), which is a whole remesh: after simplification the fields take 62 ms where it took 8 at 64³, and 94 ms where it took 12 at 128³. On a contour they take 565 ms and 2.3 s where it took 114 ms and 435 ms.
- **Against the contour:** the three balls by Uniform at 128³ take 37 ms to contour and 239 ms to contour and simplify (step 0). Their fields take 181 ms simplified, and 3.3 s on the contour's 52,224 vertices, which become 613,372.
- **Triangles** take a third to four fifths longer than quads: their search for lattice steps is over 36 pairs, each lattice's six found among sixteen.
- **The split and the hierarchy** are by one worker, and take a third to a half of a contour's time between them.

### Features at the scale of the edge length (step 3b, 2026-11-18)

**Setup:**
- **The turned cubes:** step 3's two, by every algorithm (Burst) at 32³, 64³ and 128³, contoured and simplified at the voxelizer's defaults. Their 12 edges and 8 corners are known exactly. A feature edge is **on** a true edge where both its ends are within a fiftieth of a cell of it, **beside** where within a cell, and **far** otherwise.
- **The skeletons:** the 15 models, by every algorithm at the three grids, contoured and simplified: 360 meshes.
- **The scale:** the edge length a remesh at 1 has, but no more than a cell, as `Remeshing.featureLengthOf` gives it, unless a table says otherwise. The Feature Angle is 40°.
- **The probe:** `SkeletonProbe.cs` in the probes folder, with `run_step3b_measures.sh`, and `IsotropicProbe.cs` with `run_isotropic_3b.sh`. Unity was in front throughout, and the runs were on the final code.

**The turned cubes as given,** by each edge's own angle / at scale: lines, corners (of them within a cell of a true corner), feature edges beside and far. Adaptive is Uniform's to the edge:

| Cube | Algorithm | Grid | Contour | Simplified |
|---|---|---|---|---|
| about an axis | Uniform, Manifold | 32³ | 66, 38 (8) / 12, 8 (8) | 12, 8 / 12, 8 |
| | | 64³ | 122, 73 (12) / 12, 8 (8) | 12, 8 / 12, 8 |
| | | 128³ | 190, 136 (8) / 12, 8 (8) | 12, 8 / 12, 8 |
| | Occupancy | 32³ | 12, 8 / 12, 8 | 12, 8 / 12, 8 |
| | | 64³ | 32, 25 (8) / 12, 8 (8) | 12, 8 / 12, 8 |
| | | 128³ | 24, 16 (8) / 12, 8 (8) | 12, 8 / 12, 8 |
| about a diagonal | Uniform | 32³ | 167, 114, 36 beside, 7 far / 27, 24, 6, 0 | 62, 36, 12, 19 / 26, 21, 1, 2 |
| | | 64³ | 359, 238, 111, 10 / 54, 50, 17, 0 | 144, 81, 24, 49 / 64, 50, 6, 9 |
| | | 128³ | 699, 471, 212, 29 / 102, 100, 39, 0 | 282, 165, 41, 117 / 101, 86, 4, 16 |
| | Manifold | 32³ | 118, 83 / 12, 8 | 12, 8 / 12, 8 |
| | | 64³ | 247, 169 / 17, 12 | 12, 8 / 12, 8 |
| | | 128³ | 475, 333 / 24, 20 | 12, 8 / 12, 8 |
| | Occupancy | 32³ | 127, 101, 165, 14 / 62, 59, 136, 4 | 69, 46, 57, 15 / 24, 22, 41, 2 |
| | | 64³ | 259, 214, 376, 22 / 107, 102, 301, 16 | 85, 60, 87, 14 / 27, 25, 62, 4 |
| | | 128³ | 487, 406, 725, 65 / 212, 208, 588, 39 | 166, 119, 112, 41 / 68, 64, 90, 14 |

- **Uniform's cube turned about a diagonal** keeps 12, 34 and 68 points where its sheets touch, which are hard, at the three grids.
- **Occupancy's** keeps strips that bridge its creases wider than half a cell: the feature edges beside the true ones are four in ten of its contour's at scale.

**The turned cubes remeshed** by Isotropic at the defaults, read at the remesh's own scale: lines and corners, with the features found the old way / at scale:

| Cube | Algorithm | 32³ contour | simplified | 64³ contour | simplified | 128³ contour | simplified |
|---|---|---|---|---|---|---|---|
| about an axis | Uniform, Adaptive, Manifold | 12, 8 | 12, 8 | 12, 8 | 12, 8 | 12, 8 | 12, 8 |
| | Occupancy | 12, 8 | 12, 8 | 12, 8 | 12, 8 | 12, 8 | 12, 8 |
| about a diagonal | Uniform | 28, 28 | 26, 20 | 52, 54 | 65, 50 | 101, 109 | 104, 86 |
| | Manifold | 12, 8 | 12, 8 | 12, 8 | 12, 8 | 20, 16 | 12, 8 |
| | Occupancy | 29, 25 | 12, 8 | 34, 31 | 12, 8 | 89, 85 | 24, 19 |

- **Read by each edge's own angle,** as step 3's test read them, the remeshes before 3b had 66, 122 and 189 lines on Uniform's axis cube, 63, 121 and 189 on Manifold's diagonal one, and 62, 109 and 228 on Occupancy's diagonal contours. Read so after 3b, the axis cube by Uniform keeps 32, 43 and 62 lines, every one of them on a true edge: slivers of the remesh along its creases, a matter of its triangles and not of its features.
- **Corners:** every one is within a cell of a true corner, but two of Occupancy's simplified diagonal cube at 32³, at 1.1 and 1.7 cells.
- **Far edges,** more than a cell off any true edge, read at scale: none on the contours but 1 and 3 of Occupancy's at 32³ and 128³; 4, 12 and 20 on Uniform's simplified diagonal cube and 5 on Occupancy's at 128³. Before 3b, read by each edge, there were 22, 57 and 122 on Uniform's.

**The skeletons of the 15 models,** every algorithm, lines / corners / length of the feature edges by angle in cells, by each edge's own angle → at scale:

| Grid | Contour | Simplified |
|---|---|---|
| 32³ | 6,384 / 4,301 / 12,638 → 2,702 / 2,462 / 9,708 | 3,435 / 2,317 / 10,946 → 2,381 / 1,924 / 9,779 |
| 64³ | 12,295 / 8,153 / 25,616 → 4,044 / 3,907 / 19,794 | 6,195 / 4,003 / 23,303 → 3,358 / 2,930 / 19,689 |
| 128³ | 23,572 / 15,689 / 51,535 → 6,574 / 6,799 / 39,669 | 11,077 / 7,056 / 47,314 → 5,208 / 5,069 / 39,456 |

- **The flat models,** the cube, both slabs, the plate, the hollows across faces, the two balls and the ball, have the same skeleton at scale as by each edge, on every mesh.
- **More lines or corners at scale** on one mesh: Occupancy's simplified cut-away ball at 32³, 3 lines and 10 corners → 13 and 16, where the band finds a crease spread over its edges.
- **Most of the length taken out** is the noise of finding 25: at 64³ the turned and the new shapes lose a fifth (Occupancy) to a half (Uniform) of theirs, the three balls by Uniform three fifths, and the cubes with a ball by Uniform, where the ball meets the cubes, a sixth.

**The scale on simplified meshes** (finding 26, decision 12): the 15 models at 64³ by every algorithm, simplified: lines / corners / length by angle, by each edge → at no more than a cell → at the edge length:

| Model | By each edge | At a cell | At the edge length |
|---|---|---|---|
| ThreeBalls | 483 / 276 / 1,066 | 105 / 114 / 702 | 98 / 96 / 772 |
| CubesAndSphere | 105 / 71 / 2,442 | 98 / 67 / 2,428 | 99 / 68 / 2,424 |
| SphereCutaway | 83 / 73 / 836 | 34 / 28 / 862 | 18 / 12 / 825 |
| SlabWithBand | 112 / 64 / 1,843 | 112 / 64 / 1,843 | 64 / 64 / 776 |
| TurnedShapes | 2,334 / 1,425 / 4,704 | 1,030 / 1,036 / 2,963 | 690 / 905 / 2,463 |
| NewShapes | 1,732 / 1,127 / 3,623 | 851 / 751 / 2,613 | 602 / 599 / 2,339 |
| HollowsAcrossFaces | 336 / 224 / 3,189 | 336 / 224 / 3,189 | 48 / 224 / 2,458 |
| SlabAcrossBounds | 161 / 97 / 1,094 | 161 / 97 / 1,094 | 131 / 97 / 593 |
| TurnedCubeDiagonal | 385 / 230 / 2,201 | 167 / 133 / 1,691 | 122 / 113 / 1,552 |
| All 15 | 6,195 / 4,003 / 23,303 | 3,358 / 2,930 / 19,689 | 2,336 / 2,594 / 16,506 |

The cube, the cube turned about an axis, the plate, the ball across the bounds and the two balls are the same in all three.

**What it changes in Isotropic,** at the defaults, the 15 models at 64³ by Uniform and Occupancy: the medians over the 30 meshes of each kind of step 3's measures, with the features found the old way → at scale (contour / simplified):

| Measure | Contour | Simplified |
|---|---|---|
| Edges within the paper's limits | 0.987 → 0.987 | 0.818 → 0.813 |
| Inner vertices of six edges | 0.870 → 0.872 | 0.678 → 0.689 |
| Smallest angle, mean | 52.9° → 52.9° | 45.9° → 46.0° |
| Spread of vertex areas | 0.085 → 0.084 | 0.140 → 0.137 |
| Vertices against the input | 1.01 → 1.01 | 1.16 → 1.17 |
| Distance to the shapes, mean, in cells | 0.0049 → 0.0049 | 0.018 → 0.018 |

- **Slivers:** summed over the 30 contours, the share of triangles under 30° falls from 0.19 to 0.14, and under 10° from 0.057 to 0.017. The simplified meshes are as they were.
- **Lines of the remesh,** read by each edge's own angle: on contours 4,739 → 3,668 over the 30, the turned and the new shapes a sixth fewer and the cubes with a ball by Uniform 141 → 39. On simplified meshes 2,492 → 2,514, most of the gain on Occupancy's turned shapes, 264 → 316, whose remesh is also a fifth further from the shapes, 0.018 → 0.021 cells.
- **The flat models** remesh the same, line for line.

**Time:** the remesh alone with the features found the old way (A) and at scale (B), in turns A, B, A, B, the median of three after one of each to warm up, and of one at 128³ contoured; every algorithm and the 15 models; Unity in front, but for Uniform's contours at 64³, which ran with another app in front and count only for the ratio. In ms:

| Grid | Mesh | A | B | B against A, median (least to most) | Lists, features and grid, of B |
|---|---|---|---|---|---|
| 32³ | contour | 145 | 145 | 1.00 (0.99 to 1.04) | 2.4% |
| 32³ | simplified | 18.2 | 18.2 | 1.00 (0.90 to 1.34) | 2.8% |
| 64³ | contour | 522 | 520 | 1.00 (0.97 to 1.04) | 2.5% |
| 64³ | simplified | 35.0 | 35.0 | 1.00 (0.95 to 1.34) | 2.4% |
| 128³ | contour | 2,055 | 2,063 | 1.00 (0.99 to 1.02) | 2.5% |
| 128³ | simplified | 63.7 | 63.8 | 1.00 (0.90 to 1.31) | 2.5% |

- **The skeleton at scale costs nothing that shows:** it is a part of the build stage, which is a fortieth of the remesh.
- **Not comparable with step 3's timings:** these ran about four and a half times slower than those, as the normals' measurement after step 3 did, with Normals at Faces here. The reason is still not known; A and B ran alike.

### Field Aligned's mesh (step 5, 2026-11-19)

**Setup:**
- **Models:** step 3's 15, by every algorithm (Burst) at 64³, contoured and simplified at the voxelizer's defaults: 120 meshes, each remeshed four ways at an edge length of 1, as Quad Dominant, as Triangles and as Quads, and as Quad Dominant with the lines between materials left free: 480 remeshes. The Feature Angle is 40°, and the normals out are the input's, creased at 40°.
- **Time:** Uniform from 32³ to 128³, contoured and simplified, each kind of face, the median of three after one to warm up.
- **Measures:** faces by their sides; the inner vertices, those of no open edge, with four edges for quads or six for triangles; table 1 of the paper, over the faces of the kind asked for: their areas against the mean, and their angles against a square's or a triangle's, the least, the mean of each face's least, the most and the mean of each face's most; edges against the length; the distance of every vertex to the shapes, in cells; lines of open edges and of more than two triangles, and pieces, against the mesh given; how far lines between materials moved, from the middle of every edge between two materials to the nearest such edge of the other mesh, each way.
- **The probe:** `FieldAlignedProbe.cs` in the probes folder (`Quality`, `Times`, and `Hand`, `Debug`, `Split`, `Square` and `AreaPatch` for the step's own questions), with `run_field_aligned_quality.sh`, `run_field_aligned_rest.sh` and `summarize_field_aligned.py`; the rows in `probes/remeshing-step5/`. Unity was in front throughout, and the runs were on the final code.

**The faces,** the 60 meshes of each kind together, medians over the meshes:

| Faces | Mesh | Vertices against the input's | Triangles / quads / pentagons | Regular inner vertices | Area σ, least, most | Angle σ | Least angle, mean least | Most angle, mean most | Edges against *l*, 5% / 50% / 95% | Time |
|---|---|---|---|---|---|---|---|---|---|---|
| Quad Dominant | contour | 1.002 | 1.0% / 98.5% / 0.5% | 98.1% | 0.07, 0.51, 1.41 | 3.6° | 55°, 87° | 140°, 93° | 0.92 / 1.00 / 1.06 | 669 ms |
| Quad Dominant | simplified | 1.022 | 4.2% / 93.8% / 2.1% | 91.8% | 0.12, 0.57, 1.36 | 4.7° | 65°, 86° | 117°, 94° | 0.82 / 1.00 / 1.11 | 69 ms |
| Triangles | contour | 1.006 | all | 94.2% | 0.12, 0.004, 1.70 | 4.9° | 0.3°, 57° | 179°, 64° | 0.89 / 1.00 / 1.10 | 924 ms |
| Triangles | simplified | 1.026 | all | 80.7% | 0.19, 0.26, 1.52 | 10.4° | 16°, 52° | 136°, 69° | 0.78 / 1.00 / 1.16 | 118 ms |
| Quads | contour | 1.012 | all | 98.9% | 0.10, 0.32, 1.55 | 4.9° | 42°, 87° | 172°, 93° | 0.91 / 1.00 / 1.08 | 221 ms |
| Quads | simplified | 1.056 | all | 95.7% | 0.19, 0.38, 1.50 | 8.5° | 45°, 83° | 136°, 97° | 0.71 / 0.98 / 1.13 | 26 ms |

- **About as many vertices as the mesh given,** as decision 9 has it: on contours 0.97 to 1.07 times as many. On simplified meshes the spread is wider, up to twice as many as Quad Dominant and three times as quads alone on the smallest, such as the slab with a band, whose 16 vertices are fewer than its faces need.
- **The paper's table 1** has the same columns for its own models, larger and scanned, as quad dominant: area σ 0.11 to 0.20, angle σ 4.4° to 12°, the mean least angle 79° to 86° and the mean most 94° to 100°. The contours here are in the same range or better, and the simplified meshes, of a few hundred vertices, alike.
- **Triangles are the weakest:** the least angle of a contour's triangles is under a degree, and a fifth of a simplified mesh's inner vertices are irregular. A triangular lattice meets the right angles of these models at sixty degrees, and where it does not fit, faces of more than three sides are cut into triangles of whatever shape is left.
- **Quads alone** are read off faces twice the length, split, and so are the most regular and the quickest, a third of the time of Quad Dominant: their fields are on a quarter of the vertices. On the simplified slab with a band they give up: no face comes out of the fields at twice its length.
- **The distance to the shapes** is the mesh given's: every vertex out is on that mesh (finding 28). On contours 0.005 of a cell on the mean, and on simplified meshes 0.01.
- **What the extraction meets,** over the 60 contours as Quad Dominant: 1,948 vertices of the graph spurious, 2,891 snapped onto an edge and 293 merged into a neighbour, 98 holes filled and 105 left, which are the open rims and the holes of more than six sides; no snapping that did not settle, and no vertex out of reach of the input. Two faces had nothing mapped into them.

**Topology** (finding 30), lines of open edges and of more than two triangles against the mesh given:

| Faces | Mesh | Meshes with more open lines, lines | with more lines of more than two triangles | Unfilled holes |
|---|---|---|---|---|
| Quad Dominant | contour | 11, 74 | 4 | 105 |
| Quad Dominant | simplified | 8, 4 | 2 | 33 |
| Triangles | contour | 24, 114 | 31 | 159 |
| Triangles | simplified | 8, 7 | 11 | 36 |
| Quads | contour | 8, 26 | 4 | 53 |
| Quads | simplified | 8, −3 | 7 | 25 |

**Lines between materials,** held and free (finding 29), as Quad Dominant, the medians over the four algorithms of each model's mean and furthest distance out to the input's lines, in cells:

| Model | Contour, held | Contour, free | Simplified, held | Simplified, free |
|---|---|---|---|---|
| SlabWithBand | 0.001, 0.005 | 0.32, 0.44 | 0.41, 1.25 | band gone |
| SlabAcrossBounds | 0.003, 0.06 | 0.28, 0.51 | 0.54, 1.13 | 3.7, 5.9 |
| PlateAcrossBounds | 0.001, 0.006 | 0.26, 0.45 | 0.26, 0.51 | 1.2, 2.1 |
| SphereCutaway | 0.004, 0.02 | 0.30, 0.44 | 0.07, 0.21 | 1.2, 1.8 |
| TurnedShapes | 0.03, 0.32 | 0.26, 0.51 | 0.22, 1.0 | 1.0, 2.1 |
| NewShapes | 0.08, 1.06 | 0.26, 1.28 | 0.22, 2.2 | 0.62, 2.2 |

- **Triangles,** held, keep the lines within 0.01 of a cell on the median of contours and 0.28 at the furthest; **quads alone** within 0.01 and 0.06.

**Time,** Uniform, the whole remesh with its fields, in ms, the median of the 15 models (and the most), with the shares of its stages: the sheets and the split, level 0 and the hierarchy, the two fields, the graph read off them, the faces, the materials and the smoothing, and the output:

| Grid | Mesh | Faces | Vertices of the fields | Median | Most | Sheets, split | Graph, hierarchy | Fields | Extraction | Faces | Materials, smoothing | Out |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 32³ | contour | Quad Dominant | 29,200 | 202 | 323 | 17% | 14% | 55% | 9% | 2% | 3% | 0% |
| 32³ | simplified | Quad Dominant | 2,610 | 48 | 110 | 8% | 6% | 79% | 4% | 1% | 2% | 0% |
| 64³ | contour | Quad Dominant | 119,000 | 652 | 1,230 | 22% | 16% | 45% | 11% | 2% | 4% | 0% |
| 64³ | simplified | Quad Dominant | 5,460 | 71 | 177 | 11% | 8% | 72% | 5% | 1% | 2% | 0% |
| 128³ | contour | Quad Dominant | 483,000 | 2,625 | 5,440 | 26% | 17% | 40% | 12% | 2% | 4% | 0% |
| 128³ | simplified | Quad Dominant | 10,700 | 106 | 277 | 14% | 11% | 64% | 6% | 1% | 3% | 0% |
| 128³ | contour | Triangles | 408,000 | 3,418 | 6,827 | 17% | 12% | 56% | 11% | 2% | 3% | 0% |
| 128³ | simplified | Triangles | 8,520 | 151 | 384 | 8% | 6% | 77% | 5% | 1% | 2% | 0% |
| 128³ | contour | Quads | 124,000 | 776 | 1,548 | 19% | 14% | 44% | 12% | 5% | 5% | 1% |
| 128³ | simplified | Quads | 2,680 | 50 | 117 | 8% | 6% | 75% | 5% | 2% | 3% | 1% |

- **What step 5 adds to the fields** is a sixth to a fifth of a contour's time and a tenth or less of a simplified mesh's. The extraction is the largest part of it, and but for the links' classifying it is by one worker.
- **Against Isotropic** (step 3), after simplification: 71 ms where it takes 8 at 64³, and 106 where it takes 12 at 128³. On a contour, 652 ms and 2.6 s where it takes 114 and 435.
- **The fields' limit** of 2²¹ vertices is not met at an edge length of 1: the largest contour at 128³ puts them on about a million.

### Scenes, tuning and documentation (step 8, 2026-11-19)

**Setup:** the 15 models of step 5 by every algorithm (Burst) at 64³, contoured and simplified at the voxelizer's defaults, remeshed at the defaults; times are of `Remeshing.remesh` as the voxelizer calls it, with Unity in front throughout. The probes are `FieldAlignedProbe.cs` (with a share of the split that step 8 added to it and took out again), `CompareProbe.cs` (`Compare`, `Hashes`, and `Levels`, kept in `remeshing-step8/LevelsEntry.cs.txt` as it needed the two constants made settable) and `RemeshScenes.cs`, with `run_split_share.sh`, `run_step8_final.sh`, `run_step8_times.sh`, `run_step8_hashes.sh`, `summarize_share.py` and `summarize_compare.py`; the rows in `probes/remeshing-step8/`.

**The split** (finding 31), Field Aligned over the 60 meshes of each kind, at three longest edges as shares of the lattice's length; time and the faces' measures are medians, holes and lines are totals:

| Faces | Mesh | Share | Vertices of the fields | ms | Regular inner vertices | Area σ | Angle σ | Holes left | Meshes with more open lines, lines |
|---|---|---|---|---|---|---|---|---|---|
| Quad Dominant | contour | 0.5 | 119,408 | 674 | 98.1% | 0.072 | 3.58° | 105 | 11, 74 |
| Quad Dominant | contour | 0.625 | 63,886 | 407 | 98.0% | 0.073 | 3.53° | 145 | 11, 101 |
| Quad Dominant | contour | 0.75 | 41,172 | 314 | 98.0% | 0.073 | 3.52° | 156 | 18, 107 |
| Quad Dominant | simplified | 0.5 | 5,169 | 68 | 91.8% | 0.119 | 4.67° | 33 | 8, 17 |
| Quad Dominant | simplified | 0.625 | 3,407 | 62 | 91.8% | 0.119 | 4.47° | 37 | 8, 20 |
| Quad Dominant | simplified | 0.75 | 2,620 | 49 | 91.3% | 0.124 | 5.17° | 34 | 7, 19 |
| Triangles | contour | 0.5 | 101,411 | 936 | 94.2% | 0.116 | 4.86° | 159 | 24, 114 |
| Triangles | contour | 0.625 | 62,616 | 640 | 94.1% | 0.118 | 4.96° | 190 | 23, 146 |
| Quads | contour | 0.5 | 30,012 | 221 | 98.9% | 0.102 | 4.87° | 53 | 8, 26 |
| Quads | contour | 0.625 | 16,131 | 153 | 98.6% | 0.103 | 5.14° | 65 | 8, 37 |

- **On Uniform alone** a share of 1 took a contour's remesh to 230 ms and left 72 holes where 0.5 left 36, and 8 meshes with more open lines where 0.5 left 3.

**The small levels** (finding 32), Field Aligned as quads with triangles and pentagons over Uniform's 15 models at 64³, the median of three remeshes after one, by the smallest level smoothed colour by colour and the vertices of a colour to a batch:

| Level, batch | Simplified, ms | its fields | Contour, ms | Remeshes that differ |
|---|---|---|---|---|
| 4,096, 64 (before) | 71 | 50 | 666 | — |
| 1,024, 64 | 53 | 33 | 646 | 0 |
| 256, 16 | 49 | 29 | 642 | 0 |
| 256, 4 (built) | 45 | 25 | 638 | 0 |
| 256, 1 | 43 | 22 | 634 | 0 |
| 0, 4 | 46 | 26 | 640 | 0 |

**Isotropic's checks guessed** (finding 33), against the code before step 8, over the 60 meshes of each kind by every algorithm, medians in ms; all 960 remeshes of both techniques at 32³ and 64³ hash the same:

| Grid | Mesh | Isotropic before | after | Field Aligned, Quad Dominant, before | after |
|---|---|---|---|---|---|
| 32³ | contour | 146 | 58 | 207 | 175 |
| 32³ | simplified | 18.5 | 8.7 | 47 | 26 |
| 64³ | contour | 519 | 216 | 679 | 654 |
| 64³ | simplified | 35 | 17 | 68 | 43 |

**Both measured alike,** at 64³ over the 60 meshes of each kind, medians, with the topology against the mesh given in all:

| Technique | Mesh | Vertices against the input's | Regular inner vertices | Area σ | Angle σ | Least angle, mean least | Edges against *l*, 5% / 50% / 95% | Meshes with more open lines | Lines between materials, mean / furthest, cells |
|---|---|---|---|---|---|---|---|---|---|
| Isotropic | contour | 1.006 | 84.7% | 0.105 | 8.0° | 21°, 53° | 0.87 / 1.00 / 1.17 | 0 | 0 / 0 |
| Isotropic | simplified | 1.231 | 66.3% | 0.236 | 18.6° | 19°, 45° | 0.73 / 0.98 / 1.21 | 0 | 0 / 0 |
| Field Aligned, Quad Dominant | contour | 1.002 | 98.1% | 0.072 | 3.6° | 55°, 87° | 0.92 / 1.00 / 1.06 | 11 | 0.004 / 0.054 |
| Field Aligned, Quad Dominant | simplified | 1.022 | 91.8% | 0.119 | 4.7° | 65°, 86° | 0.82 / 1.00 / 1.11 | 8 | 0.26 / 1.0 |
| Field Aligned, Triangles | contour | 1.006 | 94.2% | 0.116 | 4.9° | 0.3°, 57° | 0.89 / 1.00 / 1.10 | 24 | 0.011 / 0.28 |
| Field Aligned, Quads | contour | 1.012 | 98.9% | 0.102 | 4.9° | 42°, 87° | 0.91 / 1.00 / 1.07 | 8 | 0.011 / 0.063 |

- **Isotropic** changes no topology of any of the 120 meshes, and its lines between materials are the input's exactly. Its angles are measured against 60°, Field Aligned's quads against 90°.
- **Quads alone** gave up on the slab with a band simplified, by every algorithm: no faces came out of its fields.

**Time on the final code,** Uniform's 15 models, the median of three after one, in ms, the median (and the slowest), with the shares of the stages:

| Grid | Mesh | Isotropic | its build, split, collapse, flip, smooth, areas | Field Aligned, Quad Dominant | its sheets and split, graph and hierarchy, fields, extraction, faces, smoothing |
|---|---|---|---|---|---|
| 32³ | contour | 57 (140) | 6%, 13%, 33%, 13%, 19%, 16% | 170 (306) | 21%, 16%, 45%, 11%, 2%, 4% |
| 32³ | simplified | 8 (57) | 4%, 10%, 32%, 11%, 21%, 19% | 26 (81) | 17%, 12%, 56%, 8%, 2%, 4% |
| 64³ | contour | 209 (505) | 6%, 12%, 34%, 14%, 17%, 16% | 630 (1,207) | 25%, 18%, 38%, 12%, 2%, 4% |
| 64³ | simplified | 17 (117) | 4%, 12%, 39%, 12%, 18%, 15% | 44 (146) | 20%, 14%, 50%, 9%, 2%, 4% |
| 128³ | contour | 834 (2,113) | 7%, 11%, 34%, 15%, 17%, 16% | 2,608 (5,474) | 27%, 19%, 35%, 13%, 2%, 4% |
| 128³ | simplified | 25 (192) | 4%, 11%, 42%, 12%, 16%, 15% | 77 (248) | 22%, 16%, 45%, 10%, 2%, 4% |

- **Field Aligned's triangles and quads alone** are in the README: at 128³ triangles take 3,366 ms on a contour and 94 simplified, quads alone 741 and 29.
- **Against step 3,** whose times were from before the correctness review: Isotropic takes 1.8 to 1.9 times as long on contours and 2.2 to 2.3 times on simplified meshes. The difference is the review's searches of the input, in the collapses and flips as well as the smoothing.
- **What is left by one worker:** Isotropic's collapses, a third of its time, and its splits and flips; Field Aligned's sheets, split, graph and hierarchy, two fifths of a contour's time, and its extraction.

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
   - **Satisfiability: answered, as recommended, which leaves it open until step 7.** The greedy pass is built, step 7 measures what it leaves on this project's models, and the question is asked again then with those numbers. If it is wanted it is a small solver of its own for regions of a few hundred variables. **Still open,** with steps 6 and 7 deferred on 2026-11-19.
7. **Isotropic's density by curvature?** This was a question about the earlier paper, whose samples follow a density. **It no longer applies.** Botsch and Kobbelt's loop has one edge length for the whole mesh, and edge lengths that vary with curvature are out of scope.
8. **Burst only?** **Answered: yes, as recommended.** A managed contour is remeshed by Burst code, and a GPU contour is read back, as for simplification.

### Asked after step 0

9. **What is the Edge Length measured against?** Finding 16. **Answered on 2026-11-17: the mesh the pass is given,** as recommended, in the user's words "For decision 9, option 2." The Design's Settings have it.
   - **Cells, as the plan had it,** with a default of 2. Every mesh comes out at the same density whatever the simplifier did. At 64³ that is four times as many vertices as the simplifier leaves on round models, and 50 to 150 times as many on flat ones. With remeshing on, the simplifier would then only clean the surface, and its Error and Detail would change little of what is shown.
   - **The mesh the pass is given,** as all three papers' defaults do: 1 is the length that keeps its vertex count, worked out from its area for each technique's faces, and 0.5 gives four times the vertices. The simplifier's Error and Detail then set how many vertices there are, and the remesh how they are spread. On an unsimplified contour, 1 is about one cell. The spread is still uniform, so a round part comes out coarser than the simplifier left it and a flat part finer.
   - **Both,** as a mode beside the length: more to explain, and the two would only meet on unsimplified contours.

   Recommended: the mesh the pass is given, at 1. It is what "a pass that goes after mesh simplification" asks for, as far as I can read it, and it is what the papers do.

### Asked after step 3

10. **Features by angle at the scale of the edge length?** Finding 19. Isotropic keeps the noise of contours whose edges cross the grid: the crease of a turned cube comes out as two lines with a strip of triangles between them, and many corners.
    - **Leave it:** simplification on, which is how the pass is meant to be used, cleans most of it for Manifold and Occupancy, and the cube turned about an axis for every algorithm. Uniform's and Adaptive's turned shapes, and every unsimplified contour of such an edge, keep it.
    - **Find features by angle at the scale of *l*:** a feature edge by angle where the surface bends by more than the angle across a band about *l* wide, not across one edge, so that a strip of bridging triangles counts as one crease; and lines by angle shorter than *l* that end loose taken out. A change to `BurstFeatures`, which all three techniques use, and its own step, before Field Aligned: step 4 holds its fields to features too.
    - **Raise the Feature Angle's default:** fewer noisy lines, by a fifth to a half at 60° (finding 17). It cannot mend the bridges: both sides of a bridge across a right angle meet their faces at about 45°, so at 60° neither is a feature and the crease is lost.

    Recommended: at the scale of *l*, as a step of its own before step 4, since Field Aligned would otherwise inherit the same noise in the edges it holds its fields to.

    **Answered on 2026-11-18: as recommended,** in the user's words "For decision 10, go with your recommendation." Features by angle are found at the scale of *l*, in step 3b, before step 4.

11. **The remesh's normals.** Finding 21. The two ways out were:
    - the normals the input would show creased, where each vertex lies;
    - each vertex's own triangles' normal.

    **Answered on 2026-11-18: both, as a setting,** in the user's words "Implement both options so I can experiment with both." Input is the default.

### Asked after step 3b

12. **What scale are the features of a simplified mesh found at?** Finding 26. The edge length of a simplified mesh of flat faces is many cells, and at that scale its narrow faces are thin, and merge into their neighbours.
    - **At the edge length, as decision 10 has it:** the simplified round and turned shapes come out with somewhat fewer lines, and the simplified flat models lose features the simplifier kept: the slab with a band's sides, the hollows across faces. A remesh rounds those off.
    - **At the edge length, but no more than a cell:** the noise of the contour's cells is cleared as well on contours, where the two are the same, and somewhat less on simplified meshes, and every simplified flat model keeps its skeleton exactly.

    Recommended, and what 3b builds: no more than a cell. The noise 3b clears is of the cells, and what a simplified mesh keeps beyond a cell is the shape. It is one line, `Remeshing.featureLengthOf`, to change.

    **Answered on 2026-11-18: as recommended,** in the user's words "For decision 12, go with your recommendation." Features by angle are found at the edge length, but at no more than a cell.

