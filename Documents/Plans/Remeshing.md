# Remeshing

Plan written 2026-11-17 for a pass that rebuilds the mesh of a contour after mesh simplification, with three techniques to choose between: evenly shaped triangles from a loop of edge splits, collapses, flips and smoothing; triangles or quads that follow an orientation field; and quads with few irregular vertices.

**Status:** decided, not started. No code is written and nothing is measured. All eight decisions were answered on 2026-11-17 (Decisions). Step 0 waits for the go-ahead.

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

One **Edge Length** setting, in cells, sets the size of the faces for all three, so they can be compared like for like. With the setting off, every mesh is the one it is today, bit for bit.

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
   - **Field Aligned and QuadriFlow keep their fields on the input's vertices,** so the input has to be finer than the output (section 3.8 of the second paper). The reference code splits every input edge longer than half the output's edge length before anything else.
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
   - **Collapses and flips are refused** where they would change the topology or turn a triangle over (finding 7), and a collapse also where it would leave an edge longer than 4/3 *l*, which the next round would only split again.
   - **Vertices are put back on the input.** Moving within a tangent plane leaves a curved surface a little each time, and over five rounds and the area smoothing after them the mesh would shrink. After every move a vertex goes to the nearest point of the input: of the input's triangles in its own patch between feature lines, or of its own feature line.
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

11. **Both field papers have public reference code, and it differs from the papers.** I looked at the two repositories' READMEs and licences and had three source files summarized (Instant Meshes' `batch.cpp` and `extract.cpp`, QuadriFlow's `main.cpp`). I did not read them line by line and ran nothing. Step 0 reads the parts listed here properly.
    - **Licences:** Instant Meshes (github.com/wjakob/instant-meshes) is BSD 3-clause with one clause added: changes that are published are licensed back to the authors. QuadriFlow (github.com/hjwdzh/QuadriFlow) is MIT. Both allow this project to follow them, with credit.
    - **Instant Meshes, where the code says more than the paper:**
      - it splits input edges longer than half the output's edge length, or twice the input's mean;
      - it constrains both fields along open edges with a weight of 1;
      - after merging vertices it snaps a vertex that lies within 0.3 of an edge length of an edge onto it, and removes the diagonals of quads;
      - it walks faces of three to eight sides and fills holes of up to six;
      - for quads only it splits every face into four round its middle and does not double the edge length, where the paper's figure 1 solves at quarter resolution.
    - **QuadriFlow, where the code differs from the paper:**
      - by default it solves a maximum flow, not a minimum cost flow, which is an option;
      - the satisfiability step is off by default and calls an outside program;
      - it has a step that solves for a scale varying over the mesh, which the paper does not describe.
    - **What follows:** the papers are the plan. The code is read where a paper is silent (decision 2).
    - **Isotropic's paper has no reference code.**

12. **The output has to be the same every time.** The second paper starts its fields from random directions and positions. A fixed seed, colours visited in a fixed order and stable sorts make the same mesh give the same output, bit for bit, which the tests and the `Start` hashes rely on. Isotropic's loop is in a fixed order (finding 9). A GPU contour has its vertices in another order every run, so it will remesh alike and not the same, as it simplifies today.

13. **How long they will take is not known.** These are the papers' own figures, not measurements of this project:
    - **Isotropic:** 100,000 triangles in under 5 seconds on a Pentium 4 of 2004, on one core. Its splits, collapses and flips are one after another, as the simplifier's loop is; its smoothing and the search for nearest points are parallel. The simplifier takes 206 ms for the three balls at 128³, and this is work of the same kind and order.
    - **Field Aligned:** under a second for inputs under 100,000 triangles on twelve cores.
    - **QuadriFlow:** five seconds for a million faces, with the fields solved on a GPU and the rest on one core.

    The interactive editor's scheduler already copes with slow work: it waits for changes to settle and pauses over a second.

14. **Two pieces of solver code are QuadriFlow's alone,** and each is a known algorithm with answers in closed form to test against: conjugate gradients and minimum cost flow. They get their own files and their own tests, apart from the technique.

## Design

### Settings

On `Voxelizer`, under a **Remeshing** header after Mesh Simplification, carried over by the editor's `select`:

| Setting | Default | What it does |
|---|---|---|
| Technique | Off | Off, Isotropic, Field Aligned or QuadriFlow. Off, every mesh is what it is without this feature. |
| Edge Length | 2 | How long the edges of the new faces are, in cells. Provisional: step 0 looks at what the contours and the simplified meshes are like and sets it. |
| Feature Angle | 40 | Two triangles that meet at more than this many degrees have a sharp edge between them, which is kept. The same number as the Crease Angle's default, and a setting of its own: one decides where the mesh is held, the other how it is shaded. |
| Iterations | 5 | Isotropic only. How many times the loop of splits, collapses, flips and smoothing is gone through. The paper's "about 5". |
| Area Iterations | 10 | Isotropic only. How many rounds of smoothing by vertex area follow the loop. Zero leaves them out. The paper's "fewer than 20"; provisional until step 3 measures. |
| Faces | Quad Dominant | Field Aligned only. **Triangles**; **Quad Dominant**, quads with triangles and pentagons where rows of faces begin and end; or **Quads**, quads only. |

- **Edge length and not a vertex count.** All three papers take a length, or a count that is turned into one. A length in cells is what the project's other tolerances are in, and it means the same thing for a cube and a ball.
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

- **The feature skeleton** (finding 6): the kind of every edge, the corners, and the lines as runs of edges between corners or closed loops. In the simplifier's lists a fourth kind of sharp edge, by angle, beside the three it has. The simplifier itself never sets it, and its meshes stay what they are, which its tests and the hashes hold.
- **Splitting long edges:** the first operation of Isotropic's loop. Field Aligned and QuadriFlow prepare their input with it (finding 3).
- **Sheets** (decision 4): the mesh split along its edges of four triangles and at its vertices of several fans, so that what is left is a manifold. Field Aligned and QuadriFlow work on the sheets. Isotropic does not need them.
- **Polygons out,** for the two field techniques: each polygon as triangles with the flags of finding 8. A quad is split along the diagonal that leaves its two triangles flatter.
- **Materials out,** for the two field techniques: an output face takes the sub-mesh that most of the input's area mapped into it has.

### Isotropic

1. **The lists:** the simplifier's `Connectivity`, built from the input, with the feature skeleton marked in it. Two operations are added to it, each updating everything `collapse` updates:
   - **`split`** of an edge at a point, for an edge of any number of triangles. Every triangle of the edge becomes two of its sub-mesh.
   - **`flip`** of an edge of two triangles.
2. **The input, kept to go back to:** its triangles in a grid for the search of the nearest point, each with the patch between feature lines it belongs to, and its feature lines.
3. **The loop,** for the set number of iterations, with *l* the edge length:
   1. **Split** every edge longer than 4/3 *l* at its midpoint, longest first, the new edges among them, until none is longer.
   2. **Collapse** every edge shorter than 4/5 *l*, shortest first, into the position finding 9 gives it, where `check` allows it and no edge longer than 4/3 *l* comes of it.
   3. **Flip** every edge of two triangles that is not a feature where that brings its four vertices nearer their number of edges: six, or four on an open edge, as the paper has it. A flip is refused where the new edge exists, where a triangle would turn over, and where it would make a feature (finding 9).
   4. **Smooth:** every vertex towards the middle of its neighbours, within its tangent plane, then back onto the input. All moves of a round are worked out from the positions the round began with, in parallel.
4. **Equal areas,** for the set number of rounds: the same move towards the middle of the neighbours weighted by their Voronoi areas, damped, and back onto the input. The Voronoi area is the one the paper's section 3 describes: from the circumcentres of a vertex's triangles, or the midpoints of edges where an angle is obtuse.
5. **Out:** the positions; for each vertex the input's normal at the point it was put back on; the triangles with their sub-meshes.

### Field Aligned

1. **Preparation:** the sheets, and every edge longer than half the edge length split (finding 3).
2. **The graph:** the vertices with the mesh's edges between them, a normal and an area for each. A vertex on a feature edge takes the normal of one of its faces, as section 3.5 has it.
3. **The hierarchy** of section 3.5, and a colouring of every level so that vertices of one colour share no edge and can be visited in parallel.
4. **The orientation field,** then **the position field,** six iterations a level from the coarsest level down, from a fixed random start. Along open edges and lines between materials both fields are held: the direction to the line's, and the lattice to pass through it (section 3.7).
5. **Extraction** as section 3.8 has it, with what the code adds where the paper is silent (finding 11): merging in order of distance, leaving out vertices too few inputs agree on, walking the faces, filling small holes.
6. **Faces:**
   - **Triangles:** sixth turns and a triangular lattice.
   - **Quad Dominant:** quarter turns and a square lattice. Triangles and pentagons stay as they are.
   - **Quads:** the same at twice the edge length, then every face split into quads round its middle, so the edges come out at the length that was asked for.
7. **Out:** positions and normals as the weighted means of section 3.8, materials by area (finding 5), polygons as triangles with their flags.

### QuadriFlow

1. **Fields:** Field Aligned's steps 1 to 4 for quads. The input must be a manifold here, so the sheets are not optional.
2. **Offsets:** for every edge, from the fields' own whole numbers (definition 3.1).
3. **The flow network** of section 3.3: a breadth-first walk over the triangles turns each triangle's equation so that an edge's offset enters its two triangles with opposite signs; offsets it cannot balance, and those of open edges, are fixed; the fixed ones are adjusted until supply equals demand.
4. **The flow:** successive shortest paths, which gives a minimum cost flow exactly. Offsets start within two steps, and the limit is raised until the flow fits, as in the paper.
5. **Long offsets:** an edge whose offset is over one step is split in the middle (figure 6).
6. **Turned triangles:** the greedy pass of section 3.4. What it leaves is counted (decision 6).
7. **Positions:** equation 9 by conjugate gradients, each vertex in its tangent plane, a vertex on a feature edge along that edge, and a corner where it is.
8. **Extraction** (section 3.6): edges of offset zero collapsed, right-angled triangles paired along their long sides. A triangle left without a partner, as at an open edge, stays a triangle.
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
  - **Sheets:** two boxes that share an edge come apart into two closed manifolds; a mesh that is a manifold comes back as it was.
  - **Polygons:** the flags of a quad, a pentagon and a triangle; the welded triangles give back the polygons.
- **Isotropic:**
  - **Split and flip,** as the simplifier's collapse is tested: after any run of splits, flips and collapses the lists are those of a mesh built from nothing; a split keeps the surface point for point, every sub-mesh's area, and the kind of every feature; a flip of a flat pair gives the other diagonal; each reason for refusing a flip has a mesh that gives it and one next to it that flips. Edges of one, two and four triangles are split.
  - **Nearest point:** against a search over every triangle, on points near faces, edges and vertices, and never onto another patch.
  - **Positions of collapses:** each case of finding 9 on a box with a line between two materials.
  - **A flat square:** after the loop, the share of edges between 4/5 and 4/3 of the length, the share of inner vertices with six edges, and the mean smallest angle, each over a bar step 3 measures and sets.
  - **Equal areas:** the spread of vertex areas is smaller after the rounds than before, by a factor step 3 measures; the paper claims about five.
  - **An upright box** stays the box: every vertex on a face, every edge vertex on its edge, every corner at its corner, the volume unchanged.
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
  - With the angle kind of sharp edge added to its lists, the simplifier's meshes are what they were, collapse for collapse.
  - `remesh` after a change of edge length or technique shows what a fresh contour under those settings shows, without contouring.
  - A GPU contour is read back and remeshed; creased and split normals after remeshing; past 65,535 vertices.
  - A technique that gives up shows the input and reports why.
  - The wireframe of a quad mesh has no diagonals, by offscreen render.
- **Editor:** the settings carry over; a change of remeshing settings does not contour; a change of detail remeshes.
- **Measured and reported for every technique,** on the 13 models of the simplification probe and all four algorithms, with and without simplification: time by stage; faces by number of sides and vertices by number of edges; the spread of angles, areas and edge lengths, as table 1 of the second paper and table 2 of the third have them; distance to the shapes; open and non-manifold edges and pieces against the input's; how far lines between materials moved.

## Steps

Each step compiles, passes every test, and is reviewed before the next one starts. Nothing is committed for you. The order is by what depends on what: the frame, then Isotropic, which is the smallest and whose split the others use, then Field Aligned, then QuadriFlow, which needs Field Aligned's fields.

0. **Baseline and what the pass will meet (small).** Both suites green, the `Start` hashes, timings with Unity in front. A probe over the 13 models, every algorithm, simplified and not: edge lengths in cells; feature edges, corners and lines at angles from 20° to 60°; patches between feature lines and their sizes; edges of four triangles and what the sheets make of them. The parts of the reference code that finding 11 lists, read properly. The default edge length comes out of this.
1. **The frame (medium).** The settings and inspector, the hook, what is kept, `remesh`, the statistics, the request for shared vertices, the flags through `finish`, the wireframe's mask, the editor's signature, scheduler and `select`. A stand-in technique that turns pairs of triangles into quads where they are flat, so that the flags and the wireframe can be tested before a technique gives quads; it is taken out again at step 5.
2. **Isotropic: skeleton and operations (medium).** The feature skeleton and the angle kind of sharp edge, `split` and `flip` on the simplifier's lists, the positions of collapses, the nearest-point search, with their tests. The simplifier's meshes hashed before and after.
3. **Isotropic: the loop (medium).** The loop, the smoothing, the equalizing of areas. Wired in as a technique. **Measured here:** edge lengths, edges per vertex, angles and vertex areas after each iteration and each area round, which sets the damping factor and the two defaults; how far vertices are from the shapes; time by stage from 32³ to 128³, simplified and not.
4. **Field Aligned: the fields (large).** Sheets, the preparation, graph, normals, areas, hierarchy, colouring, both fields, for quads and triangles. **Measured here:** the energies by level, the irregular points of both fields against the edge length, time by stage from 32³ to 128³.
5. **Field Aligned: the mesh (large).** Extraction, faces, holes, the split into quads, polygons, materials and normals out. Wired in. **Measured here:** everything under Tests' last point, and what the constraints do for lines between materials.
6. **QuadriFlow: offsets and flow (large).** Offsets, the network, the flow solver with its tests, long offsets. **Measured here:** the size of the flow and the time it takes against the edge length; whether the paper's hierarchy of networks is needed.
7. **QuadriFlow: the mesh (large).** Conjugate gradients, the greedy pass, the positions, extraction. Wired in. **Measured here:** turned triangles left after the greedy pass, which is what decision 6 waits for; the rest as for step 5.
8. **Scenes, tuning and documentation (medium).** The scenes, whatever the measurements show to cost most, a README section for each technique with its settings, what it keeps and what it does not, `AGENTS.md`, and this plan marked complete.

Four of the nine are large, all of them the two field techniques. For scale: the simplifier was one large step among eight.

**Changed from the first draft:** Isotropic was three large steps after QuadriFlow, for the earlier paper. Before the new paper was read, the draft's decision 1 put the loop at one medium step. It is two here, because the paper turned out to leave features and staying on the surface to its reader (finding 9).

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

All eight were answered on 2026-11-17. The first: "I've added in the proper isotropic surface remeshing paper. We'll use that one." The rest: "For the other decisions, I'll go with your recommendations."

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
