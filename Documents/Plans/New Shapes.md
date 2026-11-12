# New Shapes

Plan written 2026-11-12 for adding seven shapes from https://iquilezles.org/articles/distfunctions/ to every implementation and to the interactive editor: Cylinder, Cone, Capsule, Torus, Pyramid, Octahedron and Hexagonal Prism.

**Status:** complete. Steps 0 to 5 were done on 2026-11-12, and the user accepted the work as complete the same day ("We can consider this completed"). The user asked for these seven on 2026-11-12, after a review of the page found they fit without new parameters. Step 0 was done the same day, and the user approved the plan as written ("Let's start"), the design choices below included. Steps 1 and 2 followed the same day. The user accepted the cost of step 2: Burst contours of cuboids and ellipsoids are 8 to 15% slower (finding 10). Step 3 followed, and the user accepted its cost too: GPU remeshes of them are 11% slower (finding 11).

## Goal

The seven shapes can be added like Cuboid and Ellipsoid: as volumes in a scene, contoured by every algorithm and every implementation, turned or not, and from buttons in the interactive editor, with outlines drawn through the Shapes library. Every one of them is an exact signed distance. Cuboid and Ellipsoid give the meshes they give today.

## What exists today

- **A shape is its type, origin, rotation and extents.** The extents are half its size along its own axes, read off the transform's scale. There are no other parameters, so a shape has to be made from these three numbers alone, and it fills the box of ±extents in its own frame.
- **Sampling is written four times:**
  - Managed: the static `sample( position, extents )` of `Cuboid` and `Ellipsoid`, called through switches in `DensityFunction.Snapshot` and the interactive editor's `Shape`, and in the test helpers `BurstTestSupport.FixedDensityFunction` and `ContouringTest`;
  - Burst: `BurstDensityFunction.sampleCuboid` and `sampleEllipsoid`, the same arithmetic operation for operation;
  - GPU: `sampleCuboid` and `sampleEllipsoid` in `UniformDualContouring.compute`, chosen by the `type` field of `DensityFunction.Data`, read as an int;
  - interactive editor: its `Shape.sample`.
- **The editor** adds shapes with "Add Cuboid" and "Add Ellipsoid", at half the grid across. It draws each shape's outline through `[DrawGizmo]` with Unity's `Gizmos`, so that a click on the outline selects the shape, which matters for subtracted shapes that are otherwise invisible. Its "outside the grid" warning needs how far a shape reaches along each axis.

### Findings that shape the work

1. **All seven fit three numbers.** Each has an axis, `y`, and a cross-section across `x` and `z`. A round or square cross-section needs `x` and `z` to be equal, and there is no exact distance for an elliptic one. See Design for the rule.
2. **The formulas were checked against distances worked out independently.**
   - **The check:** the convex shapes (pyramid, octahedron, hexagonal prism) were measured face by face, and the shapes of revolution (cylinder, cone) in their 2D profile. Each shape was tested at 3,000 random points around each of six boxes, even and uneven, in double precision, with a Python script in the session scratchpad that is not kept.
   - **Result:** all seven agree to within 3e-16, inside and outside, and lie within their box.
   - **The page's pyramid is not exact,** and is off by up to 0.375:
     - outside, below its base, it does not give the distance to the base;
     - inside, near the base, it measures only to the sloping sides.

     Two small changes make it exact (Design).
   - **The page's exact octahedron is only for the regular one,** with all three extents equal. A general one, exact for any three half-diagonals, is derived in Design and passed the same check.
3. **`sign` differs between the implementations.** GLSL, HLSL and `math.sign` give 0 for 0, and `Mathf.Sign` gives 1. The page uses `sign` in the cone, the pyramid and the hexagonal prism. The formulas here use comparisons instead. Wherever the page multiplies by `sign` of zero, the distance is zero anyway, except on the pyramid's sloping planes past its faces, where the page gives 0 and the true distance is positive.
4. **The type is switched on in nine places:** the three implementations, the snapshot, the editor's `Shape` and `addShape`, the two test helpers and the probe. A shared managed switch, `DensityFunction.sample( type, position, extents )`, replaces the four managed copies, so that a new shape needs one line there.
5. **Some tests already run over every type** through `[Values] DensityFunction.Type`, and pick up the new ones by themselves:
   - `ShapeRotationTest`'s turn and quaternion-length tests hold for any shape.
   - `aQuarterTurnSwapsTwoExtents` holds only for shapes that look the same with two axes swapped, which are Cuboid, Ellipsoid and Octahedron. A cone turned onto its side is not a cone with two extents swapped, so the test is limited to those three.
   - `InteractiveEditorTest.aTurnedShapeIsOutsideTheGridAsFarAsItReaches` builds an Ellipsoid for every type but Cuboid, so it is changed to build the shape it is given.
6. **On the GPU, more cases in the shader's switch may change Cuboid and Ellipsoid in the last bits.** That happened when rotation was added (finding 7 of Shape Rotation): the compiler arranges float operations as it sees fit. The `Start` hashes show whether it happens; if so, the size is measured and the user decides, as last time.
7. **The editor's added size suits five of the shapes.** A cube of half the grid gives a torus with no hole and a capsule that is a ball, so those two are added at sizes of their own.
8. **It is not known whether Shapes' drawing can be clicked.** Unity selects an object from a click on its gizmo when the gizmo is drawn with `Gizmos`. Shapes draws with `Graphics.DrawMeshNow` when it is used inside a gizmo callback, and whether Unity's picking sees that is not known. **Answered in step 4: it does not.** Unity's picking sees `Gizmos` lines even with no colour, so the outlines are drawn both ways.
9. **How close dual contouring puts vertices to the new shapes.** Found in step 1, at 16³ with the SVD solver, from the worst vertices of `ShapeContourTest`. Three things take vertices off a shape; the shapes themselves are exact.
   - **Curved surfaces.** A vertex goes where the tangent planes of its cell's intersections come closest. On a curved surface that is off the surface by up to the sag across the cell's diagonal, `3h²/8R` for cells of size `h` and a radius of curvature `R`.
     - Capsule: up to 0.0099 off, against a sag of 0.0098.
     - Torus: 0.0098, against 0.0146 for its tube.
     - Cylinder: 0.0036, against 0.0059.

     The tests allow the sag plus the 0.002 of the intersections.
   - **Corners where four faces meet,** the octahedron's and the pyramid's tip. A cell next to one can see only two faces that meet nowhere but at the corner. The QEF then has rank two, and the vertex can lie anywhere on the line where the two planes cross through the corner. On the turned octahedron one such vertex was 0.0063 off, 0.008 from the corner.
   - **Near the cone's tip,** the side curves more sharply than a cell can follow, and a cell there may have only a few intersections.
     - In the worst cell, four intersections give the direction toward the tip an eigenvalue of 0.081. Secret Sauce's truncation at 0.1 drops that direction, so the vertex stays 0.016 off. Without the truncation, the vertex would be at the tip, 0.0004 off.
     - This is the solver doing what the paper says. The cone's tip still gets a vertex of its own, within 0.0011.

     Along the rim, some cells show two pieces of surface (finding 6 of Shape Rotation). Uniform and Adaptive Dual Contouring put the mass point there, up to 0.018 off.
   - **Flat faces and edges** are as good as the turned cube's:

     - vertices within 0.001 of the faces;
     - corners within 0.0015;
     - the mesh within 0.0012 of every edge and rim.
10. **Burst samples cuboids and ellipsoids slower once it knows nine shapes.** Found in step 2, on the three balls of `ContouringPerformanceTest`, which are all ellipsoids.
    - **The size of it:** with the seven new cases in `BurstDensityFunction.sample`, contours took 21 to 26% longer, and the turned Adaptive one 58%. Sampling (`SampleCorners`) took 48% longer.
    - **How it was measured:** variants of the file were timed alternately in one session, each checked to be the one compiled (Measurements). The first variant was the file as committed.
    - **What it is not:**
      - not how the cases are chosen: one switch, or tests for the two old types first with the rest in a function of its own, cost the same;
      - not the new formulas themselves: keeping them out of line, or calling them from that one function, cost the same;
      - not `sample` appearing three times in `calculateDensity`: sampling once before the switch cost the same, and changed nothing for the file as committed.
    - **What helps:** forcing `sample` inline (`AggressiveInlining`) takes the cost to 8 to 14% (Measurements). So part of the cost is that Burst stops inlining `sample` once it can be any of nine shapes.
    - **Not found:** what the remaining 8 to 14% is. Burst's generated code would show it, but getting it from the command line means driving the Burst Inspector's internal API.
    - **Decision:** the user accepted the cost on 2026-11-12.
11. **The GPU too takes longer over cuboids and ellipsoids once its shader has the new shapes.** Found in step 3.
    - **The size of it:** a remesh of the three balls takes 11% longer, at 64³ and 128³ alike (Measurements).
    - **How it was measured:** the shader as committed and the new one were timed alternately in one session. A probe checked which one ran: the old shader samples a cylinder as 0, which gives no surface.
    - **Not looked into:** the cause. The likely one is that a kernel takes as many registers as its largest path needs, so with the new shapes fewer threads run at once, whatever shapes are contoured.
    - **Output unchanged:** unlike rotation (finding 7 of Shape Rotation), the GPU's meshes of cuboids and ellipsoids come out bit for bit as before.
    - **Decision:** the user accepted the cost on 2026-11-12.

## Design

### The shapes

Each shape is centred on its origin, with its axis along its own `y`, and fills the box of ±extents as far as its kind allows. `e` is the extents and `r = min( e.x, e.z )` is the radius, or half-width, across.

| Shape | Made from the extents | Formula |
|---|---|---|
| Cylinder | radius `r`, half-height `e.y` | the page's vertical capped cylinder |
| Cone | base of radius `r` at `-e.y`, tip at `+e.y` | the page's cone, given the rim `( r, -2 e.y )` from the tip instead of an angle |
| Capsule | radius `min( r, e.y )`, the rest of `e.y` the straight part | the page's vertical capsule, centred |
| Torus | tube radius `t = min( e.y, r )`, ring radius `r - t` | the page's torus |
| Pyramid | square base of half-side `r` at `-e.y`, tip at `+e.y` | the page's pyramid for a base of side 1, scaled by `2r`, with the two corrections below |
| Octahedron | corners at `±e.x`, `±e.y`, `±e.z` on the axes | derived below |
| Hexagonal Prism | regular hexagon across `x` and `z`, corners on the `x` axis, half-length `e.y` | the page's hexagonal prism with its axes swapped, of inradius `min( e.z, e.x √3/2 )` |

- **Round and square cross-sections take the smaller of `x` and `z`,** so the shape stays inside its box and outline. Where they differ, the interactive editor warns, as it does for a shear. The hexagonal prism is the largest regular one that fits and has no warning, because a box of equal `x` and `z` cannot fit a regular hexagon exactly anyway.
- **The pyramid's two corrections:**
  - **below the base:** the distance is that to the base square, `length( max( abs( p ) - ( 0.5, 0, 0.5 ), 0 ) )`;
  - **inside:** the larger of the page's value and minus the height above the base, because inside a convex shape the nearest face can be the base.
- **The octahedron, for any half-diagonals `a`, `b`, `c`.** In the positive octant, with `n = ( 1/a, 1/b, 1/c )` and `m = dot( abs( p ), n ) - 1`:
  - **Inside** (`m ≤ 0`): the distance to the face's plane, `m / |n|`, which is exact inside any convex shape.
  - **Outside:** the point projected onto the face's plane, `q = p - n m / |n|²`, has the barycentric coordinates `q.x/a`, `q.y/b` and `q.z/c`.
    - If they are all at least 0, the distance is `m / |n|`.
    - Otherwise it is the distance to the edge opposite the first negative one. That edge is enough because the face's triangle always has three acute angles.
  - With equal extents it is the page's octahedron.
- **No `sign`** (finding 3): `x * sign( s )` becomes `s < 0 ? -x : x`.
- **Degenerate sizes** are left as they are for Ellipsoid today. A zero extent can divide by zero, and the editor already warns about a shape without size along an axis.

### Where the code goes

- **Managed:** a component per shape in `Assets/Scripts`, named as in the table (`HexagonalPrism.cs` for the prism), each a `Volume` with a static `sample( position, extents )` like `Cuboid`. `DensityFunction.Type` gains the seven values after `Ellipsoid`, so the existing values and the GPU's numbers stay as they are. `DensityFunction.sample( type, position, extents )` holds the one managed switch (finding 4).
- **Burst:** `sampleCylinder` to `sampleHexagonalPrism` in `BurstDensityFunction`, operation for operation as in the managed code, with `BurstSurfaceExtractor`'s Mathf-style `min`, `max` and `magnitude`.
- **GPU:** the same seven functions and seven `DENSITY_FUNCTION_TYPE_` constants in `UniformDualContouring.compute`. `DensityFunction.Data` does not change.

### Interactive editor

- **Buttons:** a row of three per line, under "Add", one for each of the nine types, named as the types read ("Hexagonal Prism"). A new shape's object is named after its type, as today. As built, the buttons are a `GUILayout.SelectionGrid`, so that all nine are as wide, and the shape list names types the same way.
- **Sizes added at:** half the grid across, as today, except the capsule (a quarter of the grid across and half of it high) and the torus (half of the grid across, a fifth of it high).
- **Outlines** are drawn with the Shapes library, for all nine types, in the colours and opacities of today:
  - Cuboid and pyramid: their edges.
  - Ellipsoid: its three principal ellipses.
  - Cylinder, cone and capsule: the rims of their caps or base, and lines along their sides; the capsule's caps as arcs.
  - Torus: its inner, outer, top and bottom circles, and, added in step 4, four cuts through its tube, so that it reads as a torus from any side.
  - Octahedron and hexagonal prism: their edges.

  The lines are worked out in the shape's frame by one function, which the tests check, and drawn in pixels, like the debug view.
- **Clicking an outline still selects its shape** (finding 8). Step 4 first checks whether Unity picks Shapes' drawing inside `[DrawGizmo]`. If it does not, the same lines also go to `Gizmos.DrawLineList`, invisible, only so that a click finds them.
- **Warnings:** a Cylinder, Cone, Capsule, Torus or Pyramid with different sizes along `x` and `z` is named, with the size it is contoured at, "as its outline shows".
- **Outside the grid:** the new shapes reach as far as the box around them, turned. That can only miss a warning, never give a wrong one. Done in step 1 already: before it, every type but Cuboid was taken to reach as far as an ellipsoid, which a cylinder's rim goes past.

## Tests

- **Distances (managed),** in a new `ShapeDistanceTest`:
  - Each shape's `sample` against the independent distances of finding 2, written in C# in the test: face by face for the convex shapes, in the profile for the cylinder and cone, and from the definition for the capsule and torus. This covers several boxes, even and uneven, at points inside, outside and near the surface, to within float rounding.
  - The shape lies inside its box, and touches it where the table says: the cone's tip, the octahedron's six corners, the rims of the cylinder.
  - The rule for unequal `x` and `z`.
  - The page's pyramid fails the distance test; the corrected one passes.
- **Contours,** in `ShapeContourTest`, an abstract fixture with a Managed and a Burst subclass like `RotatedCubeTest`. Each of the seven shapes, upright and turned, is contoured by all three algorithms at 16³:
  - **Closed:** the surface is closed, and manifold for Manifold Dual Contouring. For Uniform and Adaptive Dual Contouring, the cells of finding 6 of Shape Rotation are left out, as for the turned cube.
  - **On the shape:** every vertex lies within a tolerance of the shape.
    - For flat faces, the tolerance is the turned cube's 0.002.
    - For curved surfaces, it is 0.002 plus the sag `3h²/8R` at the vertex's own radius of curvature (finding 9). On the cone, that radius shrinks toward the tip.
    - Vertices within two cells of a corner where four faces meet are left out, and so are the cells that show two pieces of surface for Uniform and Adaptive Dual Contouring. The corner test holds those corners.
  - **Sharp features:** a vertex at every corner of the pyramid, octahedron and hexagonal prism, and at the cone's tip. The mesh runs along the edges of those three and along the rims of the cylinder and cone.
  - **Normals:** `HermiteNormalTest` holds the normals of the pyramid and the octahedron to the normals of their faces, away from edges, for every implementation.
- **Parity:**
  - **Burst against Managed:** the two random-shape parity tests of `BurstSurfaceExtractorTest` run once more, over all nine types, from a generator of their own. The existing runs keep exactly the shapes they have.
  - **Whole meshes:** a model with every new shape, turned, in `TopologyTest`, for all three algorithms in Managed and Burst.
  - **GPU against CPU:** vertices on that model, in `UniformDualContouringGPUTest`.
- **No change for the old shapes:** the 416 `Start` hashes match the baseline from step 0. For the GPU, see finding 6.
- **Editor,** in `InteractiveEditorTest` and `InteractiveEditorAuthoringTest`:
  - every type can be added, with its component, name and size;
  - at scale 1, each type contours as its volume does, for all seven variants;
  - the outline function's lines lie on each shape's surface or edges;
  - the `x` and `z` warning, both ways, and none for the hexagonal prism;
  - the grid warning for each type.
- **Performance:** `ContouringPerformanceTest` on the three balls stays within the noise in Burst, against step 0. Sampling each new shape is timed against the Ellipsoid and recorded.

## Steps

Each step compiles, passes every test, and is reviewed before the next one starts.

0. **Baseline (small).** Both suites green, the 416 `Start` hashes against the end of Shape Rotation, and the Burst runs of `ContouringPerformanceTest` recorded with Unity in front. **Done 2026-11-12:**
   - Edit mode 878 of 878, play mode 21 of 21.
   - The 416 `Start` hashes are identical to those at the end of Shape Rotation, the GPU lines included. They are the baseline for every later step.
   - The Burst runs are under Measurements. Unity was in front for every case.
1. **Managed (medium).** The enum, the seven components, `DensityFunction.sample` and its four callers, the two tests of finding 5. `ShapeDistanceTest`, and `ShapeContourTest` for Managed. The `Start` hashes unchanged. **Done 2026-11-12:**
   - **The shapes:** `Cylinder`, `Cone`, `Capsule`, `Torus`, `Pyramid`, `Octahedron` and `HexagonalPrism` in `Assets/Scripts`, each a `Volume` with a static `sample( position, extents )`, written from the formulas of Design. `DensityFunction.Type` has the seven after `Ellipsoid`.
   - **One switch each for sampling and for components:**
     - `DensityFunction.sample( type, position, extents )` is the managed switch. The snapshot, the editor's `Shape` and the test helper `FixedDensityFunction` sample through it.
     - `Volume.componentOf( type )` gives the component of a type. The editor's `addShape` and `ContouringTest` add shapes through it.
   - **Outside the grid:** in step 1 rather than step 4, as Design says.
   - **Burst and the GPU** do not know the new types yet. Burst gives NaN for them and the shader 0, as for any unknown type, and no test gives them one.
   - **New tests:**
     - **`ShapeDistanceTest` (28).** Each shape's density is within 5e-7 of the distance worked out independently, in double precision; the largest difference is 1.2e-7. It also covers:
       - every shape lies inside its box and touches it where Design says;
       - round and square shapes take the smaller of `x` and `z`;
       - the hexagonal prism is the largest regular one that fits;
       - the article's pyramid is off below and inside near its base, where this one is exact.
     - **`ManagedShapeContourTest` (180).** For each shape, upright and turned, and each algorithm:
       - the surface is closed, simplified and not. For Manifold Dual Contouring it is one manifold surface, with the torus' one handle.
       - every vertex is within its tolerance (Tests, finding 9). The worst share of its tolerance is 0.84, the capsule's.
       - every corner has a vertex within 0.002;
       - the mesh runs within 0.002 of every edge, and of the rims within 0.002 plus their sag across a cell.
     - **`HermiteNormalTest`:** the turned pyramid's and octahedron's normals are those of their faces, for Managed. Burst and the GPU join in steps 2 and 3.
     - **Existing tests:** the 29 of `ShapeRotationTest` and the 7 of `aTurnedShapeIsOutsideTheGridAsFarAsItReaches` that now run over the new types pass. The quarter-turn test is limited to Cuboid, Ellipsoid and Octahedron (finding 5).
   - **Shared test helpers:** `closestPointOnTriangle`, `distanceToMesh` and `showsSeveralPieces` moved from `RotatedCubeTest` to `TopologyTest`. `ShapeDistanceTest.polyhedronOf` gives the corners and faces of the three flat-faced shapes to the distance, contour and normals tests.
   - **The tests catch mistakes.** The pyramid without its inside correction, the octahedron as only the distance to its face's plane, and the cone taking the larger of `x` and `z` were put in together. They failed:
     - 6 distance tests;
     - 9 contour tests;
     - the pyramid's normals.
   - **Verification:**
     - Edit mode 1124 of 1124, play mode 21 of 21.
     - The 416 `Start` hashes are identical to step 0.
     - Compiling gave no errors or warnings.
2. **Burst (small).** `BurstDensityFunction`. The Burst contour fixture, the parity tests, the model in `TopologyTest`, performance. **Done 2026-11-12,** apart from the decision on its cost:
   - **`BurstDensityFunction`:** `sampleCylinder` to `sampleHexagonalPrism`, operation for operation as the managed formulas, and the seven cases of `sample`. `BurstSurfaceExtractor` gained a `clamp` that works as `Mathf.Clamp` does. `sample` is forced inline (finding 10).
   - **New tests:**
     - **`BurstShapeContourTest` (180):** the contour tests of step 1 for Burst, all within the same tolerances.
     - **Random parity:** `densitiesMaterialsAndNormalsOfEveryTypeAreTheManagedOnes` and `intersectionsBoundariesAndCorrectionsOfEveryTypeAreTheManagedOnes`, upright and turned. They use `BurstTestSupport.randomShapesOfEveryType`, from seeds of their own; the existing runs draw exactly the shapes they did. The tolerances already in use hold.
     - **Whole meshes:** `TopologyTest.newShapes` holds the seven shapes, turned, with a second material and a subtraction. `contourIsTheManagedOneOnTheNewShapes` contours it with Managed and Burst for all three algorithms, every solver and every amount of simplification or clustering (16 cases). They agree within `BurstParity`'s tolerances.
     - **Normals:** `HermiteNormalTest` for the pyramid and the octahedron in Burst.
   - **The tests catch mistakes.** The cone taking the larger of `x` and `z` and the hexagonal prism's fold the wrong way round were put into Burst only. They failed:
     - the four random parity runs of every type;
     - all 16 whole-mesh runs;
     - 30 Burst contour tests.

     The existing parity tests, of cuboids and ellipsoids, passed, as they should.
   - **Performance:** Burst contours of cuboids and ellipsoids are 8 to 15% slower than in step 0 (finding 10, Measurements). The plan's aim was to stay within the noise. The user accepted the cost on 2026-11-12, rather than looking further into Burst's code or giving models of cuboids and ellipsoids a path of their own.
   - **Verification:**
     - Edit mode 1326 of 1326, play mode 21 of 21.
     - The 416 `Start` hashes are identical to step 0, so forcing `sample` inline changed no output.
3. **GPU (small).** The shader. GPU tests, `HermiteNormalTest` for the GPU, the GPU hashes (finding 6). **Done 2026-11-12:**
   - **The shader:** `sampleCylinder` to `sampleHexagonalPrism` in `UniformDualContouring.compute`, written from the C# ones, with `DENSITY_FUNCTION_TYPE_CYLINDER` to `DENSITY_FUNCTION_TYPE_HEXAGONAL_PRISM` (2 to 8). Two names had to change from the C# code: `half` is a type in HLSL and `distance` a function.
   - **Shader warnings:** the first version had two, "use of potentially uninitialized variable", in the pyramid and the octahedron. The code after their early `return`s is in `else` branches now, as in commit 1e87ed3, and the shader compiles without messages.
   - **New tests:**
     - `verticesMatchCPUOnTheNewShapes`: GPU vertices on `TopologyTest.newShapes` are within 0.05 cells of the CPU's;
     - `surfaceOfTheNewShapesIsClosed`, for all three Uniform implementations;
     - `HermiteNormalTest`'s pyramid and octahedron for the GPU.
   - **The tests catch mistakes.** The torus' ring the wrong way round and the pyramid without its inside correction were put into the shader only. Three tests failed:
     - the vertex test, with 239 of 510 vertices off;
     - the pyramid's normals on the GPU;
     - the GPU's closed surface.
   - **Finding 6 did not happen:** the 32 GPU lines of the `Start` hashes are identical to step 0, and so are the rest.
   - **Speed:** the GPU's remeshes of cuboids and ellipsoids take 11% longer (finding 11). The user accepted it on 2026-11-12, rather than compiling the shader with and without the new shapes.
   - **Verification:** edit mode 1332 of 1332, play mode 21 of 21, the 416 `Start` hashes identical to step 0.
4. **Interactive editor (medium).** The picking check first, then the outline function and Shapes drawing, buttons, sizes, warnings and their tests. A look at the outlines in the scene view. **Done 2026-11-12:**
   - **The picking check (finding 8):**
     - **How:** a temporary scene, never saved, held an editor with one subtracted cylinder, far from the open scene. `HandleUtility.PickGameObject` picked at three points from the scene view:
       - on the cylinder's outline;
       - on the grid box, which `Gizmos` draws;
       - at nothing.
     - **Result:** the grid box gave its editor and nothing gave nothing, but the outline drawn with Shapes was not picked.
     - **Fallback:** with the outline's lines also given to `Gizmos.DrawLineList` in a colour of no opacity, the outline picked its cylinder.
     - The scene was closed again and the scene view put back.
   - **Outlines:** `InteractiveEditorGizmos.outlineOf( type, extents )` gives the lines of each of the nine outlines in the shape's frame, made from its extents as the shape is. `[DrawGizmo]` methods for the nine components draw them with Shapes, 2 pixels wide, in the colours of before, and with `Gizmos` without colour for clicking.
   - **The editor:**
     - **Buttons:** nine Add buttons in a grid of three by three.
     - **Sizes:** `InteractiveEditor.addedScaleOf( type )` adds the capsule at (0.25, 0.5, 0.25) and the torus at (0.5, 0.2, 0.5) of the grid, every other shape at half the grid.
     - **Names:** `InteractiveEditor.nameOf( type )` names types with spaces, for the buttons, the shape list, the undo step and the warning. It is written out rather than using `ObjectNames`, because the warnings are not editor-only code.
     - **The new warning:** "{name} is not as wide along x as along z. A cylinder is round across, so it is contoured as wide as the narrower of the two, as its outline shows. Scale it the same along x and z." It is for the cylinder, cone, capsule and torus, and the pyramid is "square across". Widths within `Mathf.Approximately` of each other count as the same.
   - **New and changed tests:**
     - `everyLineOfAnOutlineEndsOnTheShape`, for all nine types in four boxes: every line ends on the shape, within its box.
     - `theOutlineOfAShapeWithFlatFacesIsItsEdges`: as many lines as edges, each on the shape from end to end.
     - `atScaleOneTheContourOfTheNewShapesIsThatOfTheVolumes`: for all seven variants, the editor contours the seven new shapes, turned, as their volumes do.
     - `aRoundOrSquareShapeOfTwoWidthsIsWarnedAbout`, both ways, and none once the widths agree; `otherShapesOfTwoWidthsAreFine` for the cuboid, ellipsoid, octahedron and hexagonal prism.
     - `addShapeAddsAUnionOfMaterialZeroAtTheCentre` for every type, with its component and size. `aShapeIsAddedAtASizeThatShowsWhatItIs`: the capsule has a straight part, the torus a hole, and none of the nine is warned about but for materials.
     - `aTurnedShapeIsOutsideTheGridAsFarAsItReaches` now turns a bar that is long along its own y. Before, it was long along x, which the new warning rightly named for the round shapes.
   - **The tests catch mistakes.** These three were put in together and failed four tests:
     - the torus' top and bottom circles at its outer radius failed the outline test;
     - the torus left out of the warning failed both of its warning tests;
     - the capsule added as a cube failed its size test.
   - **Looked at:** the nine outlines from two heights, and the inspector, in a temporary scene that was closed again (the inspector of the user's scene was too long to show the buttons). Outlines and buttons look right.
     - From a height level with the shapes' tops, a top shows as a line, as it should.
     - The first build of the buttons gave them the widths of their names, which the grid fixed.
   - **Verification:** edit mode 1382 of 1382, play mode 21 of 21, the 416 `Start` hashes identical to step 0, no scene changed.
   - **After review, 2026-11-12:**
     - **Outline thickness:** the user set the outlines to 1 pixel, then asked for the thickness to be an option. `InteractiveEditor.outlineThickness`, shown as "Outline Thickness" under Debounce, is in pixels, at least 0.5, and 1 unless set. It is not in the signature, so changing it does not contour (`theSignatureStaysTheSameWith`).
     - **Overlaps:** the user noticed the half opaque outlines are brighter wherever two of their lines overlap: at every joint along a circle, where edges meet, and where two outlines cross. Shapes blends each line by itself.
       - **The options:**
         - opaque outlines, dimmed by colour rather than opacity;
         - each circle and arc as one Shapes polyline, which would leave corners and crossings;
         - the stencil buffer, to blend each outline once per pixel.
       - **The user chose the first.** `InteractiveEditorGizmos.colourOf( combination, selected )` gives opaque colours, and dims a shape that is not selected half way to the grey of the scene view's ground (0.35). That is about how it looked half opaque over the ground.
       - **Checked:** a capture of the user's scene view shows no brighter spots any more, along lines or where outlines cross. `outlinesAreOpaqueAndDimmerUnlessSelected` (3) holds the colours to that.
       - **Not changed:** where a subtracted shape's outline lies exactly on the surface it carves, the line flickers against the mesh, which is at the same depth.
     - **On top:** the user asked for the outlines to be drawn over the scene's geometry instead of being cut by it. They are drawn with `ZTest Always` now, which also ends the flicker above. A capture of the user's scene view shows every outline over the mesh.
     - **Clicking an outline over the mesh:** a click there picks the contour, not the shape. Unity picks by depth, and the contour is in front of the outline's `Gizmos` lines.
       - **`HandleUtility.pickGameObjectCustomPasses` does not help.**
         - **Tried:** in a temporary scene, a pass found the cylinder's outline under the click. Unity gives the click in the camera's pixels from its bottom left.
         - **Result:** Unity still returned the contour in front, so a pass cannot overrule a renderer nearer the camera.
         - The pass was taken out again.
       - **What would:** a handler of the scene view's clicks that registers the outlines as controls (`HandleUtility.AddControl` on Layout, select on MouseDown). It is left for the user to decide.
       - Where nothing is in front of an outline, a click picks its shape as before.
     - **At their depth, without the contour cutting them:** the user then asked for outlines at their proper depth without lines broken by the contour, and drawing on top as a setting.
       - **The cause of the breaks:** the contour is only near the shapes, a fraction of a cell inside or outside them (finding 9), so it cuts through outlines wherever they lie on it.
       - **The fix:** every point of an outline is drawn half a cell of the selected voxelizer's grid nearer the camera, along its line of sight. That leaves it where it was on the screen. `InteractiveEditorGizmos.nearerTheCamera` does it, and `InteractiveEditor.cellSize` gives the cell, scaled by the editor's largest scale.
       - **Clicking:** the `Gizmos` lines for clicking are moved too, so a click on an outline that lies on the contour picks the shape.
       - **The setting:** "Outlines On Top" (`InteractiveEditor.outlinesOnTop`), off unless set, draws outlines over everything instead, without the move. It is not in the signature.
       - **New tests:**
         - `anOutlineDrawnNearerTheCameraStaysWhereItIsOnTheScreen` checks with a Unity camera of each kind that a moved point is as far as asked from where it was, on the same pixel, and nearer;
         - `theCellSizeIsThatOfTheSelectedGrid` for each algorithm, which also checks that the setting starts off;
         - "outlines on top" among the changes the signature ignores.
       - **Checked:** a capture of the user's scene view shows the outlines on the capsule and the ball whole, and hidden where the mesh is in front of them.
5. **Documentation (small).** README: the shapes and how each is made from its box; this plan marked complete. **Done 2026-11-12:**
   - **A "Shapes" section** with a table of the nine volumes and how each is made from its box. It also says that round and square shapes take the smaller of x and z, that the new shapes are exact distances from Inigo Quilez's article with the pyramid corrected and the octahedron made exact for any box, and what the new shapes cost Burst and the GPU over cuboids and ellipsoids.
   - **The Burst figure** at 128³ is 28 to 33 ms now.
   - **The interactive editor section:** the Add buttons, the warning about x and z, and the outlines' colours, thickness, depth and "Outlines On Top" setting, with the click that setting cannot make.
   - **The code layout** names the volume components and `DensityFunction.sample`.

## Measurements

### Before any change (step 0, 2026-11-12)

`ContouringPerformanceTest` on the three balls, Burst, with Unity in front. Each figure is the median of three contours, in ms, from two rounds in one session.

| Algorithm | Cells | Model | Voxelize | SampleCorners | SolveVertices |
|---|---|---|---|---|---|
| Uniform | 128³ | upright | 24.3, 23.9 | 5.8, 5.8 | 11.8, 11.7 |
| Uniform | 256³ | upright | 151.7, 150.2 | 46.8, 46.5 | 55.6, 55.7 |
| Adaptive | 128³ | upright | 28.8, 28.2 | 5.8, 5.8 | 13.4, 13.2 |
| Manifold | 128³ | upright | 29.4, 29.3 | 5.8, 5.8 | 12.8, 12.6 |
| Uniform | 128³ | turned | 26.6, 26.8 | 7.3, 7.3 | 12.8, 12.8 |
| Adaptive | 128³ | turned | 30.8, 30.1 | 7.2, 7.2 | 14.4, 14.2 |
| Manifold | 128³ | turned | 31.8, 32.0 | 7.2, 7.3 | 13.7, 13.7 |

The rounds agree within 2%. Sessions differed by up to 13% in Shape Rotation, so step 2 times the old and new sampling alternately in one session if these figures move by more than the noise.

### Burst with the new shapes (step 2, 2026-11-12)

Variants of `BurstDensityFunction.cs`, and in the last run of `BurstSurfaceExtractor.cs`, were timed alternately, two rounds each, in four sessions. A probe checked which variant was compiled before every round. Every session had the file as committed, `head`, and it gave the same times in all four and as in step 0. Voxelize in ms, the two rounds:

| Variant | Uniform 128³ | Uniform 256³ | Adaptive 128³ | Manifold 128³ | Adaptive 128³ turned |
|---|---|---|---|---|---|
| `head`: as committed | 24.0, 24.3 | 151.8, 150.8 | 28.5, 28.2 | 29.2, 29.1 | 31.0, 31.4 |
| one switch over nine types | 30.4, 30.5 | 188.0, 186.6 | 34.1, 35.0 | 35.5, 35.2 | 48.8, 49.5 |
| the same, new formulas not inlined | 30.1, 30.3 | 186.3, 187.0 | 34.5, 34.4 | 35.6, 35.7 | 48.6, 48.6 |
| old types first, the rest in a function not inlined | 30.8, 30.7 | 187.9, 189.3 | 35.3, 35.0 | 36.1, 36.3 | 49.1, 49.3 |
| the same, Burst choosing whether to inline it | 31.3, 30.7 | 190.4, 191.3 | 35.1, 37.4 | 36.4, 37.1 | 49.6, 49.5 |
| old types first, `sample` forced inline | 27.9, 27.6 | 171.8, 173.2 | 31.9, 32.0 | 32.8, 33.1 | 33.9, 33.6 |
| one switch, `sample` forced inline | 27.7, 27.4 | 171.8, 173.1 | 32.1, 31.0 | 32.9, 32.7 | 34.1, 33.1 |
| one switch, sampling once in `calculateDensity` | 30.1, 30.4 | 185.4, 187.5 | 33.9, 34.1 | 35.4, 35.5 | 47.7, 48.5 |
| the same, `sample` forced inline | 27.6, 27.1 | 171.6, 172.3 | 31.7, 31.7 | 32.5, 32.4 | 33.5, 33.8 |
| `head`, sampling once in `calculateDensity` | 24.4, 24.2 | 150.9, 150.9 | 27.8, 27.8 | 29.7, 29.0 | 30.6, 30.4 |

SampleCorners at Uniform 128³: 5.8 ms for `head`, 8.5 to 8.9 with the switch, 7.7 to 7.9 forced inline. The variant kept is one switch with `sample` forced inline: 8 to 14% slower than `head`, against 21 to 58%.

**`ContouringPerformanceTest` with the variant kept,** in a later session, against step 0. Voxelize in ms:

| Algorithm | Cells | Model | Step 0 | Step 2 | Change |
|---|---|---|---|---|---|
| Uniform | 128³ | upright | 24.3, 23.9 | 27.2, 28.0 | +14% |
| Uniform | 256³ | upright | 151.7, 150.2 | 173.5, 174.5 | +15% |
| Adaptive | 128³ | upright | 28.8, 28.2 | 32.3, 32.4 | +14% |
| Manifold | 128³ | upright | 29.4, 29.3 | 32.4, 32.6 | +11% |
| Uniform | 128³ | turned | 26.6, 26.8 | 29.7, 29.5 | +11% |
| Adaptive | 128³ | turned | 30.8, 30.1 | 33.9, 34.1 | +12% |
| Manifold | 128³ | turned | 31.8, 32.0 | 34.7, 34.6 | +9% |

**What each shape costs to sample,** with the variant kept. One shape of each type was contoured at Uniform 128³, turned by (30°, -60°, 75°) and half the grid across; the torus is a fifth of the grid high. Each figure is the median of 7 contours, in ms, and the two rounds agreed within 0.2 ms. `SampleCorners` samples every corner of the grid once, so it shows the cost of a sample. `SolveVertices` also depends on how much surface there is.

| Shape | Vertices | SampleCorners | SolveVertices | Voxelize |
|---|---|---|---|---|
| Cuboid | 39,490 | 4.25 | 8.05 | 17.5 |
| Ellipsoid | 19,310 | 4.64 | 5.31 | 13.0 |
| Cylinder | 29,844 | 4.64 | 7.07 | 15.9 |
| Cone | 15,615 | 5.49 | 5.08 | 13.3 |
| Capsule | 19,310 | 4.27 | 5.10 | 12.5 |
| Torus | 19,538 | 4.54 | 5.35 | 13.1 |
| Pyramid | 19,692 | 5.73 | 5.38 | 14.2 |
| Octahedron | 10,126 | 5.11 | 3.58 | 10.8 |
| Hexagonal prism | 27,442 | 5.47 | 7.27 | 16.7 |

The capsule, torus and cylinder sample as fast as the ellipsoid. The octahedron costs 10% more, the cone and the prism 18%, the pyramid 23%.

Before `sample` was forced inline, the same probe gave 6.2 to 9.8 ms for `SampleCorners`, 7.15 for the ellipsoid.

### GPU with the new shapes (step 3, 2026-11-12)

The remesh probe of Shape Rotation times a whole Uniform GPU remesh of the three balls, mean of 5. The shader as committed and the new one were timed alternately, twice each, in one session, with Unity in front:

| Cells | Shader as committed (ms) | New shader (ms) |
|---|---|---|
| 64³ | 25.2, 26.1 | 28.9, 28.3 |
| 128³ | 118.3, 115.0 | 129.0, 129.4 |

## Out of scope

- **The page's other shapes,** those that need numbers beyond the extents (rounded boxes, the box frame, the capped torus, the capped and round cones and others) and those that do not fit at all. The review of 2026-11-12 lists them.
- **Example scenes** for the new shapes.
- **Smooth blends and other operators from the page.**
