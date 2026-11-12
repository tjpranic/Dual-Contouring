# New Shapes

Plan written 2026-11-12 for adding seven shapes from https://iquilezles.org/articles/distfunctions/ to every implementation and to the interactive editor: Cylinder, Cone, Capsule, Torus, Pyramid, Octahedron and Hexagonal Prism.

**Status:** step 1 done 2026-11-12, waiting for review. The user asked for these seven on 2026-11-12, after a review of the page found they fit without new parameters. Step 0 was done the same day, and the user approved the plan as written ("Let's start"), the design choices below included.

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
8. **It is not known whether Shapes' drawing can be clicked.** Unity selects an object from a click on its gizmo when the gizmo is drawn with `Gizmos`. Shapes draws with `Graphics.DrawMeshNow` when it is used inside a gizmo callback, and whether Unity's picking sees that is not known.
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

- **Buttons:** a row of three per line, under "Add", one for each of the nine types, named as the types read ("Hexagonal Prism"). A new shape's object is named after its type, as today.
- **Sizes added at:** half the grid across, as today, except the capsule (a quarter of the grid across and half of it high) and the torus (half of the grid across, a fifth of it high).
- **Outlines** are drawn with the Shapes library, for all nine types, in the colours and opacities of today:
  - Cuboid and pyramid: their edges.
  - Ellipsoid: its three principal ellipses.
  - Cylinder, cone and capsule: the rims of their caps or base, and lines along their sides; the capsule's caps as arcs.
  - Torus: its inner, outer, top and bottom circles.
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
2. **Burst (small).** `BurstDensityFunction`. The Burst contour fixture, the parity tests, the model in `TopologyTest`, performance.
3. **GPU (small).** The shader. GPU tests, `HermiteNormalTest` for the GPU, the GPU hashes (finding 6).
4. **Interactive editor (medium).** The picking check first, then the outline function and Shapes drawing, buttons, sizes, warnings and their tests. A look at the outlines in the scene view.
5. **Documentation (small).** README: the shapes and how each is made from its box; this plan marked complete.

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

## Out of scope

- **The page's other shapes,** those that need numbers beyond the extents (rounded boxes, the box frame, the capped torus, the capped and round cones and others) and those that do not fit at all. The review of 2026-11-12 lists them.
- **Example scenes** for the new shapes.
- **Smooth blends and other operators from the page.**
