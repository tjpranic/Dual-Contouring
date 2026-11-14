# Creased Normals

Plan written 2026-11-14 for the TODO item "Edges on shared vertex meshes look horrendous": vertex normals that are smooth across curved surfaces and break at sharp edges, for every algorithm and implementation, on contoured and simplified meshes alike.

**Status:** steps 0 to 2 done on 2026-11-14. Step 3 waits for review. The investigation behind the plan was done on 2026-11-14 with a probe outside the repo. It is summarized under Findings, and its pictures are described there.

The user decided three things on 2026-11-14, in their words:
1. "Yes, replace the Shared setting". The new mode takes the place of Shared Vertex Normals.
2. "Both, selectable with an option". A vertex that is not split can keep the contourer's normal or take one made from its faces.
3. "Kernel if possible". The GPU implementations crease on the GPU.

The four smaller decisions at the end were accepted as recommended on 2026-11-14 ("Everything sounds good. Let's begin."), and that was the go-ahead for step 0.

**Technique:** splitting normals by angle. Unity's model importer does the same with its Smoothing Angle. A vertex's triangles go into groups by their face normals, and each group becomes a vertex of its own. There is no paper. The grouping rule is the probe's (finding 6).

## Goal

The Vertex Normals setting offers **Creased** in place of Shared, and Split as before. Creased is the default.
- **Creased:** around each vertex, triangles whose normals are within the **Crease Angle** of each other share one vertex. Triangles across a sharper edge get a vertex of their own.
- **Smooth Normals** decides the normal of a vertex that is not split. **Contour** keeps the normal the contourer or the simplifier gave it, unless that normal is unusable (finding 10). **Faces** uses the angle-weighted normal of its triangles. Split vertices always take the normal of their group's triangles.
- **Where it works:** every algorithm in every implementation, after Mesh Simplification, in the interactive editor, and on the GPU's direct draw.
- **Today's Shared output is still reachable.** A Crease Angle of 180° with Smooth Normals on Contour gives the mesh Shared gives today, bit for bit.

## What exists today

- **The setting:** `SurfaceExtractor.VertexNormals { Shared, Split }`, serialized as 0 and 1. The voxelizer's default is Shared.
  - The CPU implementations ignore it. They always give shared vertices, and `Voxelizer.finish` calls `splitVertices` for Split.
  - The GPU implementations get it in their constant buffer. Uniform's `contour` kernel and Occupancy's `contourSplit` kernel write six vertices per quad for Split.
- **The shared normal is an average of the cell's Hermite normals.** It is normalized from the edges' normals in Uniform (`BurstGrid.cs` around line 397 and the managed code), from the node normals in Adaptive and Manifold, and from the plane normals in Occupancy.
- **Mesh Simplification** asks the implementation for shared vertices whatever the setting (`Voxelizer.contourNormals`). The simplifier carries an area-weighted normal through every collapse (`BurstMeshSimplifier.cs:808`), and `finish` splits afterwards if Split is chosen. `simplify` builds a new detail from the record and goes through `finish` too.
- **GPU:**
  - Uniform and Occupancy share one layout: `vertices` and `normals` buffers, a `Quad` of six indices and a sub-mesh, and `counts[0]` and `counts[1]` for vertices and quads.
  - `RenderContourDiffuse` and the wireframe's quads variant draw from these buffers, and `SurfaceExtractor.readMesh` reads them back.
  - A GPU contour's vertex order differs from run to run.
- **The interactive editor** carries `vertexNormals` over in `select` and puts it in the contour signature.
- **Scenes:** the ten Sphere scenes use Shared, Mesh Simplification's Uniform Sphere among them. The other 66 use Split, as do the interactive editor scene's four voxelizers.
- **Tests:** six files name `VertexNormals.Shared` or `.Split`. `VoxelizerContourTest` holds that a simplified contour asks for Shared. The 480 `Start` hashes are all taken with Shared.
- **The README** does not describe the Vertex Normals setting.

### Findings that shape the work

From the investigation of 2026-11-14. The probe is `ShadingProbe.cs` in the probes folder outside the repo. It contours on hidden objects, applies prototype passes, and renders with a `PreviewRenderUtility`, so the open scene was never touched.

1. **Why Shared looks bad.** A vertex on a sharp edge gets a normal halfway between the faces. Every triangle touching it blends towards that normal, which makes a band one cell wide along the edge. Where an edge crosses the grid at an angle, the mix of Hermite normals from each face changes from cell to cell, and the band becomes a sawtooth.

   The table shows the share of triangles with a corner normal more than 40° off their own face, under Shared:

   | Model | Triangles >40° off |
   |---|---|
   | Turned cube, 32³ | 12.8% |
   | NewShapes, 64³ | 11.0% |
   | NewShapes, 16³ | 40.6% |
   | CubesAndSphere, 32³, Uniform, Adaptive and Manifold | 16.5% |
   | CubesAndSphere, 32³, Occupancy | 15.7% |
   | Sphere, 16³ | 0% |
2. **Big triangles make it much worse.** Simplified meshes and Manifold's octree clustering spread the blend over whole faces, and hexagonal prisms and boxes look like pebbles. Under Shared, simplified NewShapes at 64³ has 26.3% of triangles more than 40° off, and Manifold at 2e-4 on CubesAndSphere at 64³ has 17.8%.
3. **Occupancy gives zero normals** to vertices on a surface that lies in a grid plane, and they render black. The cut-away sphere at 32³ has 42 of them.
4. **Split facets every curved surface,** at three vertices per triangle.
5. **The prototype fixes all of it.** Both groupings (finding 6) were run at 40°. "Over 40°" is the share of triangles with a corner normal more than 40° off their own face.

   | Model | Shared over 40° | Edge flood fill: over 40°, vertices | Per-vertex grouping: over 40°, vertices |
   |---|---|---|---|
   | Turned cube, 32³ | 12.8% | 0.0%, +16.0% | 0.0%, +15.7% |
   | NewShapes, 64³ | 11.0% | 0.1%, +18.9% | 0.0%, +17.7% |
   | NewShapes simplified, 64³ | 26.3% | 0.3%, +56.7% | 0.0%, +51.4% |
   | NewShapes, 16³ | 40.6% | 1.0%, +105.6% | 0.0%, +105.0% |
   | CubesAndSphere, 32³, Adaptive and Manifold | 16.5% | 0.2%, +17.3% | 0.0%, +17.5% |
   | CubesAndSphere, 32³, Occupancy | 15.7% | 0.0%, +15.0% | 0.0%, +15.0% |
   | Manifold at 2e-4, 64³ | 17.8% | 0.0%, +36.4% | 0.0%, +34.5% |
   | Cut-away sphere, Occupancy, 32³ | 5.9% | — | 0.0%, +6.7%, no zero normals |

   - **For comparison,** Split adds 200% vertices.
   - **In the renders,** both passes keep edges as crisp as Split and curved surfaces as smooth as Shared. The cylinder's rim, the hexagonal prism, and the boxes cut by a ball all show it.
6. **How the triangles are grouped matters.**
   - **The classic rule** joins two triangles across an edge when the angle between them is under the threshold, and floods outwards. It can chain: two triangles straddling an edge, each within the angle of its neighbour, join the two faces into one group. That is where its leftover 0.1 to 1% comes from.
   - **The probe's rule** works around each vertex. Its triangles are taken largest corner angle first, and each joins the group whose mean normal is nearest, if that is within the angle; otherwise it starts a group. Comparing with the mean and not with a neighbour stops the chaining. This rule is the plan's.
7. **Unity's normalize cuts off at 1e-5.** `Vector3.normalized` returns zero below a length of 1e-5, and so does `BurstSurfaceExtractor.normalize`, which matches it on purpose. A triangle's cross product is twice its area, so at 32³ any triangle under about 0.005 cells² gets a zero normal. The turned cube has 83 triangles under 0.01 cells². Those zero normals broke the probe's first grouping. **The pass needs its own normalize,** and its own corner angles, since `Vector3.Angle` also gives zero for short sides.
8. **The angle.**
   - 30° to 45° works on every model tried.
   - At 40°, a sphere at 8³ gets 3 or 4 vertices split, which is barely visible. At 16³ none is split.
   - A hexagonal prism's sides turn 60°, so the 60° Unity's importer uses by default would leave them smooth.
   - The default here is 40°.
9. **Slivers along edges remain.** Some triangles straddle an edge, and they show as small notches in every mode, Split included.
   - **How many:** the share of triangles whose face is more than 20° off the true normal is 1.6% on the turned cube, 2.9% on NewShapes at 64³, 9.1% simplified and 18.2% at 16³.
   - **Not the triangulation:** taking the other quad diagonal where its middle is nearer the surface turned 23% of quads and moved the turned cube from 1.6% to 1.9%. The cause is where vertices are placed. This is out of scope.
10. **Contour normals need a fallback.**
    - Under Smooth Normals on Contour, a flat region of Occupancy that lies in a grid plane would keep its zero normals.
    - A simplified vertex's carried normal can also be off.
    - **The fallback:** the contour normal is used only when it is within the crease angle of every triangle at the vertex. Otherwise the group's normal is used.
    - **Why every triangle and not the group's normal,** as this plan first said: a contour normal within the angle of the group's normal could be up to twice the angle from one of its triangles. Checking each triangle keeps the guarantee of finding 11 under both options. Changed at step 1.
11. **A group's mean can drift.** A triangle joins when it is within the angle of the mean at that moment, and later members move the mean. In contrived cases an early, small member could end up more than the angle away.
    - Taking the largest corners first keeps this small, and the probe never saw it.
    - **The guard:** after grouping, any corner farther than the angle from its group's final normal gets a group of its own. That makes "no corner normal is more than the angle off its own face" true by construction, and the tests hold to it.
12. **The GPU can do it generically.**
    - Uniform and Occupancy share the vertex, normal and quad buffers.
    - So one set of kernels over triangles can crease both: count each vertex's corners, place its list, group it, and rewrite the quads.
    - It needs only atomic adds, which both GPU implementations already use. Nothing in it is specific to an algorithm.
13. **The GPU's corner runs work on Metal** (step 0). Throwaway versions of the first three kernels built every vertex's run of corners from real GPU contours, with atomic adds and early stops past the counts.
    - **Right everywhere:** the 13 models of the simplification probe, at 32³ and 128³, Uniform and Occupancy. Every run matched the CPU's list, every slot was used once, and the totals were the corner counts.
    - **Two-dimensional dispatches** are also right, forced with 7 groups across. 128³ needs them: NewShapes has 272,000 corners, or 4,250 groups of 64, but a capacity-sized dispatch for Uniform would be 196,000 groups, past the 65,535 limit.
    - **Cost:** the four kernels and a read-back of one value take 0.3 to 0.7 ms at 32³ and 128³, after a first call of 38 ms.
    - **Most corners at a vertex:** 15 in Uniform's GPU contours and 12 in Occupancy's. On the CPU it is 15 in Uniform and Adaptive, 12 in Manifold, 13 in Occupancy and 24 in Manifold at 2e-4, at 64³. Simplified meshes can have more.

      So neither pass gets a fixed-size array: the GPU sorts each run in place and keeps group sums in a buffer beside the runs, and the CPU uses its runs.
    - **Buffer sizes decide the dispatch.**
      - **Uniform's capacities** are one quad and six vertices per cell. A crease sized for capacity would need its corner runs and second vertex buffers sized for six corners per cell: about 350 MB at 128³.
      - **Occupancy** already reads its totals back in mid-contour and sizes its buffers from them.
      - **So both read their counts on the CPU and size and dispatch exactly.** Occupancy sizes from the totals it already reads. Uniform reads its two counts back once after its contour kernel. That read waits for the GPU contour: 100 to 350 ms at 128³ with Uniform's one-thread groups, against 2 to 20 ms for Occupancy.
      - **Why not an indirect dispatch:** it avoids the wait but not the capacity-sized buffers.

## Design

### Settings

On `Voxelizer`, next to Vertex Normals, in the voxelizer's inspector:

| Setting | Default | What it does |
|---|---|---|
| Vertex Normals | Creased | Creased or Split. Creased takes Shared's serialized value 0, so the ten Sphere scenes and every new voxelizer get it. |
| Crease Angle | 40 | In degrees, 0 to 180. Triangles further apart than this around a vertex get vertices of their own. Only used by Creased. |
| Smooth Normals | Contour | Contour or Faces: the normal of a vertex that is not split (finding 10). Only used by Creased. |

- **What the implementations are asked for.** The implementations get a request of their own instead of the setting:
  - Shared: today's shared vertices, uncreased;
  - Split;
  - Creased, with the angle and the option.

  The CPU implementations still always give shared vertices. A simplified contour asks for Shared, as today.
- **`contourNormals`** becomes that request: Shared when simplifying, otherwise the setting.

### The CPU pass

`BurstCreases`, a static class of Burst jobs in `Assets/Scripts/Burst`, beside `BurstMeshSimplifier`. It runs on the finished mesh of any implementation. It has no managed or GPU twin of its own; the GPU kernels are a separate implementation of the same rule (Tests).

1. **Faces:** each triangle's unit normal and its three corner angles, with the pass's own normalize and angle (finding 7). A triangle of zero area has a zero normal and weight. Its corners join their vertex's first group without changing its normal.
2. **Corner lists:** a count of corners per vertex, a prefix sum, and a scatter into one array, so that each vertex's corners are in one run.
3. **Grouping,** in parallel over vertices:
   - the vertex's corners are sorted by corner angle, largest first, ties by corner index;
   - they are grouped as in finding 6, with the guard of finding 11;
   - each group's normal is the sum of its faces' normals times their corner angles, normalized.
4. **Output:**
   - a prefix sum over group counts numbers the new vertices, in the order of the old vertices and then of their groups, so the output is deterministic;
   - positions are copied, normals are written by Smooth Normals (finding 10), and the triangles are rewritten with their sub-meshes;
   - the index format follows the new vertex count.

`Voxelizer.finish` takes the request instead of the `split` flag: Split splits as today, and Creased runs `BurstCreases` unless the GPU already did. Both `contour` and `simplify` go through it, so a new simplification detail is creased too.

### The GPU kernels

`CreaseNormals.compute` in `Assets/Resources/Shaders`, used by `UniformDualContouring.GPU.cs` and `OccupancyDualContouring.GPU.cs` after their contour kernel, when they are asked for Creased:

1. **Count:** one thread per triangle corner adds one to its vertex's count. Threads beyond `counts[1]` stop at once.
2. **Place:** one thread per vertex takes a run of the corner array with an atomic add, so the runs come in any order.
3. **Scatter:** one thread per corner writes its corner into its vertex's run.
4. **Group:** one thread per vertex sorts and groups its run as the CPU pass does, takes its output vertices with an atomic add, and writes their positions and normals into a second pair of vertex and normal buffers.
5. **Rewrite:** one thread per quad writes its six new indices.

- **What comes out:** the voxelization then hands out the new buffers. `RenderContourDiffuse`, the wireframe's quads variant and `readMesh` read them unchanged.
- **Buffer sizes:** the new buffers are as large as Split needs, six vertices per quad.
- **Dispatch:** decided by step 0 (finding 13).
  - The counts are known on the CPU: Occupancy already reads its totals, and Uniform reads its two counts back once after its contour kernel.
  - The crease buffers are sized from those counts, half as much again, as Occupancy's `reserve` does.
  - The threads are dispatched in groups of 64, in two dimensions past 65,535 groups.
  - Each kernel still stops past the counts, which the dispatch rounds up to.
- **When the GPU does not crease:** when the mesh is simplified. The GPU then gives shared vertices, they are read back and simplified, and the CPU pass creases them, as Split works today.

### Interactive editor

- `select` carries the two new settings over, as it does Vertex Normals.
- Both join the contour signature, where Vertex Normals already is.

### Example scenes

The ten Sphere scenes switch to Creased by themselves.
- **At 8³** (the Uniform, Adaptive and Manifold scenes), 3 or 4 vertices may split at 40° (finding 8). Under Contour only those vertices change.
- **No other scene changes** without the user's go-ahead (decision 4).

## Tests

- **`BurstCreasesTest`,** on hand-built meshes with answers in closed form:
  - **A cube** of 8 shared vertices gives 24, each with its face's normal.
  - **Regular prisms** of 3 to 12 sides: the sides are split from each other exactly when the angle between them is over the crease angle, and the caps always are.
  - **A flat grid and a finely tessellated sphere** split nothing. Under Contour every normal is the input's, and under Faces they are the faces' normals.
  - **A chain of triangles turning 15° each,** around one vertex and along a strip: no group reaches past the angle, which the flood fill of finding 6 would fail.
  - **Angle 180 under Contour** gives the input mesh bit for bit.
  - **Angle 0:** every corner's normal is its face's.
  - **Fallback:** zero, flipped and far-off contour normals are replaced by the group's (finding 10).
  - **Edge cases:** zero-area triangles, a vertex no triangle uses, sub-meshes, a non-manifold edge of four triangles, a pinched vertex, and 16-bit and 32-bit indices.
  - **The tiny-triangle case** of finding 7: a triangle under 1e-5 in cross-product length still gets its normal.
  - **The same mesh twice** gives the same output.
- **Invariants on contoured meshes,** for every algorithm and implementation, the 13 models of the simplification probe, with and without simplification:
  - every corner's normal is within the crease angle of its face (finding 11);
  - welding the vertices by position gives back the uncreased mesh's triangles;
  - no normal is zero or non-unit.
- **GPU against Burst:** the GPU's creased contour against `BurstCreases` run on the GPU's own uncreased contour, read back.
  - **Compared as sets of triangles,** since GPU vertex order varies.
  - **Mismatches allowed only near the threshold:** a corner may be grouped differently only if it is within a small tolerance of the angle. GPU and Burst round corner angles differently.
  - **Cases:** Uniform and Occupancy, both Smooth Normals options, and direct draw against read-back.
- **Voxelizer:**
  - the defaults (`VoxelizerDefaultsTest`);
  - a simplified contour asks for Shared, and its new details are creased;
  - Split is unchanged;
  - the six test files that name `VertexNormals.Shared` move to the request or to Creased.
- **Interactive editor:** the carry-over tests and two more cases in the signature tests.
- **The `Start` hashes:**
  - with Creased at 180° under Contour, the 480 hashes are identical to step 0's baseline, which shows nothing else changed;
  - at the defaults they make the new baseline.
- **The tests catch a mistake.** As in earlier plans, mistakes are made on purpose and the tests must fail:
  - the grouping compares with a neighbour instead of the mean;
  - Unity's normalize is used;
  - the guard is left out;
  - the Contour fallback is left out;
  - the GPU corner runs are mixed up.

  The GPU's buffer-validity checks are not mutated, since drawing from released buffers crashes the editor.

## Steps

Each step compiles, passes every test, and is reviewed before the next one starts.

0. **Baseline and checks (small).**
   - Both suites green, and the 480 `Start` hashes recorded.
   - Throwaway kernels in the scratchpad check atomic adds into a structured buffer and dispatching for capacity with an early stop, on Metal, against the counts the contour kernels leave.
   - The answers are written here before step 1.
   - **Done 2026-11-14:**
     - **Suites:** edit mode 2741 of 2741 in 263 s, play mode 21 of 21.
     - **`Start` hashes:** 480 configurations, identical line for line to Mesh Simplification's last baseline. These are the baseline from here on. Half of them are taken with Shared.
     - **GPU checks:** finding 13. The dispatch question is answered there and in the design.
     - **Where the check ran:** Unity cannot compile a compute shader from source in memory without an internal importer API. So the throwaway kernel sat in a temporary `Assets/CreaseCheckTemp` folder for the check, and `AssetDatabase.DeleteAsset` removed it and its `.meta` afterwards.
     - **Clean-up:** `git status` shows only this plan, the open scene is unmodified, and the console has no errors.
     - **A warning that was already there:** CS0108, `BurstMeshSimplifierTest.Shape` hides `ContouringTest.Shape`, from before this plan. It was left alone.
     - **The probe** (`Step0Probe.cs` and `CreaseCheck.compute`) is kept in the probes folder outside the repo.
1. **The CPU pass (medium).** `BurstCreases` and `BurstCreasesTest`, not yet wired in. Its time at 64³, 128³ and 256³ on the three balls and NewShapes, with Unity in front.
   - **Done 2026-11-14:**
     - **Built:**
       - `Assets/Scripts/Burst/BurstCreases.cs`: six jobs, a `Mesh` entry and an array entry, `Statistics`, and the pass's own `unit`.
         - `FacesJob` (parallel): face normals and corner angles, `atan2` of the cross and dot products, in doubles.
         - `RunsJob` (serial): each vertex's corners in corner order.
         - `GroupJob` (parallel over vertices): sort, group, guard, normals.
         - `BasesJob` (serial): numbers the new vertices.
         - `VerticesJob` and `TrianglesJob` (parallel): write the new vertices and triangles.
       - `SurfaceExtractor.SmoothNormals { Contour, Faces }`. The settings themselves are step 2's.
     - **Settled while building, beyond what the design said:**
       - **The Contour rule** checks every triangle, not the group's normal (finding 10).
       - **A vertex no triangle uses** stays in its place with its normal. Without that, 180° under Contour would not give back meshes that have such vertices, as Manifold at 2e-4 does.
       - **A group with no normal of its own** takes the contour's under either option: its triangles have no area between them, or, at 180° only, their normals cancel.
       - **Triangles without area** weigh nothing, come last in their runs, and join the first group.
       - **At 180° and above** the cosine is -2, so a dot product rounded just under -1 cannot start a second group.
       - **The guard leaves a corner alone in its group.** At an angle of 0 the group's normal can round a hair away from its only corner, and the guard would then move that corner for ever. A mutation that took this check out would have hung the editor, so it was not run.
     - **Tests:**
       - `BurstCreasesTest`, 330 tests in 50 s:
         - **Hand-built shapes:** a cube under and past 90°, prisms of 3 to 24 sides, a flat patch, a sphere, a cone's tip, the drifting group of finding 11, the order of equal corners, the Contour fallback, a triangle without area, a loose vertex, an edge of four triangles and a pinched vertex, a cube 1e-4 across, and sub-meshes with 16-bit and 32-bit indices.
         - **Contours:** the 11 contours × 11 surfaces of the simplification tests, plain and simplified, under both options, at 40°: the same triangles, no corner past the angle, and unit normals. Also 180° under Contour giving the mesh back bit for bit, 0° giving every corner its triangle's normal, and the same output twice.
       - **Moved:** the contours and surfaces moved from `MeshSimplificationTest` into a new abstract `ContourVariantsTest`, which both suites derive from. `TopologyTest` was not the place, because `BurstMeshSimplifierTest` has enums of the same names.
     - **The tests catch a mistake** (`mutations_creases_step1.py` in the probes folder). 13 mistakes were made, and 12 were caught:
       - chaining to the previous triangle (caught by the cone, which now also requires that no corner was guarded);
       - Unity's normalize;
       - no guard;
       - the contour's normal always kept;
       - no angle weights;
       - smallest corners first;
       - a loose vertex dropped;
       - the contour's normal at split vertices;
       - weights in the wrong corners;
       - a corner given the next corner's group;
       - no special cosine at 180°;
       - a group without a normal left at zero.

       **The survivor** was equal corners taken the other way round. Both orders are deterministic, but the GPU must follow Burst's, so a test now pins the order, and it catches the mistake.
     - **Time** (Uniform Burst, Unity in front, medians of 5). The pass costs about what Split's `splitVertices` costs today, and most of it is the arrays:

       | Mesh | Contour | Vertices | Crease | Split |
       |---|---|---|---|---|
       | Three balls 64³ | 5.6 ms | 13,000 → 13,684 (+5.3%) | 1.9 ms | 1.3 ms |
       | Three balls 128³ | 28 ms | 52,224 → 53,581 (+2.6%) | 6.9 ms | 5.5 ms |
       | Three balls 256³ | 173 ms | 208,900 → 211,440 (+1.2%) | 28 ms (21 of it the arrays) | 25 ms |
       | NewShapes 64³ | 9.4 ms | 11,111 → 13,093 (+17.8%) | 1.7 ms | 1.2 ms |
       | NewShapes 128³ | 49 ms | 45,270 → 49,089 (+8.4%) | 6.1 ms | 4.9 ms |
       | NewShapes 256³ | 295 ms | 182,492 → 190,003 (+4.1%) | 25 ms | 21 ms |
       | Three balls 128³ simplified | 28 + 206 ms | 2,268 → 2,732 (+20.5%) | 0.5 ms | 0.3 ms |
       | NewShapes 128³ simplified | 48 + 177 ms | 2,634 → 3,904 (+48.2%) | 0.6 ms | 0.3 ms |

       - **Fewer extra vertices at finer grids:** creases are lines and vertices fill surfaces, so the share added falls as the grid gets finer.
       - **Contour costs the same as Faces,** within 1 ms.
       - **The guard fired once** in these meshes, on NewShapes at 128³.
       - **Not tuned:** `RunsJob` and `BasesJob` are serial. They are the candidates if step 5's measurements call for it.
     - **Verification:**
       - edit mode 3071 of 3071 (2741 + 330), and play mode 21 of 21;
       - the 480 `Start` hashes identical to step 0's, since nothing calls the pass yet;
       - the open scene unmodified and the console clean. Burst's out-of-range errors during the mutation run came from the dropped-vertex mutant, and the console was clear after a run on the real source.
2. **The voxelizer (medium).**
   - Creased replaces Shared, with the two settings and the implementations' request.
   - `finish`, `contour` and `simplify` crease.
   - The existing tests are moved over.
   - The hashes at 180° under Contour against step 0's; the new baseline at the defaults.
   - **Done 2026-11-14:**
     - **Built:**
       - **`SurfaceExtractor.VertexNormals` is `{ Creased, Split }`.** Creased has Shared's value 0, and is the default.
       - **The request is `SurfaceExtractor.ContourVertices { Shared, Split }`.** Every implementation's `voxelize` takes it in place of `VertexNormals`, and `Voxelizer.contourVertices` gives it in place of `contourNormals`: Shared when simplifying or creasing, Split otherwise. Creased joins the request in step 3, when the GPU can act on it.
       - **The two settings are public fields** with tooltips, `creaseAngle` (Range 0 to 180, default 40) and `smoothNormals` (default Contour), right after Vertex Normals in the inspector, like the simplification settings.
       - **`Voxelizer.finish`** takes whether the mesh has shared vertices. It splits them for Split, creases them with `BurstCreases` for Creased (skipped for an empty mesh), destroys the mesh it creased, and hands back the mesh it showed. `contour` and `simplify` both go through it, so every new simplification detail is creased.
       - A profiler marker, `Voxelizer.Crease`.
     - **For now, the GPU's direct draw is not creased.** A Creased GPU contour drawn straight from the buffers shows the shared vertices with the contour's normals until step 3's kernels, as Shared did. The doc comment on `contour` says so. Every contour read back from the GPU is creased on the CPU.
     - **Tests:**
       - **`VoxelizerContourTest`** has a helper, `asShown`, which creases or splits an expected mesh as `contour` would. The tests that hold the shown mesh to `voxelize`'s, to the simplifier's or to a level of the record now use it.
       - **Two new tests there:**
         - `contourCreasesUnderTheVoxelizersSettings`: 9 variants × 0°, 25° and 180° × both options, the shown mesh against `BurstCreases` on the contour, with the uncreased mesh not left behind;
         - `fullCreaseAngleUnderContourShowsTheContoursOwnVertices`.
       - **`MeshComparison.welded`** makes a creased mesh one vertex per position again. The topology check in `contourWithSimplificationShowsTheSimplifiedContour` counts by vertex index, and failed on creased meshes until it compared the welded mesh.
       - **`VoxelizerDefaultsTest`** checks the three new defaults.
       - **`InteractiveEditorTest.debugDrawCountFollowsTheFlags`** counted two debug marks per vertex of the shown mesh, which only holds when nothing is creased. It now contours at 180°.
       - **Shared became Creased** in the voxelizer setups of `BurstMeshSimplifierTest`, `ContourVariantsTest` and `WireframeTest`. They read `voxelize` or draw straight from the GPU, both unchanged by this step.
     - **The tests catch a mistake** (`mutations_creases_step2.py`). All 10 were caught:
       - no crease;
       - the default angle instead of the voxelizer's;
       - Faces whatever the setting;
       - the uncreased mesh left behind;
       - `simplify` not creasing;
       - the GPU splitting whenever not simplifying;
       - Creased asking for split vertices;
       - the uncreased mesh reported;
       - two wrong defaults.
     - **`Start` hashes** (the probe now sets the crease angle, `run_hashes.sh <file> <angle>`):
       - **At 180° under Contour,** all 480 are identical to step 0's, with "Creased" read as "Shared". Nothing changed but the crease.
       - **At the defaults** (the new baseline), 220 of the 240 Creased configurations changed. The 20 that did not are smooth spheres with nothing to split. All 240 Split configurations are unchanged. Vertices added: 0.7% to 109%, median 18%.
     - **The ten scenes saved with Shared,** opened as preview scenes and contoured, read Creased at 40° under Contour, and split no vertex, so they show what they showed before.
     - **Verification:**
       - edit mode 3134 of 3134. The first full run had 3133: the one failure was the debug count test above, before it was fixed;
       - play mode 21 of 21;
       - the open scene unmodified;
       - the console clean after the mutation run's errors were cleared and the real source recompiled.
3. **The GPU kernels (medium to large).** `CreaseNormals.compute`, used by Uniform and Occupancy GPU for direct draw and read-back. The GPU against Burst tests, and a look at both GPU Sphere scenes in play mode.
4. **Interactive editor (small).** Carry-over, signature, and a live look in the editor scene with the user's scene left unmodified.
5. **Measurements and documentation (small).**
   - CPU and GPU times and vertex counts at 64³ to 256³.
   - A README section on Vertex Normals.
   - Scene changes, if decision 4 says so.
   - This plan marked complete.

## Out of scope

- **The slivers of finding 9,** which come from where vertices are placed.
- **Re-creasing without contouring** when only the angle or Smooth Normals changes in the interactive editor. It would need the uncreased mesh kept, as the simplification record keeps the contour.
- **Tangents, normal maps and UVs.**
- **Splitting by material.** A vertex where two materials meet stays shared when its normals agree, as today.
- **Creasing from the Hermite data** instead of the faces. It would need every implementation to keep its normals per cell, and the faces already work after simplification.

## Decisions

1. **Name the new mode Creased?** It says what it does, next to Split. The alternatives are Smooth, which hides that edges stay sharp, and Shared, which keeps the old name for a new behaviour. Recommended: Creased.
2. **Default for Smooth Normals?**
   - **Contour** keeps today's shading wherever nothing is split, and with it today's Sphere scenes, apart from the 3 or 4 split vertices at 8³.
   - **Faces** is the same rule everywhere, and the probe could not tell the two apart on a sphere.

   Recommended: Contour.
3. **Default Crease Angle of 40°?** Finding 8: 30° to 45° worked everywhere, and 60° misses the hexagonal prism. Recommended: 40°.
4. **Which scenes show it?**
   - The ten Sphere scenes change by themselves.
   - The 66 that use Split stay as they are unless the user says otherwise.
   - The candidates where the difference shows most are the interactive editor scene's four voxelizers and the three Mesh Simplification scenes on Split. The fourth, Uniform Sphere, is one of the ten.

   Recommended: switch those seven at step 5, after a look at each.
