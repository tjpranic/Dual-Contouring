# Interactive Editor

Plan written 2026-11-11 for an editor in which shapes are added and combined with CSG operations, and the contour follows every change, in edit mode.

**Status:** step 0 (measurement) done 2026-11-11, and the two questions it raised were answered (decisions 6 and 7). Steps 1 (re-runnable voxelizer), 2 (editor component and scene) and 3 (automatic remeshing) done 2026-11-11; step 3 awaits review.

## Goal

One scene, `Assets/Scenes/Interactive Editor.unity`, where:

- shapes are added, moved, scaled, reordered and removed, each with its own CSG operation and material;
- one dual contouring variant at a time contours them;
- everything runs in edit mode, with no need to press Play;
- with a Burst or GPU variant, any change to an inspector value, a shape's transform or the shape list remeshes after a short debounce;
- with a Managed variant, the mesh is rebuilt when "Contour" is pressed.

"Variant" here means an algorithm and an implementation together. There are seven: Uniform with Managed, Burst or GPU, and Adaptive and Manifold each with Managed or Burst.

## Decisions

Made by the user on 2026-11-11.

1. **What is selectable:** every algorithm and every implementation, so all seven variants. The algorithm is a dropdown on the editor; the implementation is the voxelizer's existing setting.
2. **Shape transforms:** the interactive editor reads real transforms. A shape is contoured where its transform says it is, under any root scale and inside groups. `Volume.origin` and `Volume.extents` keep today's reading, so the 60 existing scenes are untouched.
3. **Shapes:** Cuboid and Ellipsoid, as today. Rotation is wanted and has its own plan, `Shape Rotation.md`, which follows this one. New primitives are not planned.
4. **Managed variants never remesh automatically.** They contour only when "Contour" is pressed.
5. **Burst and GPU variants remesh automatically, during a drag as well.** While a slider or handle is being dragged they remesh as often as the last contour's time allows, as long as it was fast. Otherwise they remesh when the changes pause.

Made by the user on 2026-11-11 after the step 0 measurements, each as recommended:

6. **GPU buffers are kept between contours.** A GPU remesh took 75–81 ms at 64³ and 0.6–1.0 s at 128³, slower than Burst at every size. Most of that was on the CPU, before the GPU starts: every contour allocated all eleven compute buffers anew and uploaded arrays for voxels, corners, edges, vertices, normals and quads, about 1.5 GB at 128³. The `buildVoxelGrid` kernel overwrites the voxels, corners and edges, and the contour kernel appends the vertices, normals and quads, so none of that upload is needed.
   - Step 1 keeps the buffers while the resolution and the number of shapes stay the same, and uploads only the configuration, the shapes and the two counters.
   - Expected: 64³ about 40 ms, live during a drag; 128³ about 170 ms.
   - It is a C# change in `UniformDualContouring.GPU.cs`. The shader stays as it is, and every mesh stays the same.
7. **Automatic remeshing pauses after a remesh over a second.** With the debug view on, Uniform has to copy its corners, edges and voxels out for every remesh: GPU took 0.44 s at 32³ and 5.0 s at 64³, Burst 0.36 s at 64³. Burst Uniform without debug takes 0.2 s at 256³.
   - When a Burst or GPU variant's last remesh took more than a second, automatic remeshing pauses. The status line says so, and "Contour" works as it does for Managed.
   - It resumes after a remesh under a second, for example once the debug view is off or the resolution is lower.

## What exists today

- **Stub scene:** `Interactive Editor.unity` holds a camera with `CameraController`, a directional light and the FPS counter canvas. It is the same as `Empty.unity`. Both are untracked.
- **Voxelizers:** `UniformDualContouring`, `AdaptiveDualContouring` and `ManifoldDualContouring` derive from `Voxelizer`. `Voxelizer.Start` contours once: it collects the `Volume` components below it, calls `voxelize`, post-processes the mesh, adds a `MeshFilter` and captures debug data.
- **Shapes:** `Cuboid` and `Ellipsoid` derive from `Volume`. Position and size come from the transform; `combination` (Union, Intersection, Subtraction) and `materialIndex` are fields.
- **CSG:** a fold over the shapes in hierarchy order, starting from empty space. Order matters, and a first shape that is not a Union gives nothing.
- **Materials:** sub-mesh n is material bit n, drawn with the mesh renderer's n-th material. A new `Volume` has material `Void`, which never produces a surface.
- **Grid:** the voxelizer's object space from -0.5 to 0.5 on each axis. The Resolution slider goes from 2 to 8. Uniform has 2^resolution cells across (4³ to 256³), the octrees 2^(resolution - 1) leaves (2³ to 128³).

Step 0 measured how long a remesh takes in edit mode; see "Measurements" at the end.

### Findings that shape the work

1. **Shape transforms are not real transforms.** `Volume.origin` is `localPosition / voxelizer.localScale`, and `extents` likewise. In "CSG Operations" the voxelizer has scale 5 and "Sphere 1" has local scale 5, which contours as a sphere filling the grid, while its real world size is 25. Transforms of groups between a shape and the voxelizer are ignored; the existing groups are all at identity. Rotation is ignored.
2. **Nothing runs in edit mode.** No script has `ExecuteAlways`. Shapes' `ImmediateModeShapeDrawer` is a plain `MonoBehaviour` too, so the debug overlay does not draw in edit mode either. Putting `ExecuteAlways` on `Voxelizer` would make all 60 scenes contour when opened, some for seconds, so the edit-mode behaviour goes in a new component instead.
3. **`Voxelizer.Start` is single use and assumes play mode.**
   - It adds a new `MeshFilter` on every call.
   - `meshFilter.mesh` and `meshRenderer.materials` create instances, which leak in edit mode.
   - A mesh assigned in edit mode is written into the scene file unless it is flagged not to be saved.
   - `UniformDualContouring` creates its three implementation objects in `Start`, so `voxelize` fails before it.
4. **The GPU path needs a readback for edit mode.**
   - Drawing straight from the compute buffers happens in `Voxelizer.Update`, which does not run in edit mode.
   - `convert` builds a managed object for every voxel, with its 8 corners and 12 edges, then de-duplicates them. That is 5 million objects at 64³, for debug data nobody looks at when the debug flags are off.
   - `OnDestroy` releases the compute buffers and is not called in edit mode.
5. **`voxelize` takes any density functions.** It reads `type`, `origin`, `extents`, `combination` and `materialIndex` through the `DensityFunction` interface, in all three implementations. The editor can hand it shapes converted its own way without touching `Volume`.
6. **Transform changes raise no inspector event.** `OnValidate` covers inspector edits of a component's own fields. It does not cover moving or scaling with handles, the Transform inspector, reordering, adding, removing, deactivating, or undo of any of those.
7. **Edit mode has no steady tick.** `Update` on an `ExecuteAlways` component runs only when something in the scene changes. A debounce timer needs `EditorApplication.update`.
8. **Everything is far slower while Unity is in the background.** With another application in front, Burst took 15–18 times as long (Uniform 128³: 441 ms against 25). Probably macOS scheduling a background application's threads differently; the cause wasn't looked into. Any remesh started while Unity is not the active application, after a script recompile for example, pays that, and so would any timing taken then.
9. **The GPU path uploads what its kernels overwrite.** See decision 6.

## Design

### Scene

```
Interactive Editor      InteractiveEditor, UniformDualContouring, AdaptiveDualContouring,
                        ManifoldDualContouring, MeshFilter, MeshRenderer (one material per material bit)
  Cuboid                Cuboid
  Ellipsoid             Ellipsoid
  Group                 (optional, moves its shapes together)
    Cuboid              Cuboid
```

- **One object, three voxelizers.** The three voxelizer components sit on the root, with the shapes below it. Each keeps its own settings, and the existing presets still apply to them. Only the selected one is shown in the inspector and used; the other two are hidden.
- **Why not three objects:** a shape finds its voxelizer by looking up the hierarchy, so three sibling voxelizers could not share one set of shapes.
- **Camera, light and FPS counter** stay as the stub has them. The FPS counter and camera controller only run in play mode; in edit mode the scene view is the viewport and the inspector shows the timings.

### `InteractiveEditor` component

New, `ExecuteAlways`, in `Assets/Scripts/InteractiveEditor.cs`. It does not contour anything itself: it decides when to contour and with which voxelizer.

- **Settings:** algorithm (Uniform, Adaptive, Manifold) and the debounce interval.
- **Selecting an algorithm:**
  - The common settings carry over to the newly selected voxelizer: resolution, iterations, implementation, solver, intersection approximation, vertex normals and debug flags. The same shapes are then compared like for like.
  - Each algorithm's own settings (simplification, thresholds, manifold criterion) stay with its component.
  - GPU carried over to Adaptive or Manifold becomes Burst, which keeps remeshing automatic. The existing inspectors turn it into Managed.
- **Lifetime:**
  - When enabled (scene load, script recompile) with a Burst or GPU variant, it contours if the mesh is missing or out of date.
  - With a Managed variant it never contours on its own. After the scene is opened there is no mesh until "Contour" is pressed.
  - When disabled it releases the GPU buffers.
  - The mesh is never saved into the scene, so the scene file stays small and opening the scene does not mark it changed.
  - The mesh object should survive a script recompile, so a recompile costs no contour and a Managed mesh stays on screen. Step 2 confirms that.
- **Play mode:** the same component and the same code path. The voxelizers under it do not contour by themselves.
- **Shapes:** converted by the editor into plain density functions in the root's object space. Inactive shapes are left out.
  - A direct child's local position and scale are already in that space, whatever the root's scale: a child of scale 1 fills the grid.
  - A shape inside groups has the groups' positions and scales applied on top.

### Change detection

One mechanism covers every source in finding 6: the editor compares what it would contour now with what the current mesh was built from.

- **Signature:** a string of the selected algorithm, the selected voxelizer's contour settings, and for each active shape in order its type, position, size, operation and material. Strings are compared whole, so there are no hash collisions.
- **Polling:** worked out on every editor update. It costs about 25 µs with the starter model's four shapes.
- **What it ignores:** names, debug colours and sizes, the renderer's materials, the debounce, a shape's own rotation (until `Shape Rotation.md`), the voxelizers that aren't selected, and the root's own transform, which moves the mesh without changing it. A material added to the renderer gets an empty sub-mesh on the next update, without a contour.
- **Debug flags** are part of the signature, because the Burst implementations only copy debug data out when a flag is on.
- **Managed variants** use the same signature, only to show that the mesh is out of date.

### Scheduling

A small class with no Unity dependencies, so tests can drive it with a fake clock.

- **Managed:** never contours by itself. "Contour" is the only trigger.
- **Burst and GPU:**
  - **Debounce:** contour once the signature has been stable for the interval, 100 ms by default.
  - **Fast variants:** if the last contour took at most 50 ms, contour on a change without waiting for it to settle, and no sooner than that contour's own time after it ended, so the editor spends at most half its time contouring. A drag of a handle or slider then updates live, and a single change shows at once. The debounce only applies to slower variants.
    - Live by the measurements: Burst up to 128³ (25–34 ms), and GPU up to 64³ (28–30 ms, with its buffers kept since step 1).
    - Waiting for the pause: Burst Uniform at 256³ (160–194 ms) and GPU at 128³ (137–140 ms).
  - **Slow remeshes:** after a remesh over a second, automatic remeshing pauses until a remesh is under a second again (decision 7).
  - **Unity in the background:** automatic remeshing waits until Unity is the active application again (finding 8). The times the scheduler goes by are only taken while it is active.
- **Blocking:** a contour runs to completion on the main thread.
- **No vertex reordering:** the editor's remesh leaves out `OptimizeReorderVertexBuffer`. It takes 5–10% of a Burst remesh at 128³ and above (3.6 ms of 34 at 128³, 17 of 194 at 256³) and only makes the mesh a little faster to draw. `Start` keeps it for the existing scenes.

### Changes to `Voxelizer`

Behaviour in the 60 existing scenes stays exactly as it is.

- **Re-runnable contour:** the body of `Start` becomes a method that can be called again. It reuses the mesh filter, destroys the mesh it replaces and returns the time taken and the vertex and triangle counts. `Start` calls it.
- **Edit-mode safety:** shared mesh and shared materials in place of instances, and a mesh that is not saved with the scene.
- **Uniform:** the implementation objects are created on first use, not in `Start`.
- **GPU readback:** when the debug flags are off, read back only the vertex, normal and quad buffers and build the mesh. `convert` stays for the debug view and the tests.
- **Debug overlay:** `DrawShapes` is already public. In edit mode the editor component calls it from its own camera hook.

### Authoring

- **Inspector of `InteractiveEditor`:**
  - the algorithm dropdown and the debounce interval;
  - the shapes in CSG order, each with its operation and material inline, reorderable by dragging, with remove buttons;
  - "Add Cuboid" and "Add Ellipsoid", which create a child with material 0, Union, a sensible size, and undo support;
  - a status line: up to date, waiting, or out of date, and the last contour's time, vertices and triangles;
  - "Contour", which remeshes at once with any variant.
- **Scene view:**
  - a wire outline for each shape, coloured by operation (Subtraction shapes are otherwise invisible), clickable to select;
  - a wire box for the grid, since a surface that leaves it is cut open.
- **Warnings in the inspector:** a first shape that is not a Union, a shape with material `Void`, a material bit the renderer has no material for, a shape entirely outside the grid, and a rotated shape. The last one goes away with `Shape Rotation.md`.
- **Editor code:** the inspector goes at the bottom of the component's file, as the existing inspectors do. This adds to the editor-only code inside `GameAssembly` that was left alone on 2026-11-11.

## Steps

Each step compiles, passes every test, and is reviewed before the next one starts.

0. **Measure (small). Done 2026-11-11.** No product code.
   - Timed a full remesh in edit mode for every variant from 16³ up, as far as each goes, stage by stage. Timed the GPU path with `convert` and with a buffer-only readback. The numbers are under "Measurements".
   - Outcome: debounce 100 ms; remeshing during a drag when the last contour took at most 50 ms; no vertex reordering in the editor; automatic remeshing waits while Unity is in the background. The GPU findings led to decisions 6 and 7.
1. **Re-runnable voxelizer (medium). Done 2026-11-11.** The changes to `Voxelizer` above, and the GPU buffer change of decision 6.
   - **`Voxelizer.contour( [densityFunctions,] drawFromGPU = true, reorderVertices = true )`:** the body of `Start`, which now only calls it. It returns `ContourStatistics` (milliseconds, vertex count, triangle count; the counts are null when drawn straight from the GPU). With no density functions given it contours the volumes below it, as `Start` did.
   - **Mesh:** named `Contour`, flagged `DontSaveInEditor | DontSaveInBuild`, assigned as the shared mesh of the mesh filter, which is added only if there is none. Showing a new contour destroys the contour mesh the filter showed, whichever voxelizer on the object made it, so voxelizers on one object share one filter. The mesh can still be unloaded once nothing uses it, so one whose voxelizer is gone is not kept for the session.
   - **`Voxelizer.clear( )`:** takes away the shown contour, the mesh this voxelizer made, the drawing from the GPU and the debug data. `OnDestroy` calls it; Uniform's override calls the base before releasing its buffers.
   - **Debug data** is cleared at every contour, so a contour with the debug view off leaves no stale overlay.
   - **GPU drawing:** the material reads `sharedMaterials` for its colours, is flagged not to be saved and is destroyed when the next contour or `clear` replaces it.
   - **`readMesh( GPUVoxelization )`:** a new abstract member beside `convert`, implemented in `UniformDualContouring.GPU.cs`, which reads the two counters and then only that many vertices, normals and quads. `contour` uses it for GPU when the debug view is off and the mesh is not drawn from the GPU. Adaptive and Manifold throw, as they do for `convert`.
   - **Uniform:** each implementation is made on first use. `Start` no longer makes them, and `testing` still keeps it from contouring.
   - **GPU buffers (decision 6):** `prepareBuffers` replaces `createBuffers`. The configuration, counter, argument and vertex-count buffers are made once; the six grid buffers when the resolution changes; the density function buffer when their number changes, holding at least one element so a contour of nothing works. Each contour uploads the configuration, the density functions and the two counters, and sets the constant buffer again, because all GPU voxelizers share the one compute shader. `releaseBuffers` nulls what it releases, so the next contour makes everything anew.
   - **Tests:** `VoxelizerContourTest`, 105 tests over all seven variants: the mesh shown, a second contour, one mesh filter, the replaced mesh destroyed, not saved, statistics, no reordering, a contour of nothing (both normals), given density functions, `clear`, two voxelizers on one object, Uniform without `Start`, `readMesh` against `convert` (4 models, both normals), GPU contours in a row against fresh ones (changing models, shape counts and resolutions, and nothing), and GPU after releasing its buffers.
   - **Verified:** edit mode 618/618, play mode 20/20. A probe hashed what `Start` leaves behind (the mesh, the number of mesh filters and the captured debug data) for 416 configurations: three algorithms, every implementation, four models, both solvers, both normals, two resolutions, debug view on and off. Every hash is identical before and after. GPU meshes were hashed as sorted triangle sets, and are the same from run to run.
   - **GPU timings after:** see "GPU with its buffers kept" under Measurements.
2. **Editor component and scene (medium). Done 2026-11-11.** `InteractiveEditor` with algorithm selection, lifetime, shape conversion and the "Contour" button. No automatic remeshing yet.
   - **Component:** `Assets/Scripts/InteractiveEditor.cs`, `ExecuteAlways`. It requires the three voxelizers and a mesh filter, so adding it adds them. Its inspector, at the bottom of the same file inside `#if UNITY_EDITOR`, has the algorithm dropdown (undoable), "Contour", and a line with the last contour's time, vertices and triangles.
   - **Selecting an algorithm** (`select`): carries the common settings over as designed. Resolution carries over as the number of cells across (Uniform r is octree r + 1), clamped to the slider's 2 to 8. GPU becomes Burst for the octrees. The voxelizer left behind releases its compute buffers; its mesh stays until the next contour. Only the selected voxelizer shows in the inspector (`HideInInspector` on the other two, saved with the scene). `OnValidate` puts the flags right after an undo, on the editor's next update.
   - **Shapes** (`shapes()`): hierarchy order. Shapes on inactive objects, disabled shape components and a shape on the editor's own object are left out. Each becomes an `InteractiveEditor.Shape` (a `DensityFunction`) in the editor's object space. A direct child gives exactly its local position and half its local scale. Groups apply their position, scale and rotation to the positions in them, without a round trip through world space. A negative scale gives a positive size. Rotation of a shape itself is not read (`Shape Rotation.md`).
   - **Contour:** the selected voxelizer's `contour( shapes, drawFromGPU: false, reorderVertices: false )`, then a scene view repaint.
   - **Sub-meshes:** Unity draws every material past the last sub-mesh over the last sub-mesh. The starter model's second-material ball came out blue, the last of four materials, until the editor began padding the mesh with empty sub-meshes up to the renderer's number of materials. The example scenes don't run into this because they list exactly the materials they use.
   - **Lifetime:**
     - `Awake` in play mode disables the three voxelizers, which keeps their `Start` from contouring.
     - `OnEnable` contours a Burst or GPU variant when the mesh filter shows nothing; a Managed variant never.
     - `OnDisable` (also just before a script reload) releases every voxelizer's compute buffers and keeps the mesh.
     - `OnDestroy` clears the contour and releases.
     - `Voxelizer` gained `release()` for this: `OnDestroy` is now `clear()` then `release()`, and Uniform overrides `release` instead of `OnDestroy`.
   - **Scene:** `Interactive Editor` at scale 5 with a mesh filter, a renderer with `opaque_white`, `opaque_green`, `opaque_red` and `opaque_blue` for material bits 0 to 3, and the editor. Uniform is selected: Burst, resolution 6 (64³), SVD, no surface correction, split normals. Adaptive and Manifold keep their defaults until selected. The starter model is a rounded cube (Cube, Union; Rounding, Intersection) with a bite out of one corner (Bite, Subtraction) and a ball of material 1 in the bite (Ball, Union). It was built in the open scene through the editor API, with undo, and saved. A contour takes about 5 ms and gives 57,564 vertices and 19,188 triangles.
   - **Tests:** `InteractiveEditorTest`, 50 edit-mode tests: required components, inspector visibility, carrying settings over, GPU to Burst, keeping each algorithm's own settings, resolution limits, shape placement (scaled root, nested groups, rotated group, negative scale, order, what is left out), the contour at scale 1 matching the volumes' contour for all seven variants, the editor's scale not changing the contour, every variant contouring the starter model, sub-mesh padding, contouring on enable (Burst and GPU) or not (Managed), keeping the mesh over disable and enable, GPU after disable, removing the editor removing the mesh, switching algorithm. `InteractiveEditorPlayModeTest` (play mode): only the editor contours. The triangle comparison moved to `MeshComparison`, shared with `VoxelizerContourTest`.
   - **Verified:**
     - Edit mode 668/668, play mode 21/21. The 416 `Start` hashes are identical to the baseline.
     - In the saved scene the mesh filter holds no mesh and the file has no mesh data. Contouring replaces the mesh without marking the scene modified.
     - The mesh is the same object after a script reload.
     - Play mode: the editor contours once and the voxelizers are disabled. After leaving play mode they are enabled, the scene is unmodified, and Burst contours again, because the edit-mode mesh does not survive play mode. A Managed variant shows nothing after play mode until "Contour" is pressed.
     - A test run reopens the open scene when it ends, which also costs one contour for Burst and GPU.
3. **Automatic remeshing (medium). Done 2026-11-11.** Signature, polling, scheduler.
   - **`RemeshScheduler`** (`Assets/Scripts/RemeshScheduler.cs`, no Unity dependency) decides from the time, the signature, whether the variant is automatic and whether Unity is active. Its states are `UpToDate`, `Due`, `Waiting`, `Manual` (Managed), `Paused` (decision 7) and `Background`. A contour tells it the time it ended and how long it took; times taken in the background are not used.
   - **`InteractiveEditor`:**
     - `signature()` builds the signature.
     - `tick()` asks the scheduler and contours when it says `Due`. It runs from `EditorApplication.update`, registered in `OnEnable` and removed in `OnDisable`, and also in play mode inside the editor.
     - The time and "Unity is active" come from `EditorApplication.timeSinceStartup` and `InternalEditorUtility.isApplicationActive`. Tests replace them through `clock` and `applicationActive`.
     - A new **Debounce** setting, 0.1 s by default.
   - **On enable:** a mesh that is shown counts as up to date, which is how a script recompile costs no contour. With nothing shown, a Burst or GPU variant contours at its first update, without waiting for the debounce. The scheduler starts afresh, so a pause from a slow contour ends with a recompile.
   - **Inspector:** the Debounce field, and a status line for each state (up to date; waiting for changes to settle; out of date, Managed; paused, with the time of the contour that paused it; waiting for Unity to be active). The inspector repaints continuously so the status follows the editor's updates.
   - **Tests:**
     - `RemeshSchedulerTest` (15) on a clock of its own. It covers debouncing, a burst of changes, changing back, live contouring for fast contours, waiting for slow ones, the inclusive 50 ms limit, Managed, the background, pausing and resuming, the exclusive 1 s limit, background timings, and the debounce setting. Times at the limits are binary fractions so the differences come out exact.
     - `InteractiveEditorTest` (51 more, 101 in all):
       - 28 contour inputs that change the signature and 11 other changes that don't;
       - a change undone is up to date;
       - each Burst and GPU variant contours a change by itself;
       - selecting an algorithm contours;
       - Managed only contours when asked;
       - nothing happens in the background, then a contour once Unity is active;
       - enabling with a mesh shown is up to date;
       - a material added to the renderer gets a sub-mesh without a contour.
   - **By hand in the scene:** moving the ball contoured by itself, and moving it back contoured again to the same vertex count, within two seconds each and without marking the scene modified.
   - **Verified:** edit mode 734/734, play mode 21/21, and the 416 `Start` hashes are identical to the baseline.
4. **Authoring (medium).** Inspector list and buttons, outlines, grid box, warnings, status line, debug overlay in edit mode.
   - Tests: add, remove and reorder through the same functions the inspector calls, each undoable; each warning condition.
   - The inspector layout and the scene view drawing are checked by eye, by the user.
5. **Documentation (small).** A README section on the interactive editor, and this plan marked complete.

## Out of scope

- **Rotation.** Planned separately in `Shape Rotation.md`.
- **New primitives.**
- **Background contouring.** The editor blocks while a contour runs. Moving the managed path to a thread, or leaving Burst jobs running across frames, is a separate piece of work.
- **Partial remeshing.** Every change contours the whole grid.
- **Live editing in the existing example scenes.** They keep contouring once, on Play.
- **Player builds.** The tool lives in the editor.
- **Drawing straight from GPU buffers in edit mode.** The GPU variant reads back to a mesh here.
- **Saving the mesh as an asset.** A natural follow-up button, not part of this plan.

## Measurements

Step 0, 2026-11-11. A probe ran each remesh stage by stage on hidden objects in edit mode, the way `Voxelizer.Start` does it, with nothing added to the open scene.

**Setup:**
- Model and settings of `ContouringPerformanceTest`: the three balls, SVD, no surface correction; Adaptive simplifies and Manifold clusters at 2e-4, topology-safe and with the manifold criterion.
- Burst and GPU: medians of 5 runs after a warm-up (3 at 256³ and 128³ GPU). Managed: one run each.
- Unity was brought to the front before every run, and was still in front after every run but one (Manifold Managed, Split, where the numbers match the run beside it). See finding 8 for why.
- Times are in ms, in the editor (Mono, Burst safety checks on), on a 64 GB Mac.
- Managed was measured up to 64³; the Burst plan has its 128³ times (39–57 s).

**A remesh with the debug view off, in ms** (Shared normals / Split normals)

| Cells or leaves | Uniform Burst | Adaptive Burst | Manifold Burst | Uniform GPU | Uniform Managed | Adaptive Managed | Manifold Managed |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 16³ | 0.5 / 0.6 | 0.9 / 1.0 | 1.0 / 1.2 | 3.0 / 3.0 | — / 103 | — / 134 | — / 128 |
| 32³ | 1.4 / 1.9 | 1.9 / 2.0 | 2.4 / 2.5 | 11.5 / 11.9 | — / 661 | — / 725 | — / 738 |
| 64³ | 5.1 / 6.9 | 6.1 / 6.0 | 6.6 / 6.7 | 75 / 81 | — / 5,180 | — / 6,051 | — / 6,329 |
| 128³ | 25.5 / 33.8 | 29.4 / 28.2 | 29.7 / 30.0 | 953 / 636 | | | |
| 256³ | 160 / 194 | | | not possible | | | |

- The Burst `voxelize` times match the Burst plan's tuned figures.
- GPU at 256³ was not run: its buffers would take about 12 GB, with a single edge buffer of 4.8 GB.

**Where a GPU remesh goes, in ms** (Shared / Split)

| Cells | Allocate and upload | GPU work, until the first read returns | Read back and build mesh |
|---:|---:|---:|---:|
| 16³ | 0.3 / 0.3 | 2.1 / 2.0 | 0.5 / 0.6 |
| 32³ | 1.2 / 1.1 | 8.7 / 8.8 | 1.5 / 1.8 |
| 64³ | 41 / 41 | 31 / 36 | 3.4 / 4.0 |
| 128³ | 787 / 458 | 150 / 148 | 14 / 25 |

The first column is the `voxelize` call, which only queues the kernels; the GPU's own time shows up in the first read. The buffer-only readback reads the two counters and then only as many vertices, normals and quads as the contour wrote.

**Mesh post-processing, Uniform Burst, in ms**

| Stage | 64³ | 128³ | 256³ |
|---|---:|---:|---:|
| Split vertices | 1.2 | 5.3 | 23.5 |
| Recalculate bounds (Split) | 0.1 | 0.4 | 1.7 |
| Reorder vertex buffer (Shared / Split) | 0.3 / 0.9 | 1.2 / 3.6 | 4.9 / 16.7 |

Adaptive and Manifold meshes are a tenth the size or less, so their post-processing stays under 0.5 ms.

**A remesh with every debug flag on, Split normals, in ms**

| Cells or leaves | Uniform Burst | Adaptive Burst | Manifold Burst | Uniform GPU (`convert`) | Uniform Managed | Adaptive Managed | Manifold Managed |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 16³ | 6.5 | 5.2 | 3.7 | 62 | 166 | 126 | 120 |
| 32³ | 56 | 6.3 | 4.9 | 443 | 778 | 721 | 741 |
| 64³ | 362 | 12.9 | 11.5 | 4,956 | 6,355 | 6,056 | 6,321 |
| 128³ | not run | 43.7 | 43.7 | not run | | | |

- Uniform's debug data is every cell with its corners and edges, which is what costs. The octree variants keep theirs to the nodes that were kept.
- The capture into the overlay's arrays is part of these totals: 71 ms of Uniform Burst at 64³, 1,281 ms of GPU at 64³ (its edges and corners are de-duplicated lazily, during the capture).
- Drawing the overlay every frame was not measured. At 64³ Uniform it is 275,000 corners; step 4 looks at it.

**Unity in the background, Burst, debug off, Shared, in ms**

| Cells or leaves | Uniform, in front | Uniform, in the background | Adaptive, in front | Adaptive, in the background |
|---:|---:|---:|---:|---:|
| 16³ | 0.5 | 7.6 | 0.9 | 8.1 |
| 32³ | 1.4 | 24.6 | 1.9 | 33.8 |
| 64³ | 5.1 | 88.7 | 6.1 | 106.9 |
| 128³ | 25.5 | 440.9 | 29.4 | 511.4 |
| 256³ | 160 | 2,444 | | |

**GPU with its buffers kept, debug off, in ms** (step 1; Shared / Split, Unity in front, medians of 5 after a warm-up)

| Cells | Remesh | Before step 1 | `voxelize` call | GPU work, until the first read returns |
|---:|---:|---:|---:|---:|
| 16³ | 2.7 / 2.6 | 3.0 / 3.0 | 0.07 | 2.1 / 2.0 |
| 32³ | 8.7 / 7.3 | 11.5 / 11.9 | 0.08 | 7.3 / 5.8 |
| 64³ | 28 / 30 | 75 / 81 | 0.07 | 23 / 24 |
| 128³ | 137 / 140 | 953 / 636 | 0.08 | 122 / 119 |

- The GPU's own time went down as well, by about a fifth: it no longer copies the uploads.
- The first contour at a new resolution still makes the grid buffers. The warm-up run absorbs that here, and it no longer uploads anything into them.
- GPU is still slower than Burst at every size measured, by 4 to 5 times at 64³ and 128³. The shader's `numthreads( 1, 1, 1 )` is the likely reason, and stays out of scope.
