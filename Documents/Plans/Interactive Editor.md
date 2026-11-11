# Interactive Editor

Plan written 2026-11-11 for an editor in which shapes are added and combined with CSG operations, and the contour follows every change, in edit mode.

**Status:** planned. The five decisions below were made on 2026-11-11. No code written.

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
5. **Burst and GPU variants remesh automatically, during a drag as well.** While a slider or handle is being dragged they remesh at most once per debounce interval, as long as the last contour was fast. Otherwise they remesh when the changes pause.

## What exists today

- **Stub scene:** `Interactive Editor.unity` holds a camera with `CameraController`, a directional light and the FPS counter canvas. It is the same as `Empty.unity`. Both are untracked.
- **Voxelizers:** `UniformDualContouring`, `AdaptiveDualContouring` and `ManifoldDualContouring` derive from `Voxelizer`. `Voxelizer.Start` contours once: it collects the `Volume` components below it, calls `voxelize`, post-processes the mesh, adds a `MeshFilter` and captures debug data.
- **Shapes:** `Cuboid` and `Ellipsoid` derive from `Volume`. Position and size come from the transform; `combination` (Union, Intersection, Subtraction) and `materialIndex` are fields.
- **CSG:** a fold over the shapes in hierarchy order, starting from empty space. Order matters, and a first shape that is not a Union gives nothing.
- **Materials:** sub-mesh n is material bit n, drawn with the mesh renderer's n-th material. A new `Volume` has material `Void`, which never produces a surface.
- **Grid:** the voxelizer's object space from -0.5 to 0.5 on each axis. The Resolution slider goes from 2 to 8. Uniform has 2^resolution cells across (4³ to 256³), the octrees 2^(resolution - 1) leaves (2³ to 128³).

**Voxelize times from the Burst plan's measurements, in ms** (three balls, SVD, editor):

| Cells or leaves | Burst | Managed |
|---:|---:|---:|
| 16³ | 0.5 – 1.2 | 105 – 120 |
| 32³ | 1.3 – 2.6 | 680 – 760 |
| 64³ | 5.3 – 7.0 | 4,800 – 5,400 |
| 128³ | 24 – 29 | 39,000 – 57,000 |
| 256³ (Uniform only) | 153 | not possible, about 80 GB |

These are the `voxelize` call alone. Split normals, vertex reordering, debug capture and the GPU path in edit mode have not been measured; step 0 does that.

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

- **Signature:** a hash of the selected algorithm, the selected voxelizer's contour settings, and for each active shape in order its type, position, size, operation and material.
- **Polling:** worked out on every editor tick. It costs a handful of reads per shape.
- **What it ignores:** names, debug colours and sizes, the renderer's materials, and the root's own transform, which moves the mesh without changing it.
- **Debug flags** are part of the signature, because the Burst implementations only copy debug data out when a flag is on.
- **Managed variants** use the same signature, only to show that the mesh is out of date.

### Scheduling

A small class with no Unity dependencies, so tests can drive it with a fake clock.

- **Managed:** never contours by itself. "Contour" is the only trigger.
- **Burst and GPU:**
  - **Debounce:** contour once the signature has been stable for the interval (100 ms to start with; step 0 sets the default).
  - **During a drag:** while the signature keeps changing, contour at most once per interval if the last contour took under about 30 ms. Burst stays under that up to 128³. A slower contour waits for the pause.
- **Blocking:** a contour runs to completion on the main thread.
- **GPU caveat:** the GPU path has no edit-mode timing yet. If step 0 shows it taking seconds at the higher resolutions, that comes back to the user before step 3.

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

0. **Measure (small).** No product code.
   - Time a full remesh in edit mode for every variant at resolutions 4 to 8, as far as each can go: `voxelize`, split normals, vertex reordering and debug capture, separately.
   - Time the GPU path with `convert` and with a buffer-only readback.
   - Outcome: the default debounce, the "fast" limit for remeshing during a drag, how the GPU behaves at each resolution, and whether vertex reordering is worth keeping in the editor.
1. **Re-runnable voxelizer (medium).** The changes to `Voxelizer` above.
   - Tests first: a second contour gives the same mesh, leaves one mesh filter and no stray mesh; the buffer-only GPU readback matches `convert` as a set of triangles; a Uniform voxelizer contours without `Start`.
   - Verified by both suites (513 edit mode, 20 play mode) and a mesh-hash comparison of the existing scenes before and after.
2. **Editor component and scene (medium).** `InteractiveEditor` with algorithm selection, lifetime, shape conversion and the "Contour" button. No automatic remeshing yet.
   - Scene: the root object with its components and materials, and a starter model.
   - Tests: one voxelizer active at a time; common settings carry over and GPU falls back to Burst; shape conversion under a scaled root and inside a group; every variant contours the starter model in edit mode; the scene is unchanged after contouring and holds no mesh data.
   - By hand: the mesh is still there after a script recompile.
3. **Automatic remeshing (medium).** Signature, polling, scheduler.
   - Tests: the signature changes for each input that should change it and for none that should not; scheduler cases with a fake clock (a burst of changes gives one contour, continuous change gives one per interval when fast and none until the pause when slow, Managed never contours by itself, "Contour" always runs); undo and redo trigger a remesh.
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
