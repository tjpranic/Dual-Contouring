# Managed Implementations

Plan written 2026-11-11 for moving the managed implementation of each dual contouring algorithm out of its component file, the way the Burst implementations already are.

**Status:** steps 0 to 3 done on 2026-11-11, step 4 next. The decisions were made on 2026-11-11.

## Goal

Each algorithm's component file holds its managed implementation, and Uniform's also holds the GPU one. The Burst code lives apart from them, in `Assets/Scripts/Burst/`, as partial classes next to its own support types. The managed code moves out the same way, into `Assets/Scripts/Managed/`. Afterwards a component file holds only what all of its implementations share.

This is a move, not a rewrite. Nothing changes behaviour: every mesh, test and scene stays exactly as it is.

## Decisions

Made by the user on 2026-11-11, each as recommended:

1. **GPU code:** Uniform's GPU implementation moves out too, into `Assets/Scripts/GPU/`. Otherwise `UniformDualContouring.cs` would keep about 350 lines of GPU code (the implementation and the readback) and be the only component file that still holds implementation code.
2. **Shared files with managed parts:** `SurfaceExtractor`, `QEFSolver` and `DensityFunction` are part shared and part managed. They become partial interfaces, with the managed members in `Managed/<Name>.Managed.cs`. C# allows partial interfaces, so calls such as `SurfaceExtractor.calculateDensity( … )` stay as they are.
3. **Names:** the managed classes keep their names (`SVDQEF`, `SimpleQEF`, `Octree`, `OctreeTopology`, `OctreeContouringTables`), and the folder says they are managed. The Burst types carry a `Burst` prefix because they sit beside the managed types they port. A `Managed` prefix would touch most of the test files and gain nothing else.

## What exists today

| Component file | Lines | Managed only | GPU only | Shared |
|---|---|---|---|---|
| `UniformDualContouring.cs` | 999 | `ManagedImplementation` (about 270) | `GPUImplementation` and `convert`, the readback (about 350) | debug classes `Corner`, `Edge`, `Voxel`, `Quad` (about 290), dispatch, lifetime, inspector |
| `AdaptiveDualContouring.cs` | 682 | debug classes, octree, markers, `voxelizeManaged` and contouring (about 610) | none | settings, dispatch, inspector |
| `ManifoldDualContouring.cs` | 949 | debug classes with `Vertex` and `Triangle`, octree, markers, `voxelizeManaged`, clustering and contouring (about 880) | none | settings, dispatch, inspector |

- **Managed only:** `SVDQEF.cs`, `SimpleQEF.cs`, `Octree.cs`, `OctreeTopology.cs`, `OctreeContouringTables.cs`.
- **Mixed:**
  - `SurfaceExtractor.cs`: the interfaces, the enums, `createMesh` (managed and the GPU readback) and `hashOf` (managed and Burst debug data) are shared. `calculateMaterial`, `calculateDensity`, `approximateIntersection`, `calculateNormal`, `crossesMaterialBoundary`, `calculateMaterialBoundary` and `findHighestMaterialBit`, about 165 lines, are managed only; `BurstSurfaceExtractor` is their Burst port.
  - `QEFSolver.cs`: `QEFSolver.Type` is shared, as the voxelizer's saved setting and the Burst solver switch. The rest, the solver interface, `placeVertex` and `surfaceCorrection`, is managed.
  - `DensityFunction.cs`: the interface and its enums are shared. `Snapshot` and `snapshot( )` are managed; Burst has `BurstDensityFunction.snapshot`. `Data` is the GPU's buffer layout.
- **Shared, staying where they are:** `Voxelizer`, `Volume`, `Cuboid`, `Ellipsoid`, `Either`, `Extensions`, `CameraController`, `FPSCounter`, `Flashlight`, and `DualMarchingCubes`, whose table the managed code reads and the GPU implementation uploads.

### Findings that shape the work

1. **Component files stay put.** Unity ties a MonoBehaviour to the file named after it, and 60 scenes and 3 presets refer to the three component scripts by GUID (Uniform 20 and 1, Adaptive 18 and 1, Manifold 22 and 1). The three files keep their name, folder and `.meta`; only code leaves them. No scene or preset changes.
2. **Serialized fields stay in the component files.** Unity orders the inspector by field declaration, and across partial files that order depends on the order the compiler sees the files in. Every serialized field is in a component file today and stays there.
3. **Burst calls no managed code.** The Burst files share only interfaces and enums with the managed code (`QEFSolver.Type`, `DensityFunction.Combination`, `SurfaceExtractor.MaterialIndex`) and name managed functions only in comments. No Burst file changes.
4. **Tests need no changes.** They name managed types, such as `AdaptiveDualContouring.Voxel` as an octree type argument, `OctreeTopology`, `DualMarchingCubes` and the QEF classes. Every one of those names stays the same.
5. **Uniform's debug classes are shared.** The GPU readback builds the same `Corner`, `Edge`, `Voxel` and `Quad` the managed implementation does, through their `Data` structs, so they stay in `UniformDualContouring.cs`. In Adaptive and Manifold the same classes are managed only and move.
6. **Rule:** code that only one implementation uses goes into that implementation's folder. Code that two or more use stays at the root.

## Layout after the move

```
Assets/Scripts/
  UniformDualContouring.cs            settings, dispatch, lifetime, shared debug classes, inspector
  AdaptiveDualContouring.cs           settings, dispatch, inspector
  ManifoldDualContouring.cs           settings, dispatch, inspector
  Voxelizer.cs  SurfaceExtractor.cs  QEFSolver.cs  DensityFunction.cs  DualMarchingCubes.cs
  Volume.cs  Cuboid.cs  Ellipsoid.cs  Either.cs  Extensions.cs
  CameraController.cs  FPSCounter.cs  Flashlight.cs
  Managed/
    UniformDualContouring.Managed.cs     ManagedImplementation
    AdaptiveDualContouring.Managed.cs    debug classes, octree, voxelizeManaged, contouring
    ManifoldDualContouring.Managed.cs    debug classes, octree, voxelizeManaged, clustering, contouring
    SurfaceExtractor.Managed.cs          density, intersection, normal and material boundary functions
    QEFSolver.Managed.cs                 solver interface, placeVertex, surfaceCorrection
    DensityFunction.Managed.cs           Snapshot
    SVDQEF.cs  SimpleQEF.cs  Octree.cs  OctreeTopology.cs  OctreeContouringTables.cs
  Burst/                                unchanged
  GPU/
    UniformDualContouring.GPU.cs         GPUImplementation, convert
    DensityFunction.GPU.cs               Data
```

- The root `QEFSolver.cs` keeps only `QEFSolver.Type`.
- The component files go from 999, 682 and 949 lines to about 380, 70 and 70.
- The compute shaders stay in `Assets/Resources/Shaders/`, because the GPU implementation loads them with `Resources.Load( "Shaders/UniformDualContouring" )`.
- Each new file brings its own `using` aliases, and the component files lose the aliases they no longer need.
- Profiler marker names (`<Algorithm>DualContouring.Managed.<Stage>`) stay the same.

## Steps

Each step compiles, passes every test, gives identical output, and is reviewed before the next one starts.

0. **Baseline (small). Done 2026-11-11.**
   - **Tests:** edit mode 513/513, play mode 20/20.
   - **Compile:** a forced clean compile (`CleanBuildCache`) gave 0 errors and 0 warnings.
   - **Probe:** voxelizes each algorithm with each of its implementations and hashes the mesh (vertices, normals, each sub-mesh's indices), the mesh as a set of triangles, and the corners, edges and voxels of the debug data.
     - It covers 7 models: the 4 test models, the slab with a band, a cube corner and a sphere-cube intersection. These come with both solvers, both intersection approximations and surface correction on and off.
     - It also covers vertex normals for the GPU and three simplification settings each for Adaptive and Manifold, on grids of 16³ and 32³.
     - The full set is 1,344 configurations (Uniform 336, Adaptive 504, Manifold 504) and takes about 10 minutes.
     - A quick set of 1,134 keeps every configuration except the 32³ managed octree runs other than SVD with binary search and no surface correction. It takes about 5 minutes, and the steps below use it.
   - **Reproducible:** a second run matched the first in every hash but one: the GPU's vertex and triangle order. The GPU appends both with atomic counters, so their order changes between runs. GPU meshes are therefore compared as sets of triangles, which still hold every position and normal bit for bit. Everything else is compared in order.
   - **Tools:** the probe (run through `unity command script run`, compiled in memory, nothing added to the project), the moved-lines checker and the comparison script are in the session's scratchpad, together with the baseline results.
1. **Managed code (medium). Done 2026-11-11.**
   - **Result:**
     - The component files went from 999, 682 and 949 lines to 730, 56 and 53. Uniform's file still holds the GPU code until step 3.
     - The new files are `UniformDualContouring.Managed.cs` (285 lines), `AdaptiveDualContouring.Managed.cs` (634) and `ManifoldDualContouring.Managed.cs` (904).
     - The moved-lines check found 3,091 lines moved unchanged. Everything else that changed is `using` lines, the three `partial` declarations, braces and blank lines.
     - The compile gave 0 errors and 0 warnings. Tests: edit mode 513/513, play mode 20/20. All 1,134 probe configurations match the baseline.
     - The five whole files kept their GUIDs, and the three component scripts still resolve to their classes. No scene, preset or other asset changed.
   - **Method:** a script cut the line ranges, found by marker lines, so nothing was retyped. The folder was created through the asset database first, so it got its `.meta`. The five moves went in one `StartAssetEditing` batch, followed by a refresh, so the editor compiled once.
   - Create `Assets/Scripts/Managed/` and move the five managed-only files into it, unchanged. The moves go through the asset database in one batch, as the scene moves did, so each `.meta` and GUID moves with its file and the editor recompiles once.
   - Uniform: `ManagedImplementation` moves to `Managed/UniformDualContouring.Managed.cs`, the way `BurstImplementation` sits in `Burst/UniformDualContouring.Burst.cs`.
   - Adaptive: the debug classes, the `octree` field, the profiler markers, `voxelizeManaged`, `contourCell`, `contourFace`, `contourEdge` and `generateIndices` move to `Managed/AdaptiveDualContouring.Managed.cs`, the way `voxelizeBurst` sits in its Burst file. The `Axis` and `Position` aliases go with them.
   - Manifold: the same, plus `Vertex`, `Triangle` and the lookup helpers, to `Managed/ManifoldDualContouring.Managed.cs`.
2. **Shared files (small). Done 2026-11-11.** `SurfaceExtractor`, `QEFSolver` and `DensityFunction` become partial interfaces, and their managed members move to `Managed/`.
   - **Result:**
     - `SurfaceExtractor.cs` went from 342 to 173 lines; `calculateMaterial` to `findHighestMaterialBit` moved to `Managed/SurfaceExtractor.Managed.cs`.
     - `QEFSolver.cs` went from 48 to 7 lines and keeps only `QEFSolver.Type`.
     - `DensityFunction.cs` went from 85 to 46 lines; `Snapshot` and `snapshot( )` moved. `Data` stays there until step 3.
     - The moved-lines check found 249 lines moved unchanged, with only headers besides.
     - The compile gave 0 errors and 0 warnings. Tests: edit mode 513/513, play mode 20/20. All 1,134 probe configurations match the baseline. No asset changed.
     - No call site changed.
3. **GPU (small). Done 2026-11-11.** `GPUImplementation` and `convert` move to `GPU/UniformDualContouring.GPU.cs`, and `DensityFunction.Data` to `GPU/DensityFunction.GPU.cs`.
   - **Result:**
     - `UniformDualContouring.cs` went from 730 to 372 lines and now holds the shared debug classes, dispatch, lifetime and the inspector. `GPU/UniformDualContouring.GPU.cs` has 368 lines.
     - `DensityFunction.cs` went from 46 to 28 lines.
     - The moved-lines check found 376 lines moved unchanged, with only headers besides.
     - The compile gave 0 errors and 0 warnings. Tests: edit mode 513/513, play mode 20/20. All 1,134 probe configurations match the baseline, the 112 GPU ones as sets of triangles.
     - The compute shader still loads from `Resources`. No asset changed.
4. **Documents (small).** `README.md` gets a short code layout section naming the three folders and the rule. This plan is marked complete.

## Checks

Every step passes these:

- **Moved, not changed:** a script compares the lines a step removes with the lines it adds. Every removed line must come back unchanged. The only new lines allowed are file headers: `using` lines, the `partial` declaration and its braces. `git diff --color-moved` shows the same thing by eye.
- **Compiles cleanly:** no errors, and no warnings the baseline didn't have.
- **Tests:** both suites, with the baseline counts.
- **Output:** the probe's hashes match the baseline bit for bit, with GPU meshes compared as sets of triangles (see step 0). The code is unchanged, so any difference is a mistake.
- **Assets:** `git status` shows no `.unity`, `.preset` or `.asset` change, and no `.meta` change other than for moved and new files.

Notes:

- No scene is opened.
- Once the project has Burst jobs, a recompile can run past `unity recompile`'s 120 s default; pass `--timeout 300`.
- **History:** git records a whole-file move as a rename when it's committed without edits, so the step 1 file moves are best committed on their own. Code split out of a component file shows in `git blame` of its new file as the commit that moved it; `git blame -C -C` traces it further back.

## Out of scope

- **Dispatch:** Uniform selects its implementation through the `Implementation` interface, while Adaptive and Manifold switch inside `voxelize` and their `implementation` property throws. Making them alike changes code, not where it lives.
- **`Implementation.CPU.Voxelization`:** the name means the CPU-side result, which Managed and Burst both return. It stays.
- **Tests:** the test folders stay as they are. The Burst tests have no folder of their own either.
- **Editor code (found while planning):** the inspector classes, `VoxelizerEditor` and the three algorithm editors, are in GameAssembly, which builds for every platform. They use `UnityEditor` with no `#if UNITY_EDITOR`, so a player build would fail to compile (not tried). The user decided on 2026-11-11 to leave them as they are unless they become an actual problem. If one comes up, it goes to the user before anything is done about it.
