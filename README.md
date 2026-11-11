# Dual Contouring Unity Project
A Unity implementation of various **Dual Contouring** algorithms for mesh generation from implicit surfaces. This project includes multiple dual contouring variants with managed C#, Burst and GPU implementations.

## Overview
Dual Contouring is a technique for generating meshes from volumetric data represented by implicit functions. Unlike Marching Cubes, Dual Contouring preserves sharp features and produces better-quality meshes by solving a Quadratic Error Function (QEF) to find optimal vertex positions.

This Unity project implements three main variants of the dual contouring algorithm:

- **Uniform Dual Contouring** - Regular grid-based implementation
- **Adaptive Dual Contouring** - Octree-based implementation with level-of-detail support
- **Manifold Dual Contouring** - Specialized version that ensures manifold mesh topology

## Implementations
Each voxelizer component has an **Implementation** setting that selects how it contours.

| Algorithm | Managed | Burst | GPU |
|---|---|---|---|
| Uniform Dual Contouring | Yes | Yes | Yes |
| Adaptive Dual Contouring | Yes | Yes | No |
| Manifold Dual Contouring | Yes | Yes | No |

- **Managed** is plain C#, written to follow the papers. It is the reference the other two are tested against, and it is slow: a grid of 128³ cells takes most of a minute.
- **Burst** runs the same algorithm as jobs compiled by the Burst compiler, on all cores. It contours 128³ cells in about 25 to 30 ms in the editor, which is over a thousand times faster than Managed, and supports both QEF solvers.
- **GPU** runs Uniform Dual Contouring in a compute shader, with the SVD solver only.

Burst gives the same mesh as Managed, triangle for triangle, with vertices that differ in the last bits of their coordinates. Where octree simplification or vertex clustering decides right at the error threshold, the two can collapse different nodes.

Volumes, the `Cuboid` and `Ellipsoid` components, are placed by their transforms: position, rotation and scale. Every implementation contours turned shapes. A turned shape costs a little more to sample than an unturned one: Burst contours a model whose shapes are all turned 6 to 11% slower.

Example scenes for each implementation are in `Assets/Scenes/<Algorithm>/Managed` and `Assets/Scenes/<Algorithm>/Burst`. For Uniform Dual Contouring those two folders are under `CPU`, next to `GPU`.

## Interactive editor
`Assets/Scenes/Interactive Editor.unity` lets you build a model out of shapes and see it contoured as you edit, in edit mode, without pressing Play.

The **Interactive Editor** object holds the editor and a voxelizer for each algorithm. In its inspector:

- **Algorithm** picks Uniform, Adaptive or Manifold. The settings all voxelizers share, such as resolution, implementation and solver, carry over, so the same model is compared like for like. Resolution carries over as the number of cells across. The selected voxelizer's own settings, such as the Implementation, appear below the editor.
- **Shapes** lists every shape below the object, in the order their operations apply. Each row has an enabled box, the shape's name, which selects it, its operation (Union, Intersection or Subtraction) and material, and a remove button. Drag rows to reorder them. **Add Cuboid** and **Add Ellipsoid** add a new shape. Everything can be undone.
- **Contour** contours at once. The line below it says whether the mesh is up to date, and how long the last contour took.
- **Warnings** point out likely mistakes: a first shape that is not a Union, a shape without a material or outside the grid, a material the renderer has no material for, a shape a group would shear, a debug view that draws so much it slows the editor down.

Shapes are read off their transforms the way Unity shows them: where they are, how they are turned and how large they are. A direct child of scale 1 fills the grid, whatever the scale of the editor object, and the box around the editor in the scene view is the grid. Groups, empty objects between the editor and its shapes, move, turn and scale the shapes in them.

One arrangement cannot always be contoured as Unity shows it. A shape turned inside a group that is scaled unevenly can come out with slanted sides, and a shape is always a box or an ellipsoid. Such a shape is contoured with square sides instead, and the inspector warns about it. A group scaled the same along every axis never slants anything, and neither does an uneven scale along the axes a shape is turned about or by quarter turns.

In the scene view, an outline shows each shape where and as it is contoured, in green for Union, yellow for Intersection and red for Subtraction, so subtracted shapes can be seen and clicked.

How the mesh follows changes depends on the implementation:

| Implementation | Contours by itself |
|---|---|
| Burst, GPU | After every change, from the inspector, a handle (moving, turning or scaling), the hierarchy or an undo. Contours that take up to 50 ms follow a drag live; slower ones wait until the changes settle for the **Debounce** interval. After a contour over a second, it waits for **Contour** to be pressed until a contour is fast again. |
| Managed | Never. Managed contours take from a tenth of a second at 16³ cells to most of a minute at 128³, so it contours when **Contour** is pressed. |

Nothing contours by itself while Unity is not the active application, where everything runs many times slower. The mesh is not saved with the scene, so a Burst or GPU variant contours when the scene opens, and a Managed one waits for **Contour**.

## Code layout
Each algorithm is one component, such as `AdaptiveDualContouring`. Its class is split into partial class files: the component file, plus one file for each implementation.

| Folder | Contents |
|---|---|
| `Assets/Scripts` | The components, with their settings, the switch between implementations and their inspectors. Also everything the implementations share: the voxelizer, the volumes, the density function and surface extractor interfaces, and the dual marching cubes table. `DensityFunction.Frame` works out once how a shape is turned, and all three implementations take positions into the shape's frame with the numbers it gives. The interactive editor (`InteractiveEditor.cs`, with its inspector and shape outlines) and the scheduler that decides when it contours (`RemeshScheduler.cs`) are here too. |
| `Assets/Scripts/Managed` | The managed implementations (`<Algorithm>.Managed.cs`) and what only they use: the QEF solvers, the octree with its tables and topology checks, and the density, intersection and normal functions. |
| `Assets/Scripts/Burst` | The Burst implementations (`<Algorithm>.Burst.cs`) and the Burst code they share: ports of the QEF solvers, density functions, tables and topology checks, and the native grid, octree and mesh data. |
| `Assets/Scripts/GPU` | The GPU implementation of Uniform Dual Contouring (`UniformDualContouring.GPU.cs`) and the layout it uploads density functions in. The compute shaders are in `Assets/Resources/Shaders`. |

Code that only one implementation uses goes in that implementation's folder; code that several of them use stays in `Assets/Scripts`. `SurfaceExtractor`, `QEFSolver` and `DensityFunction` are partial interfaces for the same reason: their managed and GPU members sit in those folders, and their shared members sit in `Assets/Scripts`.

## Requirements
- **Unity 6000.4.5f1** or later
- **Unity Mathematics** (1.3.3)
- **Burst** (1.8.29) and **Unity Collections** (6.4.0) for the Burst implementations
- Compute Shader support for the GPU implementation
- **Shapes** library by Freya Holmer
- **Unity Test Framework** (1.6.0) and **Performance Testing** (3.4.0) to run the tests

## License
The MIT License (MIT)

Copyright (c) 2025 Thomas Pranic

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

---

This project implements academic algorithms for dual contouring. Refer to the original papers for algorithmic details:

- *Dual Contouring of Hermite Data* by Ju et al.
- *Dual Contouring: The Secret Sauce* by Schaefer and Warren.
- *Manifold Dual Contouring* by Schaefer et al.