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

Example scenes for each implementation are in `Assets/Scenes/<Algorithm>/Managed` and `Assets/Scenes/<Algorithm>/Burst`. For Uniform Dual Contouring those two folders are under `CPU`, next to `GPU`.

## Code layout
Each algorithm is one component, such as `AdaptiveDualContouring`. Its class is split into partial class files: the component file, plus one file for each implementation.

| Folder | Contents |
|---|---|
| `Assets/Scripts` | The components, with their settings, the switch between implementations and their inspectors. Also everything the implementations share: the voxelizer, the volumes, the density function and surface extractor interfaces, and the dual marching cubes table. |
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