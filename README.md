# Dual Contouring Unity Project
A Unity implementation of various **Dual Contouring** algorithms for mesh generation from implicit surfaces. This project includes multiple dual contouring variants with managed C#, Burst and GPU implementations.

## Overview
Dual Contouring is a technique for generating meshes from volumetric data represented by implicit functions. Unlike Marching Cubes, Dual Contouring preserves sharp features and produces better-quality meshes by solving a Quadratic Error Function (QEF) to find optimal vertex positions.

This Unity project implements four main variants of the dual contouring algorithm:

- **Uniform Dual Contouring** - Regular grid-based implementation
- **Adaptive Dual Contouring** - Octree-based implementation with level-of-detail support
- **Manifold Dual Contouring** - Specialized version that ensures manifold mesh topology
- **Occupancy-Based Dual Contouring** - Manifold version on a regular grid that only asks whether a point is inside or outside

## Implementations
Each voxelizer component has an **Implementation** setting that selects how it contours.

| Algorithm | Managed | Burst | GPU |
|---|---|---|---|
| Uniform Dual Contouring | Yes | Yes | Yes |
| Adaptive Dual Contouring | Yes | Yes | No |
| Manifold Dual Contouring | Yes | Yes | No |
| Occupancy-Based Dual Contouring | No | Yes | Yes |

- **Managed** is plain C#, written to follow the papers. It is the reference the other two are tested against, and it is slow: a grid of 128³ cells takes most of a minute.
- **Burst** runs the same algorithm as jobs compiled by the Burst compiler, on all cores. It contours 128³ cells in about 28 to 33 ms in the editor, which is over a thousand times faster than Managed, and supports both QEF solvers. Occupancy-Based Dual Contouring samples the volumes several times as often and takes about 52 ms.
- **GPU** runs Uniform or Occupancy-Based Dual Contouring in a compute shader, with the SVD solver only. Contouring 128³ cells and reading the mesh back takes about 133 ms for Uniform and 32 ms for Occupancy-Based Dual Contouring, whose searches suit a GPU and whose shader keeps far less for each cell.

Burst gives the same mesh as Managed, triangle for triangle, with vertices that differ in the last bits of their coordinates. Where octree simplification or vertex clustering decides right at the error threshold, the two can collapse different nodes.

Occupancy-Based Dual Contouring has no managed implementation. Its tests hold it to answers known in closed form, and its two implementations to each other: the GPU's vertices are within 0.02 of a cell of Burst's, and every polygon is split the same way.

Volumes are placed by their transforms: position, rotation and scale. Every implementation contours turned shapes. A turned shape costs a little more to sample than an unturned one: Burst contours a model whose shapes are all turned 6 to 11% slower.

## Shapes
There are nine kinds of volume, each its own component. Each is made from the box its transform gives it: centred on the box, with its axis along the box's own y.

| Shape | Made from its box |
|---|---|
| `Cuboid` | The box itself. |
| `Ellipsoid` | The ellipsoid that touches the middle of each face of the box. |
| `Cylinder` | Standing along y, as high as the box. |
| `Cone` | Its base fills the bottom of the box, and its tip is in the middle of the top. |
| `Capsule` | A cylinder along y with a half ball on each end, as high as the box. A box lower than it is wide gives a ball. |
| `Torus` | Lying across x and z, with a tube as thick as the box is high. |
| `Pyramid` | Its square base fills the bottom of the box, and its tip is in the middle of the top. |
| `Octahedron` | Its six corners are in the middle of the box's six faces. |
| `HexagonalPrism` | Standing along y, the largest regular hexagon that fits across x and z, with two corners on the x axis. |

A cylinder, cone, capsule or torus is round across x and z, and a pyramid is square. They take the smaller of the box's x and z sizes for both.

The seven shapes after the ellipsoid come from Inigo Quilez's [distance functions](https://iquilezles.org/articles/distfunctions/). Each is an exact signed distance, checked against distances worked out another way. Two of them differ from the article:
- **Pyramid:** the article's pyramid is off below its base and inside near it; corrected here.
- **Octahedron:** the article's exact one needs all three sizes equal; this one is exact for any box.

Knowing the new shapes makes the Burst and GPU implementations slower over cuboids and ellipsoids: about 8 to 15% for Burst and 11% for the GPU.

Example scenes for each implementation are in `Assets/Scenes/<Algorithm>/Managed` and `Assets/Scenes/<Algorithm>/Burst`. For Uniform Dual Contouring those two folders are under `CPU`, next to `GPU`. Occupancy-Based Dual Contouring has `Burst` and `GPU`.

## Wireframe
Every voxelizer can draw the edges of its contour's triangles over the contour. Its inspector has a **Wireframe** section:

- **Wireframe** is Off, **Over Surface** or **Lines Only**.
  - **Over Surface** draws the lines over the shaded surface.
  - **Lines Only** draws them over a flat fill, which hides the surface and the lines behind it. Each material's lines are in that material's colour.
- **Wireframe Line Width** is in pixels, and stays the same at any distance.
- **Wireframe Line Color** is the colour of the lines over the surface. Its alpha blends them.
- **Wireframe Fill Color** is the fill under Lines Only.

**How the lines are drawn:**
- **Edges:** the lines are the edges of the triangles, so every quad shows its diagonal.
- **The contour:** it is drawn as before, by its renderer or straight from the GPU, and the lines go over it. Picking, materials and shadows stay as they were; under Lines Only the hidden surface still casts its shadows.
- **Where it works:** every implementation, in play mode, and in the interactive editor in edit mode as well.
- **No contouring:** the settings change only what is drawn, so changing them never contours again.

**Cost:** on the three balls at 1920 × 1080, drawing the lines adds about 0.1 ms to a frame of 418,000 triangles from Burst, and about 0.2 ms from the GPU.

**Dense meshes:** seen whole, they fill up with lines. With the grid 600 pixels across, a cell at 256³ is two or three pixels wide.

## Occupancy-Based Dual Contouring
`OccupancyDualContouring` follows *Occupancy-Based Dual Contouring* (Hwang and Sung, 2024). It is Manifold Dual Contouring on a uniform grid, without simplification, and it only ever asks the volumes whether a point is inside or outside, never how far away the surface is. A point is inside where the volumes give it a material.

- **Edges:** where the contour crosses an edge of the grid is found by halving the edge.
- **Faces:** on every face of a cell that the contour crosses, three searches find the point where the surface's two straight pieces would meet on that face.
- **Vertices:** the plane through a point on an edge and the two points on the faces next to it takes the place of the gradient. The vertex of each piece of surface in a cell is the minimizer of the QEF of those planes.
- **Polygons:** each is split along the diagonal that keeps its triangles from passing through those of other polygons, or into four triangles around the point on its edge.
- **Manifold:** where two cells meet at a face that both would join their surface across, both take the other way of joining it, so every edge of the mesh has two triangles.

Its settings, beside those every voxelizer has:

| Setting | Default | What it does |
|---|---|---|
| Edge Search Iterations | 15 | How many times an edge is halved. |
| Face Search Samples | 4 | How many samples a search across a face takes to find where it first crosses the contour. |
| Face Search Iterations | 11 | How many times it then halves the step it found the contour in. |
| Keep Vertices In Cells | On | Moves a vertex that the QEF places outside its cell back into it. |
| Split Quads | On | Splits polygons so that triangles do not pass through each other. Off, always along the same diagonal, as the other algorithms do. |

Binary Search Iterations, Intersection Approximation and Surface Correction Iterations place points by distance. This algorithm does not use them, and its inspector does not show them.

How it compares with Manifold Dual Contouring on the same grid, measured on single shapes at 16³ cells:

- **Curved surfaces:** its vertices are ten to twenty times closer to the shape. On a ball they are 0.0016 of a cell off on average, against 0.039.
- **Flat faces, and edges that every face of the grid sees bend once:** exact, as on an upright cuboid.
- **Corners:** where a face of the grid sees three faces of a shape, the search on that face goes wrong, and the corner can be missed by up to 0.44 of a cell. Manifold Dual Contouring has a vertex within 0.03 of a cell of the corners its tests look at.
- **Keep Vertices In Cells** decides between two faults:
  - **On,** no triangles passed through each other on 27 test models at 16³ and 32³ cells, and four pairs did at 64³. Edges of turned shapes are blunted where they pass just outside a cell: one vertex in six to ten is moved, by up to 0.4 of a cell.
  - **Off,** edges are as sharp as the planes give them, and triangles pass through each other as they do in Manifold Dual Contouring: 436 pairs on the same models at 16³ and 1,849 at 64³. Split Quads then does nothing.
- **Materials:** each material has its sub-mesh, but vertices are not drawn to where two materials meet on the surface, so that boundary follows the grid.
- **Subtractions and materials:** a Subtraction takes away only the materials it names. Where one passes through a shape of another material, this algorithm leaves that shape whole. The other algorithms dent it there: they keep its triangles, but place their vertices by a density that every subtraction is taken out of. To cut a shape with any algorithm, give the Subtraction that shape's material as well.

## Interactive editor
`Assets/Scenes/Interactive Editor.unity` lets you build a model out of shapes and see it contoured as you edit, in edit mode, without pressing Play.

The **Interactive Editor** object holds the editor and a voxelizer for each algorithm. In its inspector:

- **Algorithm** picks Uniform, Adaptive, Manifold or Occupancy. The settings all voxelizers share, such as resolution, implementation, solver and wireframe, carry over, so the same model is compared like for like. Resolution carries over as the number of cells across. An implementation the new algorithm does not have becomes Burst. The selected voxelizer's own settings, such as the Implementation, appear below the editor.
- **Shapes** lists every shape below the object, in the order their operations apply. Each row has an enabled box, the shape's name, which selects it, its operation (Union, Intersection or Subtraction) and material, and a remove button. Drag rows to reorder them. The **Add** buttons, one for each kind of shape, add a new one at the centre of the grid. Everything can be undone.
- **Contour** contours at once. The line below it says whether the mesh is up to date, and how long the last contour took.
- **Warnings** point out likely mistakes: a first shape that is not a Union, a shape without a material or outside the grid, a material the renderer has no material for, a shape a group would shear, a round or square shape whose x and z sizes differ, a debug view that draws so much it slows the editor down.

Shapes are read off their transforms the way Unity shows them: where they are, how they are turned and how large they are. A direct child of scale 1 fills the grid, whatever the scale of the editor object, and the box around the editor in the scene view is the grid. Groups, empty objects between the editor and its shapes, move, turn and scale the shapes in them.

One arrangement cannot always be contoured as Unity shows it. A shape turned inside a group that is scaled unevenly can come out with slanted sides, and a shape is always made from a box with square sides. Such a shape is contoured with square sides instead, and the inspector warns about it. A group scaled the same along every axis never slants anything, and neither does an uneven scale along the axes a shape is turned about or by quarter turns.

In the scene view, an outline shows each shape where and as it is contoured, so subtracted shapes can be seen and clicked. Outlines are green for Union, yellow for Intersection and red for Subtraction, and dimmer for shapes that are not selected.
- **Thickness:** **Outline Thickness** sets how thick they are, in pixels.
- **Depth:** outlines are hidden where the mesh is in front of them. They are drawn half a cell nearer the camera than they are, so that the mesh, which is only near the shapes, does not cut through them.
- **Outlines On Top:** draws them over everything instead. A click on an outline where the mesh is in front of it then still selects the mesh.

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
| `Assets/Scripts` | The components, with their settings, the switch between implementations and their inspectors. Also everything the implementations share: the voxelizer, the volumes, the density function and surface extractor interfaces, and the dual marching cubes table. `DensityFunction.Frame` works out once how a shape is turned, and all three implementations take positions into the shape's frame with the numbers it gives. The nine volume components (`Cuboid.cs` to `HexagonalPrism.cs`), each with its distance function, are here, and `DensityFunction.sample` chooses between them for the managed code. The interactive editor (`InteractiveEditor.cs`, with its inspector and shape outlines) and the scheduler that decides when it contours (`RemeshScheduler.cs`) are here too, as is the wireframe (`Wireframe.cs`), whose shader is `Assets/Resources/Shaders/RenderContourWireframe.shader`. |
| `Assets/Scripts/Managed` | The managed implementations (`<Algorithm>.Managed.cs`) and what only they use: the QEF solvers, the octree with its tables and topology checks, and the density, intersection and normal functions. |
| `Assets/Scripts/Burst` | The Burst implementations (`<Algorithm>.Burst.cs`) and the Burst code they share: ports of the QEF solvers, density functions, tables and topology checks, and the native grid, octree and mesh data. The searches of Occupancy-Based Dual Contouring are in `BurstOccupancy.cs`. |
| `Assets/Scripts/GPU` | The GPU implementations of Uniform and Occupancy-Based Dual Contouring (`<Algorithm>.GPU.cs`), the mesh readback they share and the layout they upload density functions in. The compute shaders are in `Assets/Resources/Shaders`, one for each algorithm, with the shape functions both use in `DensityFunctions.hlsl`. |

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
- *Occupancy-Based Dual Contouring* by Hwang and Sung.