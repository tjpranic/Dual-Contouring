# Dual Contouring Unity Project

A Unity implementation of various **Dual Contouring** algorithms for mesh generation from implicit surfaces. This project includes multiple dual contouring variants with both CPU and GPU implementations.

## Overview

Dual Contouring is a technique for generating meshes from volumetric data represented by implicit functions. Unlike Marching Cubes, Dual Contouring preserves sharp features and produces better-quality meshes by solving a Quadratic Error Function (QEF) to find optimal vertex positions.

This Unity project implements three main variants of the dual contouring algorithm:

- **Uniform Dual Contouring** - Regular grid-based implementation
- **Adaptive Dual Contouring** - Octree-based implementation with level-of-detail support
- **Manifold Dual Contouring** - Specialized version that ensures manifold mesh topology

## Requirements

- **Unity 2022.3.40f1** or later
- **Unity Mathematics Package** (1.2.6)
- Compute Shader support for GPU implementations

## Getting Started

### Usage

1. Open the project in Unity
2. Navigate to the `Assets/Scenes` folder and open a test scene
3. Create a new GameObject and add one of the dual contouring components:
   - `UniformDualContouring`
   - `AdaptiveDualContouring`
   - `ManifoldDualContouring`

## Implementation Details

### Uniform Dual Contouring
- Uses a regular 3D grid subdivision
- Supports both CPU and GPU implementations
- GPU version uses compute shaders for parallel processing
- Best for uniform detail requirements

### Adaptive Dual Contouring
- Uses octree subdivision for level-of-detail
- Optional mesh simplification based on error thresholds
- CPU-only implementation
- Best for scenes with varying detail requirements

### Manifold Dual Contouring
- Ensures manifold mesh topology
- Specialized vertex clustering to prevent non-manifold edges
- CPU-only implementation
- Best when topological correctness is critical

### QEF Solvers
- **Simple QEF**: Fast iterative solver
- **SVD QEF**: More accurate Singular Value Decomposition solver

## License

The MIT License (MIT)

Copyright (c) 2025 Thomas Pranic

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

---

This project implements academic algorithms for dual contouring. Refer to the original papers for algorithmic details:

- *Dual Contouring of Hermite Data* by Ju et al.
- *Manifold Dual Contouring* by Schaefer et al.