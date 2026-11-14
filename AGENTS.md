# Project Overview
Dual contouring algorithms implemented in C# in Unity. See README.md for more details.

# Algorithms
Copies of the papers go in Documents/Papers/.

## Dual Contouring
Based off the original dual contouring papers *Dual Contouring of Hermite Data* and *Dual Contouring: The Secret Sauce*.

## Adaptive Dual Contouring
An extension to the original dual contouring algorithm to allow for mesh simplification.

## Manifold Dual Contouring
Based off the *Manifold Dual Contouring* paper; a technique that allows for guaranteed watertight (2-manifold) mesh generation and simplification.

## Occupancy-Based Dual Contouring
Based off the *Occupancy-Based Dual Contouring* paper; manifold dual contouring on a uniform grid that only asks whether a point is inside or outside, with searches in place of distances and gradients. It has Burst and GPU implementations and no managed one, so its tests hold it to answers known in closed form and the two implementations to each other.

The authors' reference code (github.com/KAIST-Visual-AI-Group/ODC) is licensed CC BY-NC-SA 4.0 and this project is MIT. Do not port its code or its tables. Write from the paper, and generate tables from the project's own `DualMarchingCubes`.

## Mesh Simplification
Based off *Surface Simplification Using Quadric Error Metrics*: edge collapses in order of quadric error, as a Burst pass over the finished mesh of any algorithm and implementation (`BurstMeshSimplifier`). Three more papers go into it:
- *Controlling Quadric Error Simplification with Line Quadrics*: a second error term that settles flat regions.
- *Progressive Meshes*: the record of collapses behind the detail setting, and the rules that keep the lines between materials.
- *Topology Preserving Edge Contraction*: which collapses keep the topology, for meshes that are not manifolds too.

It has no managed or GPU implementation, so its tests hold it to answers known in closed form and to what a simplification may not change about a mesh. Where it departs from the papers, `Documents/Plans/Mesh Simplification.md` says how and why.

# Unity
You should be able to interact with the Unity editor via an MCP server backed by the Unity CLI. Prefer the MCP when you need to interact with the editor. Recompilation takes a while and the server will respond as busy while that happens, so if you do encounter that, try to wait it out. Sometimes the Unity editor may not respond for other reasons. Bringing the editor into focus can help unstick it.