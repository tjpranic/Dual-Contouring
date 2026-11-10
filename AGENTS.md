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

# Unity
You should be able to interact with the Unity editor via an MCP server backed by the Unity CLI. Prefer the MCP when you need to interact with the editor. Recompilation takes a while and the server will respond as busy while that happens, so if you do encounter that, try to wait it out. Sometimes the Unity editor may not respond for other reasons. Bringing the editor into focus can help unstick it.