# Dual Contouring

## TODO
- Remove any LINQ code
- Creased normal shader warnings:
    ```
    Shader warning in 'CreaseNormals': use of potentially uninitialized variable (before) at kernel groupCorners at CreaseNormals.compute(202) (on metal)
    Shader warning in 'CreaseNormals': use of potentially uninitialized variable (contourNormalFits) at kernel groupCorners at CreaseNormals.compute(237) (on metal)
    Shader warning in 'CreaseNormals': array reference cannot be used as an l-value; not natively addressable, forcing loop to unroll at kernel rewrite at CreaseNormals.compute(386) (on metal)
    ```

## IN PROGRESS

## DONE
- Research warping functions and function compression techniques
    Doable. Warping functions are well known. Compression techniques exist for SDFs (see Keeter and Barbier papers in Documents/Papers/).
- See if it's possible to parallelize Manifold Dual Contouring
    Possible. Plan has been drafted.
- Better custom inspectors
- Edges on shared vertex meshes look horrendous
    Fixed with the creased normals feature