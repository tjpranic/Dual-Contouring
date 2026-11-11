using UnityEngine;

// list of signed distance functions:
// https://iquilezles.org/articles/distfunctions/

using MaterialIndex = SurfaceExtractor.MaterialIndex;

public partial interface DensityFunction {

    public enum Type {
        Cuboid,
        Ellipsoid
    }

    public enum Combination {
        Union,
        Intersection,
        Subtraction
    }

    public Type          type          { get; }
    public Vector3       origin        { get; }
    public Vector3       extents       { get; }
    public Combination   combination   { get; set; }
    public MaterialIndex materialIndex { get; set; }

    public float sample( Vector3 position );

}