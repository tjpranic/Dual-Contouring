using UnityEngine;

// list of signed distance functions:
// https://iquilezles.org/articles/distfunctions/

using MaterialIndex = SurfaceExtractor.MaterialIndex;

public interface DensityFunction {

    public enum Type {
        Cuboid,
        Ellipsoid
    }

    public Type type { get; }

    public enum Combination {
        Union,
        Intersection,
        Subtraction
    }

    public Combination   combination   { get; set; }
    public MaterialIndex materialIndex { get; set; }

    public float sample( Vector3 position );

}