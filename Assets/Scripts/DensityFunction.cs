using UnityEngine;

// list of signed distance functions:
// https://iquilezles.org/articles/distfunctions/

public interface DensityFunction {

    public enum Combination {
        Union,
        Intersection,
        Subtraction
    }

    public Combination                    combination   { get; set; }
    public SurfaceExtractor.MaterialIndex materialIndex { get; set; }

    public float sample( Vector3 position );

}