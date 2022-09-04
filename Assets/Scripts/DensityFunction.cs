using UnityEngine;

// list of signed distance functions:
// https://iquilezles.org/articles/distfunctions/

public interface DensityFunction {

    public enum CombinationMode {
        Union,
        Intersection,
        Subtraction
    }

    public CombinationMode getCombinationMode( );

    public float sample( Vector3 position );

}