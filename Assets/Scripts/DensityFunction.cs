using UnityEngine;

// List of signed distance functions:
// https://iquilezles.org/articles/distfunctions/

public interface DensityFunction {

    public float sample( Vector3 position, Vector3 origin, Vector3 scale );

}