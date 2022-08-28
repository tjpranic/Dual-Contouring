using UnityEngine;

public class Cuboid : DensityFunction {

    private static Vector3 max( Vector3 a, Vector3 b ) {
        return new Vector3( Mathf.Max( a.x, b.x ), Mathf.Max( a.y, b.x ), Mathf.Max( a.z, b.x ) );
    }

    private static Vector3 absolute( Vector3 vector ) {
        return new Vector3( Mathf.Abs( vector.x ), Mathf.Abs( vector.y ), Mathf.Abs( vector.z ) );
    }

    public float sample( Vector3 position, Vector3 origin, Vector3 scale ) {
        var q = absolute( position - origin ) - scale;
        return Vector3.Magnitude( max( q, Vector3.zero ) ) + Mathf.Min( Mathf.Max( q.x, Mathf.Max( q.y, q.z ) ), 0.0f );
    }

}