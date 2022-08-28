using UnityEngine;

public class Ellipsoid : DensityFunction {

    private static Vector3 multiply( Vector3 a, Vector3 b ) {
        return new Vector3( a.x * b.x, a.y * b.y, a.z * b.z );
    }

    private static Vector3 divide( Vector3 a, Vector3 b ) {
        return new Vector3( a.x / b.x, a.y / b.y, a.z / b.z );
    }

    public float sample( Vector3 position, Vector3 origin, Vector3 scale ) {
        if( position == Vector3.zero ) {
            return -1.0f;
        }

        var k0 = Vector3.Magnitude( divide( position - origin, scale ) );
        var k1 = Vector3.Magnitude( divide( position - origin, multiply( scale, scale ) ) );
        return k0 * ( k0 - 1.0f ) / k1;
    }

}