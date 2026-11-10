using UnityEngine;

public class Ellipsoid : Volume {

    public override DensityFunction.Type type {
        get { return DensityFunction.Type.Ellipsoid; }
    }

    public override float sample( Vector3 position ) {
        return sample( position, this.origin, this.extents );
    }

    public static float sample( Vector3 position, Vector3 origin, Vector3 extents ) {
        // avoid division by zero
        if( position - origin == Vector3.zero ) {
            return -1.0f;
        }

        var k0 = Vector3.Magnitude( ( position - origin ).divide( extents ) );
        var k1 = Vector3.Magnitude( ( position - origin ).divide( extents.multiply( extents ) ) );

        return k0 * ( k0 - 1.0f ) / k1;
    }

}
