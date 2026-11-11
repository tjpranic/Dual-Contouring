using UnityEngine;

public class Ellipsoid : Volume {

    public override DensityFunction.Type type {
        get { return DensityFunction.Type.Ellipsoid; }
    }

    public override float sample( Vector3 position ) {
        return sample( new DensityFunction.Frame( this ).local( position ), this.extents );
    }

    // an ellipsoid centred on the origin of its own frame, at a position in that frame
    public static float sample( Vector3 position, Vector3 extents ) {
        // avoid division by zero
        if( position == Vector3.zero ) {
            return -1.0f;
        }

        var k0 = Vector3.Magnitude( position.divide( extents ) );
        var k1 = Vector3.Magnitude( position.divide( extents.multiply( extents ) ) );

        return k0 * ( k0 - 1.0f ) / k1;
    }

}
