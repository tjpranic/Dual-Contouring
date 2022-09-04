using UnityEngine;

public class Ellipsoid : Volume {

    public override float sample( Vector3 position ) {
        var origin  = this.getOrigin( );
        var extents = this.getExtents( );

        // avoid division by zero
        if( position - origin == Vector3.zero ) {
            return -1.0f;
        }

        var k0 = Vector3.Magnitude( ( position - origin ).Divide( extents ) );
        var k1 = Vector3.Magnitude( ( position - origin ).Divide( extents.Multiply( extents ) ) );

        return k0 * ( k0 - 1.0f ) / k1;
    }

}