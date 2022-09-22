using UnityEngine;

public class Ellipsoid : Volume {

    public override float sample( Vector3 position ) {
        // avoid division by zero
        if( position - origin == Vector3.zero ) {
            return -1.0f;
        }

        var k0 = Vector3.Magnitude( ( position - this.origin ).divide( this.extents ) );
        var k1 = Vector3.Magnitude( ( position - this.origin ).divide( this.extents.multiply( this.extents ) ) );

        return k0 * ( k0 - 1.0f ) / k1;
    }

}