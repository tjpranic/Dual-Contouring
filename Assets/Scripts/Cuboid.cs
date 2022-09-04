using UnityEngine;

public class Cuboid : Volume {

    public override float sample( Vector3 position ) {
        var origin  = this.getOrigin( );
        var extents = this.getExtents( );

        var q = ( position - origin ).Absolute( ) - extents;

        return Vector3.Magnitude( Vector3.Max( q, Vector3.zero ) ) + Mathf.Min( Mathf.Max( q.x, Mathf.Max( q.y, q.z ) ), 0.0f );
    }

}