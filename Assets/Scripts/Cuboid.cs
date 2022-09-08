using UnityEngine;

public class Cuboid : Volume {

    public override float sample( Vector3 position ) {
        var q = ( position - this.origin ).Absolute( ) - this.extents;
        return Vector3.Magnitude( Vector3.Max( q, Vector3.zero ) ) + Mathf.Min( Mathf.Max( q.x, Mathf.Max( q.y, q.z ) ), 0.0f );
    }

}