using UnityEngine;

public class Cuboid : Volume {

    public override DensityFunction.Type type {
        get { return DensityFunction.Type.Cuboid; }
    }

    public override float sample( Vector3 position ) {
        return sample( position, this.origin, this.extents );
    }

    public static float sample( Vector3 position, Vector3 origin, Vector3 extents ) {
        var q = ( position - origin ).absolute( ) - extents;
        return Vector3.Magnitude( Vector3.Max( q, Vector3.zero ) ) + Mathf.Min( Mathf.Max( q.x, Mathf.Max( q.y, q.z ) ), 0.0f );
    }

}
