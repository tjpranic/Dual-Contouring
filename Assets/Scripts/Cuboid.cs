using UnityEngine;

public class Cuboid : Volume {

    public override DensityFunction.Type type {
        get { return DensityFunction.Type.Cuboid; }
    }

    public override float sample( Vector3 position ) {
        return sample( new DensityFunction.Frame( this ).local( position ), this.extents );
    }

    // a cuboid centred on the origin of its own frame, at a position in that frame
    public static float sample( Vector3 position, Vector3 extents ) {
        var q = position.absolute( ) - extents;
        return Vector3.Magnitude( Vector3.Max( q, Vector3.zero ) ) + Mathf.Min( Mathf.Max( q.x, Mathf.Max( q.y, q.z ) ), 0.0f );
    }

}
