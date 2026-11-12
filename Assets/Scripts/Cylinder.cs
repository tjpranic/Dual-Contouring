using UnityEngine;

public class Cylinder : Volume {

    public override DensityFunction.Type type {
        get { return DensityFunction.Type.Cylinder; }
    }

    public override float sample( Vector3 position ) {
        return sample( new DensityFunction.Frame( this ).local( position ), this.extents );
    }

    // a cylinder centred on the origin of its own frame, at a position in that frame. It stands along y, as high as its
    // box, and is as wide as the smaller of the box's x and z extents allow
    public static float sample( Vector3 position, Vector3 extents ) {
        var radius = Mathf.Min( extents.x, extents.z );

        // how far outside the side and outside the caps
        var side = Mathf.Sqrt( ( position.x * position.x ) + ( position.z * position.z ) ) - radius;
        var caps = Mathf.Abs( position.y ) - extents.y;

        var outsideSide = Mathf.Max( side, 0.0f );
        var outsideCaps = Mathf.Max( caps, 0.0f );

        return Mathf.Min( Mathf.Max( side, caps ), 0.0f ) + Mathf.Sqrt( ( outsideSide * outsideSide ) + ( outsideCaps * outsideCaps ) );
    }

}
