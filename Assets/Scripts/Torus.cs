using UnityEngine;

public class Torus : Volume {

    public override DensityFunction.Type type {
        get { return DensityFunction.Type.Torus; }
    }

    public override float sample( Vector3 position ) {
        return sample( new DensityFunction.Frame( this ).local( position ), this.extents );
    }

    // a torus centred on the origin of its own frame, at a position in that frame, lying in the plane of x and z. Its
    // tube is as thick as the box is high, and it reaches as far across as the smaller of the box's x and z extents
    // allow. A tube thicker than that leaves a ball
    public static float sample( Vector3 position, Vector3 extents ) {
        var across = Mathf.Min( extents.x, extents.z );
        var tube   = Mathf.Min( extents.y, across );

        // the radius of the circle through the middle of the tube
        var ring = across - tube;

        var fromRing = Mathf.Sqrt( ( position.x * position.x ) + ( position.z * position.z ) ) - ring;

        return Mathf.Sqrt( ( fromRing * fromRing ) + ( position.y * position.y ) ) - tube;
    }

}
