using UnityEngine;

public class Capsule : Volume {

    public override DensityFunction.Type type {
        get { return DensityFunction.Type.Capsule; }
    }

    public override float sample( Vector3 position ) {
        return sample( new DensityFunction.Frame( this ).local( position ), this.extents );
    }

    // a capsule centred on the origin of its own frame, at a position in that frame: a cylinder along y with a half ball
    // on each end, as high as its box. It is as wide as the smaller of the box's x and z extents allow, and a box lower
    // than it is wide leaves a ball as high as the box
    public static float sample( Vector3 position, Vector3 extents ) {
        var radius = Mathf.Min( Mathf.Min( extents.x, extents.z ), extents.y );

        // half the length of the line the capsule is the ball around
        var half = extents.y - radius;

        var along = position.y - Mathf.Clamp( position.y, -half, half );

        return Mathf.Sqrt( ( position.x * position.x ) + ( along * along ) + ( position.z * position.z ) ) - radius;
    }

}
