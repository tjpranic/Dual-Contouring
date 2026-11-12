using UnityEngine;

public class Cone : Volume {

    public override DensityFunction.Type type {
        get { return DensityFunction.Type.Cone; }
    }

    public override float sample( Vector3 position ) {
        return sample( new DensityFunction.Frame( this ).local( position ), this.extents );
    }

    // a cone centred on the origin of its own frame, at a position in that frame. Its base fills the bottom of its box,
    // as wide as the smaller of the box's x and z extents allow, and its tip is in the middle of the top
    public static float sample( Vector3 position, Vector3 extents ) {
        var radius = Mathf.Min( extents.x, extents.z );

        // in the half plane through the axis and the position, measured from the tip: the position, and the rim of the base
        var positionAcross = Mathf.Sqrt( ( position.x * position.x ) + ( position.z * position.z ) );
        var positionAlong  = position.y - extents.y;
        var rimAcross      = radius;
        var rimAlong       = -2.0f * extents.y;

        // the nearest point of the side, and that of the base
        var t = Mathf.Clamp( ( ( positionAcross * rimAcross ) + ( positionAlong * rimAlong ) ) / ( ( rimAcross * rimAcross ) + ( rimAlong * rimAlong ) ), 0.0f, 1.0f );

        var toSideAcross = positionAcross - ( rimAcross * t );
        var toSideAlong  = positionAlong  - ( rimAlong  * t );
        var toBaseAcross = positionAcross - ( rimAcross * Mathf.Clamp( positionAcross / rimAcross, 0.0f, 1.0f ) );
        var toBaseAlong  = positionAlong  - rimAlong;

        var squared = Mathf.Min(
            ( toSideAcross * toSideAcross ) + ( toSideAlong * toSideAlong ),
            ( toBaseAcross * toBaseAcross ) + ( toBaseAlong * toBaseAlong )
        );

        // inside, the position is behind the side and above the base
        var outside = Mathf.Max( ( positionAlong * rimAcross ) - ( positionAcross * rimAlong ), rimAlong - positionAlong );

        return outside < 0.0f ? -Mathf.Sqrt( squared ) : Mathf.Sqrt( squared );
    }

}
