using UnityEngine;

public class HexagonalPrism : Volume {

    public override DensityFunction.Type type {
        get { return DensityFunction.Type.HexagonalPrism; }
    }

    public override float sample( Vector3 position ) {
        return sample( new DensityFunction.Frame( this ).local( position ), this.extents );
    }

    // cos 30 and tan 30, which a regular hexagon is made of
    public const float cos30 = 0.8660254f;
    public const float tan30 = 0.57735027f;

    // the distance from the middle of a regular hexagon to its sides, for the largest one with its corners on the x axis
    // that fits in a box of the extents
    public static float inradiusOf( Vector3 extents ) {
        return Mathf.Min( extents.z, extents.x * cos30 );
    }

    // a hexagonal prism centred on the origin of its own frame, at a position in that frame. It stands along y, as high
    // as its box, and across x and z it is the largest regular hexagon that fits, with two corners on the x axis and two
    // sides facing z
    public static float sample( Vector3 position, Vector3 extents ) {
        var inradius = inradiusOf( extents );

        // the hexagon is the same in every quarter, and on either side of the line at 60 degrees through its corner there,
        // so the position is taken to the quarter and reflected across that line into the sixth of the plane around the
        // side that faces z
        var across = Mathf.Abs( position.x );
        var up     = Mathf.Abs( position.z );
        var along  = Mathf.Abs( position.y );

        var reflection = 2.0f * Mathf.Min( ( -cos30 * across ) + ( 0.5f * up ), 0.0f );

        across -= reflection * -cos30;
        up     -= reflection * 0.5f;

        // how far outside that side, and outside the ends
        var offSideAcross = across - Mathf.Clamp( across, -tan30 * inradius, tan30 * inradius );
        var offSideUp     = up - inradius;

        var fromSide = Mathf.Sqrt( ( offSideAcross * offSideAcross ) + ( offSideUp * offSideUp ) );

        var side = offSideUp < 0.0f ? -fromSide : fromSide;
        var ends = along - extents.y;

        var outsideSide = Mathf.Max( side, 0.0f );
        var outsideEnds = Mathf.Max( ends, 0.0f );

        return Mathf.Min( Mathf.Max( side, ends ), 0.0f ) + Mathf.Sqrt( ( outsideSide * outsideSide ) + ( outsideEnds * outsideEnds ) );
    }

}
