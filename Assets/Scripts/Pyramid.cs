using UnityEngine;

public class Pyramid : Volume {

    public override DensityFunction.Type type {
        get { return DensityFunction.Type.Pyramid; }
    }

    public override float sample( Vector3 position ) {
        return sample( new DensityFunction.Frame( this ).local( position ), this.extents );
    }

    // a pyramid centred on the origin of its own frame, at a position in that frame. Its square base fills the bottom of
    // its box, as wide as the smaller of the box's x and z extents allow, and its tip is in the middle of the top.
    // The pyramid of the article, with a base of side 1 at y = 0, is scaled to the base. As the article gives it, it is
    // not exact below the base, nor inside near it, where the base is nearer than the sides; both are handled here
    public static float sample( Vector3 position, Vector3 extents ) {
        var side   = 2.0f * Mathf.Min( extents.x, extents.z );
        var height = 2.0f * extents.y / side;

        var x = position.x / side;
        var y = ( position.y + extents.y ) / side;
        var z = position.z / side;

        // below the base, the nearest point is on the base
        if( y <= 0.0f ) {
            var offBaseX = Mathf.Max( Mathf.Abs( x ) - 0.5f, 0.0f );
            var offBaseZ = Mathf.Max( Mathf.Abs( z ) - 0.5f, 0.0f );

            return Mathf.Sqrt( ( offBaseX * offBaseX ) + ( y * y ) + ( offBaseZ * offBaseZ ) ) * side;
        }

        var m2 = ( height * height ) + 0.25f;

        // the pyramid is the same in every quarter around its axis and on either side of the diagonals, so the position
        // is taken to the eighth where x >= z >= 0, and measured from the corner of the base there
        var px = Mathf.Abs( x );
        var pz = Mathf.Abs( z );

        if( pz > px ) {
            ( px, pz ) = ( pz, px );
        }

        px -= 0.5f;
        pz -= 0.5f;

        // in the plane of the side that faces x, and how far in front of that plane
        var qx = pz;
        var qy = ( height * y ) - ( 0.5f * px );
        var qz = ( height * px ) + ( 0.5f * y );

        var s = Mathf.Max( -qx, 0.0f );
        var t = Mathf.Clamp( ( qy - ( 0.5f * pz ) ) / ( m2 + 0.25f ), 0.0f, 1.0f );

        var a = ( m2 * ( qx + s ) * ( qx + s ) ) + ( qy * qy );
        var b = ( m2 * ( qx + ( 0.5f * t ) ) * ( qx + ( 0.5f * t ) ) ) + ( ( qy - ( m2 * t ) ) * ( qy - ( m2 * t ) ) );

        var squared  = Mathf.Min( qy, ( -qx * m2 ) - ( qy * 0.5f ) ) > 0.0f ? 0.0f : Mathf.Min( a, b );
        var distance = Mathf.Sqrt( ( squared + ( qz * qz ) ) / m2 );

        // above the base, the position is inside when it is behind the side. Then the nearest face can be the base
        if( qz < 0.0f ) {
            return Mathf.Max( -distance, -y ) * side;
        }

        return distance * side;
    }

}
