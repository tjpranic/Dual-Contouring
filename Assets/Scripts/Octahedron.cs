using UnityEngine;

public class Octahedron : Volume {

    public override DensityFunction.Type type {
        get { return DensityFunction.Type.Octahedron; }
    }

    public override float sample( Vector3 position ) {
        return sample( new DensityFunction.Frame( this ).local( position ), this.extents );
    }

    // an octahedron centred on the origin of its own frame, at a position in that frame, with its six corners where the
    // axes meet the faces of its box. The article's exact octahedron has all three the same; this one is exact for any
    public static float sample( Vector3 position, Vector3 extents ) {
        // the octahedron is the same in every octant, and in the positive one its face is the triangle of the three
        // corners on the axes, in the plane dot( p, n ) = 1
        var px = Mathf.Abs( position.x );
        var py = Mathf.Abs( position.y );
        var pz = Mathf.Abs( position.z );

        var nx = 1.0f / extents.x;
        var ny = 1.0f / extents.y;
        var nz = 1.0f / extents.z;

        var nn = ( nx * nx ) + ( ny * ny ) + ( nz * nz );
        var m  = ( px * nx ) + ( py * ny ) + ( pz * nz ) - 1.0f;

        var plane = m / Mathf.Sqrt( nn );

        // inside a convex shape, the distance to the nearest face is that to its plane
        if( m <= 0.0f ) {
            return plane;
        }

        // outside, the position seen on the plane. Its coordinates over the extents are where it lies in the triangle:
        // if none is negative it lies on the face, else the nearest point is on the edge across from the corner of the
        // first negative one. The triangle's angles are all less than a right angle, so no other edge can be nearer
        var scale = m / nn;

        if( px - ( nx * scale ) < 0.0f ) {
            return segment( px, py, pz, 0.0f, extents.y, 0.0f, 0.0f, 0.0f, extents.z );
        }
        if( py - ( ny * scale ) < 0.0f ) {
            return segment( px, py, pz, 0.0f, 0.0f, extents.z, extents.x, 0.0f, 0.0f );
        }
        if( pz - ( nz * scale ) < 0.0f ) {
            return segment( px, py, pz, extents.x, 0.0f, 0.0f, 0.0f, extents.y, 0.0f );
        }

        return plane;
    }

    // the distance from the point p to the segment from a to b
    private static float segment( float px, float py, float pz, float ax, float ay, float az, float bx, float by, float bz ) {
        var pax = px - ax;
        var pay = py - ay;
        var paz = pz - az;
        var bax = bx - ax;
        var bay = by - ay;
        var baz = bz - az;

        var t = Mathf.Clamp( ( ( pax * bax ) + ( pay * bay ) + ( paz * baz ) ) / ( ( bax * bax ) + ( bay * bay ) + ( baz * baz ) ), 0.0f, 1.0f );

        var dx = pax - ( bax * t );
        var dy = pay - ( bay * t );
        var dz = paz - ( baz * t );

        return Mathf.Sqrt( ( dx * dx ) + ( dy * dy ) + ( dz * dz ) );
    }

}
