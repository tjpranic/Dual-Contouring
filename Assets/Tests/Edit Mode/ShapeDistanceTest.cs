using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using UnityEngine;

// the shapes of the article other than the box and the ellipsoid, each made from the three extents of its box. Every
// one is an exact signed distance: its density at a point is how far the point is from its surface, negative inside.
// That is checked against the distance worked out another way, in double precision: face by face for the shapes with
// flat faces, in the profile through the axis for the cylinder and the cone, and for the capsule and the torus as the
// points within a radius of a line and of a circle
public class ShapeDistanceTest {

    public static readonly DensityFunction.Type[] types = {
        DensityFunction.Type.Cylinder,
        DensityFunction.Type.Cone,
        DensityFunction.Type.Capsule,
        DensityFunction.Type.Torus,
        DensityFunction.Type.Pyramid,
        DensityFunction.Type.Octahedron,
        DensityFunction.Type.HexagonalPrism
    };

    // the shapes that are round or square across x and z, and so take the smaller of the two
    public static readonly DensityFunction.Type[] roundOrSquare = {
        DensityFunction.Type.Cylinder,
        DensityFunction.Type.Cone,
        DensityFunction.Type.Capsule,
        DensityFunction.Type.Torus,
        DensityFunction.Type.Pyramid
    };

    // even and uneven, wider than high and higher than wide
    private static readonly Vector3[] boxes = {
        new( 0.5f,  0.5f,  0.5f  ),
        new( 0.3f,  0.45f, 0.3f  ),
        new( 0.4f,  0.2f,  0.25f ),
        new( 0.15f, 0.4f,  0.35f ),
        new( 0.45f, 0.1f,  0.45f ),
        new( 0.2f,  0.2f,  0.42f )
    };

    // the density is a float worked out from floats, the distance it is checked against a double. They differ by at most
    // 1.2e-7, one or two units in the last place of a density near 1
    private const double tolerance = 5e-7;

    // ---------------------------------------------------------------------------------------------------- doubles

    private readonly struct D3 {

        public readonly double x;
        public readonly double y;
        public readonly double z;

        public D3( double x, double y, double z ) {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public D3( Vector3 vector ) : this( vector.x, vector.y, vector.z ) {
        }

        public static D3 operator +( D3 a, D3 b ) => new( a.x + b.x, a.y + b.y, a.z + b.z );
        public static D3 operator -( D3 a, D3 b ) => new( a.x - b.x, a.y - b.y, a.z - b.z );
        public static D3 operator *( D3 a, double s ) => new( a.x * s, a.y * s, a.z * s );

        public static double dot( D3 a, D3 b ) => ( a.x * b.x ) + ( a.y * b.y ) + ( a.z * b.z );

        public static D3 cross( D3 a, D3 b ) => new( ( a.y * b.z ) - ( a.z * b.y ), ( a.z * b.x ) - ( a.x * b.z ), ( a.x * b.y ) - ( a.y * b.x ) );

        public double length => Math.Sqrt( dot( this, this ) );

    }

    private static double clamp( double value, double minimum, double maximum ) {
        return Math.Max( minimum, Math.Min( maximum, value ) );
    }

    private static double distanceToSegment( D3 point, D3 start, D3 end ) {
        var along = end - start;
        var t     = clamp( D3.dot( point - start, along ) / D3.dot( along, along ), 0.0, 1.0 );

        return ( point - ( start + ( along * t ) ) ).length;
    }

    // ---------------------------------------------------------------------------------------------------- distances

    // the signed distance to a convex polygon in the plane, its corners given counterclockwise
    private static double distanceToPolygon( double x, double y, (double x, double y)[] corners ) {
        var nearest = double.PositiveInfinity;
        var inside  = true;

        for( var index = 0; index < corners.Length; ++index ) {
            var start = corners[index];
            var end   = corners[( index + 1 ) % corners.Length];

            nearest = Math.Min( nearest, distanceToSegment( new D3( x, y, 0.0 ), new D3( start.x, start.y, 0.0 ), new D3( end.x, end.y, 0.0 ) ) );

            if( ( ( end.x - start.x ) * ( y - start.y ) ) - ( ( end.y - start.y ) * ( x - start.x ) ) < 0.0 ) {
                inside = false;
            }
        }

        return inside ? -nearest : nearest;
    }

    // a shape of revolution about y, by the distance in the half plane through the axis to its profile, given across
    // and along the axis. The nearest point of such a shape lies in that half plane
    private static Func<D3, double> revolution( params (double x, double y)[] profile ) {
        return ( point ) => distanceToPolygon( Math.Sqrt( ( point.x * point.x ) + ( point.z * point.z ) ), point.y, profile );
    }

    // a convex polyhedron, by the distance to the nearest of its faces, each a list of corners in order around it.
    // Inside, the point is behind the plane of every face
    private static Func<D3, double> polyhedron( D3[] corners, int[][] faces ) {
        var centre = corners.Aggregate( new D3( 0.0, 0.0, 0.0 ), ( sum, corner ) => sum + corner ) * ( 1.0 / corners.Length );

        var planes = faces.Select( ( face ) => {
            var points = face.Select( ( index ) => corners[index] ).ToArray( );
            var normal = D3.cross( points[1] - points[0], points[2] - points[0] );

            normal = normal * ( 1.0 / normal.length );

            // facing out
            if( D3.dot( normal, centre - points[0] ) > 0.0 ) {
                normal = normal * -1.0;
            }

            return ( normal, points );
        } ).ToArray( );

        return ( point ) => {
            var inside  = planes.All( ( plane ) => D3.dot( plane.normal, point - plane.points[0] ) <= 0.0 );
            var nearest = double.PositiveInfinity;

            foreach( var ( normal, points ) in planes ) {
                var height    = D3.dot( normal, point - points[0] );
                var projected = point - ( normal * height );

                // within the face when on the inner side of every edge, which is the side its other corners are on
                var within = true;

                for( var index = 0; index < points.Length; ++index ) {
                    var start = points[index];
                    var end   = points[( index + 1 ) % points.Length];
                    var other = points[( index + 2 ) % points.Length];

                    var side      = D3.dot( D3.cross( end - start, projected - start ), normal );
                    var otherSide = D3.dot( D3.cross( end - start, other - start ), normal );

                    if( side * otherSide < 0.0 ) {
                        within = false;
                    }
                }

                if( within ) {
                    nearest = Math.Min( nearest, Math.Abs( height ) );
                }
                else {
                    for( var index = 0; index < points.Length; ++index ) {
                        nearest = Math.Min( nearest, distanceToSegment( point, points[index], points[( index + 1 ) % points.Length] ) );
                    }
                }
            }

            return inside ? -nearest : nearest;
        };
    }

    // the corners of each shape with flat faces, as it is made from its box, and its faces, each the corners in order
    // around it. The pyramid's tip comes first, and the octahedron's corners come in opposite pairs
    public static ( Vector3[] corners, int[][] faces ) polyhedronOf( DensityFunction.Type type, Vector3 extents ) {
        var across = Mathf.Min( extents.x, extents.z );
        var y      = extents.y;

        switch( type ) {
            case DensityFunction.Type.Pyramid: {
                var corners = new[] {
                    new Vector3( 0.0f, y, 0.0f ),
                    new Vector3( -across, -y, -across ),
                    new Vector3(  across, -y, -across ),
                    new Vector3(  across, -y,  across ),
                    new Vector3( -across, -y,  across )
                };

                return ( corners, new[] { new[] { 1, 2, 3, 4 }, new[] { 0, 1, 2 }, new[] { 0, 2, 3 }, new[] { 0, 3, 4 }, new[] { 0, 4, 1 } } );
            }
            case DensityFunction.Type.Octahedron: {
                var corners = new[] {
                    new Vector3( extents.x, 0.0f, 0.0f ), new Vector3( -extents.x, 0.0f, 0.0f ),
                    new Vector3( 0.0f, y, 0.0f ),         new Vector3( 0.0f, -y, 0.0f ),
                    new Vector3( 0.0f, 0.0f, extents.z ), new Vector3( 0.0f, 0.0f, -extents.z )
                };

                var faces = new[] { 0, 1 }.SelectMany(
                    ( a ) => new[] { 2, 3 }.SelectMany( ( b ) => new[] { 4, 5 }.Select( ( c ) => new[] { a, b, c } ) )
                ).ToArray( );

                return ( corners, faces );
            }
            case DensityFunction.Type.HexagonalPrism: {
                // the corners at every 60 degrees, the first on the x axis, at the bottom and then at the top
                var corner = HexagonalPrism.inradiusOf( extents ) / HexagonalPrism.cos30;

                var ring = Enumerable.Range( 0, 6 ).Select(
                    ( index ) => ( x: corner * Mathf.Cos( index * Mathf.PI / 3.0f ), z: corner * Mathf.Sin( index * Mathf.PI / 3.0f ) )
                ).ToArray( );

                var corners = ring.Select( ( c ) => new Vector3( c.x, -y, c.z ) ).Concat( ring.Select( ( c ) => new Vector3( c.x, y, c.z ) ) ).ToArray( );

                var faces = new List<int[]> { Enumerable.Range( 0, 6 ).ToArray( ), Enumerable.Range( 6, 6 ).ToArray( ) };

                for( var index = 0; index < 6; ++index ) {
                    faces.Add( new[] { index, ( index + 1 ) % 6, 6 + ( ( index + 1 ) % 6 ), 6 + index } );
                }

                return ( corners, faces.ToArray( ) );
            }
            default: {
                throw new Exception( "This type has no flat faces" );
            }
        }
    }

    // the edges of a shape with flat faces, each once, as the corners at its ends
    public static ( Vector3 start, Vector3 end )[] edgesOf( DensityFunction.Type type, Vector3 extents ) {
        var ( corners, faces ) = polyhedronOf( type, extents );

        return faces.SelectMany(
            ( face ) => face.Select( ( corner, index ) => ( a: Math.Min( corner, face[( index + 1 ) % face.Length] ), b: Math.Max( corner, face[( index + 1 ) % face.Length] ) ) )
        ).Distinct( ).Select( ( edge ) => ( corners[edge.a], corners[edge.b] ) ).ToArray( );
    }

    // the distance to the shape of the type in a box of the extents, as the shape is made from its box
    private static Func<D3, double> distanceTo( DensityFunction.Type type, Vector3 extents ) {
        double x = extents.x;
        double y = extents.y;
        double z = extents.z;

        var across = Math.Min( x, z );

        switch( type ) {
            case DensityFunction.Type.Cylinder: {
                return revolution( ( -across, -y ), ( across, -y ), ( across, y ), ( -across, y ) );
            }
            case DensityFunction.Type.Cone: {
                return revolution( ( -across, -y ), ( across, -y ), ( 0.0, y ) );
            }
            case DensityFunction.Type.Capsule: {
                var radius = Math.Min( across, y );
                var start  = new D3( 0.0, -( y - radius ), 0.0 );
                var end    = new D3( 0.0,    y - radius,   0.0 );

                return ( point ) => distanceToSegment( point, start, end ) - radius;
            }
            case DensityFunction.Type.Torus: {
                var tube = Math.Min( y, across );
                var ring = across - tube;

                // the nearest point of the circle through the middle of the tube is the one in the point's direction
                return ( point ) => {
                    var direction = Math.Sqrt( ( point.x * point.x ) + ( point.z * point.z ) );
                    var nearest   = direction > 0.0 ? new D3( point.x * ring / direction, 0.0, point.z * ring / direction ) : new D3( ring, 0.0, 0.0 );

                    return ( point - nearest ).length - tube;
                };
            }
            case DensityFunction.Type.Pyramid:
            case DensityFunction.Type.Octahedron:
            case DensityFunction.Type.HexagonalPrism: {
                var ( corners, faces ) = polyhedronOf( type, extents );

                return polyhedron( corners.Select( ( corner ) => new D3( corner ) ).ToArray( ), faces );
            }
            default: {
                throw new Exception( "No distance for this type" );
            }
        }
    }

    // ---------------------------------------------------------------------------------------------------- tests

    private static float sample( DensityFunction.Type type, Vector3 position, Vector3 extents ) {
        return DensityFunction.sample( type, position, extents );
    }

    // points all over the box and around it
    private static IEnumerable<Vector3> pointsAround( System.Random random, Vector3 extents, int count ) {
        return Enumerable.Range( 0, count ).Select( ( _ ) => BurstTestSupport.range( random, -extents * 1.6f, extents * 1.6f ) );
    }

    [Test]
    public void theDensityIsTheDistanceToTheShape( [ValueSource( nameof( types ) )] DensityFunction.Type type ) {
        var random = new System.Random( 1 );
        var worst  = 0.0;
        var where  = "";

        foreach( var extents in boxes ) {
            var distance = distanceTo( type, extents );

            foreach( var point in pointsAround( random, extents, 3000 ) ) {
                var error = Math.Abs( sample( type, point, extents ) - distance( new D3( point ) ) );

                if( error > worst ) {
                    worst = error;
                    where = $"at {point:F4} in the box of {extents}: density {sample( type, point, extents )}, distance {distance( new D3( point ) )}";
                }
            }
        }

        Assert.That( worst, Is.LessThanOrEqualTo( tolerance ), $"the largest difference, {where}" );
    }

    // the pyramid as the article gives it, of a base of side 1 at y = 0 and its tip at the height, without the corrections
    // below the base and inside near it
    private static float articlesPyramid( Vector3 position, float height ) {
        var m2 = ( height * height ) + 0.25f;

        var px = Mathf.Abs( position.x );
        var pz = Mathf.Abs( position.z );

        if( pz > px ) {
            ( px, pz ) = ( pz, px );
        }

        px -= 0.5f;
        pz -= 0.5f;

        var qx = pz;
        var qy = ( height * position.y ) - ( 0.5f * px );
        var qz = ( height * px ) + ( 0.5f * position.y );

        var s = Mathf.Max( -qx, 0.0f );
        var t = Mathf.Clamp( ( qy - ( 0.5f * pz ) ) / ( m2 + 0.25f ), 0.0f, 1.0f );

        var a = ( m2 * ( qx + s ) * ( qx + s ) ) + ( qy * qy );
        var b = ( m2 * ( qx + ( 0.5f * t ) ) * ( qx + ( 0.5f * t ) ) ) + ( ( qy - ( m2 * t ) ) * ( qy - ( m2 * t ) ) );

        var d2 = Mathf.Min( qy, ( -qx * m2 ) - ( qy * 0.5f ) ) > 0.0f ? 0.0f : Mathf.Min( a, b );

        return Mathf.Sqrt( ( d2 + ( qz * qz ) ) / m2 ) * Mathf.Sign( Mathf.Max( qz, -position.y ) );
    }

    // the distances find what the corrections are for: the article's pyramid is far off below its base, and inside near
    // its base, and the pyramid here is not
    [Test]
    public void theArticlesPyramidIsNotExactAndThisOneIs( ) {
        // the pyramid of the article with height 1 is the one in the box of extents 0.5, moved up by half
        var extents  = Vector3.one * 0.5f;
        var distance = distanceTo( DensityFunction.Type.Pyramid, extents );

        var belowTheBase      = new Vector3( 0.05f, -0.6f, 0.03f );
        var insideNearTheBase = new Vector3( 0.05f, -0.48f, 0.03f );

        foreach( var point in new[] { belowTheBase, insideNearTheBase } ) {
            var article = articlesPyramid( point + new Vector3( 0.0f, 0.5f, 0.0f ), 1.0f );
            var exact   = distance( new D3( point ) );

            Assert.That( Math.Abs( article - exact ), Is.GreaterThan( 0.1 ), $"the article's pyramid at {point}" );
            Assert.That( Math.Abs( sample( DensityFunction.Type.Pyramid, point, extents ) - exact ), Is.LessThanOrEqualTo( tolerance ), $"the pyramid at {point}" );
        }
    }

    [Test]
    public void everyShapeLiesInsideItsBox( [ValueSource( nameof( types ) )] DensityFunction.Type type ) {
        var random = new System.Random( 2 );

        foreach( var extents in boxes ) {
            var outside = pointsAround( random, extents, 6000 ).Where(
                ( point ) => Mathf.Abs( point.x ) > extents.x || Mathf.Abs( point.y ) > extents.y || Mathf.Abs( point.z ) > extents.z
            );

            foreach( var point in outside ) {
                Assert.That( sample( type, point, extents ), Is.GreaterThan( 0.0f ), $"density at {point:F4}, outside the box of {extents}" );
            }
        }
    }

    // points on the surface where the shape meets its box, as each is made from it: the rims and caps, the tips, the
    // corners of the faces
    private static Vector3[] touchingPointsOf( DensityFunction.Type type, Vector3 extents ) {
        var across = Mathf.Min( extents.x, extents.z );
        var y      = extents.y;

        switch( type ) {
            case DensityFunction.Type.Cylinder: {
                return new[] {
                    new Vector3( across, 0.0f, 0.0f ), new Vector3( 0.0f, 0.0f, -across ), new Vector3( 0.0f, y, 0.0f ), new Vector3( 0.0f, -y, 0.0f ),
                    new Vector3( -across, y, 0.0f ), new Vector3( 0.0f, -y, across )
                };
            }
            case DensityFunction.Type.Cone: {
                return new[] { new Vector3( 0.0f, y, 0.0f ), new Vector3( 0.0f, -y, 0.0f ), new Vector3( across, -y, 0.0f ), new Vector3( 0.0f, -y, -across ) };
            }
            case DensityFunction.Type.Capsule: {
                var radius = Mathf.Min( across, y );

                return new[] { new Vector3( 0.0f, y, 0.0f ), new Vector3( 0.0f, -y, 0.0f ), new Vector3( radius, 0.0f, 0.0f ), new Vector3( 0.0f, 0.0f, -radius ) };
            }
            case DensityFunction.Type.Torus: {
                var tube = Mathf.Min( y, across );
                var ring = across - tube;

                return new[] { new Vector3( across, 0.0f, 0.0f ), new Vector3( 0.0f, 0.0f, -across ), new Vector3( ring, tube, 0.0f ), new Vector3( 0.0f, -tube, ring ) };
            }
            case DensityFunction.Type.Pyramid: {
                return new[] {
                    new Vector3( 0.0f, y, 0.0f ),
                    new Vector3( across, -y, across ), new Vector3( -across, -y, across ), new Vector3( across, -y, -across ), new Vector3( -across, -y, -across )
                };
            }
            case DensityFunction.Type.Octahedron: {
                return new[] {
                    new Vector3( extents.x, 0.0f, 0.0f ), new Vector3( -extents.x, 0.0f, 0.0f ),
                    new Vector3( 0.0f, y, 0.0f ),         new Vector3( 0.0f, -y, 0.0f ),
                    new Vector3( 0.0f, 0.0f, extents.z ), new Vector3( 0.0f, 0.0f, -extents.z )
                };
            }
            case DensityFunction.Type.HexagonalPrism: {
                var inradius = HexagonalPrism.inradiusOf( extents );
                var corner   = inradius / HexagonalPrism.cos30;

                return new[] {
                    new Vector3( corner, y, 0.0f ), new Vector3( -corner, -y, 0.0f ),
                    new Vector3( corner / 2.0f, y, inradius ), new Vector3( -corner / 2.0f, -y, -inradius ),
                    new Vector3( 0.0f, 0.0f, inradius )
                };
            }
            default: {
                throw new Exception( "No points for this type" );
            }
        }
    }

    [Test]
    public void everyShapeTouchesItsBoxWhereItIsMadeTo( [ValueSource( nameof( types ) )] DensityFunction.Type type ) {
        foreach( var extents in boxes ) {
            foreach( var point in touchingPointsOf( type, extents ) ) {
                Assert.That( Mathf.Abs( sample( type, point, extents ) ), Is.LessThanOrEqualTo( 1e-6f ), $"density at {point:F4}, in the box of {extents}" );
            }
        }
    }

    // a box wider along x than along z, or the other way round, gives the shape of the narrower box
    [Test]
    public void aRoundOrSquareShapeTakesTheSmallerOfXAndZ( [ValueSource( nameof( roundOrSquare ) )] DensityFunction.Type type ) {
        var random = new System.Random( 3 );

        var narrow = new Vector3( 0.2f, 0.3f, 0.2f );

        foreach( var wider in new[] { new Vector3( 0.4f, 0.3f, 0.2f ), new Vector3( 0.2f, 0.3f, 0.45f ) } ) {
            foreach( var point in pointsAround( random, wider, 2000 ) ) {
                Assert.AreEqual( sample( type, point, narrow ), sample( type, point, wider ), $"density at {point:F4}, in the box of {wider}" );
            }
        }
    }

    // the hexagon is regular, so a box wider or deeper than the largest one that fits gives the same prism
    [Test]
    public void theHexagonalPrismIsTheLargestRegularOneThatFits( ) {
        var random = new System.Random( 4 );

        var cases = new[] {
            // deep enough, so as wide as the box: its sides facing z are set back from the box
            ( new Vector3( 0.3f, 0.3f, 0.4f ), new Vector3( 0.3f, 0.3f, 0.3f * HexagonalPrism.cos30 ) ),
            // wide enough, so as deep as the box: its corners on x are set back from the box
            ( new Vector3( 0.5f, 0.3f, 0.2f ), new Vector3( 0.3f, 0.3f, 0.2f ) )
        };

        foreach( var ( box, fitting ) in cases ) {
            Assert.AreEqual( HexagonalPrism.inradiusOf( fitting ), HexagonalPrism.inradiusOf( box ), $"the inradius in the box of {box}" );

            foreach( var point in pointsAround( random, box, 2000 ) ) {
                Assert.AreEqual(
                    sample( DensityFunction.Type.HexagonalPrism, point, fitting ),
                    sample( DensityFunction.Type.HexagonalPrism, point, box ),
                    $"density at {point:F4}, in the box of {box}"
                );
            }
        }
    }

}
