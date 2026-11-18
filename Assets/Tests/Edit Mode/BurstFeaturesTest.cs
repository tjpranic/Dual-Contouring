using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using Unity.Mathematics;
using UnityEngine;

using Kind  = BurstIsotropic.Kind;
using Sharp = BurstMeshSimplifier.Sharp;

// features by angle at the scale of a length (BurstFeatures; Documents/Plans/Remeshing.md, step 3b): on meshes built by
// hand with the noise that contours have along their creases, a sliver, a sliver folded over, a strip of triangles
// that bridges the crease, a crease spread over several edges, the feature edges come out as one line at the scale of
// a length where each edge's own angle gives more or fewer; and at a length of 0 as each edge's own angle gives them
public class BurstFeaturesTest {

    private const double angle = 40.0;

    // two flat faces that meet along the x axis from 0 to n, bent by an angle: z = 0 out to y = n, facing up, and from
    // the axis out to n along (0, -cos bend, sin bend), open round their rims. At a right angle, between them, from
    // y = w on the first to z = w on the second, a quarter round of so many strips, or the faces meet at the axis where
    // w is 0. Rows of unit squares along x
    private static HandBuiltMesh roof( int n, float w = 0.0f, int strips = 1, double bend = 90.0, Action<HandBuiltMesh, int> crease = null ) {
        var built = new HandBuiltMesh( );
        var rows  = new List<float>( );

        if( w > 0.0f ) {
            rows.Add( w );
        }
        for( var row = 1; row <= n; ++row ) {
            rows.Add( row );
        }
        if( w == 0.0f ) {
            rows.Insert( 0, 0.0f );
        }

        // rounded, so that at a right angle the second face is exactly y = 0, where the strips end
        var cosine = ( float )Math.Round( Math.Cos( bend * Math.PI / 180.0 ), 12 );
        var sine   = ( float )Math.Round( Math.Sin( bend * Math.PI / 180.0 ), 12 );

        Vector3 onSecond( int x, float t ) {
            return t == 0.0f ? new Vector3( x, 0.0f, 0.0f ) : new Vector3( x, -t * cosine, t * sine );
        }

        for( var x = 0; x < n; ++x ) {
            for( var row = 0; row + 1 < rows.Count; ++row ) {
                if( w == 0.0f && row == 0 && crease != null ) {
                    crease( built, x );
                }
                else {
                    built.quad( new Vector3( x, rows[row], 0 ), new Vector3( x + 1, rows[row], 0 ), new Vector3( x + 1, rows[row + 1], 0 ), new Vector3( x, rows[row + 1], 0 ) );
                }

                built.quad( onSecond( x, rows[row] ), onSecond( x, rows[row + 1] ), onSecond( x + 1, rows[row + 1] ), onSecond( x + 1, rows[row] ) );
            }

            for( var strip = 0; w > 0.0f && strip < strips; ++strip ) {
                Vector3 at( int along, int index ) {
                    var turn = 0.5 * Math.PI * index / strips;

                    return new Vector3( along, ( float )( w - ( w * Math.Sin( turn ) ) ), ( float )( w - ( w * Math.Cos( turn ) ) ) );
                }

                built.quad( at( x, strip + 1 ), at( x + 1, strip + 1 ), at( x + 1, strip ), at( x, strip ) );
            }
        }

        return built;
    }

    // the first row of the first face at x = 1, its edge on the crease split at a point by a sliver that the second
    // face's edge is across. At the point on the axis, the sliver has no area
    private static Action<HandBuiltMesh, int> sliverAt( Vector3 point ) {
        return ( built, x ) => {
            if( x != 1 ) {
                built.quad( new Vector3( x, 0, 0 ), new Vector3( x + 1, 0, 0 ), new Vector3( x + 1, 1, 0 ), new Vector3( x, 1, 0 ) );

                return;
            }

            built.triangle( new Vector3( 1, 0, 0 ), point, new Vector3( 1, 1, 0 ) );
            built.triangle( point, new Vector3( 2, 0, 0 ), new Vector3( 2, 1, 0 ) );
            built.triangle( point, new Vector3( 2, 1, 0 ), new Vector3( 1, 1, 0 ) );
            built.triangle( new Vector3( 1, 0, 0 ), new Vector3( 2, 0, 0 ), point );
        };
    }

    // the feature edges by angle, which are not hard, and their lines
    private static ( int lines, double length, int edges ) byAngle( FeatureMeshRun run ) {
        var edges = run.edges( ).Where( ( edge ) => run.mesh.edgeLines[edge.edge] != -1 && run.mesh.mesh.edgeSharp[edge.edge] == Sharp.None ).ToArray( );

        return ( edges.Select( ( edge ) => run.mesh.edgeLines[edge.edge] ).Distinct( ).Count( ), edges.Sum( ( edge ) => math.distance( run.positionOf( edge.a ), run.positionOf( edge.b ) ) ), edges.Length );
    }

    // the corners that are not on the rim
    private static int innerCorners( FeatureMeshRun run ) {
        return run.cornerVertices( ).Count( ( vertex ) => {
            for( var end = run.mesh.mesh.firstEdges[vertex]; end != -1; end = run.mesh.mesh.nextEdges[end] ) {
                if( run.mesh.mesh.edgeSharp[end >> 1] != Sharp.None ) {
                    return false;
                }
            }
            return true;
        } );
    }

    private static void assertOneLineAlongTheCrease( FeatureMeshRun run, int n, double offTheAxis, string message ) {
        var ( lines, length, _ ) = byAngle( run );

        Assert.AreEqual( 1, lines, $"{message}: lines by angle" );
        Assert.AreEqual( n, length, 0.05 * n, $"{message}: their length" );
        Assert.AreEqual( 0, innerCorners( run ), $"{message}: corners off the rim" );

        foreach( var edge in run.edges( ).Where( ( edge ) => run.mesh.edgeLines[edge.edge] != -1 && run.mesh.mesh.edgeSharp[edge.edge] == Sharp.None ) ) {
            var a = run.positionOf( edge.a );
            var b = run.positionOf( edge.b );

            Assert.LessOrEqual( math.max( math.length( a.yz ), math.length( b.yz ) ), offTheAxis + 1e-6, $"{message}: edge {edge.a}-{edge.b} from {a} to {b} is by the crease" );
        }
    }

    // ---- at a length of 0, as each edge's own angle has it ----

    [Test]
    public void atALengthOf0TheFeaturesAreThoseOfEachEdgesOwnAngle( ) {
        foreach( var ( built, which ) in new[] {
            ( new HandBuiltMesh( ).box( Vector3Int.zero, Vector3Int.one * 2 ), "a box" ),
            ( new HandBuiltMesh( ).prism( 8, 2.0f, 3.0f ), "an octagonal prism" ),
            ( new HandBuiltMesh( ).cone( 8, 2.0f, 3.0f ), "a cone" ),
            ( roof( 4, 0.3f ), "a crease bridged by a strip" ),
            ( roof( 4, 0.3f, 3 ), "a crease spread over three edges" ),
            ( roof( 4, crease: sliverAt( new Vector3( 1.5f, 0, 0 ) ) ), "a crease with a sliver on it" )
        } ) {
            using var run = new FeatureMeshRun( built, angle );

            for( var edge = 0; edge < run.mesh.mesh.edgeCount; ++edge ) {
                if( run.mesh.mesh.edges[edge].x == -1 || run.mesh.mesh.edgeSharp[edge] != Sharp.None ) {
                    continue;
                }

                var ends      = run.mesh.mesh.edges[edge];
                var triangles = run.triangles( ).Where( ( t ) => math.any( t.corners == ends.x ) && math.any( t.corners == ends.y ) ).Select( ( t ) => t.corners ).ToArray( );
                var normals   = triangles.Select( ( c ) => math.cross( run.positionOf( c.y ) - run.positionOf( c.x ), run.positionOf( c.z ) - run.positionOf( c.x ) ) ).ToArray( );

                Assert.AreEqual( BurstFeatures.bent( normals[0], normals[1], math.cos( math.radians( angle ) ) ), run.mesh.edgeLines[edge] != -1, $"{which}: edge {ends.x}-{ends.y}" );
            }
        }
    }

    // ---- at the scale of a length ----

    [Test]
    public void aCleanCreaseIsOneLineAtEveryScale( [Values( 0.0, 1.0, 2.0 )] double length ) {
        using var run = new FeatureMeshRun( roof( 6 ), angle, length );

        assertOneLineAlongTheCrease( run, 6, 0.0, "a clean crease" );
    }

    [Test]
    public void aSliverWithoutAreaOnACreaseIsMergedIntoItsPatches( ) {
        var built = roof( 6, crease: sliverAt( new Vector3( 1.5f, 0, 0 ) ) );

        // its edges bend against nothing, which leaves a gap in the crease
        using var each = new FeatureMeshRun( built, angle );

        Assert.AreEqual( 2, byAngle( each ).lines, "at a length of 0: lines by angle" );
        Assert.AreEqual( 2, innerCorners( each ), "at a length of 0: corners where the crease stops" );

        using var run = new FeatureMeshRun( built, angle, 1.0 );

        assertOneLineAlongTheCrease( run, 6, 0.0, "at a length of 1" );
    }

    [Test]
    public void aSliverFoldedOverACreaseIsMergedIntoItsPatches( ) {
        // a sliver a ten thousandth high, facing into the solid
        var built = roof( 6, crease: sliverAt( new Vector3( 1.5f, -1e-4f, 1e-4f ) ) );

        // its three edges all bend: the crease is twice over beside it, with a corner at each end
        using var each = new FeatureMeshRun( built, angle );

        Assert.AreEqual( 8, byAngle( each ).edges, "at a length of 0: feature edges by angle, the crease's six and two more" );
        Assert.AreEqual( 2, innerCorners( each ), "at a length of 0: corners where the sliver ends" );

        using var run = new FeatureMeshRun( built, angle, 1.0 );

        assertOneLineAlongTheCrease( run, 6, 2e-4, "at a length of 1" );
    }

    [Test]
    public void aStripThatBridgesACreaseIsOneLineWhereItIsThinnerThanHalfTheLength( ) {
        // 0.42 wide, its two edges at 45 degrees each
        var built = roof( 6, 0.3f );

        using var each = new FeatureMeshRun( built, angle );

        Assert.AreEqual( 2, byAngle( each ).lines, "at a length of 0: lines by angle, either side of the strip" );

        using var run = new FeatureMeshRun( built, angle, 1.0 );

        assertOneLineAlongTheCrease( run, 6, 0.3, "at a length of 1" );

        // 0.85 wide: at a length of 1 a face of its own, and at 2 not
        var wide = roof( 6, 0.6f );

        using var wideAtOne = new FeatureMeshRun( wide, angle, 1.0 );

        Assert.AreEqual( 2, byAngle( wideAtOne ).lines, "wider, at a length of 1: lines by angle" );

        using var wideAtTwo = new FeatureMeshRun( wide, angle, 2.0 );

        assertOneLineAlongTheCrease( wideAtTwo, 6, 0.6, "wider, at a length of 2" );
    }

    [Test]
    public void aCreaseSpreadOverEdgesThatEachBendByLessThanTheAngleIsFoundAcrossTheBand( ) {
        // a quarter round of radius 0.3 in three strips, its edges at 30 degrees each
        var built = roof( 6, 0.3f, 3 );

        using var each = new FeatureMeshRun( built, angle );

        Assert.AreEqual( 0, byAngle( each ).lines, "at a length of 0: no edge bends by more than the angle" );

        using var run = new FeatureMeshRun( built, angle, 1.0 );

        assertOneLineAlongTheCrease( run, 6, 0.3, "at a length of 1" );
    }

    [Test]
    public void aRoundThatBendsByLessThanTheAngleWithinTheBandIsNoCrease( ) {
        // a quarter round of radius 3 in three strips 1.6 wide, each edge at 30 degrees: the band of a length 1 either
        // side of an edge holds the strips next to it and no more
        using var run = new FeatureMeshRun( roundOf( 6, 3.0f, 3 ), angle, 1.0 );

        Assert.AreEqual( 0, byAngle( run ).lines, "lines by angle" );
        Assert.AreEqual( 0, innerCorners( run ), "corners off the rim" );
    }

    // the faces of the roof meeting through a quarter round of a radius in so many strips, its rows out to n from it
    private static HandBuiltMesh roundOf( int n, float radius, int strips ) {
        var built = new HandBuiltMesh( );

        Vector3 at( int along, int index ) {
            var turn = 0.5 * Math.PI * index / strips;

            return new Vector3( along, ( float )( radius - ( radius * Math.Sin( turn ) ) ), ( float )( radius - ( radius * Math.Cos( turn ) ) ) );
        }

        for( var x = 0; x < n; ++x ) {
            for( var row = 0; row < n; ++row ) {
                built.quad( new Vector3( x, radius + row, 0 ), new Vector3( x + 1, radius + row, 0 ), new Vector3( x + 1, radius + row + 1, 0 ), new Vector3( x, radius + row + 1, 0 ) );
                built.quad( new Vector3( x, 0, radius + row ), new Vector3( x, 0, radius + row + 1 ), new Vector3( x + 1, 0, radius + row + 1 ), new Vector3( x + 1, 0, radius + row ) );
            }

            for( var strip = 0; strip < strips; ++strip ) {
                built.quad( at( x, strip + 1 ), at( x + 1, strip + 1 ), at( x + 1, strip ), at( x, strip ) );
            }
        }

        return built;
    }

    [Test]
    public void aBendOfLessThanTheAngleBesideALargeTriangleIsNoCrease( ) {
        // the first face a fan of triangles 40 high on the crease, the second face at 30 degrees in unit squares. In the
        // band either side of the crease, a large triangle counts for no more of itself than lies in the band
        var built = new HandBuiltMesh( );
        var n     = 6;
        var tilt  = 30.0 * Math.PI / 180.0;

        Vector3 onSecond( int x, float t ) {
            return t == 0.0f ? new Vector3( x, 0.0f, 0.0f ) : new Vector3( x, ( float )( -t * Math.Cos( tilt ) ), ( float )( t * Math.Sin( tilt ) ) );
        }

        for( var x = 0; x < n; ++x ) {
            built.triangle( new Vector3( x, 0, 0 ), new Vector3( x + 1, 0, 0 ), new Vector3( n * 0.5f, 40.0f, 0 ) );

            for( var row = 0; row < n; ++row ) {
                built.quad( onSecond( x, row ), onSecond( x, row + 1 ), onSecond( x + 1, row + 1 ), onSecond( x + 1, row ) );
            }
        }

        using var run = new FeatureMeshRun( built, angle, 1.0 );

        Assert.AreEqual( 0, byAngle( run ).lines, "lines by angle" );
    }

    [Test]
    public void aShortCreaseThatEndsLooseIsTakenOutAndALongOneKept( [Values( 0.25, 2.0 )] double length ) {
        // two faces one row deep that meet along the x axis from 0 to 4, in steps of a quarter: the first flat, the
        // second a row of triangles each with a corner on the rim, at a bend from the first that is 62 degrees at x = 0
        // and ten degrees less every quarter after, down to none. Only the edges on the axis from 0 to 0.5 bend by more
        // than the angle, at 57 and 47 degrees, and the next by 37: the crease runs from the open rim to a loose end
        // half a length long. At a length of 2 the band carries it on through the edges at 37 and 27 degrees to x = 1,
        // which is still shorter than the length
        var built = new HandBuiltMesh( );

        Vector3 axis( int step ) {
            return new Vector3( step * 0.25f, 0.0f, 0.0f );
        }

        Vector3 rim( int step ) {
            var x    = ( step * 0.25 ) + 0.125;
            var bend = Math.Max( 0.0, 62.0 - ( 40.0 * x ) ) * Math.PI / 180.0;

            return new Vector3( ( float )x, ( float )-Math.Cos( bend ), ( float )Math.Sin( bend ) );
        }

        for( var step = 0; step < 16; ++step ) {
            built.quad( axis( step ), axis( step + 1 ), axis( step + 1 ) + Vector3.up, axis( step ) + Vector3.up );
            built.triangle( axis( step + 1 ), axis( step ), rim( step ) );

            if( step + 1 < 16 ) {
                built.triangle( axis( step + 1 ), rim( step ), rim( step + 1 ) );
            }
        }

        using var each = new FeatureMeshRun( built, angle );

        Assert.AreEqual( 0.5, byAngle( each ).length, 1e-6, "at a length of 0: the crease" );

        using var run = new FeatureMeshRun( built, angle, length );

        Assert.AreEqual( length < 0.5 ? 0.5 : 0.0, byAngle( run ).length, 1e-6, $"at a length of {length}: the crease" );

        // a crease that is kept ends at a corner, where it ends loose
        Assert.AreEqual( length < 0.5 ? 1 : 0, innerCorners( run ), $"at a length of {length}: corners off the rim" );
    }

    // the faces of the roof at a right angle, and between them strips through the points of a profile in (y, z) from
    // (a, 0) on the first face to (0, c) on the second, as a function gives it at each column x. The faces' first rows
    // run from the profile's ends to y = 1 and z = 1. The second face of a sub-mesh of its own
    private static HandBuiltMesh profiled( int n, Func<int, Vector2[]> profileAt, int secondSubMesh = 0 ) {
        var built = new HandBuiltMesh( );

        for( var x = 0; x < n; ++x ) {
            var here  = profileAt( x );
            var there = profileAt( x + 1 );

            for( var row = 0; row < n; ++row ) {
                var fromA     = row == 0 ? here[0].x : row;
                var fromAThen = row == 0 ? there[0].x : row;
                var fromC     = row == 0 ? here[here.Length - 1].y : row;
                var fromCThen = row == 0 ? there[there.Length - 1].y : row;

                built.quad( new Vector3( x, fromA, 0 ), new Vector3( x + 1, fromAThen, 0 ), new Vector3( x + 1, row + 1, 0 ), new Vector3( x, row + 1, 0 ) );
                built.quad( new Vector3( x, 0, fromC ), new Vector3( x, 0, row + 1 ), new Vector3( x + 1, 0, row + 1 ), new Vector3( x + 1, 0, fromCThen ), secondSubMesh );
            }

            for( var strip = 0; strip + 1 < here.Length; ++strip ) {
                built.quad(
                    new Vector3( x, here[strip + 1].x, here[strip + 1].y ), new Vector3( x + 1, there[strip + 1].x, there[strip + 1].y ),
                    new Vector3( x + 1, there[strip].x, there[strip].y ), new Vector3( x, here[strip].x, here[strip].y )
                );
            }
        }

        return built;
    }

    // a strip from (0.2, 0) to (0, 0.35): 0.4 wide, at 60 degrees to the first face and 30 to the second
    private static readonly Vector2[] leaning = { new( 0.2f, 0.0f ), new( 0.0f, 0.34641016f ) };

    [Test]
    public void aThinPatchIsMergedIntoTheFaceItLiesNearest( ) {
        using var run = new FeatureMeshRun( profiled( 6, ( x ) => leaning ), angle, 1.0 );

        // into the second face, which leaves the crease along the first
        assertOneLineAlongTheCrease( run, 6, 0.2, "at a length of 1" );

        foreach( var edge in run.edges( ).Where( ( edge ) => run.mesh.edgeLines[edge.edge] != -1 && run.mesh.mesh.edgeSharp[edge.edge] == Sharp.None ) ) {
            Assert.AreEqual( 0.0, run.positionOf( edge.a ).z + run.positionOf( edge.b ).z, 1e-6, $"edge {edge.a}-{edge.b} is on the first face's side of the strip" );
        }
    }

    [Test]
    public void aThinPatchIsNotMergedAcrossALineBetweenMaterials( ) {
        // the second face of another material: the strip lies nearer it, but its only neighbour across an edge that is
        // not hard is the first face, and once merged into it there is no line by angle left
        var built = profiled( 6, ( x ) => leaning, 1 );

        using var each = new FeatureMeshRun( built, angle );

        Assert.AreEqual( 1, byAngle( each ).lines, "at a length of 0: lines by angle" );

        using var run = new FeatureMeshRun( built, angle, 1.0 );

        Assert.AreEqual( 0, byAngle( run ).lines, "at a length of 1: lines by angle" );
    }

    [Test]
    public void aThinPatchThatIsMergedIntoAnotherIsMergedAgain( ) {
        // three strips 0.1, 0.2 and 0.1 wide that turn by 30, 25 and 25 degrees from the first face and the last 10 more
        // into the second, which joins it: the first strip lies nearer the second than the first face, is merged into it,
        // and the two together are still thin, and are merged into the third and the second face
        var turns   = new[] { 30.0, 55.0, 80.0 };
        var widths  = new[] { 0.1, 0.2, 0.1 };
        var profile = new Vector2[4];
        var y       = 0.0;
        var z       = 0.0;

        for( var strip = 2; strip >= 0; --strip ) {
            y += widths[strip] * Math.Cos( turns[strip] * Math.PI / 180.0 );
        }

        profile[0] = new Vector2( ( float )y, 0.0f );

        for( var strip = 0; strip < 3; ++strip ) {
            y -= widths[strip] * Math.Cos( turns[strip] * Math.PI / 180.0 );
            z += widths[strip] * Math.Sin( turns[strip] * Math.PI / 180.0 );

            profile[strip + 1] = new Vector2( strip == 2 ? 0.0f : ( float )y, ( float )z );
        }

        using var run = new FeatureMeshRun( profiled( 6, ( x ) => profile ), angle, 1.0 );

        assertOneLineAlongTheCrease( run, 6, 0.35, "at a length of 1" );
    }

    [Test]
    public void aCreaseInsideAPatchThatIsMergedAwayGoesWithIt( ) {
        // a strip of two halves, which bend by 45 degrees against each other for x up to 1, lie flat together from 5,
        // and go from the one to the other between, by less than half the angle a column: one patch, with a crease
        // inside it where it bends, which is merged away whole. Within a length of the crease an edge that bends by more
        // than half the angle is cut, so the strip has to come round slowly to stay one patch
        Vector2[] profileAt( int x ) {
            var share  = Math.Clamp( ( x - 1 ) / 4.0f, 0.0f, 1.0f );
            var middle = Vector2.Lerp( new Vector2( 0.08786797f, 0.08786797f ), new Vector2( 0.15f, 0.15f ), share );

            return new[] { new Vector2( 0.3f, 0.0f ), middle, new Vector2( 0.0f, 0.3f ) };
        }

        var built = profiled( 8, profileAt );

        using var each = new FeatureMeshRun( built, angle );

        Assert.Greater( byAngle( each ).lines, 1, "at a length of 0: lines by angle" );

        using var run = new FeatureMeshRun( built, angle, 1.0 );

        assertOneLineAlongTheCrease( run, 8, 0.3, "at a length of 1" );
    }

    [Test]
    public void aSliverAmongLargeTrianglesDoesNotJoinTheTwoSidesOfACrease( ) {
        // two faces that meet at 60 degrees, in squares of 10, with a sliver a thousandth high on the crease between 10
        // and 20 that faces half way between them: 30 degrees from each, which is not more than the angle. Within a
        // length of 1 of its edges the band holds the sliver and one face's triangle, and its side of the band weighs
        // next to nothing
        var built   = new HandBuiltMesh( );
        var sine    = ( float )Math.Sin( Math.PI / 3.0 );
        var cosine  = 0.5f;
        var epsilon = 1e-3f;
        var point   = new Vector3( 15.0f, sine * epsilon, -cosine * epsilon );

        Vector3 onSecond( int x, int t ) {
            return t == 0 ? new Vector3( 10 * x, 0.0f, 0.0f ) : new Vector3( 10 * x, -10 * t * cosine, 10 * t * sine );
        }

        for( var x = 0; x < 6; ++x )
        for( var row = 0; row < 3; ++row ) {
            if( x == 1 && row == 0 ) {
                built.triangle( new Vector3( 10, 0, 0 ), point, new Vector3( 10, 10, 0 ) );
                built.triangle( point, new Vector3( 20, 0, 0 ), new Vector3( 20, 10, 0 ) );
                built.triangle( point, new Vector3( 20, 10, 0 ), new Vector3( 10, 10, 0 ) );
                built.triangle( new Vector3( 10, 0, 0 ), new Vector3( 20, 0, 0 ), point );
            }
            else {
                built.quad( new Vector3( 10 * x, 10 * row, 0 ), new Vector3( 10 * ( x + 1 ), 10 * row, 0 ), new Vector3( 10 * ( x + 1 ), 10 * ( row + 1 ), 0 ), new Vector3( 10 * x, 10 * ( row + 1 ), 0 ) );
            }

            built.quad( onSecond( x, row ), onSecond( x, row + 1 ), onSecond( x + 1, row + 1 ), onSecond( x + 1, row ) );
        }

        using var each = new FeatureMeshRun( built, angle );

        Assert.AreEqual( 2, innerCorners( each ), "at a length of 0: corners where the crease stops at the sliver" );

        using var run = new FeatureMeshRun( built, angle, 1.0 );

        assertOneLineAlongTheCrease( run, 60, 0.01, "at a length of 1" );
    }

    [Test]
    public void aLineThatZigzagsTurnsOnlyWhereItTurnsOverALength( ) {
        // a flat face z = 0, and a face that rises from a crease along x that zigzags in it, y = 0.3 and -0.3 by turns, to
        // a straight row at z = 4 above the axis, and on up: it leans by less than 5 degrees, so only the crease bends.
        // Each step of the crease turns by 62 degrees from the one before, and over a length of 2 either way not at all
        var built = new HandBuiltMesh( );
        var n     = 12;

        Vector3 onCrease( int x ) {
            return new Vector3( x, x % 2 == 0 ? 0.3f : -0.3f, 0.0f );
        }

        Vector3 onSecond( int x, int t ) {
            return t == 0 ? onCrease( x ) : new Vector3( x, 0.0f, 4.0f * t );
        }

        for( var x = 0; x < n; ++x )
        for( var row = 0; row < 3; ++row ) {
            var from     = row == 0 ? onCrease( x ) : new Vector3( x, row, 0 );
            var fromThen = row == 0 ? onCrease( x + 1 ) : new Vector3( x + 1, row, 0 );

            built.quad( from, fromThen, new Vector3( x + 1, row + 1, 0 ), new Vector3( x, row + 1, 0 ) );
            built.quad( onSecond( x, row ), onSecond( x, row + 1 ), onSecond( x + 1, row + 1 ), onSecond( x + 1, row ) );
        }

        using var each = new FeatureMeshRun( built, angle );

        Assert.AreEqual( n, byAngle( each ).edges, "at a length of 0: the crease's edges" );
        Assert.AreEqual( n - 1, innerCorners( each ), "at a length of 0: a corner where every step turns" );

        using var run = new FeatureMeshRun( built, angle, 2.0 );

        Assert.AreEqual( n, byAngle( run ).edges, "at a length of 2: the crease's edges" );
        Assert.AreEqual( 1, byAngle( run ).lines, "at a length of 2: lines by angle" );
        Assert.AreEqual( 0, innerCorners( run ), "at a length of 2: corners off the rim" );
    }

    [Test]
    public void aConesTipIsACornerAtTheScaleOfALength( ) {
        // of 24 sides, whose edges bend by 14 degrees each, at a length a sixth of its radius
        using var run = new FeatureMeshRun( new HandBuiltMesh( ).cone( 24, 6.0f, 18.0f ), 40.0, 1.0 );

        var tip = Enumerable.Range( 0, run.vertexCount ).Single( ( vertex ) => run.hasTriangles( vertex ) && math.all( run.positionOf( vertex ) == new double3( 0.0, 0.0, 18.0 ) ) );

        Assert.AreEqual( 0, run.mesh.featureEdgesOf( tip ), "feature edges at the tip" );

        Assert.AreEqual( Kind.Corner, run.kinds[tip], "the tip" );
        Assert.AreEqual( BurstFeatures.Corner.Tip, run.mesh.corners[tip], "a tip by its triangles" );
    }

    [Test]
    public void hardFeaturesAreTheSameAtEveryScale( ) {
        // a box of two materials, whose line between them goes round it at x = 2, and a flat patch, open round its rim,
        // of two materials, whose line between them is straight
        foreach( var ( built, length, which ) in new[] {
            ( new HandBuiltMesh( ).box( Vector3Int.zero, new Vector3Int( 4, 2, 2 ), ( centre, axis, side ) => centre.x < 2.0f ? 0 : 1 ), 1.0, "a box of two materials" ),
            ( new HandBuiltMesh( ).patch( 6, 6, ( i, j ) => i < 3 ? 0 : 1 ), 4.0, "a patch of two materials" )
        } ) {
            using var each = new FeatureMeshRun( built, angle );
            using var run  = new FeatureMeshRun( built, angle, length );

            for( var edge = 0; edge < each.mesh.mesh.edgeCount; ++edge ) {
                if( each.mesh.mesh.edgeSharp[edge] != Sharp.None ) {
                    Assert.AreNotEqual( -1, run.mesh.edgeLines[edge], $"{which}: hard edge {edge} at a length of {length}" );
                }
            }

            CollectionAssert.AreEqual( each.kinds, run.kinds, $"{which}: what every vertex is" );
            Assert.AreEqual( each.lineCount, run.lineCount, $"{which}: lines" );
        }
    }

    [Test]
    public void aBoxIsTwelveLinesAndEightCornersAtEveryScale( [Values( 0.0, 0.5, 1.0 )] double length ) {
        using var run = new FeatureMeshRun( new HandBuiltMesh( ).box( Vector3Int.zero, Vector3Int.one * 3 ), angle, length );

        Assert.AreEqual( 12, run.lineCount, "lines" );
        Assert.AreEqual( 8, run.cornerVertices( ).Length, "corners" );
    }

}
