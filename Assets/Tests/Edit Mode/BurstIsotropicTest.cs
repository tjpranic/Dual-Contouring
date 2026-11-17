using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using Unity.Mathematics;
using UnityEngine;

using Corner      = BurstFeatures.Corner;
using Kind        = BurstIsotropic.Kind;
using FlipRefusal = BurstMeshSimplifier.FlipRefusal;
using Operation   = FeatureOperation;

// the features a remesh keeps (BurstFeatures) and the mesh the isotropic remesh works on (BurstIsotropic), on meshes
// built by hand whose features are known: which edges are features, the corners and the lines, what a split, a flip
// and a collapse do with them, and where each kind of collapse goes (Documents/Plans/Remeshing.md, findings 6 and 9)
public class BurstIsotropicTest {

    private static double3 at( Vector3 position ) {
        return new double3( position.x, position.y, position.z );
    }

    private static string describe( FeatureMeshRun run, int vertex ) {
        return $"vertex {vertex} at {run.positionOf( vertex )}";
    }

    // ---- the skeleton ----

    [Test]
    public void aBoxHasTwelveLinesAndEightCorners( [Values( 1, 2 )] int size ) {
        var built = new HandBuiltMesh( ).box( Vector3Int.zero, Vector3Int.one * size );

        using var run = new FeatureMeshRun( built, 40.0 );

        Assert.AreEqual( 12,        run.lineCount,        "lines" );
        Assert.AreEqual( 12 * size, run.featureEdgeCount, "feature edges" );

        for( var vertex = 0; vertex < run.vertexCount; ++vertex ) {
            var position = built.positions[vertex];
            var extremes = Enumerable.Range( 0, 3 ).Count( ( axis ) => position[axis] == 0.0f || position[axis] == size );
            var expected = extremes == 3 ? Kind.Corner : extremes == 2 ? Kind.Line : Kind.Surface;

            Assert.AreEqual( expected,    run.kinds[vertex],              describe( run, vertex ) );
            Assert.AreEqual( Corner.None, run.mesh.corners[vertex],       $"{describe( run, vertex )}: a corner by the count of its lines, not by angle" );
        }

        // every line is one edge of the box, from corner to corner
        foreach( var line in run.edges( ).Where( ( edge ) => run.mesh.edgeLines[edge.edge] != -1 ).GroupBy( ( edge ) => run.mesh.edgeLines[edge.edge] ) ) {
            var direction = math.abs( math.normalize( run.positionOf( line.First( ).a ) - run.positionOf( line.First( ).b ) ) );

            Assert.AreEqual( size, line.Count( ), $"edges of line {line.Key}" );

            foreach( var edge in line ) {
                Assert.AreEqual( 1.0, math.dot( direction, math.abs( math.normalize( run.positionOf( edge.a ) - run.positionOf( edge.b ) ) ) ), 1e-12, $"line {line.Key} is straight" );
            }
        }

        using var flat = new FeatureMeshRun( built, 100.0 );

        Assert.AreEqual( 0, flat.lineCount,                "lines at more than a right angle" );
        Assert.AreEqual( 0, flat.cornerVertices( ).Length, "corners at more than a right angle" );
    }

    [Test]
    public void aPrismsSidesAreFeaturesExactlyWhenTheyTurnMoreThanTheAngle( ) {
        // the sides of an octagonal prism meet at 45 degrees, and its rims turn by as much at every corner
        var built = new HandBuiltMesh( ).prism( 8, 2.0f, 3.0f );

        using var sharp = new FeatureMeshRun( built, 40.0 );

        Assert.AreEqual( 8 + 16, sharp.featureEdgeCount,          "at 40 degrees: the sides and both rims" );
        Assert.AreEqual( 16,     sharp.cornerVertices( ).Length,  "at 40 degrees: where the sides meet the rims" );
        Assert.AreEqual( 8 + 16, sharp.lineCount,                 "at 40 degrees: lines" );

        using var smooth = new FeatureMeshRun( built, 50.0 );

        Assert.AreEqual( 16, smooth.featureEdgeCount,         "at 50 degrees: the rims" );
        Assert.AreEqual( 0,  smooth.cornerVertices( ).Length, "at 50 degrees: corners" );
        Assert.AreEqual( 2,  smooth.lineCount,                "at 50 degrees: the rims, as loops" );
    }

    [Test]
    public void aSquareOfASecondMaterialOnAFlatFaceHasFourCorners( ) {
        var built = new HandBuiltMesh( ).patch( 4, 4, ( i, j ) => i >= 1 && i <= 2 && j >= 1 && j <= 2 ? 1 : 0 );

        bool onSquare( Vector3 position ) {
            return position.x >= 1 && position.x <= 3 && position.y >= 1 && position.y <= 3 && ( position.x == 1 || position.x == 3 || position.y == 1 || position.y == 3 );
        }

        bool cornerOfSquare( Vector3 position ) {
            return ( position.x == 1 || position.x == 3 ) && ( position.y == 1 || position.y == 3 );
        }

        using var run = new FeatureMeshRun( built, 40.0 );

        for( var vertex = 0; vertex < run.vertexCount; ++vertex ) {
            var position = built.positions[vertex];

            if( !onSquare( position ) ) {
                continue;
            }

            Assert.AreEqual( cornerOfSquare( position ) ? Kind.Corner : Kind.Line,   run.kinds[vertex],        describe( run, vertex ) );
            Assert.AreEqual( cornerOfSquare( position ) ? Corner.Turn : Corner.None, run.mesh.corners[vertex], $"{describe( run, vertex )}: where the line turns" );
        }

        // the square's four sides and the open rim's four
        Assert.AreEqual( 8,      run.lineCount,                "lines" );
        Assert.AreEqual( 4 + 4,  run.cornerVertices( ).Length, "corners of the square and of the rim" );
        Assert.AreEqual( 8 + 16, run.featureEdgeCount,         "feature edges" );

        // a line between materials is a feature at any angle, and its corners are by angle
        using var wide = new FeatureMeshRun( built, 100.0 );

        Assert.AreEqual( 8 + 16, wide.featureEdgeCount,         "feature edges at 100 degrees" );
        Assert.AreEqual( 2,      wide.lineCount,                "the square and the rim as loops at 100 degrees" );
        Assert.AreEqual( 0,      wide.cornerVertices( ).Length, "corners at 100 degrees" );
    }

    [Test]
    public void aConeHasATip( ) {
        // the sides of a cone of 32 sides lean 45 degrees from its axis, and meet each other at about 8
        var built = new HandBuiltMesh( ).cone( 32, 1.0f, 1.0f );
        var tip   = built.at( 0.0f, 0.0f, 1.0f );

        using var sharp = new FeatureMeshRun( built, 40.0 );

        CollectionAssert.AreEqual( new[] { tip }, sharp.cornerVertices( ), "corners at 40 degrees" );

        Assert.AreEqual( Corner.Tip, sharp.mesh.corners[tip], "the tip is a corner by the spread of its triangles" );
        Assert.AreEqual( 0,          sharp.mesh.featureEdgesOf( tip ), "the tip has no feature edges" );
        Assert.AreEqual( 1,          sharp.lineCount,         "the rim of the base, as a loop" );
        Assert.AreEqual( 32,         sharp.featureEdgeCount,  "the edges of the rim" );

        using var smooth = new FeatureMeshRun( built, 50.0 );

        Assert.AreEqual( 0,            smooth.cornerVertices( ).Length, "corners at 50 degrees" );
        Assert.AreEqual( Kind.Surface, smooth.kinds[tip],               "the tip at 50 degrees" );
    }

    [Test]
    public void aFlatDiscOfASecondMaterialIsOneClosedLineWithoutACorner( ) {
        var built = new HandBuiltMesh( ).disc( 16, 1.0f, 2.0f );

        using var run = new FeatureMeshRun( built, 40.0 );

        Assert.AreEqual( 0, run.cornerVertices( ).Length, "corners" );
        Assert.AreEqual( 2, run.lineCount,                "the disc and the open rim" );

        var disc = new HashSet<int>( );
        var rim  = new HashSet<int>( );

        foreach( var ( a, b, edge ) in run.edges( ) ) {
            var radius = math.length( run.positionOf( a ) ) + math.length( run.positionOf( b ) );

            if( math.abs( radius - 2.0 ) < 1e-5 ) {
                disc.Add( run.mesh.edgeLines[edge] );
            }
            else if( math.abs( radius - 4.0 ) < 1e-5 ) {
                rim.Add( run.mesh.edgeLines[edge] );
            }
            else {
                Assert.AreEqual( -1, run.mesh.edgeLines[edge], $"the edge from {run.positionOf( a )} to {run.positionOf( b )} is no feature" );
            }
        }

        Assert.AreEqual( 1, disc.Count, "lines round the disc" );
        Assert.AreEqual( 1, rim.Count,  "lines round the rim" );
        Assert.AreNotEqual( disc.Single( ), rim.Single( ), "the disc and the rim are lines of their own" );
        Assert.AreNotEqual( -1, disc.Single( ), "the disc's edges are features" );
    }

    // ---- split and flip with features ----

    // a box of 4 cells, its half at x < 2 of a second material: a line between the two runs round it at x = 2 and
    // crosses four of its edges, at corners
    private static HandBuiltMesh boxWithALine( int size = 4 ) {
        return new HandBuiltMesh( ).box( Vector3Int.zero, Vector3Int.one * size, ( centre, axis, side ) => centre.x < size / 2.0f ? 1 : 0 );
    }

    [Test]
    public void aSplitOfAFeatureEdgeLeavesItsHalvesOnItsLine( ) {
        var built = boxWithALine( );

        var materialA = built.at( 2, 1, 4 );
        var materialB = built.at( 2, 2, 4 );
        var edgeA     = built.at( 0, 1, 4 );
        var edgeB     = built.at( 0, 2, 4 );
        var surfaceA  = built.at( 1, 1, 4 );
        var surfaceB  = built.at( 1, 2, 4 );

        using var before = new FeatureMeshRun( built, 40.0 );
        using var run    = new FeatureMeshRun(
            built, 40.0,
            Operation.split( materialA, materialB, ( at( built.positions[materialA] ) + at( built.positions[materialB] ) ) / 2.0 ),
            Operation.split( edgeA, edgeB, ( at( built.positions[edgeA] ) + at( built.positions[edgeB] ) ) / 2.0 ),
            Operation.split( surfaceA, surfaceB, ( at( built.positions[surfaceA] ) + at( built.positions[surfaceB] ) ) / 2.0 )
        );

        var pairs = new[] { ( materialA, materialB, "between materials" ), ( edgeA, edgeB, "of the box" ), ( surfaceA, surfaceB, "of the surface" ) };

        for( var index = 0; index < pairs.Length; ++index ) {
            var ( a, b, what ) = pairs[index];
            var middle         = run.vertices[index];
            var line           = before.lineOf( a, b );

            Assert.AreEqual( built.positions.Count + index, middle, $"{what}: the new vertex" );

            Assert.AreEqual( line, run.lineOf( a, middle ), $"{what}: the first half" );
            Assert.AreEqual( line, run.lineOf( middle, b ), $"{what}: the second half" );

            Assert.AreEqual( line == -1 ? Kind.Surface : Kind.Line, run.kinds[middle], $"{what}: the new vertex" );

            foreach( var ( x, y, edge ) in run.edges( ).Where( ( edge ) => ( edge.a == middle || edge.b == middle ) && edge.a != a && edge.b != a && edge.a != b && edge.b != b ) ) {
                Assert.AreEqual( -1, run.mesh.edgeLines[edge], $"{what}: the edge across a triangle, from {run.positionOf( x )} to {run.positionOf( y )}" );
            }
        }

        Assert.AreNotEqual( -1, before.lineOf( materialA, materialB ), "the edge between materials is a feature" );
        Assert.AreNotEqual( -1, before.lineOf( edgeA, edgeB ),         "the edge of the box is a feature" );
        Assert.AreEqual( -1,    before.lineOf( surfaceA, surfaceB ),   "the edge of the surface is none" );

        // every other edge is on the line it was on
        foreach( var ( a, b, edge ) in before.edges( ) ) {
            var now = run.edgeBetween( a, b );

            if( now != -1 ) {
                Assert.AreEqual( before.mesh.edgeLines[edge], run.mesh.edgeLines[now], $"the edge from {before.positionOf( a )} to {before.positionOf( b )}" );
            }
        }
    }

    [Test]
    public void aFeatureEdgeByAngleIsNotFlipped( ) {
        var built = new HandBuiltMesh( ).box( Vector3Int.zero, Vector3Int.one * 2 );
        var a     = built.at( 0, 1, 2 );
        var b     = built.at( 0, 0, 2 );

        using var sharp = new FeatureMeshRun( built, 40.0, Operation.checkFlip( a, b ) );
        using var flat  = new FeatureMeshRun( built, 100.0, Operation.checkFlip( a, b ) );

        Assert.AreEqual( FlipRefusal.Feature, sharp.flips[0], "an edge of the box at 40 degrees" );
        Assert.AreNotEqual( FlipRefusal.Feature, flat.flips[0], "an edge of the box at 100 degrees, which is no feature" );
    }

    // ---- collapses ----

    // what the box with a line is at every vertex, worked out from where the vertex is
    private static Kind kindOnBoxWithALine( Vector3 position, int size ) {
        var extremes = Enumerable.Range( 0, 3 ).Count( ( axis ) => position[axis] == 0.0f || position[axis] == size );
        var onLine   = position.x == size / 2.0f;

        if( extremes == 3 || ( onLine && extremes == 2 ) ) {
            return Kind.Corner;
        }
        return extremes == 2 || onLine ? Kind.Line : Kind.Surface;
    }

    // whether an edge of the box with a line runs along one of its lines: an edge of the box, or the line between the
    // materials
    private static bool alongALine( Vector3 a, Vector3 b, int size ) {
        var shared = Enumerable.Range( 0, 3 ).Count( ( axis ) => a[axis] == b[axis] && ( a[axis] == 0.0f || a[axis] == size ) );

        return shared >= 2 || ( a.x == size / 2.0f && b.x == size / 2.0f );
    }

    // every edge of a box with a line, where placeCollapse puts it against what finding 9 says, by the kinds of its
    // ends worked out from where they are. Gives how many edges there were of each case
    private static Dictionary<string, int> placeEveryEdgeOfABoxWithALine( int size ) {
        var built      = boxWithALine( size );
        var edges      = new List<( int, int )>( );
        var operations = new List<Operation>( );

        using( var plain = new FeatureMeshRun( built, 40.0 ) ) {
            foreach( var ( a, b, _ ) in plain.edges( ) ) {
                edges.Add( ( a, b ) );
                operations.Add( Operation.place( a, b ) );
            }

            // the kinds this test expects are those of the mesh
            for( var vertex = 0; vertex < built.positions.Count; ++vertex ) {
                Assert.AreEqual( kindOnBoxWithALine( built.positions[vertex], size ), plain.kinds[vertex], $"box of {size}: {describe( plain, vertex )}" );
            }
        }

        using var run = new FeatureMeshRun( built, 40.0, operations.ToArray( ) );

        var cases = new Dictionary<string, int>( );

        for( var index = 0; index < edges.Count; ++index ) {
            var ( a, b ) = edges[index];
            var pa       = built.positions[a];
            var pb       = built.positions[b];
            var kindA    = kindOnBoxWithALine( pa, size );
            var kindB    = kindOnBoxWithALine( pb, size );
            var feature  = alongALine( pa, pb, size );
            var what     = $"box of {size}: the edge from {pa} ({kindA}) to {pb} ({kindB}){( feature ? ", a feature" : "" )}";

            Assert.AreEqual( feature, run.lineOf( a, b ) != -1, $"{what}: is a feature" );

            // what finding 9 says, by the kinds of the two ends, the lesser first
            var lesser  = ( Kind )Math.Min( ( int )kindA, ( int )kindB );
            var greater = ( Kind )Math.Max( ( int )kindA, ( int )kindB );
            var allowed = true;
            var into    = -1;
            string kase;

            if( greater == Kind.Surface ) {
                kase = "surface to surface, into the middle";
            }
            else if( lesser == Kind.Surface ) {
                kase = $"surface to {greater}, into the {greater}";
                into = kindA == Kind.Surface ? b : a;
            }
            else if( !feature ) {
                kase    = $"{lesser} to {greater} across the surface, refused";
                allowed = false;
            }
            else if( greater == Kind.Line ) {
                kase = "along a line, into the middle";
            }
            else if( lesser == Kind.Line ) {
                kase = "along a line, into its corner";
                into = kindA == Kind.Corner ? a : b;
            }
            else {
                // a corner of the box and one where the line between materials crosses its edge, where four lines meet
                kase = "corner to corner along a line, into the one of more lines";
                into = pa.x == size / 2.0f ? a : b;
            }

            cases[kase] = cases.TryGetValue( kase, out var count ) ? count + 1 : 1;

            Assert.AreEqual( allowed, run.allowed[index], $"{what}: whether it collapses" );

            if( !allowed ) {
                continue;
            }

            var position = into == -1 ? ( at( pa ) + at( pb ) ) / 2.0 : at( built.positions[into] );

            Assert.AreEqual( into, run.vertices[index], $"{what}: the end it goes into" );
            Assert.LessOrEqual( math.distance( position, run.placed[index] ), 1e-12, $"{what}: where it goes" );
        }

        Debug.Log( $"box of {size}: " + string.Join( ", ", cases.Select( ( pair ) => $"{pair.Key}: {pair.Value}" ) ) );

        return cases;
    }

    [Test]
    public void eachKindOfCollapseGoesWhereItsEndsSay( ) {
        var cases = placeEveryEdgeOfABoxWithALine( 2 ).Keys.Union( placeEveryEdgeOfABoxWithALine( 4 ).Keys );

        CollectionAssert.AreEquivalent(
            new[] {
                "surface to surface, into the middle",
                "surface to Line, into the Line",
                "surface to Corner, into the Corner",
                "Line to Line across the surface, refused",
                "Line to Corner across the surface, refused",
                "along a line, into the middle",
                "along a line, into its corner",
                "corner to corner along a line, into the one of more lines"
            },
            cases.ToArray( ),
            "the cases the two boxes have between them"
        );
    }

    [Test]
    public void twoCornersAlongALineBecomeOneUnlessBothArePoints( ) {
        // a box of two cells with a line at x = 1: the edges of the box from its corners run to the corners where the
        // line crosses them, where more lines meet
        var halves     = boxWithALine( 2 );
        var boxCorner  = halves.at( 0, 0, 2 );
        var lineCorner = halves.at( 1, 0, 2 );

        using var run = new FeatureMeshRun( halves, 40.0, Operation.place( boxCorner, lineCorner ), Operation.place( lineCorner, boxCorner ) );

        Assert.AreEqual( Kind.Corner, run.kinds[boxCorner],  "the box's corner" );
        Assert.AreEqual( Kind.Corner, run.kinds[lineCorner], "the line's corner" );

        Assert.AreEqual( 3, run.mesh.featureEdgesOf( boxCorner ),  "lines at the box's corner" );
        Assert.AreEqual( 4, run.mesh.featureEdgesOf( lineCorner ), "lines at the line's corner" );

        for( var index = 0; index < 2; ++index ) {
            Assert.IsTrue( run.allowed[index], $"either way round {index}" );
            Assert.AreEqual( lineCorner, run.vertices[index], $"into the corner of more lines, either way round {index}" );
        }

        // a box of one cell whose faces at x = 1 and y = 1 are of two more materials: three lines between materials meet
        // at its corners at x = y = 1, which are points the simplifier keeps, and two at its corner at x = 1, y = 0, z = 1
        var three = new HandBuiltMesh( ).box( Vector3Int.zero, Vector3Int.one, ( centre, axis, side ) => axis == 0 && side == 1 ? 1 : axis == 1 && side == 1 ? 2 : 0 );
        var point = three.at( 1, 1, 1 );
        var other = three.at( 1, 1, 0 );
        var plain = three.at( 1, 0, 1 );

        using var points = new FeatureMeshRun( three, 40.0, Operation.place( point, other ), Operation.place( plain, point ), Operation.place( point, plain ) );

        Assert.IsTrue( points.mesh.mesh.isPoint( point ),  "a point where three materials meet" );
        Assert.IsTrue( points.mesh.mesh.isPoint( other ),  "another" );
        Assert.IsFalse( points.mesh.mesh.isPoint( plain ), "where two meet" );

        Assert.IsFalse( points.allowed[0], "two points" );

        for( var index = 1; index <= 2; ++index ) {
            Assert.IsTrue( points.allowed[index], $"a point and a corner, either way round {index}" );
            Assert.AreEqual( point, points.vertices[index], $"into the point, either way round {index}" );
        }
    }

    [Test]
    public void aCollapseCarriesTheFeaturesOfTheEndItGoesInto( ) {
        // a surface vertex goes into one of the line, and is the vertex kept: it takes the line's place and its edges
        var built   = boxWithALine( );
        var surface = built.at( 1, 1, 4 );
        var line    = built.at( 2, 1, 4 );

        Assert.Less( surface, line, "the surface vertex is the lower, which a collapse keeps" );

        using var before = new FeatureMeshRun( built, 40.0 );
        using var run    = new FeatureMeshRun( built, 40.0, Operation.collapse( surface, line ) );

        Assert.AreEqual( BurstMeshSimplifier.Refusal.None, run.refusals[0], "the collapse is allowed" );
        Assert.AreEqual( surface, run.vertices[0], "the vertex kept" );

        Assert.AreEqual( at( built.positions[line] ), run.positionOf( surface ), "at the line's vertex" );
        Assert.AreEqual( Kind.Line, run.kinds[surface], "on the line" );

        var materialLine = before.lineOf( line, built.at( 2, 2, 4 ) );

        Assert.AreEqual( materialLine, run.lineOf( surface, built.at( 2, 2, 4 ) ), "the edge to the next vertex of the line, which was the surface vertex's diagonal" );
        Assert.AreEqual( materialLine, run.lineOf( surface, built.at( 2, 0, 4 ) ), "the edge to the corner at the line's end, which was the line vertex's" );

        // a surface vertex goes into a corner found by angle, the corner of a square of a second material
        var square      = new HandBuiltMesh( ).patch( 6, 6, ( i, j ) => i >= 2 && i <= 3 && j >= 2 && j <= 3 ? 1 : 0 );
        var inside      = square.at( 1, 1, 0 );
        var cornerOfSquare = square.at( 2, 2, 0 );

        Assert.Less( inside, cornerOfSquare, "the surface vertex is the lower" );

        using var cornered = new FeatureMeshRun( square, 40.0, Operation.collapse( inside, cornerOfSquare ) );

        Assert.AreEqual( BurstMeshSimplifier.Refusal.None, cornered.refusals[0], "the collapse into the corner is allowed" );

        Assert.AreEqual( Corner.Turn, cornered.mesh.corners[inside], "the vertex kept is the corner where the line turns" );
        Assert.AreEqual( Kind.Corner, cornered.kinds[inside],        "and a corner" );
    }

    [Test]
    public void theLongestEdgeAfterACollapseIsToTheFarthestNeighbourOfEitherEnd( ) {
        var built    = new HandBuiltMesh( ).patch( 4, 4 );
        var a        = built.at( 1, 1, 0 );
        var b        = built.at( 2, 2, 0 );
        var position = new double3( 1.25, 1.5, 0.0 );

        using var run = new FeatureMeshRun( built, 40.0, Operation.longest( a, b, position ) );

        var neighbours = run.edges( )
                            .Where( ( edge ) => ( edge.a == a || edge.b == a || edge.a == b || edge.b == b ) && !( ( edge.a == a && edge.b == b ) || ( edge.a == b && edge.b == a ) ) )
                            .Select( ( edge ) => edge.a == a || edge.a == b ? edge.b : edge.a )
                            .Distinct( );

        var expected = neighbours.Max( ( neighbour ) => math.distance( position, run.positionOf( neighbour ) ) );

        Assert.AreEqual( expected, run.placed[0].x, 1e-12, "the longest edge" );
        Assert.AreEqual( math.distance( position, new double3( 3, 3, 0 ) ), expected, 1e-12, "which is to the far corner of b's squares" );
    }

}
