using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using Unity.Jobs;
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
    public void twoCornersAlongALineBecomeOneUnlessBothAreHeld( ) {
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

        Assert.IsFalse( run.mesh.held( boxCorner ), "the box's corner, of lines by angle only, is not held" );
        Assert.IsTrue( run.mesh.held( lineCorner ), "the line's corner, on a hard line, is held" );

        // a box of one cell whose faces at x = 1 and y = 1 are of two more materials: three lines between materials meet
        // at its corners at x = y = 1, which are points the simplifier keeps, and two at its corner at x = 1, y = 0, z = 1,
        // which is held as a corner of hard lines. Neither of two held corners moves
        var three = new HandBuiltMesh( ).box( Vector3Int.zero, Vector3Int.one, ( centre, axis, side ) => axis == 0 && side == 1 ? 1 : axis == 1 && side == 1 ? 2 : 0 );
        var point = three.at( 1, 1, 1 );
        var other = three.at( 1, 1, 0 );
        var plain = three.at( 1, 0, 1 );

        using var points = new FeatureMeshRun( three, 40.0, Operation.place( point, other ), Operation.place( plain, point ), Operation.place( point, plain ) );

        Assert.IsTrue( points.mesh.mesh.isPoint( point ),  "a point where three materials meet" );
        Assert.IsTrue( points.mesh.mesh.isPoint( other ),  "another" );
        Assert.IsFalse( points.mesh.mesh.isPoint( plain ), "where two meet" );
        Assert.IsTrue( points.mesh.held( plain ),          "where two meet, on hard lines" );

        Assert.IsFalse( points.allowed[0], "two points" );
        Assert.IsFalse( points.allowed[1], "a point and a corner of hard lines" );
        Assert.IsFalse( points.allowed[2], "a point and a corner of hard lines, the other way round" );

        // a box of one cell and one material: its corners, of lines by angle, become one into the lower
        var cube = new HandBuiltMesh( ).box( Vector3Int.zero, Vector3Int.one );
        var low  = cube.at( 0, 0, 0 );
        var high = cube.at( 1, 0, 0 );

        using var plainCube = new FeatureMeshRun( cube, 40.0, Operation.place( high, low ) );

        Assert.IsTrue( plainCube.allowed[0], "two corners by angle" );
        Assert.AreEqual( Math.Min( low, high ), plainCube.vertices[0], "into the lower of two alike" );
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

    // ---- the loop ----

    // what a remesh gave
    private sealed class Remeshed {

        public double3[]              positions;
        public float3[]               normals;
        public BurstMesh.Triangle[]   triangles;
        public BurstIsotropic.Report  report;

        // the remeshed mesh, to be taken into a FeatureMeshRun
        public HandBuiltMesh built( ) {
            var built = new HandBuiltMesh( );

            built.positions.AddRange( this.positions.Select( ( position ) => new Vector3( ( float )position.x, ( float )position.y, ( float )position.z ) ) );
            built.triangles.AddRange( this.triangles );

            return built;
        }

    }

    // a hand built mesh remeshed at a length, the other settings at their defaults unless given, and with the normals
    // of its own triangles unless given. The normals out are had from the corners of the input creased at creaseAngle,
    // at which 180 leaves the normals given as they are, or from the remesh's own triangles with Normals.Faces
    private static Remeshed remesh( HandBuiltMesh built, double length, int iterations = 5, int areaRounds = 5, double angle = 40.0, Vector3[] normalsGiven = null, Remeshing.Normals normalsOut = Remeshing.Normals.Input, double creaseAngle = 180.0 ) {
        Assert.IsTrue( Unity.Burst.BurstCompiler.IsEnabled, "Burst compilation is off (Jobs > Burst > Enable Compilation)" );

        using var positions    = built.nativePositions( );
        using var normals      = normalsGiven == null ? built.nativeNormals( ) : new Unity.Collections.NativeArray<float3>( normalsGiven.Select( ( normal ) => ( float3 )normal ).ToArray( ), Unity.Collections.Allocator.TempJob );
        using var triangles    = built.nativeTriangles( );
        using var outPositions = new Unity.Collections.NativeList<float3>( Unity.Collections.Allocator.TempJob );
        using var outNormals   = new Unity.Collections.NativeList<float3>( Unity.Collections.Allocator.TempJob );
        using var outTriangles = new Unity.Collections.NativeList<BurstMesh.Triangle>( Unity.Collections.Allocator.TempJob );
        using var corners      = normalsOut == Remeshing.Normals.Input
            ? BurstRemeshing.cornerNormalsOf( positions, normals, triangles, creaseAngle, Unity.Collections.Allocator.TempJob )
            : new Unity.Collections.NativeArray<float3>( 0, Unity.Collections.Allocator.TempJob );

        BurstIsotropic.remesh(
            positions,
            normals,
            triangles,
            corners,
            new BurstIsotropic.Settings { length = length, featureCosine = math.cos( math.radians( angle ) ), iterations = iterations, areaRounds = areaRounds, damping = BurstIsotropic.damping },
            outPositions,
            outNormals,
            outTriangles,
            out var report
        );

        return new Remeshed {
            positions = outPositions.AsArray( ).ToArray( ).Select( ( position ) => ( double3 )position ).ToArray( ),
            normals   = outNormals.AsArray( ).ToArray( ),
            triangles = outTriangles.AsArray( ).ToArray( ),
            report    = report
        };
    }

    // what makes a mesh evenly meshed, against a length: how many of its edges are within the paper's limits, how many
    // of its inner vertices have six edges, the mean smallest angle of its triangles, and how far the Voronoi areas of
    // its inner vertices spread, as the deviation over the mean. And what it is: its Euler characteristic, its pieces,
    // and the lines its open edges make
    private struct Shape {

        public double inBand;
        public double valence6;
        public double smallestAngle;
        public double areaSpread;
        public int    euler;
        public int    pieces;
        public int    rims;

        public override string ToString( ) {
            return $"{this.inBand:P1} of edges within the limits, {this.valence6:P1} of inner vertices of six edges, smallest angle {this.smallestAngle:F1} degrees on the mean, " +
                   $"areas spread by {this.areaSpread:F3}, euler characteristic {this.euler}, {this.pieces} pieces, {this.rims} rims";
        }

    }

    private static Shape shapeOf( double3[] positions, BurstMesh.Triangle[] triangles, double length ) {
        var edges = new Dictionary<( int, int ), int>( );

        foreach( var triangle in triangles ) {
            foreach( var ( a, b ) in new[] { ( triangle.a, triangle.b ), ( triangle.b, triangle.c ), ( triangle.c, triangle.a ) } ) {
                var key = ( Math.Min( a, b ), Math.Max( a, b ) );

                edges[key] = edges.TryGetValue( key, out var count ) ? count + 1 : 1;
            }
        }

        var used      = triangles.SelectMany( ( triangle ) => new[] { triangle.a, triangle.b, triangle.c } ).Distinct( ).ToArray( );
        var neighbours = used.ToDictionary( ( vertex ) => vertex, ( vertex ) => new List<( int other, int triangles )>( ) );

        foreach( var edge in edges ) {
            neighbours[edge.Key.Item1].Add( ( edge.Key.Item2, edge.Value ) );
            neighbours[edge.Key.Item2].Add( ( edge.Key.Item1, edge.Value ) );
        }

        var inner = used.Where( ( vertex ) => neighbours[vertex].All( ( neighbour ) => neighbour.triangles == 2 ) ).ToArray( );

        var areas = inner.ToDictionary( ( vertex ) => vertex, ( vertex ) => 0.0 );

        foreach( var triangle in triangles ) {
            var corners = new[] { triangle.a, triangle.b, triangle.c };

            for( var k = 0; k < 3; ++k ) {
                if( areas.ContainsKey( corners[k] ) ) {
                    areas[corners[k]] += BurstIsotropic.voronoiArea( positions[corners[k]], positions[corners[( k + 1 ) % 3]], positions[corners[( k + 2 ) % 3]] );
                }
            }
        }

        var mean = areas.Values.DefaultIfEmpty( 0.0 ).Average( );

        double angleAt( double3 p, double3 q, double3 r ) {
            return math.degrees( math.atan2( math.length( math.cross( q - p, r - p ) ), math.dot( q - p, r - p ) ) );
        }

        // pieces, by the vertices the triangles join
        var parents = used.ToDictionary( ( vertex ) => vertex, ( vertex ) => vertex );

        int find( int vertex ) {
            while( parents[vertex] != vertex ) {
                vertex = parents[vertex] = parents[parents[vertex]];
            }
            return vertex;
        }

        foreach( var triangle in triangles ) {
            parents[find( triangle.a )] = find( triangle.b );
            parents[find( triangle.b )] = find( triangle.c );
        }

        // rims, by the vertices the open edges join
        var rimParents = used.ToDictionary( ( vertex ) => vertex, ( vertex ) => vertex );

        int findRim( int vertex ) {
            while( rimParents[vertex] != vertex ) {
                vertex = rimParents[vertex] = rimParents[rimParents[vertex]];
            }
            return vertex;
        }

        var open = edges.Where( ( edge ) => edge.Value == 1 ).Select( ( edge ) => edge.Key ).ToArray( );

        foreach( var ( a, b ) in open ) {
            rimParents[findRim( a )] = findRim( b );
        }

        return new Shape {
            inBand        = edges.Keys.Count( ( edge ) => math.distance( positions[edge.Item1], positions[edge.Item2] ) is var l && l >= BurstIsotropic.collapseBelow * length && l <= BurstIsotropic.splitAbove * length ) / ( double )edges.Count,
            valence6      = inner.Count( ( vertex ) => neighbours[vertex].Count == 6 ) / ( double )Math.Max( 1, inner.Length ),
            smallestAngle = triangles.Average( ( t ) => Math.Min( angleAt( positions[t.a], positions[t.b], positions[t.c] ), Math.Min( angleAt( positions[t.b], positions[t.c], positions[t.a] ), angleAt( positions[t.c], positions[t.a], positions[t.b] ) ) ) ),
            areaSpread    = mean > 0.0 ? Math.Sqrt( areas.Values.Average( ( area ) => ( area - mean ) * ( area - mean ) ) ) / mean : double.NaN,
            euler         = used.Length - edges.Count + triangles.Length,
            pieces        = used.Select( find ).Distinct( ).Count( ),
            rims          = open.Select( ( edge ) => findRim( edge.Item1 ) ).Distinct( ).Count( )
        };
    }

    private static Shape shapeOf( Remeshed remeshed, double length ) {
        return shapeOf( remeshed.positions, remeshed.triangles, length );
    }

    private static Shape shapeOf( HandBuiltMesh built, double length ) {
        return shapeOf( built.positions.Select( ( position ) => ( double3 )( float3 )position ).ToArray( ), built.triangles.ToArray( ), length );
    }

    // the volume a closed mesh encloses, from the divergence theorem
    private static double volumeOf( double3[] positions, IEnumerable<BurstMesh.Triangle> triangles ) {
        return triangles.Sum( ( t ) => math.dot( positions[t.a], math.cross( positions[t.b], positions[t.c] ) ) ) / 6.0;
    }

    [Test]
    public void aFlatSquareComesOutEvenlyMeshed( [Values( "one square", "a patch" )] string input ) {
        // a square of side 8, as two triangles or as a patch of unit squares cut along their diagonals
        var built = input == "one square"
            ? new HandBuiltMesh( ).quad( new Vector3( 0, 0, 0 ), new Vector3( 8, 0, 0 ), new Vector3( 8, 8, 0 ), new Vector3( 0, 8, 0 ) )
            : new HandBuiltMesh( ).patch( 8, 8 );

        var remeshed = remesh( built, 1.0 );
        var before   = shapeOf( built, 1.0 );
        var after    = shapeOf( remeshed, 1.0 );

        Debug.Log( $"{input}: before {before}; after {after}; {remeshed.positions.Length} vertices; {remeshed.report.splits} splits, {remeshed.report.collapses} collapses, {remeshed.report.flips} flips" );

        // the square, and only the square
        foreach( var position in remeshed.positions ) {
            Assert.AreEqual( 0.0, position.z, 1e-9, $"{input}: the vertex at {position} is in the plane" );
            Assert.IsTrue( math.all( position.xy >= -1e-9 ) && math.all( position.xy <= 8.0 + 1e-9 ), $"{input}: the vertex at {position} is in the square" );
        }

        foreach( var corner in new[] { new double3( 0, 0, 0 ), new double3( 8, 0, 0 ), new double3( 8, 8, 0 ), new double3( 0, 8, 0 ) } ) {
            Assert.IsTrue( remeshed.positions.Any( ( position ) => math.all( position == corner ) ), $"{input}: the corner at {corner} is where it was" );
        }

        Assert.AreEqual( ( 1, 1, 1 ), ( after.euler, after.pieces, after.rims ), $"{input}: a disk with one rim" );
        Assert.AreEqual( 0, remeshed.report.unplaced, $"{input}: vertices without a point of the square within reach" );

        // evenly meshed: bars a little under what this gave when it was measured, 0.89 to 0.92 of the edges, 0.73 to
        // 0.79 of the inner vertices and 49.5 to 49.9 degrees (step 3 of the plan). A square of 8 has a large share of
        // its vertices on or next to its rim, where they can only move along it
        Assert.GreaterOrEqual( after.inBand,        0.85, $"{input}: edges within the limits" );
        Assert.GreaterOrEqual( after.valence6,      0.65, $"{input}: inner vertices of six edges" );
        Assert.GreaterOrEqual( after.smallestAngle, 48.0, $"{input}: mean smallest angle" );
        Assert.Greater( after.smallestAngle, before.smallestAngle, $"{input}: a larger mean smallest angle than before" );
    }

    // a vertex's normal is the input's at the point it is on, from the normals of that triangle's corners, and not its
    // own triangles': a flat patch whose normals all lean comes out with them leaning the same way
    [Test]
    public void theNormalsAreTheInputsWhereTheVerticesAre( ) {
        var built   = new HandBuiltMesh( ).patch( 6, 6 );
        var leaning = new Vector3( 0.2f, 0.3f, 1.0f ).normalized;

        var remeshed = remesh( built, 0.8, normalsGiven: Enumerable.Repeat( leaning, built.positions.Count ).ToArray( ) );

        Assert.Greater( remeshed.report.splits, 0, "the patch was remeshed" );

        foreach( var normal in remeshed.normals ) {
            Assert.LessOrEqual( math.distance( normal, ( float3 )leaning ), 1e-5f, $"the normal {normal}" );
        }
    }

    // a box whose vertices have the normals of all their triangles, which lie between its faces at its edges and
    // corners, as a contour's do. Its normals as the box would show them creased at 40 degrees are its faces' own, so
    // every vertex the remesh puts inside a face has the face's normal, and none of the leaning normals at the box's
    // edges comes into a face (finding 21)
    [Test]
    public void aVertexInAFaceHasTheFacesNormalAsTheCreasedInputShowsIt( [Values( 0.6, 1.5 )] double length ) {
        var built    = new HandBuiltMesh( ).box( Vector3Int.zero, Vector3Int.one * 4 );
        var remeshed = remesh( built, length, creaseAngle: 40.0 );
        var inFaces  = 0;

        for( var vertex = 0; vertex < remeshed.positions.Length; ++vertex ) {
            var position = remeshed.positions[vertex];
            var planes   = Enumerable.Range( 0, 3 ).Where( ( axis ) => Math.Abs( position[axis] ) < 1e-9 || Math.Abs( position[axis] - 4.0 ) < 1e-9 ).ToArray( );

            if( planes.Length != 1 ) {
                continue;
            }

            var face = double3.zero;

            face[planes[0]] = position[planes[0]] < 1e-9 ? -1.0 : 1.0;

            Assert.LessOrEqual( math.distance( ( double3 )remeshed.normals[vertex], face ), 1e-5, $"at {length}: the normal of the vertex at {position}" );

            ++inFaces;
        }

        Assert.Greater( inFaces, 0, $"at {length}: vertices inside faces" );
    }

    // a vertex's normal is the normals at the corners of the input's triangle it lies in, mixed by where it lies: on a
    // flat patch whose normals turn from vertex to vertex, against the mix worked out here from the patch's triangles
    [Test]
    public void aVertexsNormalIsTheInputsCornersMixedByWhereItLies( ) {
        var built   = new HandBuiltMesh( ).patch( 6, 6 );
        var turning = built.positions.Select( ( p ) => new Vector3( 0.1f * p.x, ( 0.05f * p.y ) + ( 0.02f * p.x * p.x ), 1.0f ).normalized ).ToArray( );

        var remeshed = remesh( built, 0.7, normalsGiven: turning );

        Assert.Greater( remeshed.report.splits, 0, "the patch was remeshed" );

        for( var vertex = 0; vertex < remeshed.positions.Length; ++vertex ) {
            var p     = remeshed.positions[vertex];
            var found = false;

            foreach( var triangle in built.triangles ) {
                var a  = ( double3 )( float3 )built.positions[triangle.a];
                var b  = ( double3 )( float3 )built.positions[triangle.b];
                var c  = ( double3 )( float3 )built.positions[triangle.c];
                var d  = ( ( b.x - a.x ) * ( c.y - a.y ) ) - ( ( c.x - a.x ) * ( b.y - a.y ) );
                var wb = ( ( ( p.x - a.x ) * ( c.y - a.y ) ) - ( ( c.x - a.x ) * ( p.y - a.y ) ) ) / d;
                var wc = ( ( ( b.x - a.x ) * ( p.y - a.y ) ) - ( ( p.x - a.x ) * ( b.y - a.y ) ) ) / d;
                var wa = 1.0 - wb - wc;

                if( wa < -1e-9 || wb < -1e-9 || wc < -1e-9 ) {
                    continue;
                }

                var mixed = math.normalize( ( wa * ( double3 )( float3 )turning[triangle.a] ) + ( wb * ( double3 )( float3 )turning[triangle.b] ) + ( wc * ( double3 )( float3 )turning[triangle.c] ) );

                Assert.LessOrEqual( math.distance( ( double3 )remeshed.normals[vertex], mixed ), 1e-5, $"the normal of the vertex at {p}" );

                found = true;

                break;
            }

            Assert.IsTrue( found, $"the vertex at {p} is on the patch" );
        }
    }

    // with no normals at the input's corners, a vertex's normal is that of its own triangles, weighted by their areas
    [Test]
    public void withFacesTheNormalsAreThoseOfTheVertexsOwnTriangles( ) {
        var built    = new HandBuiltMesh( ).cone( 32, 4.0f, 4.0f );
        var remeshed = remesh( built, 0.5, normalsOut: Remeshing.Normals.Faces );
        var sums     = new double3[remeshed.positions.Length];

        foreach( var triangle in remeshed.triangles ) {
            var a = remeshed.positions[triangle.a];
            var n = math.cross( remeshed.positions[triangle.b] - a, remeshed.positions[triangle.c] - a );

            sums[triangle.a] += n;
            sums[triangle.b] += n;
            sums[triangle.c] += n;
        }

        for( var vertex = 0; vertex < sums.Length; ++vertex ) {
            Assert.LessOrEqual( math.distance( ( double3 )remeshed.normals[vertex], math.normalize( sums[vertex] ) ), 1e-5, $"the normal of the vertex at {remeshed.positions[vertex]}" );
        }
    }

    [Test]
    public void equalizingAreasNarrowsTheirSpread( ) {
        // a cone, whose round side the loop leaves with vertices of other than six edges, nested as the paper says
        var built = new HandBuiltMesh( ).cone( 32, 4.0f, 4.0f );

        var loop     = shapeOf( remesh( built, 0.5, areaRounds: 0 ), 0.5 );
        var equalled = shapeOf( remesh( built, 0.5, areaRounds: 5 ), 0.5 );

        Debug.Log( $"after the loop: {loop}; after five rounds: {equalled}" );

        Assert.Less( equalled.areaSpread, 0.8 * loop.areaSpread, "the spread of the areas after five rounds, against after the loop" );
        Assert.AreEqual( ( loop.euler, loop.pieces ), ( equalled.euler, equalled.pieces ), "the topology" );
    }

    [Test]
    public void anUprightBoxStaysTheBox( [Values( 0.6, 1.5 )] double length ) {
        var built    = new HandBuiltMesh( ).box( Vector3Int.zero, Vector3Int.one * 4 );
        var remeshed = remesh( built, length );
        var after    = shapeOf( remeshed, length );

        Debug.Log( $"at {length}: {after}; {remeshed.positions.Length} vertices" );

        // every vertex on a face of the box
        foreach( var position in remeshed.positions ) {
            var onFace = Enumerable.Range( 0, 3 ).Any( ( axis ) => Math.Abs( position[axis] ) < 1e-9 || Math.Abs( position[axis] - 4.0 ) < 1e-9 );
            var inside = math.all( position >= -1e-9 ) && math.all( position <= 4.0 + 1e-9 );

            Assert.IsTrue( onFace && inside, $"at {length}: the vertex at {position} is on the box" );
        }

        Assert.AreEqual( 64.0, volumeOf( remeshed.positions, remeshed.triangles ), 1e-9, $"at {length}: the volume" );
        Assert.AreEqual( ( 2, 1, 0 ), ( after.euler, after.pieces, after.rims ), $"at {length}: a closed surface of one piece" );

        // the box's own lines and corners, and no other, each corner where it was
        using var features = new FeatureMeshRun( remeshed.built( ), 40.0 );

        Assert.AreEqual( 12, features.lineCount, $"at {length}: lines" );

        var corners = features.cornerVertices( ).Select( ( vertex ) => features.positionOf( vertex ) ).ToArray( );

        Assert.AreEqual( 8, corners.Length, $"at {length}: corners" );

        foreach( var corner in corners ) {
            Assert.IsTrue( math.all( corner == 0.0 | corner == 4.0 ), $"at {length}: the corner at {corner} is a corner of the box" );
        }

        // and every feature edge on an edge of the box
        foreach( var ( a, b, edge ) in features.edges( ).Where( ( edge ) => features.mesh.edgeLines[edge.edge] != -1 ) ) {
            var pa = features.positionOf( a );
            var pb = features.positionOf( b );

            var shared = Enumerable.Range( 0, 3 ).Count( ( axis ) => pa[axis] == pb[axis] && ( pa[axis] == 0.0 || pa[axis] == 4.0 ) );

            Assert.GreaterOrEqual( shared, 2, $"at {length}: the feature edge from {pa} to {pb} is on an edge of the box" );
        }
    }

    [Test]
    public void aLineBetweenMaterialsStaysWhereItIs( ) {
        var built    = boxWithALine( );
        var remeshed = remesh( built, 0.7 );

        double areaOf( IEnumerable<BurstMesh.Triangle> triangles, Func<int, double3> at ) {
            return triangles.Sum( ( t ) => 0.5 * math.length( math.cross( at( t.b ) - at( t.a ), at( t.c ) - at( t.a ) ) ) );
        }

        for( var subMesh = 0; subMesh < 2; ++subMesh ) {
            var mine = remeshed.triangles.Where( ( triangle ) => triangle.subMeshIndex == subMesh ).ToArray( );

            // the second material is the half at x < 2, and the first the half beyond
            foreach( var triangle in mine ) {
                foreach( var vertex in new[] { triangle.a, triangle.b, triangle.c } ) {
                    var x = remeshed.positions[vertex].x;

                    Assert.IsTrue( subMesh == 1 ? x <= 2.0 + 1e-9 : x >= 2.0 - 1e-9, $"a vertex of sub-mesh {subMesh} at {remeshed.positions[vertex]} is on its side of the line" );
                }
            }

            var was = areaOf( built.triangles.Where( ( triangle ) => triangle.subMeshIndex == subMesh ), ( vertex ) => ( float3 )built.positions[vertex] );
            var now = areaOf( mine, ( vertex ) => remeshed.positions[vertex] );

            Assert.AreEqual( was, now, 1e-9, $"the area of sub-mesh {subMesh}" );
        }
    }

    // ---- the stages of the loop, one at a time ----

    // the Voronoi areas of a triangle's three corners are the triangle cut by the perpendicular bisectors of its sides,
    // or where an angle is obtuse, half the triangle at that corner and a quarter at each of the others (Meyer et al.)
    [Test]
    public void theVoronoiAreasOfATrianglesCornersAddUpToItsArea( ) {
        foreach( var ( what, a, b, c ) in new[] {
            ( "equilateral", new double3( 0, 0, 0 ), new double3( 2, 0, 0 ), new double3( 1, math.sqrt( 3.0 ), 0 ) ),
            ( "right-angled", new double3( 0, 0, 0 ), new double3( 1, 0, 0 ), new double3( 0, 1, 0 ) ),
            ( "acute and uneven", new double3( 0, 0, 0 ), new double3( 3, 0, 0.5 ), new double3( 1.2, 2.1, -0.3 ) ),
            ( "obtuse at the first corner", new double3( 0, 0, 0 ), new double3( 3, 0, 0 ), new double3( -1, 0.5, 0 ) )
        } ) {
            var area    = 0.5 * math.length( math.cross( b - a, c - a ) );
            var corners = new[] { BurstIsotropic.voronoiArea( a, b, c ), BurstIsotropic.voronoiArea( b, c, a ), BurstIsotropic.voronoiArea( c, a, b ) };

            Assert.AreEqual( area, corners.Sum( ), 1e-12, $"{what}: the corners add up to the triangle" );

            if( what == "equilateral" ) {
                Assert.AreEqual( area / 3.0, corners[1], 1e-12, $"{what}: a third at every corner" );
            }
            if( what.StartsWith( "obtuse" ) ) {
                Assert.AreEqual( area / 2.0, corners[0], 1e-12, $"{what}: half at the obtuse corner" );
                Assert.AreEqual( area / 4.0, corners[1], 1e-12, $"{what}: a quarter at another" );
            }
        }
    }

    // flips bring the vertices of a fan of eight round a centre, on an open rim, nearer six edges for the centre and
    // four for the rim
    [Test]
    public void flipsBringAFanNearerSixEdgesAtItsCentre( ) {
        var built = new HandBuiltMesh( );

        for( var k = 0; k < 8; ++k ) {
            built.triangle( Vector3.zero, HandBuiltMesh.onRing( k, 8, 2.0f, 0.0f ), HandBuiltMesh.onRing( k + 1, 8, 2.0f, 0.0f ) );
        }

        var centre = built.at( 0, 0, 0 );

        using var run   = new FeatureMeshRun( built, 40.0 );
        using var flips = new Unity.Collections.NativeArray<int>( 1, Unity.Collections.Allocator.TempJob );
        using var grid  = SurfaceGrid.of( run.mesh, 1.0, Unity.Collections.Allocator.TempJob );

        Assert.AreEqual( 8, run.edges( ).Count( ( edge ) => edge.a == centre || edge.b == centre ), "edges at the centre before" );

        new BurstIsotropic.FlipJob { mesh = run.mesh, grid = grid, reach = 2.0, cosine = math.cos( math.radians( 40.0 ) ), flips = flips }.Run( );

        var after = run.edges( ).Count( ( edge ) => edge.a == centre || edge.b == centre );

        Debug.Log( $"{flips[0]} flips leave the centre {after} edges" );

        Assert.Greater( flips[0], 0, "flips" );
        Assert.Less( after, 8, "edges at the centre after" );
    }

    // one round of smoothing: in the loop a vertex of a flat patch goes the whole way to the middle of its
    // neighbours, and to equalize areas the damped part of the way to their middle weighted by their Voronoi areas
    [Test]
    public void aRoundOfSmoothingMovesAsThePaperSays( [Values] bool byArea ) {
        var built  = new HandBuiltMesh( ).patch( 4, 4 );
        var vertex = built.at( 2, 2, 0 );

        using var run  = new FeatureMeshRun( built, 40.0 );
        using var grid = SurfaceGrid.of( run.mesh, 1.0, Unity.Collections.Allocator.TempJob );

        var positions = run.mesh.mesh.positions;
        var moved     = new double3( 2.3, 1.8, 0.0 );

        positions[vertex] = moved;

        var vertexCount = run.vertexCount;

        using var areas   = new Unity.Collections.NativeArray<double>( vertexCount, Unity.Collections.Allocator.TempJob );
        using var targets = new Unity.Collections.NativeArray<double3>( vertexCount, Unity.Collections.Allocator.TempJob );
        using var moves   = new Unity.Collections.NativeArray<byte>( vertexCount, Unity.Collections.Allocator.TempJob );

        new BurstIsotropic.AreasJob {
            positions = run.mesh.mesh.positions.AsArray( ), firstCorners = run.mesh.mesh.firstCorners.AsArray( ), nextCorners = run.mesh.mesh.nextCorners.AsArray( ), triangles = run.mesh.mesh.triangles.AsArray( ), areas = areas
        }.Schedule( vertexCount, 16 ).Complete( );

        new BurstIsotropic.SmoothJob {
            grid          = grid,
            reach         = 2.0,
            byArea        = byArea,
            damping       = BurstIsotropic.damping,
            positions     = run.mesh.mesh.positions.AsArray( ),
            firstCorners  = run.mesh.mesh.firstCorners.AsArray( ),
            nextCorners   = run.mesh.mesh.nextCorners.AsArray( ),
            triangles     = run.mesh.mesh.triangles.AsArray( ),
            subMeshes     = run.mesh.mesh.subMeshes.AsArray( ),
            firstEdges    = run.mesh.mesh.firstEdges.AsArray( ),
            nextEdges     = run.mesh.mesh.nextEdges.AsArray( ),
            edges         = run.mesh.mesh.edges.AsArray( ),
            edgeLines     = run.mesh.edgeLines.AsArray( ),
            orders        = run.mesh.mesh.orders.AsArray( ),
            sharpCounts   = run.mesh.mesh.sharpCounts.AsArray( ),
            corners       = run.mesh.corners.AsArray( ),
            fixedVertices = run.mesh.fixedVertices.AsArray( ),
            areas         = areas,
            targets       = targets,
            moves         = moves
        }.Schedule( vertexCount, 16 ).Complete( );

        var neighbours = run.edges( ).Where( ( edge ) => edge.a == vertex || edge.b == vertex ).Select( ( edge ) => edge.a == vertex ? edge.b : edge.a ).ToArray( );
        var weights    = neighbours.Select( ( neighbour ) => byArea ? areas[neighbour] : 1.0 ).ToArray( );
        var middle     = double3.zero;

        for( var index = 0; index < neighbours.Length; ++index ) {
            middle += run.positionOf( neighbours[index] ) * weights[index];
        }

        middle /= weights.Sum( );

        var expected = byArea ? moved + ( BurstIsotropic.damping * ( middle - moved ) ) : middle;

        Assert.AreEqual( 1, moves[vertex], "the vertex moves" );
        Assert.LessOrEqual( math.distance( expected, targets[vertex] ), 1e-12, $"where it goes, by area {byArea}" );

        if( byArea ) {
            Assert.AreNotEqual( areas[neighbours[0]], areas[neighbours[1]], "the neighbours' areas differ, so the weights matter" );
        }
    }

    // a split puts its vertex on the input, where the edge it splits is not on it: here the diagonal a flip has made
    // across a ridge, whose middle is under the ridge
    [Test]
    public void aSplitPutsItsVertexOnTheInput( ) {
        var built = new HandBuiltMesh( ).quad( new Vector3( 0, 0, 0.4f ), new Vector3( 2, 0, 0 ), new Vector3( 2, 2, 0.4f ), new Vector3( 0, 2, 0 ) );
        var a     = built.at( 0, 0, 0.4f );
        var c     = built.at( 2, 2, 0.4f );

        using var input   = new FeatureMeshRun( built, 40.0 );
        using var grid    = SurfaceGrid.of( input.mesh, 1.0, Unity.Collections.Allocator.TempJob );
        using var flipped = new FeatureMeshRun( built, 40.0, Operation.flip( a, c ) );
        using var splits  = new Unity.Collections.NativeArray<int>( 2, Unity.Collections.Allocator.TempJob );

        Assert.AreEqual( FlipRefusal.None, flipped.flips[0], "the ridge's diagonal flips" );

        var before = flipped.vertexCount;

        new BurstIsotropic.SplitJob { mesh = flipped.mesh, grid = grid, reach = 2.0, longest = 1.5, splits = splits }.Run( );

        Assert.Greater( flipped.vertexCount, before, "splits" );
        Assert.AreEqual( 0, splits[1], "splits refused" );

        for( var vertex = before; vertex < flipped.vertexCount; ++vertex ) {
            var position = flipped.positionOf( vertex );
            var distance = Enumerable.Range( 0, 2 ).Min( ( triangle ) => math.distance( position, SurfaceGrid.nearestOnTriangle( position, grid.corners[triangle * 3], grid.corners[( triangle * 3 ) + 1], grid.corners[( triangle * 3 ) + 2] ) ) );

            Assert.LessOrEqual( distance, 1e-9, $"the split vertex at {position} is on the input" );
        }
    }

    // Both hard polylines keep their segments, not just their vertices: a chord across a bend changes the area.
    [Test]
    public void aLineBetweenMaterialsRoundADiscStaysOnItsPolygon( [Values( 0.5, 2.0 )] double length ) {
        var built    = new HandBuiltMesh( ).disc( 16, 2.0f, 4.0f );
        var remeshed = remesh( built, length );

        using var features = new FeatureMeshRun( remeshed.built( ), 40.0 );

        Assert.Greater( remeshed.report.moves, 0, "vertices moved" );

        foreach( var ( sharp, radius ) in new[] { ( BurstMeshSimplifier.Sharp.Materials, 2.0f ), ( BurstMeshSimplifier.Sharp.Open, 4.0f ) } ) {
            var polygon = Enumerable.Range( 0, 16 ).Select( ( k ) => ( double3 )( float3 )HandBuiltMesh.onRing( k, 16, radius, 0.0f ) ).ToArray( );
            var edges   = features.edges( ).Where( ( edge ) => features.mesh.mesh.edgeSharp[edge.edge] == sharp ).ToArray( );

            foreach( var ( a, b, _ ) in edges ) {
                var pa = features.positionOf( a );
                var pb = features.positionOf( b );
                var off = Enumerable.Range( 0, polygon.Length ).Min( ( k ) => math.max(
                    math.distance( pa, SurfaceGrid.nearestOnSegment( pa, polygon[k], polygon[( k + 1 ) % polygon.Length] ) ),
                    math.distance( pb, SurfaceGrid.nearestOnSegment( pb, polygon[k], polygon[( k + 1 ) % polygon.Length] ) )
                ) );

                Assert.LessOrEqual( off, 1e-6, $"{sharp}: the whole edge {pa} to {pb} lies on one input segment" );
            }

            foreach( var bend in polygon ) {
                Assert.IsTrue( edges.Any( ( edge ) => math.distance( bend, features.positionOf( edge.a ) ) < 1e-6 || math.distance( bend, features.positionOf( edge.b ) ) < 1e-6 ),
                    $"{sharp}: the bend at {bend} remains a boundary vertex" );
            }
        }

        for( var subMesh = 0; subMesh < 2; ++subMesh ) {
            var before = built.triangles.Where( ( t ) => t.subMeshIndex == subMesh ).Sum( ( t ) => 0.5 * math.length( math.cross(
                ( double3 )( float3 )built.positions[t.b] - ( double3 )( float3 )built.positions[t.a],
                ( double3 )( float3 )built.positions[t.c] - ( double3 )( float3 )built.positions[t.a]
            ) ) );
            var after = remeshed.triangles.Where( ( t ) => t.subMeshIndex == subMesh ).Sum( ( t ) => 0.5 * math.length( math.cross(
                remeshed.positions[t.b] - remeshed.positions[t.a], remeshed.positions[t.c] - remeshed.positions[t.a]
            ) ) );

            Assert.AreEqual( before, after, 1e-5, $"the area of material {subMesh}, to float precision" );
        }
    }

    [Test]
    public void aBentNonManifoldLineKeepsItsBreakpoints( ) {
        var built = new HandBuiltMesh( ).box( Vector3Int.zero, new Vector3Int( 2, 2, 4 ) )
                                       .box( new Vector3Int( 2, 2, 0 ), new Vector3Int( 2, 2, 4 ) );
        var line = Enumerable.Range( 0, 5 ).Select( ( z ) => new Vector3( 2.0f + ( 0.1f * ( z % 2 ) ), 2.0f, z ) ).ToArray( );

        for( var z = 0; z < line.Length; ++z ) {
            built.positions[built.at( 2, 2, z )] = line[z];
        }

        var remeshed = remesh( built, 2.0 );

        using var features = new FeatureMeshRun( remeshed.built( ), 40.0 );

        var edges = features.edges( ).Where( ( edge ) => ( features.mesh.mesh.edgeSharp[edge.edge] & BurstMeshSimplifier.Sharp.NonManifold ) != 0 ).ToArray( );

        foreach( var point in line ) {
            var bend = ( double3 )( float3 )point;

            Assert.IsTrue( edges.Any( ( edge ) => math.distance( bend, features.positionOf( edge.a ) ) < 1e-6 || math.distance( bend, features.positionOf( edge.b ) ) < 1e-6 ),
                $"the non-manifold line keeps its bend at {bend}" );
        }

        foreach( var ( a, b, _ ) in edges ) {
            var pa = features.positionOf( a );
            var pb = features.positionOf( b );
            var off = Enumerable.Range( 0, line.Length - 1 ).Min( ( k ) => math.max(
                math.distance( pa, SurfaceGrid.nearestOnSegment( pa, ( float3 )line[k], ( float3 )line[k + 1] ) ),
                math.distance( pb, SurfaceGrid.nearestOnSegment( pb, ( float3 )line[k], ( float3 )line[k + 1] ) )
            ) );

            Assert.LessOrEqual( off, 1e-6, $"the whole non-manifold edge {pa} to {pb} lies on an input segment" );
        }
    }

    [Test]
    public void aRemeshKeepsFacingAFoldFreeInput( [Values( 1, 5 )] int iterations ) {
        var built = new HandBuiltMesh( ).patch( 6, 6 );

        for( var vertex = 0; vertex < built.positions.Count; ++vertex ) {
            var position = built.positions[vertex];

            position.z = 2.0f * ( float )( Math.Sin( position.x * 1.3 ) * Math.Cos( position.y * 1.7 ) );
            built.positions[vertex] = position;
        }

        var remeshed = remesh( built, 2.0, iterations: iterations );

        Assert.Greater( remeshed.report.collapses, 0, "the mesh was coarsened" );

        foreach( var triangle in remeshed.triangles ) {
            var a         = remeshed.positions[triangle.a];
            var b         = remeshed.positions[triangle.b];
            var c         = remeshed.positions[triangle.c];
            var normal    = math.cross( b - a, c - a );
            var middle    = ( a + b + c ) / 3.0;
            var closest   = double.MaxValue;
            var reference = double3.zero;

            foreach( var input in built.triangles ) {
                var pa       = ( double3 )( float3 )built.positions[input.a];
                var pb       = ( double3 )( float3 )built.positions[input.b];
                var pc       = ( double3 )( float3 )built.positions[input.c];
                var distance = math.distancesq( middle, SurfaceGrid.nearestOnTriangle( middle, pa, pb, pc ) );

                if( distance < closest ) {
                    closest   = distance;
                    reference = math.cross( pb - pa, pc - pa );
                }
            }

            Assert.Greater( math.dot( normal, reference ), 0.0, $"the triangle {triangle.a}, {triangle.b}, {triangle.c} faces the nearest input surface" );
        }
    }

    [Test]
    public void theSameMeshComesOutTwice( ) {
        var built  = boxWithALine( );
        var first  = remesh( built, 0.8 );
        var second = remesh( built, 0.8 );

        CollectionAssert.AreEqual( first.positions, second.positions, "positions" );
        CollectionAssert.AreEqual( first.normals,   second.normals,   "normals" );
        CollectionAssert.AreEqual( first.triangles, second.triangles, "triangles" );
    }

    // an island shorter than the length stays a closed piece of its own, though not its shape: its corners are closer
    // than the length, and become one as any detail of the shape under the remesh's resolution does (finding 9). The
    // lists' check keeps it from going altogether
    [Test]
    public void anIslandShorterThanTheLengthStaysClosed( ) {
        // a box of 6 and, apart from it, one of a single cell, remeshed at three times the small one's side
        var built    = new HandBuiltMesh( ).box( Vector3Int.zero, Vector3Int.one * 6 ).box( Vector3Int.one * 10, Vector3Int.one );
        var remeshed = remesh( built, 3.0 );
        var after    = shapeOf( remeshed, 3.0 );

        Debug.Log( $"two boxes at 3: {after}" );

        Assert.AreEqual( ( 4, 2, 0 ), ( after.euler, after.pieces, after.rims ), "two closed surfaces" );

        var island = remeshed.triangles.Where( ( triangle ) => remeshed.positions[triangle.a].x > 8.0 ).SelectMany( ( triangle ) => new[] { triangle.a, triangle.b, triangle.c } ).Distinct( ).ToArray( );

        Assert.GreaterOrEqual( island.Length, 4, "the island has at least the vertices of a tetrahedron" );

        foreach( var vertex in island ) {
            Assert.IsTrue( math.all( remeshed.positions[vertex] >= 10.0 - 1e-9 ) && math.all( remeshed.positions[vertex] <= 11.0 + 1e-9 ), $"the island's vertex at {remeshed.positions[vertex]} is within its cell" );
        }
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
