using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;

using Object        = UnityEngine.Object;
using SmoothNormals = SurfaceExtractor.SmoothNormals;

// BurstCreases on meshes built by hand, whose answers are known in closed form, and on the contours of every algorithm
// and implementation, simplified and not, which are held to what a crease may not change about a mesh. There is no
// second implementation on the CPU to compare with (Documents/Plans/Creased Normals.md, Tests)
public class BurstCreasesTest : ContourVariantsTest {

    // the default crease angle of a voxelizer, in degrees
    private const double angle = 40.0;

    // how far a normal may be from where it should be, in degrees, for rounding to floats
    private const double tolerance = 1e-3;

    // the meshes a test made beyond the contours, which ContouringTest destroys
    private readonly List<Object> made = new( );

    [TearDown]
    public void destroyMade( ) {
        foreach( var item in this.made ) {
            Object.DestroyImmediate( item );
        }
        this.made.Clear( );
    }

    // a mesh by hand: corners at the same position are one vertex, which keeps the normal it was first given
    private class Builder {

        public readonly List<float3>             positions = new( );
        public readonly List<float3>             normals   = new( );
        public readonly List<BurstMesh.Triangle> triangles = new( );

        private readonly Dictionary<float3, int> indices = new( );

        public int vertex( float3 position, float3 normal = default ) {
            if( !this.indices.TryGetValue( position, out var index ) ) {
                index = this.positions.Count;

                this.positions.Add( position );
                this.normals.Add( normal );
                this.indices.Add( position, index );
            }
            return index;
        }

        // a vertex that no triangle uses, even if another is at its position
        public int loose( float3 position, float3 normal ) {
            this.positions.Add( position );
            this.normals.Add( normal );

            return this.positions.Count - 1;
        }

        public void triangle( float3 a, float3 b, float3 c, int subMesh = 0 ) {
            this.triangles.Add( new BurstMesh.Triangle( this.vertex( a ), this.vertex( b ), this.vertex( c ), subMesh ) );
        }

        // the triangle turned to face away from a point inside the shape it is part of
        public void triangleFacingAwayFrom( float3 inside, float3 a, float3 b, float3 c, int subMesh = 0 ) {
            if( math.dot( math.cross( b - a, c - a ), ( ( a + b + c ) / 3.0f ) - inside ) < 0.0f ) {
                this.triangle( a, c, b, subMesh );
            }
            else {
                this.triangle( a, b, c, subMesh );
            }
        }

        // two triangles, a b c and a c d
        public void quad( float3 a, float3 b, float3 c, float3 d, int subMesh = 0 ) {
            this.triangle( a, b, c, subMesh );
            this.triangle( a, c, d, subMesh );
        }

        // the box from a corner by a size, one quad a face, facing out. Every vertex is given the normal along the
        // diagonal from the box's middle, which no face has. The sub-mesh of each face is given its axis and side
        public Builder box( float3 minimum, float3 size, Func<int, int, int> subMeshOf = null ) {
            var middle = minimum + ( size / 2.0f );

            for( var axis = 0; axis < 3; ++axis )
            for( var side = 0; side < 2; ++side ) {
                var u = ( axis + 1 ) % 3;
                var v = ( axis + 2 ) % 3;

                float3 corner( int i, int j ) {
                    var position = minimum;

                    position[axis] += side * size[axis];
                    position[u]    += i * size[u];
                    position[v]    += j * size[v];

                    this.vertex( position, math.normalize( position - middle ) );

                    return position;
                }

                var subMesh = subMeshOf?.Invoke( axis, side ) ?? 0;

                if( side == 1 ) {
                    this.quad( corner( 0, 0 ), corner( 1, 0 ), corner( 1, 1 ), corner( 0, 1 ), subMesh );
                }
                else {
                    this.quad( corner( 0, 0 ), corner( 0, 1 ), corner( 1, 1 ), corner( 1, 0 ), subMesh );
                }
            }
            return this;
        }

    }

    // a crease as managed arrays
    private struct Creased {

        public float3[]                 positions;
        public float3[]                 normals;
        public BurstMesh.Triangle[]     triangles;
        public BurstCreases.Statistics  statistics;

    }

    private static Creased crease( float3[] positions, float3[] normals, BurstMesh.Triangle[] triangles, double angle, SmoothNormals smoothNormals ) {
        using var positionsIn = new NativeArray<float3>( positions, Allocator.TempJob );
        using var normalsIn   = new NativeArray<float3>( normals, Allocator.TempJob );
        using var trianglesIn = new NativeArray<BurstMesh.Triangle>( triangles, Allocator.TempJob );

        BurstCreases.crease( positionsIn, normalsIn, trianglesIn, angle, smoothNormals, Allocator.TempJob, out var creasedPositions, out var creasedNormals, out var creasedTriangles, out var statistics );

        try {
            return new Creased {
                positions  = creasedPositions.ToArray( ),
                normals    = creasedNormals.ToArray( ),
                triangles  = creasedTriangles.ToArray( ),
                statistics = statistics
            };
        }
        finally {
            creasedPositions.Dispose( );
            creasedNormals.Dispose( );
            creasedTriangles.Dispose( );
        }
    }

    private static Creased crease( Builder builder, double angle, SmoothNormals smoothNormals ) {
        return crease( builder.positions.ToArray( ), builder.normals.ToArray( ), builder.triangles.ToArray( ), angle, smoothNormals );
    }

    // a mesh as the arrays the pass takes
    private static ( float3[] positions, float3[] normals, BurstMesh.Triangle[] triangles ) arraysOf( Mesh mesh ) {
        BurstMeshSimplifier.read( mesh, Allocator.TempJob, out var positions, out var normals, out var triangles );

        try {
            return ( positions.ToArray( ), normals.ToArray( ), triangles.ToArray( ) );
        }
        finally {
            positions.Dispose( );
            normals.Dispose( );
            triangles.Dispose( );
        }
    }

    private static float3 cornerOf( float3[] positions, BurstMesh.Triangle triangle, int corner ) {
        return positions[corner switch { 0 => triangle.a, 1 => triangle.b, _ => triangle.c }];
    }

    private static int indexOf( BurstMesh.Triangle triangle, int corner ) {
        return corner switch { 0 => triangle.a, 1 => triangle.b, _ => triangle.c };
    }

    private static double3 faceOf( float3[] positions, BurstMesh.Triangle triangle ) {
        var a = ( double3 )positions[triangle.a];
        var b = ( double3 )positions[triangle.b];
        var c = ( double3 )positions[triangle.c];

        return BurstCreases.unit( math.cross( b - a, c - a ) );
    }

    private static double degreesBetween( double3 a, double3 b ) {
        return math.degrees( math.atan2( math.length( math.cross( a, b ) ), math.dot( a, b ) ) );
    }

    // the angle between a corner's normal and its triangle's, for every corner of a triangle with area
    private static IEnumerable<( int triangle, int corner, double degrees )> cornerAnglesOf( Creased creased ) {
        for( var triangle = 0; triangle < creased.triangles.Length; ++triangle ) {
            var t    = creased.triangles[triangle];
            var face = faceOf( creased.positions, t );

            if( math.all( face == double3.zero ) ) {
                continue;
            }

            for( var corner = 0; corner < 3; ++corner ) {
                yield return ( triangle, corner, degreesBetween( face, creased.normals[indexOf( t, corner )] ) );
            }
        }
    }

    private static void assertCornersWithin( Creased creased, double degrees, string what ) {
        foreach( var ( triangle, corner, between ) in cornerAnglesOf( creased ) ) {
            Assert.LessOrEqual( between, degrees + tolerance, $"{what}: degrees between the normal of corner {corner} of triangle {triangle} and its triangle's" );
        }
    }

    // the creased triangles are the triangles of the mesh: in the same order and sub-meshes, with their corners where
    // they were
    private static void assertSameTriangles( float3[] positions, BurstMesh.Triangle[] triangles, Creased creased, string what ) {
        Assert.AreEqual( triangles.Length, creased.triangles.Length, $"{what}: triangles" );

        for( var triangle = 0; triangle < triangles.Length; ++triangle ) {
            Assert.AreEqual( triangles[triangle].subMeshIndex, creased.triangles[triangle].subMeshIndex, $"{what}: sub-mesh of triangle {triangle}" );

            for( var corner = 0; corner < 3; ++corner ) {
                Assert.AreEqual( cornerOf( positions, triangles[triangle], corner ), cornerOf( creased.positions, creased.triangles[triangle], corner ), $"{what}: corner {corner} of triangle {triangle}" );
            }
        }
    }

    private static void assertUnitNormals( Creased creased, string what ) {
        for( var vertex = 0; vertex < creased.normals.Length; ++vertex ) {
            Assert.AreEqual( 1.0, math.length( ( double3 )creased.normals[vertex] ), 1e-5, $"{what}: length of the normal of vertex {vertex}" );
        }
    }

    private static void assertNormal( float3 expected, float3 actual, string what ) {
        Assert.LessOrEqual( degreesBetween( expected, actual ), tolerance, $"{what}: degrees between {actual} and {expected}" );
    }

    // ---- shapes with known answers ----

    // a cube of eight shared vertices whose faces meet at 90 degrees: under any angle below that every face gets
    // vertices of its own with its normal, whatever normals the vertices came with
    [Test]
    public void cubeGetsTheNormalsOfItsFaces( [Values( 0.0, 10.0, angle, 89.0 )] double degrees, [Values] SmoothNormals smoothNormals ) {
        var builder = new Builder( ).box( float3.zero, new float3( 1.0f ) );
        var creased = crease( builder, degrees, smoothNormals );

        Assert.AreEqual( 8, builder.positions.Count, "vertices of the cube" );
        Assert.AreEqual( 24, creased.positions.Length, "creased vertices" );
        Assert.AreEqual( 24, creased.statistics.outputVertexCount, "creased vertices in the statistics" );
        Assert.AreEqual( 8, creased.statistics.splitVertexCount, "split vertices" );
        Assert.AreEqual( 0, creased.statistics.guardedCornerCount, "guarded corners" );

        assertSameTriangles( builder.positions.ToArray( ), builder.triangles.ToArray( ), creased, "cube" );
        assertCornersWithin( creased, 0.0, "cube" );
    }

    // past 90 degrees the three faces at every corner are one group. Under Faces the normal is the diagonal, since the
    // corner angles of a square face add up to 90 degrees however it is cut into triangles. Under Contour the
    // diagonal the vertices came with is within the angle of every face, and stays
    [Test]
    public void cubeIsSmoothPastNinetyDegrees( [Values( 91.0, 179.0, 180.0 )] double degrees, [Values] SmoothNormals smoothNormals ) {
        var builder = new Builder( ).box( new float3( -0.5f ), new float3( 1.0f ) );
        var creased = crease( builder, degrees, smoothNormals );

        Assert.AreEqual( 8, creased.positions.Length, "creased vertices" );
        Assert.AreEqual( 0, creased.statistics.splitVertexCount, "split vertices" );

        for( var vertex = 0; vertex < 8; ++vertex ) {
            assertNormal( math.normalize( creased.positions[vertex] ), creased.normals[vertex], $"vertex {vertex}" );
        }

        Assert.AreEqual( smoothNormals == SmoothNormals.Contour ? 8 : 0, creased.statistics.keptNormalCount, "contour normals kept" );
    }

    // a prism of n sides turns 360 / n degrees from one side to the next, and its caps are at 90 degrees to its sides.
    // Each side is cut into two triangles along the same diagonal, so a rim vertex has one triangle of one side and
    // two of the other: the sides that stay together still give the rim the direction straight out, since a corner
    // of a rectangle is 90 degrees however it is cut
    [Test]
    public void prismSidesSplitWhereTheyTurnPastTheAngle( [Values( 3, 4, 5, 6, 8, 10, 12, 24 )] int sides, [Values] SmoothNormals smoothNormals ) {
        var builder = new Builder( );
        var bottom  = new float3( 0.0f, 0.0f, 0.0f );
        var top     = new float3( 0.0f, 1.0f, 0.0f );

        // the last side ends where the first begins: sin( 2 pi ) is not 0 in doubles
        float3 rim( int side, float height ) {
            var turn = 2.0 * math.PI_DBL * ( side % sides ) / sides;

            return new float3( ( float )math.cos( turn ), height, ( float )math.sin( turn ) );
        }

        for( var side = 0; side < sides; ++side ) {
            var b0 = rim( side, 0.0f );
            var b1 = rim( side + 1, 0.0f );
            var t0 = rim( side, 1.0f );
            var t1 = rim( side + 1, 1.0f );

            builder.triangle( top, t1, t0 );
            builder.triangle( bottom, b0, b1 );
            builder.quad( b0, t0, t1, b1 );
        }

        var creased = crease( builder, angle, smoothNormals );
        var split   = 360.0 / sides > angle;

        Assert.AreEqual( 2 + ( 2 * sides * ( split ? 3 : 2 ) ), creased.positions.Length, "creased vertices" );

        assertSameTriangles( builder.positions.ToArray( ), builder.triangles.ToArray( ), creased, "prism" );
        assertCornersWithin( creased, angle, "prism" );

        // the normals at the rim, other than the caps'
        for( var vertex = 0; vertex < creased.positions.Length; ++vertex ) {
            var position = creased.positions[vertex];
            var normal   = creased.normals[vertex];

            if( math.abs( normal.y ) > 0.5f ) {
                assertNormal( new float3( 0.0f, math.sign( normal.y ), 0.0f ), normal, $"cap normal at {position}" );
                continue;
            }

            var outward = math.normalize( new float3( position.x, 0.0f, position.z ) );

            if( split ) {
                Assert.AreEqual( 180.0 / sides, degreesBetween( outward, normal ), tolerance, $"degrees between the normal of a side at {position} and straight out" );
            }
            else {
                assertNormal( outward, normal, $"side normal at {position}" );
            }
        }
    }

    // a flat patch of triangles of all shapes splits at no angle, and its normals are the patch's
    [Test]
    public void flatPatchSplitsNothing( [Values( 0.0, angle, 180.0 )] double degrees, [Values] SmoothNormals smoothNormals ) {
        var builder = new Builder( );
        var random  = new Unity.Mathematics.Random( 7 );
        var up      = new float3( 0.0f, 1.0f, 0.0f );
        var points  = new float3[9, 9];

        for( var i = 0; i < 9; ++i )
        for( var j = 0; j < 9; ++j ) {
            points[i, j] = new float3( i + random.NextFloat( -0.3f, 0.3f ), 0.0f, j + random.NextFloat( -0.3f, 0.3f ) );

            builder.vertex( points[i, j], up );
        }

        for( var i = 0; i < 8; ++i )
        for( var j = 0; j < 8; ++j ) {
            builder.quad( points[i, j], points[i, j + 1], points[i + 1, j + 1], points[i + 1, j] );
        }

        var creased = crease( builder, degrees, smoothNormals );

        Assert.AreEqual( builder.positions.Count, creased.positions.Length, "creased vertices" );

        for( var vertex = 0; vertex < creased.normals.Length; ++vertex ) {
            Assert.AreEqual( up, creased.normals[vertex], $"normal of vertex {vertex}" );
        }
    }

    // a sphere of 48 sides and 24 rings turns 7.5 degrees from one triangle to the next, and splits nowhere. Under
    // Contour its normals are the ones it came with, and under Faces they are within a few degrees of them
    [Test]
    public void sphereSplitsNothing( [Values] SmoothNormals smoothNormals ) {
        const int sides = 48;
        const int rings = 24;

        var builder = new Builder( );

        float3 at( int ring, int side ) {
            if( ring == 0 ) {
                return new float3( 0.0f, 1.0f, 0.0f );
            }
            if( ring == rings ) {
                return new float3( 0.0f, -1.0f, 0.0f );
            }

            var down   = math.PI_DBL * ring / rings;
            var around = 2.0 * math.PI_DBL * ( side % sides ) / sides;

            var position = new float3( ( float )( math.sin( down ) * math.cos( around ) ), ( float )math.cos( down ), ( float )( math.sin( down ) * math.sin( around ) ) );

            builder.vertex( position, math.normalize( position ) );

            return position;
        }

        builder.vertex( at( 0, 0 ), new float3( 0.0f, 1.0f, 0.0f ) );
        builder.vertex( at( rings, 0 ), new float3( 0.0f, -1.0f, 0.0f ) );

        for( var ring = 0; ring < rings; ++ring )
        for( var side = 0; side < sides; ++side ) {
            var a = at( ring, side );
            var b = at( ring, side + 1 );
            var c = at( ring + 1, side + 1 );
            var d = at( ring + 1, side );

            if( ring != 0 ) {
                builder.triangleFacingAwayFrom( float3.zero, a, b, c );
            }
            if( ring != rings - 1 ) {
                builder.triangleFacingAwayFrom( float3.zero, a, c, d );
            }
        }

        var creased = crease( builder, angle, smoothNormals );

        Assert.AreEqual( builder.positions.Count, creased.positions.Length, "creased vertices" );
        Assert.AreEqual( 0, creased.statistics.splitVertexCount, "split vertices" );

        for( var vertex = 0; vertex < creased.normals.Length; ++vertex ) {
            if( smoothNormals == SmoothNormals.Contour ) {
                Assert.AreEqual( builder.normals[vertex], creased.normals[vertex], $"normal of vertex {vertex}" );
            }
            else {
                Assert.LessOrEqual( degreesBetween( builder.normals[vertex], creased.normals[vertex] ), 2.0, $"degrees between the normal of vertex {vertex} and the sphere's" );
            }
        }
    }

    // the tip of a cone of 24 sides whose faces lean 60 degrees from its axis. Neighbouring faces are 13 degrees
    // apart, so joining a triangle to its neighbours would gather the whole tip into one group with the axis for its
    // normal, 60 degrees from every face. Joining by the group's normal keeps every face within the angle
    [Test]
    public void coneTipDoesNotGatherIntoOneGroup( [Values] SmoothNormals smoothNormals ) {
        const int sides = 24;

        var builder = new Builder( );
        var tip     = new float3( 0.0f, ( float )( 1.0 / math.tan( math.radians( 30.0 ) ) ), 0.0f );
        var inside  = new float3( 0.0f, tip.y / 3.0f, 0.0f );

        float3 rim( int side ) {
            var turn = 2.0 * math.PI_DBL * ( side % sides ) / sides;

            return new float3( ( float )math.cos( turn ), 0.0f, ( float )math.sin( turn ) );
        }

        for( var side = 0; side < sides; ++side ) {
            builder.triangleFacingAwayFrom( inside, tip, rim( side ), rim( side + 1 ) );
        }

        var creased = crease( builder, angle, smoothNormals );
        var tips    = Enumerable.Range( 0, creased.positions.Length ).Count( ( vertex ) => creased.positions[vertex].Equals( tip ) );

        Assert.GreaterOrEqual( tips, 3, "groups at the tip" );

        // and no group's normal moved away from a corner in it, which the guard would have caught (finding 11):
        // joining each triangle to the one before it gathers the tip into one group, and the guard takes 17 out
        Assert.AreEqual( 0, creased.statistics.guardedCornerCount, "guarded corners" );

        assertCornersWithin( creased, angle, "cone" );
    }

    // four triangles at a vertex whose normals turn 0, 39, 58.5 and 71 degrees about one axis, with corners of 1,
    // 0.99, 0.98 and 0.97 radians. Taken largest first each is within 40 degrees of the group's normal when it joins,
    // and the group's normal ends up 42.4 degrees from the first. The guard gives the first a group of its own
    [Test]
    public void groupThatMovesAwayFromACornerLetsItGo( [Values] SmoothNormals smoothNormals ) {
        var builder = new Builder( );
        var turns   = new[] { 0.0, 39.0, 58.5, 71.0 };
        var corners = new[] { 1.0, 0.99, 0.98, 0.97 };
        var axis    = new double3( 0.0, 1.0, 0.0 );

        for( var index = 0; index < turns.Length; ++index ) {
            var normal = new double3( math.sin( math.radians( turns[index] ) ), 0.0, math.cos( math.radians( turns[index] ) ) );
            var across = math.cross( normal, axis );
            var scale  = 1.0 + index;

            var b = axis * scale;
            var c = ( ( axis * math.cos( corners[index] ) ) + ( across * math.sin( corners[index] ) ) ) * scale;

            builder.triangle( float3.zero, ( float3 )b, ( float3 )c );
        }

        var creased = crease( builder, angle, smoothNormals );

        Assert.AreEqual( 1, creased.statistics.guardedCornerCount, "guarded corners" );
        Assert.AreEqual( 2 + 8, creased.positions.Length, "creased vertices: two at the shared corner and one at each other" );

        assertCornersWithin( creased, angle, "turning triangles" );
    }

    // corners of the same size go in the order of their triangles, which the GPU must follow too. Three triangles at a
    // vertex with corners of exactly 90 degrees and normals 45 degrees apart: at 50 degrees the first two make a group
    // and the third is alone. Taken the other way round, the last two would make the group
    [Test]
    public void cornersOfTheSameSizeGoInTheOrderOfTheirTriangles( [Values] SmoothNormals smoothNormals ) {
        var builder = new Builder( );

        builder.triangle( float3.zero, new float3( 1.0f, 0.0f, 0.0f ), new float3( 0.0f, 0.0f, 1.0f ) );
        builder.triangle( float3.zero, new float3( 1.0f, 1.0f, 0.0f ), new float3( 0.0f, 0.0f, 2.0f ) );
        builder.triangle( float3.zero, new float3( 0.0f, 1.0f, 0.0f ), new float3( 0.0f, 0.0f, 3.0f ) );

        var creased = crease( builder, 50.0, smoothNormals );

        Assert.AreEqual( 0, creased.statistics.guardedCornerCount, "guarded corners" );
        Assert.AreEqual( creased.triangles[0].a, creased.triangles[1].a, "the first two triangles share their corner" );
        Assert.AreNotEqual( creased.triangles[1].a, creased.triangles[2].a, "the third has its own" );
    }

    // ---- what a crease keeps ----

    // under Contour, a vertex of one group keeps the normal it came with where that is within the angle of every one
    // of its triangles, and takes its triangles' otherwise. A zero normal never stays, but at 180 degrees every
    // normal does
    [Test]
    public void contourNormalGivesWayWhereItDoesNotFit( [Values( angle, 180.0 )] double degrees ) {
        var builder = new Builder( );
        var up      = new float3( 0.0f, 1.0f, 0.0f );

        for( var i = 0; i < 4; ++i )
        for( var j = 0; j < 4; ++j ) {
            builder.vertex( new float3( i, 0.0f, j ), up );
        }

        for( var i = 0; i < 3; ++i )
        for( var j = 0; j < 3; ++j ) {
            builder.quad( new float3( i, 0.0f, j ), new float3( i, 0.0f, j + 1 ), new float3( i + 1, 0.0f, j + 1 ), new float3( i + 1, 0.0f, j ) );
        }

        float3 tilted( double degrees ) {
            return new float3( ( float )math.sin( math.radians( degrees ) ), ( float )math.cos( math.radians( degrees ) ), 0.0f );
        }

        // four vertices of the middle of the patch: one without a normal, one flipped, one 60 degrees off and one 10
        var normals = new Dictionary<int, float3> {
            [builder.vertex( new float3( 1.0f, 0.0f, 1.0f ) )] = float3.zero,
            [builder.vertex( new float3( 1.0f, 0.0f, 2.0f ) )] = -up,
            [builder.vertex( new float3( 2.0f, 0.0f, 1.0f ) )] = tilted( 60.0 ),
            [builder.vertex( new float3( 2.0f, 0.0f, 2.0f ) )] = tilted( 10.0 )
        };

        foreach( var ( vertex, normal ) in normals ) {
            builder.normals[vertex] = normal;
        }

        var creased = crease( builder, degrees, SmoothNormals.Contour );
        var all     = degrees >= 180.0;

        Assert.AreEqual( builder.positions.Count, creased.positions.Length, "creased vertices" );
        Assert.AreEqual( all ? 16 : 13, creased.statistics.keptNormalCount, "contour normals kept" );
        Assert.AreEqual( all ? 0 : 3, creased.statistics.replacedNormalCount, "contour normals replaced" );

        for( var vertex = 0; vertex < creased.normals.Length; ++vertex ) {
            var kept     = all || !normals.TryGetValue( vertex, out var given ) || given.Equals( tilted( 10.0 ) );
            var expected = kept ? builder.normals[vertex] : up;

            Assert.AreEqual( expected, creased.normals[vertex], $"normal of vertex {vertex}" );
        }
    }

    // a triangle without area, along an edge of a patch, has no normal and has no say in the normals of its vertices
    [Test]
    public void triangleWithoutAreaHasNoSay( [Values] SmoothNormals smoothNormals ) {
        var builder = new Builder( ).box( float3.zero, new float3( 1.0f ) );

        // along the edge from (0, 0, 0) to (1, 0, 0), through a vertex of its own halfway. That vertex has no other
        // triangle, so it keeps the normal it came with
        builder.vertex( new float3( 0.5f, 0.0f, 0.0f ), math.normalize( new float3( 0.0f, -1.0f, -1.0f ) ) );
        builder.triangle( new float3( 0.0f, 0.0f, 0.0f ), new float3( 0.5f, 0.0f, 0.0f ), new float3( 1.0f, 0.0f, 0.0f ) );

        var creased = crease( builder, angle, smoothNormals );

        Assert.AreEqual( 1, creased.statistics.zeroAreaTriangleCount, "triangles without area" );
        Assert.AreEqual( 24 + 1, creased.positions.Length, "creased vertices: the cube's and the one halfway" );

        assertSameTriangles( builder.positions.ToArray( ), builder.triangles.ToArray( ), creased, "cube with a flat triangle" );
        assertCornersWithin( creased, 0.0, "cube with a flat triangle" );

        for( var vertex = 0; vertex < creased.normals.Length; ++vertex ) {
            Assert.AreEqual( 1.0, math.length( ( double3 )creased.normals[vertex] ), 1e-5, $"length of the normal of vertex {vertex}, at {creased.positions[vertex]}" );
        }
    }

    // a vertex no triangle uses stays, in its place among the vertices, with its position and its normal
    [Test]
    public void looseVertexStaysInItsPlace( [Values] SmoothNormals smoothNormals ) {
        var builder = new Builder( );
        var normal  = new float3( 0.6f, 0.0f, 0.8f );

        builder.box( float3.zero, new float3( 1.0f ) );
        builder.loose( new float3( 5.0f, 6.0f, 7.0f ), normal );
        builder.box( new float3( 2.0f, 0.0f, 0.0f ), new float3( 1.0f ) );

        var creased = crease( builder, angle, smoothNormals );

        Assert.AreEqual( 24 + 1 + 24, creased.positions.Length, "creased vertices" );
        Assert.AreEqual( new float3( 5.0f, 6.0f, 7.0f ), creased.positions[24], "position of the loose vertex" );
        Assert.AreEqual( normal, creased.normals[24], "normal of the loose vertex" );

        assertSameTriangles( builder.positions.ToArray( ), builder.triangles.ToArray( ), creased, "two cubes" );
    }

    // an edge of four triangles where two boxes meet along it, and a vertex where two more meet at a corner. The
    // corner they share has six faces around it, each its own group
    [Test]
    public void edgeOfFourTrianglesAndPinchedVertex( [Values] SmoothNormals smoothNormals ) {
        var builder = new Builder( );

        builder.box( new float3( 0.0f, 0.0f, 0.0f ), new float3( 1.0f ) );
        builder.box( new float3( -1.0f, -1.0f, 0.0f ), new float3( 1.0f ) );
        builder.box( new float3( 3.0f, 0.0f, 0.0f ), new float3( 1.0f ) );
        builder.box( new float3( 2.0f, -1.0f, -1.0f ), new float3( 1.0f ) );

        var creased = crease( builder, angle, smoothNormals );

        assertSameTriangles( builder.positions.ToArray( ), builder.triangles.ToArray( ), creased, "boxes" );
        assertCornersWithin( creased, 0.0, "boxes" );
        assertUnitNormals( creased, "boxes" );

        var pinched = new float3( 3.0f, 0.0f, 0.0f );

        Assert.AreEqual( 6, creased.positions.Count( ( position ) => position.Equals( pinched ) ), "vertices at the pinched corner" );
    }

    // a cube a ten thousandth across: the cross products of its triangles are under 1e-5 long, which Unity's own
    // normalize would leave at zero (finding 7)
    [Test]
    public void tinyCubeGetsTheNormalsOfItsFaces( [Values] SmoothNormals smoothNormals ) {
        var builder = new Builder( ).box( new float3( 0.3f, 0.2f, 0.1f ), new float3( 1e-4f ) );
        var creased = crease( builder, angle, smoothNormals );

        Assert.AreEqual( 24, creased.positions.Length, "creased vertices" );

        assertCornersWithin( creased, 0.0, "tiny cube" );
        assertUnitNormals( creased, "tiny cube" );
    }

    // ---- meshes ----

    // through meshes: the sub-meshes keep their triangles, an empty sub-mesh between two others stays, and the index
    // format follows the creased vertices. 3,000 cubes have 24,000 vertices, which take 16 bit indices, and 72,000
    // once creased, which do not
    [Test]
    public void meshKeepsItsSubMeshesAndGetsTheIndicesItNeeds( [Values( 1, 3000 )] int cubes ) {
        var builder = new Builder( );

        for( var cube = 0; cube < cubes; ++cube ) {
            builder.box( new float3( 2 * ( cube % 60 ), 2 * ( cube / 60 ), 0.0f ), new float3( 1.0f ), ( axis, side ) => axis == 2 ? 3 : axis );
        }

        using var positions = new NativeArray<float3>( builder.positions.ToArray( ), Allocator.TempJob );
        using var normals   = new NativeArray<float3>( builder.normals.ToArray( ), Allocator.TempJob );
        using var triangles = new NativeArray<BurstMesh.Triangle>( builder.triangles.ToArray( ), Allocator.TempJob );

        var mesh    = BurstMesh.createMesh( positions, normals, triangles );
        var creased = BurstCreases.crease( mesh, angle, SmoothNormals.Contour, out _ );

        this.made.Add( mesh );
        this.made.Add( creased );

        Assert.AreEqual( IndexFormat.UInt16, mesh.indexFormat, "index format of the mesh" );
        Assert.AreEqual( cubes == 1 ? IndexFormat.UInt16 : IndexFormat.UInt32, creased.indexFormat, "index format of the creased mesh" );
        Assert.AreEqual( 24 * cubes, creased.vertexCount, "creased vertices" );
        Assert.AreEqual( 4, creased.subMeshCount, "sub-meshes" );

        var vertices        = mesh.vertices;
        var creasedVertices = creased.vertices;

        for( var subMesh = 0; subMesh < 4; ++subMesh ) {
            var indices        = mesh.GetIndices( subMesh );
            var creasedIndices = creased.GetIndices( subMesh );

            Assert.AreEqual( subMesh == 2 ? 0 : 12 * cubes, creasedIndices.Length, $"indices of sub-mesh {subMesh}" );

            for( var index = 0; index < indices.Length; ++index ) {
                if( !vertices[indices[index]].Equals( creasedVertices[creasedIndices[index]] ) ) {
                    Assert.Fail( $"index {index} of sub-mesh {subMesh} is at {creasedVertices[creasedIndices[index]]} instead of {vertices[indices[index]]}" );
                }
            }
        }
    }

    // ---- contours ----

    // the contour of a surface, simplified as a voxelizer does by default when asked to
    private Mesh contourOf( Contour contour, Surface surface, bool simplified ) {
        var voxelizer = this.voxelizerOf( contour, shapesOf( surface ), isManaged( contour ) ? 4 : 5 );
        var mesh      = this.voxelize( voxelizer ).mesh;

        if( !simplified ) {
            return mesh;
        }

        var simplifiedMesh = BurstMeshSimplifier.simplify( mesh, voxelizer.simplificationError * voxelizer.cellSize, voxelizer.lineQuadricWeight, out _ );

        this.made.Add( simplifiedMesh );

        return simplifiedMesh;
    }

    // every contour, simplified or not, keeps its triangles, and has unit normals no further than the angle from the
    // normal of any triangle they are a corner of, under either option
    [Test]
    public void creasedContourKeepsItsTrianglesAndTurnsNoCornerPastTheAngle( [Values] Contour contour, [Values] Surface surface, [Values] bool simplified ) {
        var ( positions, normals, triangles ) = arraysOf( this.contourOf( contour, surface, simplified ) );

        foreach( var smoothNormals in new[] { SmoothNormals.Contour, SmoothNormals.Faces } ) {
            var creased = crease( positions, normals, triangles, angle, smoothNormals );
            var what    = $"{contour} {surface}{( simplified ? " simplified" : "" )} {smoothNormals}";

            Assert.GreaterOrEqual( creased.positions.Length, positions.Length, $"{what}: creased vertices" );

            assertSameTriangles( positions, triangles, creased, what );
            assertCornersWithin( creased, angle, what );
            assertUnitNormals( creased, what );
        }
    }

    // at 180 degrees under Contour the crease gives the mesh back as it was, bit for bit, which is today's Shared
    [Test]
    public void fullAngleUnderContourGivesTheMeshBack(
        [Values( Contour.UniformBurst, Contour.UniformManaged, Contour.ManifoldClustered, Contour.OccupancyBurst )] Contour contour,
        [Values( Surface.TurnedShapes, Surface.SphereCutaway, Surface.PlateAcrossBounds )] Surface surface,
        [Values] bool simplified
    ) {
        var mesh = this.contourOf( contour, surface, simplified );

        var ( positions, normals, triangles ) = arraysOf( mesh );

        var arrays = crease( positions, normals, triangles, 180.0, SmoothNormals.Contour );

        CollectionAssert.AreEqual( positions, arrays.positions, "positions" );
        CollectionAssert.AreEqual( normals, arrays.normals, "normals" );
        CollectionAssert.AreEqual( triangles, arrays.triangles, "triangles" );

        var creased = BurstCreases.crease( mesh, 180.0, SmoothNormals.Contour, out _ );

        this.made.Add( creased );

        Assert.AreEqual( mesh.indexFormat, creased.indexFormat, "index format" );
        Assert.AreEqual( mesh.subMeshCount, creased.subMeshCount, "sub-meshes" );
        CollectionAssert.AreEqual( mesh.vertices, creased.vertices, "mesh vertices" );
        CollectionAssert.AreEqual( mesh.normals, creased.normals, "mesh normals" );

        for( var subMesh = 0; subMesh < mesh.subMeshCount; ++subMesh ) {
            CollectionAssert.AreEqual( mesh.GetIndices( subMesh ), creased.GetIndices( subMesh ), $"indices of sub-mesh {subMesh}" );
        }
    }

    // at 0 degrees every corner has the normal of its own triangle, as Split gives it
    [Test]
    public void zeroAngleGivesEveryCornerItsTriangle( [Values( Contour.UniformBurst, Contour.ManifoldClustered, Contour.OccupancyBurst )] Contour contour, [Values] SmoothNormals smoothNormals ) {
        var ( positions, normals, triangles ) = arraysOf( this.contourOf( contour, Surface.TurnedShapes, false ) );

        assertCornersWithin( crease( positions, normals, triangles, 0.0, smoothNormals ), 0.0, $"{contour}" );
    }

    // the same mesh twice gives the same crease
    [Test]
    public void sameMeshGivesTheSameCrease( [Values( Contour.UniformBurst, Contour.OccupancyBurst )] Contour contour ) {
        var ( positions, normals, triangles ) = arraysOf( this.contourOf( contour, Surface.NewShapes, false ) );

        var first  = crease( positions, normals, triangles, angle, SmoothNormals.Faces );
        var second = crease( positions, normals, triangles, angle, SmoothNormals.Faces );

        CollectionAssert.AreEqual( first.positions, second.positions, "positions" );
        CollectionAssert.AreEqual( first.normals, second.normals, "normals" );
        CollectionAssert.AreEqual( first.triangles, second.triangles, "triangles" );
    }

}
