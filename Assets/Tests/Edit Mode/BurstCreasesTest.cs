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
using Builder       = CreaseShapes.Builder;

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

    // the normal of a triangle as the crease has it, zero for one too flat to have a normal
    private static double3 faceOf( float3[] positions, BurstMesh.Triangle triangle ) {
        return BurstCreases.faceOf( positions[triangle.a], positions[triangle.b], positions[triangle.c] );
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
        var builder = CreaseShapes.prism( sides );
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
        var builder = CreaseShapes.flatPatch( );
        var up      = new float3( 0.0f, 1.0f, 0.0f );

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
        var builder = CreaseShapes.sphere( );
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
        var builder = CreaseShapes.coneTip( out var tip );
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
        var builder = CreaseShapes.turningTriangles( );
        var creased = crease( builder, angle, smoothNormals );

        Assert.AreEqual( 1, creased.statistics.guardedCornerCount, "guarded corners" );
        Assert.AreEqual( 2 + 8, creased.positions.Length, "creased vertices: two at the shared corner and one at each other" );

        assertCornersWithin( creased, angle, "turning triangles" );
    }

    // corners of the same size go by their triangles' normals, x first, whatever the order of the triangles, which the
    // GPU follows too. Three triangles at a vertex with corners of exactly 90 degrees and normals 45 degrees apart, at
    // (0, -1, 0), (1, -1, 0) / sqrt( 2 ) and (1, 0, 0): at 50 degrees the first two by x make a group and the third is
    // alone, in either order of the triangles. Taken the other way round, the last two would make the group
    [Test]
    public void cornersOfTheSameSizeGoByTheirNormals( [Values] SmoothNormals smoothNormals, [Values] bool reversed ) {
        var builder = CreaseShapes.cornersOfTheSameSize( reversed );
        var creased = crease( builder, 50.0, smoothNormals );

        // the triangles in the order of their normals' x
        var first  = creased.triangles[reversed ? 2 : 0].a;
        var second = creased.triangles[1].a;
        var third  = creased.triangles[reversed ? 0 : 2].a;

        Assert.AreEqual( 0, creased.statistics.guardedCornerCount, "guarded corners" );
        Assert.AreEqual( first, second, "the first two by their normals share their corner" );
        Assert.AreNotEqual( second, third, "the third has its own" );
    }

    // ---- what a crease keeps ----

    // under Contour, a vertex of one group keeps the normal it came with where that is within the angle of every one
    // of its triangles, and takes its triangles' otherwise. A zero normal never stays, but at 180 degrees every
    // normal does
    [Test]
    public void contourNormalGivesWayWhereItDoesNotFit( [Values( angle, 180.0 )] double degrees ) {
        var builder = CreaseShapes.patchWithOddNormals( out var normals );
        var up      = new float3( 0.0f, 1.0f, 0.0f );

        var creased = crease( builder, degrees, SmoothNormals.Contour );
        var all     = degrees >= 180.0;

        Assert.AreEqual( builder.positions.Count, creased.positions.Length, "creased vertices" );
        Assert.AreEqual( all ? 16 : 13, creased.statistics.keptNormalCount, "contour normals kept" );
        Assert.AreEqual( all ? 0 : 3, creased.statistics.replacedNormalCount, "contour normals replaced" );

        for( var vertex = 0; vertex < creased.normals.Length; ++vertex ) {
            var kept     = all || !normals.TryGetValue( vertex, out var given ) || given.Equals( CreaseShapes.tilted( 10.0 ) );
            var expected = kept ? builder.normals[vertex] : up;

            Assert.AreEqual( expected, creased.normals[vertex], $"normal of vertex {vertex}" );
        }
    }

    // a triangle too flat to have a normal, along an edge of a cube, has no say in the normals of its vertices. With
    // one, its corners at the ends of the edge would start groups of their own, 45 degrees from both faces
    [Test]
    public void triangleTooFlatToHaveANormalHasNoSay( [Values] SmoothNormals smoothNormals ) {
        var builder = CreaseShapes.cubeWithFlatTriangle( );
        var creased = crease( builder, angle, smoothNormals );

        Assert.AreEqual( 1, creased.statistics.flatTriangleCount, "triangles too flat to have a normal" );
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
        var normal  = new float3( 0.6f, 0.0f, 0.8f );
        var builder = CreaseShapes.cubesWithALooseVertex( normal );
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
        var builder = CreaseShapes.boxesAlongAnEdgeAndAtACorner( );
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
