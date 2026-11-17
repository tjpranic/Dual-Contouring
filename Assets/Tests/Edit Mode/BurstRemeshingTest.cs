using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

using Object = UnityEngine.Object;

// what every remeshing technique shares (BurstRemeshing, Remeshing) and the stand-in technique that tests the frame,
// on meshes built by hand whose answers are known: the edge length that keeps a mesh's vertex count, its area, its
// open and non-manifold edges, its faces by their sides, which triangles lie flat together, and the order of the flags
public class BurstRemeshingTest {

    // builds a mesh from triangles given by the positions of their corners. Corners at the same position are one vertex
    private class Builder {

        public readonly List<Vector3>            positions = new( );
        public readonly List<BurstMesh.Triangle> triangles = new( );

        private readonly Dictionary<Vector3, int> indices = new( );

        public int vertex( Vector3 position ) {
            if( !this.indices.TryGetValue( position, out var index ) ) {
                index = this.positions.Count;

                this.positions.Add( position );
                this.indices.Add( position, index );
            }
            return index;
        }

        public Builder triangle( Vector3 a, Vector3 b, Vector3 c, int subMesh = 0 ) {
            this.triangles.Add( new BurstMesh.Triangle( this.vertex( a ), this.vertex( b ), this.vertex( c ), subMesh ) );
            return this;
        }

        // two triangles, a b c and a c d, whose diagonal runs from a to c
        public Builder quad( Vector3 a, Vector3 b, Vector3 c, Vector3 d, int subMesh = 0 ) {
            this.triangle( a, b, c, subMesh );
            return this.triangle( a, c, d, subMesh );
        }

        // a flat patch of squares of the size in the plane z = 0, facing up, with open edges all around
        public Builder patch( int width, int height, float size = 1.0f, Func<int, int, int> subMeshOf = null ) {
            for( var i = 0; i < width; ++i )
            for( var j = 0; j < height; ++j ) {
                this.quad(
                    new Vector3( i, j, 0 ) * size, new Vector3( i + 1, j, 0 ) * size, new Vector3( i + 1, j + 1, 0 ) * size, new Vector3( i, j + 1, 0 ) * size,
                    subMeshOf == null ? 0 : subMeshOf( i, j )
                );
            }
            return this;
        }

        // the surface of a box of unit cells, facing out
        public Builder box( Vector3Int minimum, Vector3Int size ) {
            for( var axis = 0; axis < 3; ++axis )
            for( var side = 0; side < 2; ++side ) {
                var u = ( axis + 1 ) % 3;
                var v = ( axis + 2 ) % 3;

                for( var i = 0; i < size[u]; ++i )
                for( var j = 0; j < size[v]; ++j ) {
                    Vector3 corner( int di, int dj ) {
                        var position = Vector3.zero;

                        position[axis] = minimum[axis] + ( side * size[axis] );
                        position[u]    = minimum[u] + i + di;
                        position[v]    = minimum[v] + j + dj;

                        return position;
                    }

                    if( side == 1 ) {
                        this.quad( corner( 0, 0 ), corner( 1, 0 ), corner( 1, 1 ), corner( 0, 1 ) );
                    }
                    else {
                        this.quad( corner( 0, 0 ), corner( 0, 1 ), corner( 1, 1 ), corner( 1, 0 ) );
                    }
                }
            }
            return this;
        }

        // the regular octahedron with corners on the axes at 1, facing out
        public Builder octahedron( ) {
            var corners = new[] { Vector3.right, Vector3.up, Vector3.forward, Vector3.left, Vector3.down, Vector3.back };

            foreach( var x in new[] { 0, 3 } )
            foreach( var y in new[] { 1, 4 } )
            foreach( var z in new[] { 2, 5 } ) {
                var a = corners[x];
                var b = corners[y];
                var c = corners[z];

                // wound so that its normal points away from the centre
                if( Vector3.Dot( Vector3.Cross( b - a, c - a ), a + b + c ) > 0.0f ) {
                    this.triangle( a, b, c );
                }
                else {
                    this.triangle( a, c, b );
                }
            }
            return this;
        }

        // a sphere of latitudes and longitudes, facing out
        public Builder sphere( int latitudes, int longitudes ) {
            Vector3 at( int i, int j ) {
                if( i == 0 ) {
                    return Vector3.down;
                }
                if( i == latitudes ) {
                    return Vector3.up;
                }

                var polar   = Mathf.PI * i / latitudes;
                var azimuth = 2.0f * Mathf.PI * ( j % longitudes ) / longitudes;

                return new Vector3( Mathf.Sin( polar ) * Mathf.Cos( azimuth ), -Mathf.Cos( polar ), Mathf.Sin( polar ) * Mathf.Sin( azimuth ) );
            }

            for( var i = 0; i < latitudes; ++i )
            for( var j = 0; j < longitudes; ++j ) {
                var a = at( i, j );
                var b = at( i, j + 1 );
                var c = at( i + 1, j + 1 );
                var d = at( i + 1, j );

                if( i > 0 ) {
                    this.triangle( a, d, b );
                }
                if( i < latitudes - 1 ) {
                    this.triangle( b, d, c );
                }
            }
            return this;
        }

        public NativeArray<float3> nativePositions( ) {
            return new NativeArray<float3>( this.positions.Select( ( position ) => ( float3 )position ).ToArray( ), Allocator.TempJob );
        }

        public NativeArray<BurstMesh.Triangle> nativeTriangles( ) {
            return new NativeArray<BurstMesh.Triangle>( this.triangles.ToArray( ), Allocator.TempJob );
        }

        // the sum of the normals of the triangles of each vertex, made unit length
        public NativeArray<float3> nativeNormals( ) {
            var normals = new Vector3[this.positions.Count];

            foreach( var triangle in this.triangles ) {
                var normal = Vector3.Cross( this.positions[triangle.b] - this.positions[triangle.a], this.positions[triangle.c] - this.positions[triangle.a] );

                normals[triangle.a] += normal;
                normals[triangle.b] += normal;
                normals[triangle.c] += normal;
            }

            return new NativeArray<float3>( normals.Select( ( normal ) => ( float3 )normal.normalized ).ToArray( ), Allocator.TempJob );
        }

        public Mesh mesh( ) {
            using var positions = this.nativePositions( );
            using var normals   = this.nativeNormals( );
            using var triangles = this.nativeTriangles( );

            var mesh = BurstMesh.createMesh( positions, normals, triangles );

            mesh.hideFlags = HideFlags.HideAndDontSave;

            return mesh;
        }

        public float area {
            get {
                return this.triangles.Sum( ( triangle ) => 0.5f * Vector3.Cross( this.positions[triangle.b] - this.positions[triangle.a], this.positions[triangle.c] - this.positions[triangle.a] ).magnitude );
            }
        }

    }

    private readonly List<Object> made = new( );

    [TearDown]
    public void destroyMade( ) {
        foreach( var item in this.made ) {
            Object.DestroyImmediate( item );
        }
        this.made.Clear( );
    }

    private Mesh meshOf( Builder builder ) {
        var mesh = builder.mesh( );
        this.made.Add( mesh );
        return mesh;
    }

    // the stand-in's flags for the triangles, in their own order
    private static byte[] flatPairsOf( Builder builder, out int pairs ) {
        using var positions = builder.nativePositions( );
        using var triangles = builder.nativeTriangles( );
        using var sides     = new NativeArray<BurstRemeshing.Side>( triangles.Length * 3, Allocator.TempJob );
        using var flags     = new NativeArray<byte>( triangles.Length, Allocator.TempJob );
        using var count     = new NativeArray<int>( 1, Allocator.TempJob );

        new BurstRemeshing.SidesJob { triangles = triangles, sides = sides }.Run( );
        new BurstRemeshing.FlatPairsJob { positions = positions, triangles = triangles, sides = sides, flags = flags, pairs = count }.Run( );

        pairs = count[0];

        return flags.ToArray( );
    }

    private static ( BurstRemeshing.Topology topology, BurstRemeshing.FaceCounts faces ) countsOf( Builder builder, byte[] flags = null ) {
        using var triangles = builder.nativeTriangles( );
        using var native    = new NativeArray<byte>( flags ?? Array.Empty<byte>( ), Allocator.TempJob );

        BurstRemeshing.count( triangles, native, out var topology, out var faces );

        return ( topology, faces );
    }

    // the edge of a triangle from corner k to k + 1, as the vertices of its two ends, the lower first
    private static ( int, int ) edgeOf( BurstMesh.Triangle triangle, int k ) {
        var a = BurstRemeshing.vertexOf( triangle, k );
        var b = BurstRemeshing.vertexOf( triangle, ( k + 1 ) % 3 );
        return a < b ? ( a, b ) : ( b, a );
    }

    // the edges each triangle leaves out
    private static List<( int, int )>[] leftOut( Builder builder, byte[] flags ) {
        return builder.triangles.Select(
            ( triangle, index ) => Enumerable.Range( 0, 3 ).Where( ( k ) => ( flags[index] & ( 1 << k ) ) == 0 ).Select( ( k ) => edgeOf( triangle, k ) ).ToList( )
        ).ToArray( );
    }

    // ---- the edge length ----

    [Test]
    public void theLengthKeepsTheVertexCountOfItsFaces( ) {
        // 100 of area and 50 vertices is 2 of area for every vertex: an equilateral triangle mesh has a vertex for
        // every (sqrt 3 / 2) l^2, a square one for every l^2
        Assert.AreEqual( Math.Sqrt( 2.0 / ( Math.Sqrt( 3.0 ) / 2.0 ) ), BurstRemeshing.lengthOf( 100.0, 50, BurstRemeshing.Faces.Triangles ), 1e-12, "triangles" );
        Assert.AreEqual( Math.Sqrt( 2.0 ),                              BurstRemeshing.lengthOf( 100.0, 50, BurstRemeshing.Faces.Quads ),     1e-12, "quads" );
    }

    [Test]
    public void aMeshWithoutAreaOrVerticesHasNoLength( [Values] BurstRemeshing.Faces faces ) {
        Assert.AreEqual( 0.0, BurstRemeshing.lengthOf( 0.0, 10, faces ),        "without area" );
        Assert.AreEqual( 0.0, BurstRemeshing.lengthOf( 10.0, 0, faces ),        "without vertices" );
        Assert.AreEqual( 0.0, BurstRemeshing.lengthOf( double.NaN, 10, faces ), "with an area that is not a number" );
    }

    [Test]
    public void theAreaIsOfTheTrianglesAndTheVerticesTheyUse( ) {
        var patch = new Builder( ).patch( 4, 3, 0.5f );

        // a vertex no triangle uses, which the count leaves out
        patch.vertex( new Vector3( 10, 10, 10 ) );

        using var positions   = patch.nativePositions( );
        using var triangles   = patch.nativeTriangles( );
        using var area        = new NativeArray<double>( 1, Allocator.TempJob );
        using var vertexCount = new NativeArray<int>( 1, Allocator.TempJob );

        new BurstRemeshing.AreaJob { positions = positions, triangles = triangles, area = area, vertexCount = vertexCount }.Run( );

        Assert.AreEqual( 3.0, area[0], 1e-9, "area of 4 by 3 squares half a unit across" );
        Assert.AreEqual( 20,  vertexCount[0], "vertices of the patch, without the one no triangle uses" );
    }

    // through what a voxelizer keeps: the setting times the length that keeps the vertex count, on a flat patch and on a
    // fine sphere, whose areas the test works out by itself
    [Test]
    public void theKeptMeshGivesTheLengthOfItsAreaAndVertices( [Values( "patch", "sphere" )] string model ) {
        var builder = model == "patch" ? new Builder( ).patch( 6, 5, 0.25f ) : new Builder( ).sphere( 24, 48 );
        var kept    = Remeshing.of( this.meshOf( builder ), 0.1f );

        // the stand-in makes quads
        var expected = Math.Sqrt( builder.area / builder.positions.Count );

        Assert.AreEqual( expected,       kept.lengthOf( Remeshing.Technique.FlatPairs, 1.0f ), expected * 1e-5, "at 1" );
        Assert.AreEqual( expected * 0.5, kept.lengthOf( Remeshing.Technique.FlatPairs, 0.5f ), expected * 1e-5, "at 0.5" );
    }

    // ---- edges and faces ----

    [Test]
    public void openAndNonManifoldEdgesAreCounted( ) {
        var triangle = countsOf( new Builder( ).triangle( Vector3.zero, Vector3.right, Vector3.up ) ).topology;

        Assert.AreEqual( 3, triangle.openEdges,        "a triangle: open" );
        Assert.AreEqual( 0, triangle.nonManifoldEdges, "a triangle: of more than two triangles" );

        var patch = countsOf( new Builder( ).patch( 2, 2 ) ).topology;

        Assert.AreEqual( 8, patch.openEdges,        "a patch of 2 by 2: open" );
        Assert.AreEqual( 0, patch.nonManifoldEdges, "a patch of 2 by 2: of more than two triangles" );

        var box = countsOf( new Builder( ).box( Vector3Int.zero, new Vector3Int( 2, 1, 3 ) ) ).topology;

        Assert.AreEqual( 0, box.openEdges,        "a box: open" );
        Assert.AreEqual( 0, box.nonManifoldEdges, "a box: of more than two triangles" );

        // two boxes that share the edge from (1, 1, 0) to (1, 1, 1)
        var two = countsOf( new Builder( ).box( Vector3Int.zero, Vector3Int.one ).box( new Vector3Int( 1, 1, 0 ), Vector3Int.one ) ).topology;

        Assert.AreEqual( 0, two.openEdges,        "two boxes that share an edge: open" );
        Assert.AreEqual( 1, two.nonManifoldEdges, "two boxes that share an edge: of more than two triangles" );
    }

    [Test]
    public void withoutFlagsEveryTriangleIsAFace( ) {
        var faces = countsOf( new Builder( ).patch( 2, 2 ) ).faces;

        Assert.AreEqual( 8, faces.triangles );
        Assert.AreEqual( 0, faces.quads );
        Assert.AreEqual( 0, faces.larger );
        Assert.AreEqual( 0, faces.inconsistent );
    }

    [Test]
    public void facesAreTheTrianglesJoinedAcrossEdgesBothLeaveOut( ) {
        // a b c and a c d leave out a c, which runs from corner 2 to 0 of the first and from corner 0 to 1 of the second
        var square = new Builder( ).quad( Vector3.zero, Vector3.right, new Vector3( 1, 1, 0 ), Vector3.up );

        var quad = countsOf( square, new byte[] { 0b011, 0b110 } ).faces;

        Assert.AreEqual( 0, quad.triangles, "a quad: triangles" );
        Assert.AreEqual( 1, quad.quads,     "a quad: quads" );

        // a fan of three triangles round a: a pentagon when the two edges inside it are left out
        var a   = Vector3.zero;
        var fan = new Builder( ).triangle( a, new Vector3( 2, 0, 0 ), new Vector3( 2, 1, 0 ) ).triangle( a, new Vector3( 2, 1, 0 ), new Vector3( 1, 2, 0 ) ).triangle( a, new Vector3( 1, 2, 0 ), new Vector3( 0, 1, 0 ) );

        var pentagon = countsOf( fan, new byte[] { 0b011, 0b010, 0b110 } ).faces;

        Assert.AreEqual( 0, pentagon.triangles,    "a pentagon: triangles" );
        Assert.AreEqual( 0, pentagon.quads,        "a pentagon: quads" );
        Assert.AreEqual( 1, pentagon.larger,       "a pentagon: larger faces" );
        Assert.AreEqual( 0, pentagon.inconsistent, "a pentagon: flags that make no face" );
    }

    [Test]
    public void flagsThatMakeNoFaceAreCountedApart( ) {
        var square = new Builder( ).quad( Vector3.zero, Vector3.right, new Vector3( 1, 1, 0 ), Vector3.up );

        // the diagonal left out by one of its triangles only
        Assert.Greater( countsOf( square, new byte[] { 0b011, 0b111 } ).faces.inconsistent, 0, "an edge left out on one side" );

        // an open edge left out
        Assert.Greater( countsOf( square, new byte[] { 0b110, 0b111 } ).faces.inconsistent, 0, "an open edge left out" );
    }

    // ---- the stand-in ----

    [Test]
    public void flatPairsMakesTheQuadsOfAPatch( ) {
        var patch = new Builder( ).patch( 3, 3 );
        var flags = flatPairsOf( patch, out var pairs );

        Assert.AreEqual( 9, pairs, "pairs" );

        var edges = leftOut( patch, flags );

        for( var triangle = 0; triangle < edges.Length; ++triangle ) {
            Assert.AreEqual( 1, edges[triangle].Count, $"edges triangle {triangle} leaves out" );

            // the diagonal it was built with, from its first corner to its third
            var built    = patch.triangles[triangle];
            var diagonal = triangle % 2 == 0 ? edgeOf( built, 2 ) : edgeOf( built, 0 );

            Assert.AreEqual( diagonal, edges[triangle][0], $"the edge triangle {triangle} leaves out" );
        }

        var faces = countsOf( patch, flags ).faces;

        Assert.AreEqual( 9, faces.quads,        "quads" );
        Assert.AreEqual( 0, faces.triangles,    "triangles" );
        Assert.AreEqual( 0, faces.inconsistent, "flags that make no face" );
    }

    [Test]
    public void flatPairsMakesTheQuadsOfABox( [Values( 1, 2 )] int length ) {
        var box   = new Builder( ).box( Vector3Int.zero, new Vector3Int( length, 1, 1 ) );
        var flags = flatPairsOf( box, out var pairs );

        Assert.AreEqual( box.triangles.Count / 2, pairs, "pairs" );
        Assert.AreEqual( pairs, countsOf( box, flags ).faces.quads, "quads" );
    }

    // across the longest edge first: of two squares side by side, each triangle pairs with the one across its diagonal,
    // and not with the one across the side the squares share
    [Test]
    public void flatPairsPairsAcrossTheLongestEdgeFirst( ) {
        var two   = new Builder( ).patch( 2, 1 );
        var flags = flatPairsOf( two, out var pairs );

        Assert.AreEqual( 2, pairs, "pairs" );

        var ends   = new[] { two.vertex( Vector3.right ), two.vertex( new Vector3( 1, 1, 0 ) ) }.OrderBy( ( vertex ) => vertex ).ToArray( );
        var middle = ( ends[0], ends[1] );

        Assert.IsFalse( leftOut( two, flags ).Any( ( edges ) => edges.Contains( middle ) ), "the side the squares share is left out" );
    }

    [Test]
    public void flatPairsLeavesWhatDoesNotLieFlatTogether( ) {
        flatPairsOf( new Builder( ).octahedron( ), out var octahedron );

        Assert.AreEqual( 0, octahedron, "an octahedron" );

        // two triangles across the edge from the origin to (1, 0, 0), the second turned about it
        Builder bent( float degrees, int secondSubMesh = 0, bool flipped = false ) {
            var turn = Quaternion.AngleAxis( degrees, Vector3.right );
            var far  = turn * new Vector3( 0.5f, -1, 0 );

            var builder = new Builder( ).triangle( Vector3.zero, Vector3.right, new Vector3( 0.5f, 1, 0 ) );

            return flipped ? builder.triangle( Vector3.zero, Vector3.right, far, secondSubMesh ) : builder.triangle( Vector3.right, Vector3.zero, far, secondSubMesh );
        }

        flatPairsOf( bent( 0.0f ), out var flat );
        flatPairsOf( bent( 0.5f ), out var nearly );
        flatPairsOf( bent( 2.0f ), out var folded );
        flatPairsOf( bent( 0.0f, 1 ), out var materials );
        flatPairsOf( bent( 0.0f, 0, true ), out var wound );

        Assert.AreEqual( 1, flat,      "flat" );
        Assert.AreEqual( 1, nearly,    "half a degree apart" );
        Assert.AreEqual( 0, folded,    "two degrees apart" );
        Assert.AreEqual( 0, materials, "of two materials" );
        Assert.AreEqual( 0, wound,     "wound the other way" );

        // a dart: flat and facing the same way, but the quad it would make bends in at (1, 0, 0)
        var dart = new Builder( ).triangle( Vector3.zero, Vector3.right, new Vector3( 0.5f, 1, 0 ) ).triangle( Vector3.right, Vector3.zero, new Vector3( 2, -0.1f, 0 ) );

        flatPairsOf( dart, out var concave );

        Assert.AreEqual( 0, concave, "a quad that is not convex" );
    }

    [Test]
    public void flatPairsGivesTheSameFlagsEveryTime( ) {
        var box = new Builder( ).box( Vector3Int.zero, new Vector3Int( 3, 2, 2 ) );

        CollectionAssert.AreEqual( flatPairsOf( box, out _ ), flatPairsOf( box, out _ ) );
    }

    // ---- flags and meshes ----

    [Test]
    public void flagsGoInTheOrderOfTheIndexBuffer( ) {
        using var triangles = new NativeArray<BurstMesh.Triangle>(
            new[] { new BurstMesh.Triangle( 0, 1, 2, 1 ), new BurstMesh.Triangle( 0, 2, 3, 0 ), new BurstMesh.Triangle( 1, 2, 3, 1 ), new BurstMesh.Triangle( 0, 1, 3, 0 ) }, Allocator.TempJob
        );
        using var flags = new NativeArray<byte>( new byte[] { 1, 2, 3, 4 }, Allocator.TempJob );

        CollectionAssert.AreEqual( new byte[] { 2, 4, 1, 3 }, Remeshing.inIndexOrder( triangles, flags ) );
    }

    // the flags a remesh gives are those of the mesh it gives, triangle for triangle in its index buffer, of whatever
    // materials: each triangle leaves out the diagonal its partner leaves out
    [Test]
    public void aRemeshsFlagsAreThoseOfItsMesh( ) {
        // squares of three materials, given in an order that the mesh's sub-meshes change
        var patch = new Builder( ).patch( 3, 2, 1.0f, ( i, j ) => ( i + j ) % 3 );
        var kept  = Remeshing.of( this.meshOf( patch ), 1.0f );
        var mesh  = kept.remesh( Remeshing.Technique.FlatPairs, 1.0f, out var flags, out var statistics );

        this.made.Add( mesh );

        Assert.IsNull( statistics.gaveUp, "gave up" );
        Assert.AreEqual( 6, statistics.quads, "quads" );

        var vertices  = mesh.vertices;
        var triangles = Enumerable.Range( 0, mesh.subMeshCount ).SelectMany( ( subMesh ) => mesh.GetTriangles( subMesh ) ).ToArray( );

        Assert.AreEqual( triangles.Length / 3, flags.Length, "a flag for every triangle" );

        var sides = new Dictionary<( Vector3, Vector3 ), int>( );

        for( var triangle = 0; triangle < flags.Length; ++triangle ) {
            for( var k = 0; k < 3; ++k ) {
                if( ( flags[triangle] & ( 1 << k ) ) != 0 ) {
                    continue;
                }

                var a = vertices[triangles[( 3 * triangle ) + k]];
                var b = vertices[triangles[( 3 * triangle ) + ( ( k + 1 ) % 3 )]];

                // the longest side of a half square
                Assert.AreEqual( Mathf.Sqrt( 2.0f ), ( b - a ).magnitude, 1e-5f, $"the edge triangle {triangle} leaves out" );

                var key = a.x < b.x || ( a.x == b.x && a.y < b.y ) ? ( a, b ) : ( b, a );

                sides[key] = sides.TryGetValue( key, out var count ) ? count + 1 : 1;
            }
        }

        Assert.AreEqual( 6, sides.Count, "diagonals left out" );
        Assert.IsTrue( sides.Values.All( ( count ) => count == 2 ), "every diagonal left out by both its triangles" );
    }

    // a technique that cannot do its work gives the mesh back as it came, without flags, and says why
    [Test]
    public void aRemeshThatGivesUpGivesTheMeshAsItCame( ) {
        var octahedron = new Builder( ).octahedron( );
        var given      = this.meshOf( octahedron );
        var kept       = Remeshing.of( given, 0.5f );
        var mesh       = kept.remesh( Remeshing.Technique.FlatPairs, 1.0f, out var flags, out var statistics );

        this.made.Add( mesh );

        Assert.IsNull( flags, "flags" );
        Assert.IsNotNull( statistics.gaveUp, "why it gave up" );

        CollectionAssert.AreEqual( MeshComparison.trianglesOf( given ), MeshComparison.trianglesOf( mesh ), "triangles" );

        Assert.AreEqual( 8, statistics.triangles, "faces: triangles" );
        Assert.AreEqual( 0, statistics.quads,     "faces: quads" );
    }

    [Test]
    public void aRemeshSaysWhatItDid( ) {
        var box   = new Builder( ).box( Vector3Int.zero, new Vector3Int( 2, 1, 1 ) );
        var given = this.meshOf( box );
        var kept  = Remeshing.of( given, 0.5f );
        var mesh  = kept.remesh( Remeshing.Technique.FlatPairs, 2.0f, out _, out var statistics );

        this.made.Add( mesh );

        Assert.AreEqual( Remeshing.Technique.FlatPairs, statistics.technique );
        Assert.AreEqual( given.vertexCount,             statistics.inputVertexCount,   "vertices given" );
        Assert.AreEqual( box.triangles.Count,           statistics.inputTriangleCount, "triangles given" );
        Assert.AreEqual( 0,                             statistics.triangles,          "faces: triangles" );
        Assert.AreEqual( 10,                            statistics.quads,              "faces: quads" );
        Assert.AreEqual( 0,                             statistics.largerFaces,        "faces: larger" );
        Assert.AreEqual( 0,                             statistics.newOpenEdges,       "new open edges" );
        Assert.AreEqual( 0,                             statistics.newNonManifoldEdges, "new edges of more than two triangles" );
        Assert.GreaterOrEqual( statistics.milliseconds, 0.0,                           "time" );

        // twice the length that keeps the vertex count, in cells of half a unit
        Assert.AreEqual( 2.0 * Math.Sqrt( box.area / box.positions.Count ) / 0.5, statistics.edgeLength, 1e-5, "edge length in cells" );
    }

    // edges that are open or have more than two triangles count only where the mesh it was given did not have them: a
    // patch's rim and the edge two boxes share are the mesh's own
    [Test]
    public void aRemeshCountsOnlyTheEdgesItAdds( [Values( "patch", "two boxes" )] string model ) {
        var builder = model == "patch" ? new Builder( ).patch( 3, 2 ) : new Builder( ).box( Vector3Int.zero, Vector3Int.one ).box( new Vector3Int( 1, 1, 0 ), Vector3Int.one );
        var kept    = Remeshing.of( this.meshOf( builder ), 1.0f );
        var mesh    = kept.remesh( Remeshing.Technique.FlatPairs, 1.0f, out _, out var statistics );

        this.made.Add( mesh );

        var given = countsOf( builder ).topology;

        Assert.Greater( given.openEdges + given.nonManifoldEdges, 0, "open and non-manifold edges of the mesh given" );

        Assert.AreEqual( 0, statistics.newOpenEdges,        "new open edges" );
        Assert.AreEqual( 0, statistics.newNonManifoldEdges, "new edges of more than two triangles" );
    }

    [Test]
    public void theKeptMeshIsTheMeshAsItCame( ) {
        var given = this.meshOf( new Builder( ).patch( 3, 2, 1.0f, ( i, j ) => i % 2 ) );
        var mesh  = Remeshing.of( given, 1.0f ).mesh( );

        this.made.Add( mesh );

        CollectionAssert.AreEqual( MeshComparison.trianglesOf( given ), MeshComparison.trianglesOf( mesh ) );
        Assert.AreEqual( given.subMeshCount, mesh.subMeshCount, "sub-meshes" );
    }

}
