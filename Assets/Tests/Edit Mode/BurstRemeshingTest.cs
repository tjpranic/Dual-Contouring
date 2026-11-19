using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

using Object = UnityEngine.Object;

// what every remeshing technique shares (BurstRemeshing, Remeshing), on meshes built by hand whose answers are known:
// the edge length that keeps a mesh's vertex count, its area, its open and non-manifold edges, its faces by their
// sides, and the order of the flags
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

    private static ( BurstRemeshing.Topology topology, BurstRemeshing.FaceCounts faces ) countsOf( Builder builder, byte[] flags = null ) {
        using var triangles = builder.nativeTriangles( );
        using var native    = new NativeArray<byte>( flags ?? Array.Empty<byte>( ), Allocator.TempJob );

        BurstRemeshing.count( triangles, native, out var topology, out var faces );

        return ( topology, faces );
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

        // Instant Meshes' quads, with or without triangles and pentagons among them, and triangles, its or Isotropic's
        var quads     = Math.Sqrt( builder.area / builder.positions.Count );
        var triangles = Math.Sqrt( builder.area / builder.positions.Count / ( Math.Sqrt( 3.0 ) / 2.0 ) );

        Assert.AreEqual( quads,       kept.lengthOf( Remeshing.Technique.InstantMeshes, 1.0f ), quads * 1e-5, "quad dominant at 1" );
        Assert.AreEqual( quads * 0.5, kept.lengthOf( Remeshing.Technique.InstantMeshes, 0.5f ), quads * 1e-5, "quad dominant at 0.5" );
        Assert.AreEqual( quads,       kept.lengthOf( Remeshing.Technique.InstantMeshes, 1.0f, BurstInstantMeshes.Faces.Quads ), quads * 1e-5, "quads at 1" );
        Assert.AreEqual( triangles,   kept.lengthOf( Remeshing.Technique.InstantMeshes, 1.0f, BurstInstantMeshes.Faces.Triangles ), triangles * 1e-5, "Instant Meshes' triangles at 1" );
        Assert.AreEqual( triangles,   kept.lengthOf( Remeshing.Technique.Isotropic, 1.0f ), triangles * 1e-5, "Isotropic's triangles at 1" );
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

        // the lines the edges make
        Assert.AreEqual( ( 1, 0 ), ( triangle.openLines, triangle.nonManifoldLines ), "a triangle: lines" );
        Assert.AreEqual( ( 1, 0 ), ( patch.openLines, patch.nonManifoldLines ),       "a patch of 2 by 2: lines" );
        Assert.AreEqual( ( 0, 0 ), ( box.openLines, box.nonManifoldLines ),           "a box: lines" );
        Assert.AreEqual( ( 0, 1 ), ( two.openLines, two.nonManifoldLines ),           "two boxes that share an edge: lines" );

        // two triangles apart have two rims, and boxes that share an edge of two segments one line of two edges
        var apart = countsOf( new Builder( ).triangle( Vector3.zero, Vector3.right, Vector3.up ).triangle( Vector3.forward * 2, Vector3.forward * 2 + Vector3.right, Vector3.forward * 2 + Vector3.up ) ).topology;

        Assert.AreEqual( ( 6, 2 ), ( apart.openEdges, apart.openLines ), "two triangles apart: open edges and lines" );

        var longer = countsOf( new Builder( ).box( Vector3Int.zero, new Vector3Int( 1, 1, 2 ) ).box( new Vector3Int( 1, 1, 0 ), new Vector3Int( 1, 1, 2 ) ) ).topology;

        Assert.AreEqual( ( 2, 1 ), ( longer.nonManifoldEdges, longer.nonManifoldLines ), "two boxes that share an edge of two segments: edges and lines" );
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
    // materials: every edge a triangle leaves out is left out by the one other triangle on it, of the same sub-mesh
    [Test]
    public void aRemeshsFlagsAreThoseOfItsMesh( ) {
        // bands of three materials, given in an order that the mesh's sub-meshes change
        var patch = new Builder( ).patch( 12, 12, 1.0f, ( i, j ) => ( i / 4 + j / 6 ) % 3 );
        var kept  = Remeshing.of( this.meshOf( patch ), 1.0f );
        var mesh  = kept.remesh( new Remeshing.Settings( Remeshing.Technique.InstantMeshes, 1.0f ), out var flags, out var statistics );

        this.made.Add( mesh );

        Assert.IsNull( statistics.gaveUp, "gave up" );
        Assert.Greater( statistics.quads, 50, "quads" );
        Assert.AreEqual( 3, mesh.subMeshCount, "sub-meshes" );

        var vertices  = mesh.vertices;
        var triangles = Enumerable.Range( 0, mesh.subMeshCount ).SelectMany( ( subMesh ) => mesh.GetTriangles( subMesh ).Select( ( index ) => ( index, subMesh ) ) ).ToArray( );

        Assert.AreEqual( triangles.Length / 3, flags.Length, "a flag for every triangle" );

        var sides = new Dictionary<( int, int ), List<int>>( );

        for( var triangle = 0; triangle < flags.Length; ++triangle ) {
            for( var k = 0; k < 3; ++k ) {
                if( ( flags[triangle] & ( 1 << k ) ) != 0 ) {
                    continue;
                }

                var a   = triangles[( 3 * triangle ) + k].index;
                var b   = triangles[( 3 * triangle ) + ( ( k + 1 ) % 3 )].index;
                var key = a < b ? ( a, b ) : ( b, a );

                if( !sides.TryGetValue( key, out var list ) ) {
                    sides.Add( key, list = new List<int>( ) );
                }
                list.Add( triangle );
            }
        }

        Assert.Greater( sides.Count, 50, "edges left out" );

        foreach( var side in sides ) {
            Assert.AreEqual( 2, side.Value.Count, $"triangles that leave out the edge {side.Key} at {vertices[side.Key.Item1]}" );
            Assert.AreEqual( triangles[3 * side.Value[0]].subMesh, triangles[3 * side.Value[1]].subMesh, $"sub-meshes of the triangles across {side.Key}" );
        }
    }

    // a technique that cannot do its work gives the mesh back as it came, without flags, and says why
    [Test]
    public void aRemeshThatGivesUpGivesTheMeshAsItCame( ) {
        var octahedron = new Builder( ).octahedron( );
        var given      = this.meshOf( octahedron );
        var kept       = Remeshing.of( given, 0.5f );

        // edges of a thousandth of the octahedron's would make millions of vertices
        var mesh = kept.remesh( new Remeshing.Settings( Remeshing.Technique.InstantMeshes, 0.001f ), out var flags, out var statistics );

        this.made.Add( mesh );

        Assert.IsNull( flags, "flags" );
        Assert.IsNotNull( statistics.gaveUp, "why it gave up" );

        CollectionAssert.AreEqual( MeshComparison.trianglesOf( given ), MeshComparison.trianglesOf( mesh ), "triangles" );

        Assert.AreEqual( 8, statistics.triangles, "faces: triangles" );
        Assert.AreEqual( 0, statistics.quads,     "faces: quads" );
    }

    // Instant Meshes gives up where no faces come out of its fields: an octahedron at four times its edge length has one
    // lattice point for all of it
    [Test]
    public void anInstantMeshesRemeshOfNoFacesGivesTheMeshAsItCame( ) {
        var octahedron = new Builder( ).octahedron( );
        var given      = this.meshOf( octahedron );
        var kept       = Remeshing.of( given, 0.5f );
        var mesh       = kept.remesh( new Remeshing.Settings( Remeshing.Technique.InstantMeshes, 4.0f ), out var flags, out var statistics );

        this.made.Add( mesh );

        Assert.AreEqual( "no faces came out of the fields", statistics.gaveUp, "why it gave up" );
        Assert.IsNull( flags, "flags" );
        Assert.IsTrue( statistics.instantMeshes.HasValue, "what Instant Meshes did" );

        CollectionAssert.AreEqual( MeshComparison.trianglesOf( given ), MeshComparison.trianglesOf( mesh ), "triangles" );
    }

    [Test]
    public void aRemeshSaysWhatItDid( ) {
        var box   = new Builder( ).box( Vector3Int.zero, new Vector3Int( 8, 4, 4 ) );
        var given = this.meshOf( box );
        var kept  = Remeshing.of( given, 0.5f );
        var mesh  = kept.remesh( new Remeshing.Settings( Remeshing.Technique.InstantMeshes, 2.0f ), out _, out var statistics );

        this.made.Add( mesh );

        Assert.AreEqual( Remeshing.Technique.InstantMeshes, statistics.technique );
        Assert.AreEqual( given.vertexCount,                 statistics.inputVertexCount,    "vertices given" );
        Assert.AreEqual( box.triangles.Count,               statistics.inputTriangleCount,  "triangles given" );
        Assert.AreEqual( 0,                                 statistics.newOpenLines,        "new open lines" );
        Assert.AreEqual( 0,                                 statistics.newNonManifoldLines, "new lines of edges of more than two triangles" );
        Assert.GreaterOrEqual( statistics.milliseconds, 0.0,                                "time" );
        Assert.IsTrue( statistics.instantMeshes.HasValue,                                   "what Instant Meshes did" );
        Assert.IsNull( statistics.isotropic,                                                "what Isotropic did" );

        // the faces the mesh's flags make
        Assert.Greater( statistics.quads, 20, "faces: quads" );
        Assert.AreEqual( mesh.triangles.Length / 3, ( 2 * statistics.quads ) + statistics.triangles + ( 3 * statistics.largerFaces ), "triangles: two to a quad, three to a pentagon" );

        // twice the length that keeps the vertex count, in cells of half a unit
        Assert.AreEqual( 2.0 * Math.Sqrt( box.area / box.positions.Count ) / 0.5, statistics.edgeLength, 1e-5, "edge length in cells" );
    }

    // edges that are open or have more than two triangles count only where the mesh it was given did not have them: a
    // patch's rim and the edge two boxes share are the mesh's own
    [Test]
    public void aRemeshCountsOnlyTheEdgesItAdds( [Values( "patch", "two boxes" )] string model ) {
        var builder = model == "patch" ? new Builder( ).patch( 3, 2 ) : new Builder( ).box( Vector3Int.zero, Vector3Int.one ).box( new Vector3Int( 1, 1, 0 ), Vector3Int.one );
        var kept    = Remeshing.of( this.meshOf( builder ), 1.0f );
        var mesh    = kept.remesh( new Remeshing.Settings( Remeshing.Technique.InstantMeshes, 1.0f ), out _, out var statistics );

        this.made.Add( mesh );

        var given = countsOf( builder ).topology;

        Assert.Greater( given.openEdges + given.nonManifoldEdges, 0, "open and non-manifold edges of the mesh given" );

        Assert.AreEqual( 0, statistics.newOpenLines,        "new open lines" );
        Assert.AreEqual( 0, statistics.newNonManifoldLines, "new lines of edges of more than two triangles" );
    }

    // the normals at a mesh's corners are those it would show creased: on a box whose vertices lean between its faces,
    // creased at 40 degrees, every corner has its own face's normal; at 180, nothing is creased and every corner has its
    // vertex's normal (finding 21)
    [Test]
    public void theNormalsAtTheCornersAreThoseTheCreasedMeshShows( ) {
        var built = new HandBuiltMesh( ).box( Vector3Int.zero, Vector3Int.one * 2 );

        using var positions = built.nativePositions( );
        using var normals   = built.nativeNormals( );
        using var triangles = built.nativeTriangles( );

        using var creased = BurstRemeshing.cornerNormalsOf( positions, normals, triangles, 40.0, Allocator.TempJob );
        using var asGiven = BurstRemeshing.cornerNormalsOf( positions, normals, triangles, 180.0, Allocator.TempJob );

        Assert.AreEqual( triangles.Length * 3, creased.Length, "three to a triangle" );

        for( var index = 0; index < triangles.Length; ++index ) {
            var triangle = triangles[index];
            var face     = math.normalize( math.cross( positions[triangle.b] - positions[triangle.a], positions[triangle.c] - positions[triangle.a] ) );
            var vertices = new[] { triangle.a, triangle.b, triangle.c };

            for( var corner = 0; corner < 3; ++corner ) {
                Assert.LessOrEqual( math.distance( creased[( index * 3 ) + corner], face ), 1e-6f, $"creased: corner {corner} of triangle {index}" );
                Assert.AreEqual( normals[vertices[corner]], asGiven[( index * 3 ) + corner], $"at 180: corner {corner} of triangle {index}" );
            }
        }
    }

    // a remesh that samples a rim again has more or fewer open edges than the mesh it was given, and as many rims: it
    // adds no line of open edges
    [Test]
    public void aRemeshThatSamplesARimAgainAddsNoLine( ) {
        var builder = new Builder( ).patch( 6, 6 );
        var kept    = Remeshing.of( this.meshOf( builder ), 1.0f );
        var mesh    = kept.remesh( new Remeshing.Settings( Remeshing.Technique.Isotropic, 0.5f ), out _, out var statistics );

        this.made.Add( mesh );

        var given = countsOf( builder ).topology;

        using var triangles = new NativeArray<BurstMesh.Triangle>( Enumerable.Range( 0, mesh.subMeshCount ).SelectMany(
            ( subMesh ) => {
                var indices = mesh.GetTriangles( subMesh );

                return Enumerable.Range( 0, indices.Length / 3 ).Select( ( triangle ) => new BurstMesh.Triangle( indices[triangle * 3], indices[( triangle * 3 ) + 1], indices[( triangle * 3 ) + 2], subMesh ) );
            }
        ).ToArray( ), Allocator.TempJob );
        using var every = new NativeArray<byte>( 0, Allocator.TempJob );

        BurstRemeshing.count( triangles, every, out var made, out _ );

        Assert.Greater( made.openEdges, given.openEdges, "more open edges than the patch had" );
        Assert.AreEqual( ( 1, 1 ), ( given.openLines, made.openLines ), "one rim before and after" );
        Assert.AreEqual( 0, statistics.newOpenLines, "new lines of open edges" );
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
