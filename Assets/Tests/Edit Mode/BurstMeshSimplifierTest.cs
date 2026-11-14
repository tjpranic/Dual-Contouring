using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

using Object             = UnityEngine.Object;
using Combination        = DensityFunction.Combination;
using MaterialIndex      = SurfaceExtractor.MaterialIndex;
using ImplementationType = SurfaceExtractor.Implementation.Type;
using VertexNormals      = SurfaceExtractor.VertexNormals;
using QEFSolverType      = QEFSolver.Type;
using Connectivity       = BurstMeshSimplifier.Connectivity;
using Refusal            = BurstMeshSimplifier.Refusal;
using Sharp              = BurstMeshSimplifier.Sharp;

// the connectivity of BurstMeshSimplifier, which collapses are allowed, and what a collapse does, on meshes built by
// hand and on contours. The meshes built by hand are made of boxes on a grid of whole numbers, of flat patches, and of
// the few solids that the papers' own examples are
public class BurstMeshSimplifierTest : TopologyTest {

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

        public int at( float x, float y, float z ) {
            return this.indices[new Vector3( x, y, z )];
        }

        public void triangle( Vector3 a, Vector3 b, Vector3 c, int subMesh = 0 ) {
            this.triangles.Add( new BurstMesh.Triangle( this.vertex( a ), this.vertex( b ), this.vertex( c ), subMesh ) );
        }

        // two triangles, a b c and a c d
        public void quad( Vector3 a, Vector3 b, Vector3 c, Vector3 d, int subMesh = 0 ) {
            this.triangle( a, b, c, subMesh );
            this.triangle( a, c, d, subMesh );
        }

        // the surface of a box of unit cells, facing out. The sub-mesh of each quad is given its center and the axis
        // and direction it faces
        public Builder box( Vector3Int minimum, Vector3Int size, Func<Vector3, int, int, int> subMeshOf = null ) {
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

                    var subMesh = subMeshOf == null ? 0 : subMeshOf( ( corner( 0, 0 ) + corner( 1, 1 ) ) / 2, axis, ( side * 2 ) - 1 );

                    if( side == 1 ) {
                        this.quad( corner( 0, 0 ), corner( 1, 0 ), corner( 1, 1 ), corner( 0, 1 ), subMesh );
                    }
                    else {
                        this.quad( corner( 0, 0 ), corner( 0, 1 ), corner( 1, 1 ), corner( 1, 0 ), subMesh );
                    }
                }
            }
            return this;
        }

        // a flat patch of unit quads in the plane z = 0, facing up, with open edges all around
        public Builder patch( int width, int height, Func<int, int, int> subMeshOf = null ) {
            for( var i = 0; i < width; ++i )
            for( var j = 0; j < height; ++j ) {
                this.quad( new Vector3( i, j, 0 ), new Vector3( i + 1, j, 0 ), new Vector3( i + 1, j + 1, 0 ), new Vector3( i, j + 1, 0 ), subMeshOf == null ? 0 : subMeshOf( i, j ) );
            }
            return this;
        }

        // the sub-mesh of the triangle on three vertices
        public void setSubMesh( int a, int b, int c, int subMesh ) {
            var index = this.triangles.FindIndex( ( triangle ) => new[] { triangle.a, triangle.b, triangle.c }.OrderBy( ( vertex ) => vertex ).SequenceEqual( new[] { a, b, c }.OrderBy( ( vertex ) => vertex ) ) );

            Assert.GreaterOrEqual( index, 0, "the triangle exists" );

            var triangle = this.triangles[index];

            this.triangles[index] = new BurstMesh.Triangle( triangle.a, triangle.b, triangle.c, subMesh );
        }

        // the sum of the normals of the triangles of each vertex, weighted by their areas, made unit length
        public Vector3[] normals( ) {
            var normals = new Vector3[this.positions.Count];

            foreach( var triangle in this.triangles ) {
                var normal = Vector3.Cross( this.positions[triangle.b] - this.positions[triangle.a], this.positions[triangle.c] - this.positions[triangle.a] );

                normals[triangle.a] += normal;
                normals[triangle.b] += normal;
                normals[triangle.c] += normal;
            }

            return normals.Select( ( normal ) => normal.normalized ).ToArray( );
        }

    }

    // something done to an edge, given by its two vertices: asking whether it may be collapsed with the new vertex at
    // the position, or collapsing it there whatever the answer
    private struct Operation {

        public bool    collapse;
        public int     a;
        public int     b;
        public double3 position;

    }

    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    private struct ConnectivityJob : IJob {

        [ReadOnly] public NativeArray<float3>             positions;
        [ReadOnly] public NativeArray<float3>             normals;
        [ReadOnly] public NativeArray<BurstMesh.Triangle> triangles;
        [ReadOnly] public NativeArray<Operation>          operations;

        // after the operations: goes through the edges this often, in their order, and collapses each that may be
        // collapsed into its midpoint
        public int passes;

        public double lineWeight;

        public Connectivity mesh;

        // for every operation: what check said, and the vertex a collapse kept
        public NativeArray<Refusal> refusals;
        public NativeArray<int>     kept;

        // how many edges the passes collapsed, and how often each refusal kept one from collapsing
        public NativeArray<int> collapsed;
        public NativeArray<int> refused;

        public NativeList<float3>             writtenPositions;
        public NativeList<float3>             writtenNormals;
        public NativeList<BurstMesh.Triangle> writtenTriangles;

        public NativeArray<bool> managed;

        public void Execute( ) {
            this.managed[0] = BurstTestSupport.runningManaged( );

            this.mesh.build( this.positions, this.normals, this.triangles );
            this.mesh.buildQuadrics( this.lineWeight );

            for( var index = 0; index < this.operations.Length; ++index ) {
                var operation = this.operations[index];
                var edge      = this.mesh.edgeBetween( operation.a, operation.b );

                this.refusals[index] = this.mesh.check( edge, operation.position );
                this.kept[index]     = operation.collapse ? this.mesh.collapse( edge, operation.position ) : -1;
            }

            for( var pass = 0; pass < this.passes; ++pass ) {
                for( var edge = 0; edge < this.mesh.edgeCount; ++edge ) {
                    var ends = this.mesh.edges[edge];

                    if( ends.x == -1 ) {
                        continue;
                    }

                    var midpoint = ( this.mesh.positions[ends.x] + this.mesh.positions[ends.y] ) * 0.5;
                    var refusal  = this.mesh.check( edge, midpoint );

                    if( refusal == Refusal.None ) {
                        this.mesh.collapse( edge, midpoint );

                        ++this.collapsed[0];
                    }
                    else {
                        ++this.refused[( int )refusal];
                    }
                }
            }

            this.mesh.write( this.writtenPositions, this.writtenNormals, this.writtenTriangles );
        }

    }

    // a mesh, what was done to it, and the connectivity afterwards
    private class Run : IDisposable {

        public Connectivity mesh;

        public Refusal[] refusals;
        public int[]     kept;
        public int       collapsed;
        public int[]     refused;

        public Vector3[]            writtenPositions;
        public Vector3[]            writtenNormals;
        public BurstMesh.Triangle[] writtenTriangles;

        public Run( Vector3[] positions, Vector3[] normals, BurstMesh.Triangle[] triangles, Operation[] operations = null, int passes = 0, double lineWeight = 0.0 ) {
            operations ??= new Operation[0];

            this.mesh = new Connectivity( positions.Length, triangles.Length, Allocator.TempJob );

            using var nativePositions  = new NativeArray<float3>( positions.Select( ( position ) => ( float3 )position ).ToArray( ), Allocator.TempJob );
            using var nativeNormals    = new NativeArray<float3>( normals.Select( ( normal ) => ( float3 )normal ).ToArray( ), Allocator.TempJob );
            using var nativeTriangles  = new NativeArray<BurstMesh.Triangle>( triangles, Allocator.TempJob );
            using var nativeOperations = new NativeArray<Operation>( operations, Allocator.TempJob );
            using var refusals         = new NativeArray<Refusal>( operations.Length, Allocator.TempJob );
            using var kept             = new NativeArray<int>( operations.Length, Allocator.TempJob );
            using var collapsed        = new NativeArray<int>( 1, Allocator.TempJob );
            using var refused          = new NativeArray<int>( Enum.GetValues( typeof( Refusal ) ).Length, Allocator.TempJob );
            using var writtenPositions = new NativeList<float3>( Allocator.TempJob );
            using var writtenNormals   = new NativeList<float3>( Allocator.TempJob );
            using var writtenTriangles = new NativeList<BurstMesh.Triangle>( Allocator.TempJob );
            using var managed          = new NativeArray<bool>( 1, Allocator.TempJob );

            new ConnectivityJob {
                positions        = nativePositions,
                normals          = nativeNormals,
                triangles        = nativeTriangles,
                operations       = nativeOperations,
                passes           = passes,
                lineWeight       = lineWeight,
                mesh             = this.mesh,
                refusals         = refusals,
                kept             = kept,
                collapsed        = collapsed,
                refused          = refused,
                writtenPositions = writtenPositions,
                writtenNormals   = writtenNormals,
                writtenTriangles = writtenTriangles,
                managed          = managed
            }.Run( );

            BurstTestSupport.assertRanUnderBurst( managed );

            this.refusals         = refusals.ToArray( );
            this.kept             = kept.ToArray( );
            this.collapsed        = collapsed[0];
            this.refused          = refused.ToArray( );
            this.writtenPositions = writtenPositions.AsArray( ).ToArray( ).Select( ( position ) => ( Vector3 )position ).ToArray( );
            this.writtenNormals   = writtenNormals.AsArray( ).ToArray( ).Select( ( normal ) => ( Vector3 )normal ).ToArray( );
            this.writtenTriangles = writtenTriangles.AsArray( ).ToArray( );
        }

        public Run( Builder builder, Operation[] operations = null, int passes = 0, double lineWeight = 0.0 )
            : this( builder.positions.ToArray( ), builder.normals( ), builder.triangles.ToArray( ), operations, passes, lineWeight ) {
        }

        public void Dispose( ) {
            this.mesh.Dispose( );
        }

        // the edges that are left, by their two vertices, the lower first
        public Dictionary<( int, int ), ( int triangles, Sharp sharp )> edges( ) {
            var edges = new Dictionary<( int, int ), ( int, Sharp )>( );

            for( var edge = 0; edge < this.mesh.edgeCount; ++edge ) {
                var ends = this.mesh.edges[edge];

                if( ends.x != -1 ) {
                    edges.Add( ( math.min( ends.x, ends.y ), math.max( ends.x, ends.y ) ), ( this.mesh.edgeTriangles[edge], this.mesh.edgeSharp[edge] ) );
                }
            }
            return edges;
        }

        public int[] verticesWithTriangles( ) {
            return Enumerable.Range( 0, this.mesh.vertexCount ).Where( ( vertex ) => this.mesh.firstCorners[vertex] != -1 ).ToArray( );
        }

        public int triangleCount {
            get { return Enumerable.Range( 0, this.mesh.triangleCount ).Count( ( triangle ) => this.mesh.subMeshes[triangle] != -1 ); }
        }

    }

    private static Operation check( int a, int b, Vector3 position ) {
        return new Operation { collapse = false, a = a, b = b, position = new double3( position.x, position.y, position.z ) };
    }

    private static Operation collapse( int a, int b, Vector3 position ) {
        return new Operation { collapse = true, a = a, b = b, position = new double3( position.x, position.y, position.z ) };
    }

    // what check says about the edge between two vertices, with the new vertex halfway between them
    private static Refusal checkMidpoint( Builder builder, int a, int b ) {
        using var run = new Run( builder, new[] { check( a, b, ( builder.positions[a] + builder.positions[b] ) / 2 ) } );

        return run.refusals[0];
    }

    private static Vector3Int cells( int x, int y, int z ) {
        return new Vector3Int( x, y, z );
    }

    // the six corners of an octahedron and its eight faces, facing out
    private static readonly Vector3 px = Vector3.right, nx = Vector3.left, py = Vector3.up, ny = Vector3.down, pz = Vector3.forward, nz = Vector3.back;

    private static Builder octahedron( bool withoutFirstFace = false ) {
        var builder = new Builder( );

        if( !withoutFirstFace ) {
            builder.triangle( px, py, pz );
        }
        builder.triangle( py, nx, pz );
        builder.triangle( nx, ny, pz );
        builder.triangle( ny, px, pz );
        builder.triangle( py, px, nz );
        builder.triangle( nx, py, nz );
        builder.triangle( ny, nx, nz );
        builder.triangle( px, ny, nz );

        return builder;
    }

    // two boxes that share an edge of the given number of segments, along z at x = y = 1. Every segment has four
    // triangles, two of each box, as where plain dual contouring puts two sheets of surface through a cell
    private static Builder boxesSharingAnEdge( int segments, Func<Vector3, int, int, int> subMeshOf = null ) {
        return new Builder( ).box( cells( 0, 0, 0 ), cells( 1, 1, segments ), subMeshOf ).box( cells( 1, 1, 0 ), cells( 1, 1, segments ) );
    }

    // ---- the lists ----

    [Test]
    public void aBoxIsAClosedSurfaceOfEdgesWithTwoTriangles( ) {
        using var run = new Run( new Builder( ).box( cells( 0, 0, 0 ), cells( 2, 3, 4 ) ) );

        // 2 ( 2 x 3 + 3 x 4 + 2 x 4 ) = 52 quads
        Assert.AreEqual( 104, run.triangleCount, "triangles" );
        Assert.AreEqual( 54,  run.verticesWithTriangles( ).Length, "vertices" );
        Assert.AreEqual( 156, run.edges( ).Count, "edges" );

        Assert.IsTrue( run.edges( ).Values.All( ( edge ) => edge.triangles == 2 && edge.sharp == Sharp.None ), "every edge has two triangles and is not sharp" );

        for( var vertex = 0; vertex < run.mesh.vertexCount; ++vertex ) {
            Assert.AreEqual( 0, run.mesh.orders[vertex],      $"order of vertex {vertex}" );
            Assert.AreEqual( 0, run.mesh.sharpCounts[vertex], $"sharp edges of vertex {vertex}" );
            Assert.IsFalse( run.mesh.locked[vertex],          $"vertex {vertex} is locked" );
        }
    }

    [Test]
    public void edgesBetweenSubMeshesAreSharp( ) {
        // the half of the box below z = 2 is of a second material: the line between the two runs around the box
        var builder = new Builder( ).box( cells( 0, 0, 0 ), cells( 2, 2, 4 ), ( center, axis, side ) => center.z < 2 ? 1 : 0 );

        using var run = new Run( builder );

        var sharp = run.edges( ).Where( ( edge ) => edge.Value.sharp != Sharp.None ).ToArray( );

        Assert.AreEqual( 8, sharp.Length, "sharp edges" );

        foreach( var edge in sharp ) {
            Assert.AreEqual( Sharp.Materials, edge.Value.sharp, "why the edge is sharp" );
            Assert.AreEqual( 2.0f, builder.positions[edge.Key.Item1].z, "where the edge starts" );
            Assert.AreEqual( 2.0f, builder.positions[edge.Key.Item2].z, "where the edge ends" );
        }

        for( var vertex = 0; vertex < run.mesh.vertexCount; ++vertex ) {
            Assert.AreEqual( builder.positions[vertex].z == 2 ? 2 : 0, run.mesh.sharpCounts[vertex], $"sharp edges of vertex {vertex}" );
            Assert.AreEqual( 0, run.mesh.orders[vertex], $"order of vertex {vertex}" );
        }
    }

    [Test]
    public void openEdgesAreTheFirstBoundaryOfAPatch( ) {
        var builder = new Builder( ).patch( 4, 3 );

        using var run = new Run( builder );

        bool onRim( Vector3 position ) {
            return position.x == 0 || position.x == 4 || position.y == 0 || position.y == 3;
        }

        foreach( var edge in run.edges( ) ) {
            var a = builder.positions[edge.Key.Item1];
            var b = builder.positions[edge.Key.Item2];

            // an edge along the rim has one triangle
            var along = ( a.x == b.x && ( a.x == 0 || a.x == 4 ) ) || ( a.y == b.y && ( a.y == 0 || a.y == 3 ) );

            Assert.AreEqual( along ? 1 : 2,                   edge.Value.triangles, $"triangles of the edge from {a} to {b}" );
            Assert.AreEqual( along ? Sharp.Open : Sharp.None, edge.Value.sharp,     $"why the edge from {a} to {b} is sharp" );
        }

        for( var vertex = 0; vertex < run.mesh.vertexCount; ++vertex ) {
            var rim = onRim( builder.positions[vertex] );

            // the rim passes straight through each of its vertices, the corners of the patch among them
            Assert.AreEqual( rim ? 1 : 0, run.mesh.orders[vertex],      $"order of the vertex at {builder.positions[vertex]}" );
            Assert.AreEqual( rim ? 2 : 0, run.mesh.sharpCounts[vertex], $"sharp edges of the vertex at {builder.positions[vertex]}" );
        }
    }

    [Test]
    public void edgeOfFourTrianglesIsTheFirstBoundaryAndItsEndsAreOfOrderTwo( ) {
        var builder = boxesSharingAnEdge( 3 );

        using var run = new Run( builder );

        foreach( var edge in run.edges( ) ) {
            var a = builder.positions[edge.Key.Item1];
            var b = builder.positions[edge.Key.Item2];

            var shared = a.x == 1 && a.y == 1 && b.x == 1 && b.y == 1;

            Assert.AreEqual( shared ? 4 : 2,                          edge.Value.triangles, $"triangles of the edge from {a} to {b}" );
            Assert.AreEqual( shared ? Sharp.NonManifold : Sharp.None, edge.Value.sharp,     $"why the edge from {a} to {b} is sharp" );
        }

        for( var vertex = 0; vertex < run.mesh.vertexCount; ++vertex ) {
            var position = builder.positions[vertex];
            var shared   = position.x == 1 && position.y == 1;
            var end      = shared && ( position.z == 0 || position.z == 3 );

            // four half disks meet along the shared edge, which is the surface around a point of an edge of four
            // triangles. At its two ends the edge stops
            Assert.AreEqual( end ? 2 : shared ? 1 : 0, run.mesh.orders[vertex],      $"order of the vertex at {position}" );
            Assert.AreEqual( end ? 1 : shared ? 2 : 0, run.mesh.sharpCounts[vertex], $"sharp edges of the vertex at {position}" );
        }
    }

    [Test]
    public void vertexWhereTwoSurfacesTouchIsOfOrderTwo( ) {
        var builder = new Builder( ).box( cells( 0, 0, 0 ), cells( 2, 2, 2 ) ).box( cells( 2, 2, 2 ), cells( 2, 2, 2 ) );

        using var run = new Run( builder );

        Assert.IsTrue( run.edges( ).Values.All( ( edge ) => edge.triangles == 2 && edge.sharp == Sharp.None ), "every edge has two triangles and is not sharp" );

        for( var vertex = 0; vertex < run.mesh.vertexCount; ++vertex ) {
            Assert.AreEqual( builder.positions[vertex] == new Vector3( 2, 2, 2 ) ? 2 : 0, run.mesh.orders[vertex], $"order of the vertex at {builder.positions[vertex]}" );
            Assert.AreEqual( 0, run.mesh.sharpCounts[vertex], $"sharp edges of the vertex at {builder.positions[vertex]}" );
        }
    }

    [Test]
    public void verticesOfTwoTrianglesOnTheSameThreeVerticesAreLocked( ) {
        var builder = new Builder( ).box( cells( 0, 0, 0 ), cells( 2, 2, 2 ) );

        // back to back, on a corner of the box and two vertices of their own, and a vertex that no triangle uses
        var corner = new Vector3( 2, 2, 2 );
        var first  = new Vector3( 3, 2, 3 );
        var second = new Vector3( 2, 3, 3 );

        builder.triangle( corner, first, second );
        builder.triangle( second, first, corner );

        var unused = builder.vertex( new Vector3( 9, 9, 9 ) );

        using var run = new Run( builder );

        for( var vertex = 0; vertex < run.mesh.vertexCount; ++vertex ) {
            var position = builder.positions[vertex];

            Assert.AreEqual( position == corner || position == first || position == second, run.mesh.locked[vertex], $"the vertex at {position} is locked" );
        }

        Assert.AreEqual( -1, run.mesh.firstCorners[unused], "the first triangle of the vertex without triangles" );
        Assert.AreEqual( -1, run.mesh.firstEdges[unused],   "the first edge of the vertex without triangles" );

        // the mesh that is written holds the vertices that triangles use and no other
        Assert.AreEqual( builder.positions.Count - 1, run.writtenPositions.Length, "vertices written" );
        Assert.AreEqual( builder.triangles.Count,     run.writtenTriangles.Length, "triangles written" );

        Assert.IsFalse( run.writtenPositions.Contains( new Vector3( 9, 9, 9 ) ), "the vertex without triangles is written" );
    }

    [Test]
    public void trianglesThatUseAVertexTwiceAreLeftOut( ) {
        var builder = new Builder( ).box( cells( 0, 0, 0 ), cells( 1, 1, 1 ) );

        builder.triangles.Add( new BurstMesh.Triangle( 0, 0, 1, 0 ) );
        builder.triangles.Add( new BurstMesh.Triangle( 2, 3, 2, 0 ) );

        using var run = new Run( builder );

        Assert.AreEqual( 12, run.triangleCount, "triangles" );
        Assert.AreEqual( 18, run.edges( ).Count, "edges" );

        Assert.IsTrue( run.edges( ).Values.All( ( edge ) => edge.triangles == 2 ), "every edge has two triangles" );
    }

    // ---- the link conditions ----

    // Dey et al., figure 2. To the left the links of a and b share the two vertices opposite the edge and nothing else.
    // To the right they also share a vertex z and the edge from it to one of the two: here z is a corner of an
    // octahedron and the vertex x was put into a face of it, which gives x three triangles
    [Test]
    public void figureTwoOfDeyEtAl( ) {
        var left = octahedron( );

        Assert.AreEqual( Refusal.None, checkMidpoint( left, left.vertex( px ), left.vertex( py ) ), "to the left" );

        var x     = new Vector3( 1, 1, 1 );
        var right = octahedron( withoutFirstFace: true );

        right.triangle( px, py, x );
        right.triangle( py, pz, x );
        right.triangle( pz, px, x );

        Assert.AreEqual( Refusal.LinkVertex, checkMidpoint( right, right.vertex( px ), right.vertex( py ) ), "to the right" );

        // an edge of x collapses: it takes x out of the face again, which leaves the octahedron
        Assert.AreEqual( Refusal.None, checkMidpoint( right, right.vertex( x ), right.vertex( px ) ), "from x" );
    }

    [Test]
    public void edgesOfABoxCollapse( ) {
        var builder = new Builder( ).box( cells( 0, 0, 0 ), cells( 3, 3, 3 ) );

        using var counted = new Run( builder );

        foreach( var edge in counted.edges( ).Keys ) {
            Assert.AreEqual( Refusal.None, checkMidpoint( builder, edge.Item1, edge.Item2 ), $"the edge from {builder.positions[edge.Item1]} to {builder.positions[edge.Item2]}" );
        }
    }

    // the two vertices opposite an edge of a tetrahedron are joined by an edge that is in the links of both ends
    [Test]
    public void noEdgeOfATetrahedronCollapses( ) {
        var corners = new[] { new Vector3( 1, 1, 1 ), new Vector3( 1, -1, -1 ), new Vector3( -1, 1, -1 ), new Vector3( -1, -1, 1 ) };
        var builder = new Builder( );

        builder.triangle( corners[0], corners[2], corners[1] );
        builder.triangle( corners[0], corners[1], corners[3] );
        builder.triangle( corners[0], corners[3], corners[2] );
        builder.triangle( corners[1], corners[2], corners[3] );

        for( var a = 0; a < 4; ++a )
        for( var b = a + 1; b < 4; ++b ) {
            Assert.AreEqual( Refusal.LinkEdge, checkMidpoint( builder, a, b ), $"the edge from {a} to {b}" );
        }
    }

    [Test]
    public void edgeOfFourTrianglesCollapsesUnlessBothEndsAreItsEnds( ) {
        // one segment: both of its ends are of order 2
        var one = boxesSharingAnEdge( 1 );

        Assert.AreEqual( Refusal.LinkCorners, checkMidpoint( one, one.at( 1, 1, 0 ), one.at( 1, 1, 1 ) ), "the only segment" );

        // three segments: the two in the middle are of order 1
        var three = boxesSharingAnEdge( 3 );

        Assert.AreEqual( Refusal.None, checkMidpoint( three, three.at( 1, 1, 0 ), three.at( 1, 1, 1 ) ), "the first segment" );
        Assert.AreEqual( Refusal.None, checkMidpoint( three, three.at( 1, 1, 1 ), three.at( 1, 1, 2 ) ), "the segment in the middle" );
        Assert.AreEqual( Refusal.None, checkMidpoint( three, three.at( 1, 1, 2 ), three.at( 1, 1, 3 ) ), "the last segment" );

        // from the shared edge onto a face of one box: one end is of order 0
        Assert.AreEqual( Refusal.None, checkMidpoint( three, three.at( 1, 1, 1 ), three.at( 0, 1, 1 ) ), "from the middle of the shared edge onto a face" );
        Assert.AreEqual( Refusal.None, checkMidpoint( three, three.at( 1, 1, 0 ), three.at( 0, 1, 0 ) ), "from the end of the shared edge along an edge of a box" );
    }

    [Test]
    public void pinchedVertexCollapsesWithAVertexOfTheSurfaceButNotWithAnotherPinchedVertex( ) {
        var one = new Builder( ).box( cells( 0, 0, 0 ), cells( 2, 2, 2 ) ).box( cells( 2, 2, 2 ), cells( 2, 2, 2 ) );

        Assert.AreEqual( Refusal.None, checkMidpoint( one, one.at( 2, 2, 2 ), one.at( 1, 2, 2 ) ), "from the pinched vertex along an edge of a box" );
        Assert.AreEqual( Refusal.None, checkMidpoint( one, one.at( 2, 2, 2 ), one.at( 2, 2, 3 ) ), "from the pinched vertex along an edge of the other box" );

        // a box one cell wide with another box at each end of one of its edges. The edge has two triangles, so it is
        // not in the first boundary, and both of its ends are. No sharp edge is anywhere near
        var two = new Builder( ).box( cells( 0, 0, 0 ), cells( 1, 2, 2 ) ).box( cells( -1, 2, 2 ), cells( 1, 1, 1 ) ).box( cells( 1, 2, 2 ), cells( 1, 1, 1 ) );

        using var counted = new Run( two );

        Assert.AreEqual( 2, counted.mesh.orders[two.at( 0, 2, 2 )], "order of one end" );
        Assert.AreEqual( 2, counted.mesh.orders[two.at( 1, 2, 2 )], "order of the other end" );
        Assert.AreEqual( 0, counted.mesh.sharpCounts[two.at( 0, 2, 2 )] + counted.mesh.sharpCounts[two.at( 1, 2, 2 )], "sharp edges at the ends" );

        Assert.AreEqual( Refusal.LinkBoundary, checkMidpoint( two, two.at( 0, 2, 2 ), two.at( 1, 2, 2 ) ), "between the two pinched vertices" );
    }

    [Test]
    public void openEdgesCollapseAlongTheRimAndNotAcrossIt( ) {
        var patch = new Builder( ).patch( 4, 4 );

        Assert.AreEqual( Refusal.None, checkMidpoint( patch, patch.at( 1, 1, 0 ), patch.at( 2, 1, 0 ) ), "inside the patch" );
        Assert.AreEqual( Refusal.None, checkMidpoint( patch, patch.at( 1, 0, 0 ), patch.at( 2, 0, 0 ) ), "along the rim" );
        Assert.AreEqual( Refusal.None, checkMidpoint( patch, patch.at( 1, 0, 0 ), patch.at( 1, 1, 0 ) ), "from the rim inwards" );
        Assert.AreEqual( Refusal.None, checkMidpoint( patch, patch.at( 0, 0, 0 ), patch.at( 1, 0, 0 ) ), "from a corner of the patch along the rim" );

        // a strip one quad wide: every edge across it joins two vertices of the rim and is not of the rim
        var strip = new Builder( ).patch( 4, 1 );

        Assert.AreEqual( Refusal.LinkBoundary, checkMidpoint( strip, strip.at( 2, 0, 0 ), strip.at( 2, 1, 0 ) ), "across the strip" );
        Assert.AreEqual( Refusal.LinkBoundary, checkMidpoint( strip, strip.at( 2, 0, 0 ), strip.at( 3, 1, 0 ) ), "across the strip along a diagonal" );
        Assert.AreEqual( Refusal.None,         checkMidpoint( strip, strip.at( 1, 0, 0 ), strip.at( 2, 0, 0 ) ), "along the strip" );

        // a hole of three edges: collapsing one would leave the other two as one edge with a triangle on either side
        // and no hole. The third vertex of the hole is next to both ends of each edge and opposite to neither
        var hole = octahedron( withoutFirstFace: true );

        Assert.AreEqual( Refusal.LinkVertex, checkMidpoint( hole, hole.vertex( px ), hole.vertex( py ) ), "an edge of a hole of three" );
        Assert.AreEqual( Refusal.LinkVertex, checkMidpoint( hole, hole.vertex( py ), hole.vertex( pz ) ), "another edge of the hole" );

        // a triangle by itself: collapsing an edge would leave the other two as one edge without a triangle
        var triangle = new Builder( );

        triangle.triangle( px, py, pz );

        Assert.AreEqual( Refusal.LinkBoundaryEdge, checkMidpoint( triangle, 0, 1 ), "an edge of a single triangle" );
    }

    // a triangle that stands on an edge of a box like a fin. The edge has three triangles, the other two edges of the
    // fin have one, and collapsing the edge would leave those two as one edge without a triangle
    [Test]
    public void edgeThatAFinStandsOnDoesNotCollapse( ) {
        var builder = new Builder( ).box( cells( 0, 0, 0 ), cells( 3, 3, 3 ) );

        builder.triangle( new Vector3( 1, 1, 3 ), new Vector3( 2, 1, 3 ), new Vector3( 1.5f, 1.0f, 4.0f ) );

        var a   = builder.at( 1, 1, 3 );
        var b   = builder.at( 2, 1, 3 );
        var tip = builder.at( 1.5f, 1.0f, 4.0f );

        using var counted = new Run( builder );

        Assert.AreEqual( ( 3, Sharp.NonManifold ), counted.edges( )[( a, b )],   "the edge the fin stands on" );
        Assert.AreEqual( ( 1, Sharp.Open ),        counted.edges( )[( a, tip )], "an edge of the fin" );

        // the edge of three triangles ends at both of its vertices
        Assert.AreEqual( 2, counted.mesh.orders[a],   "order of one end" );
        Assert.AreEqual( 2, counted.mesh.orders[b],   "order of the other end" );
        Assert.AreEqual( 1, counted.mesh.orders[tip], "order of the tip of the fin" );

        Assert.AreEqual( Refusal.LinkBoundaryEdge, checkMidpoint( builder, a, b ), "the edge the fin stands on" );

        // nor does an edge of the fin, which would leave the edge it stands on and its other edge as one
        Assert.AreEqual( Refusal.LinkBoundaryEdge, checkMidpoint( builder, a, tip ), "an edge of the fin" );
    }

    [Test]
    public void noEdgeAtALockedVertexCollapses( ) {
        var builder = new Builder( ).box( cells( 0, 0, 0 ), cells( 2, 2, 2 ) );

        var corner = new Vector3( 2, 2, 2 );
        var first  = new Vector3( 3, 2, 3 );
        var second = new Vector3( 2, 3, 3 );

        builder.triangle( corner, first, second );
        builder.triangle( second, first, corner );

        Assert.AreEqual( Refusal.Locked, checkMidpoint( builder, builder.vertex( first ), builder.vertex( second ) ),      "an edge of the two triangles" );
        Assert.AreEqual( Refusal.Locked, checkMidpoint( builder, builder.vertex( corner ), builder.vertex( first ) ),      "another edge of the two triangles" );
        Assert.AreEqual( Refusal.Locked, checkMidpoint( builder, builder.vertex( corner ), builder.at( 1, 2, 2 ) ),        "from a vertex of the two triangles along the box" );
        Assert.AreEqual( Refusal.None,   checkMidpoint( builder, builder.at( 0, 0, 0 ), builder.at( 1, 0, 0 ) ),           "away from the two triangles" );
    }

    // ---- the lines of sharp edges ----

    // the top of a box of 4 x 4 x 4 cells, where the tests below draw lines between materials. The vertex at ( i, j ) of
    // the top is at ( i, j, 4 ), and a quad of the top is split from its corner ( i, j ) to ( i + 1, j + 1 )
    private static Builder boxWithTop( Func<int, int, int> subMeshOf ) {
        return new Builder( ).box( cells( 0, 0, 0 ), cells( 4, 4, 4 ), ( center, axis, side ) => axis == 2 && side == 1 ? subMeshOf( Mathf.FloorToInt( center.x ), Mathf.FloorToInt( center.y ) ) : 0 );
    }

    // Hoppe's first two tests: sharp( s, l ) and sharp( t, l ), and the same for r. A single triangle of a second
    // material has three sharp edges, and collapsing any of them would leave the other two as one edge and no triangle
    [Test]
    public void sharpEdgesToTheSameVertexKeepAnEdgeFromCollapsing( ) {
        var builder = boxWithTop( ( i, j ) => 0 );

        var a = builder.at( 1, 1, 4 );
        var b = builder.at( 2, 1, 4 );
        var c = builder.at( 2, 2, 4 );
        var d = builder.at( 1, 2, 4 );

        // the triangle on one side of the diagonal of a quad, and that on the other side of the diagonal of another
        builder.setSubMesh( a, b, c, 1 );

        Assert.AreEqual( Refusal.SharpNeighbour, checkMidpoint( builder, a, b ), "a b, with c to one side" );
        Assert.AreEqual( Refusal.SharpNeighbour, checkMidpoint( builder, b, c ), "b c, with a to one side" );
        Assert.AreEqual( Refusal.SharpNeighbour, checkMidpoint( builder, a, c ), "the diagonal, with b to one side" );

        builder.setSubMesh( a, b, c, 0 );
        builder.setSubMesh( a, c, d, 1 );

        Assert.AreEqual( Refusal.SharpNeighbour, checkMidpoint( builder, a, c ), "the diagonal, with d to the other side" );
        Assert.AreEqual( Refusal.SharpNeighbour, checkMidpoint( builder, c, d ), "c d, with a to the other side" );
    }

    // Hoppe's third test: both ends on sharp edges and the edge not sharp. A band one quad wide has a line along either
    // side, and collapsing an edge across it would join the two lines
    [Test]
    public void edgeBetweenTwoLinesOfSharpEdgesDoesNotCollapse( ) {
        var builder = boxWithTop( ( i, j ) => i == 2 ? 1 : 0 );

        Assert.AreEqual( Refusal.SharpEnds, checkMidpoint( builder, builder.at( 2, 2, 4 ), builder.at( 3, 2, 4 ) ), "across the band" );
        Assert.AreEqual( Refusal.SharpEnds, checkMidpoint( builder, builder.at( 2, 2, 4 ), builder.at( 3, 3, 4 ) ), "across the band along a diagonal" );

        // along a line, and from a line into the surface on either side of it
        Assert.AreEqual( Refusal.None, checkMidpoint( builder, builder.at( 2, 1, 4 ), builder.at( 2, 2, 4 ) ), "along one side of the band" );
        Assert.AreEqual( Refusal.None, checkMidpoint( builder, builder.at( 3, 2, 4 ), builder.at( 3, 3, 4 ) ), "along the other side of the band" );
        Assert.AreEqual( Refusal.None, checkMidpoint( builder, builder.at( 2, 2, 4 ), builder.at( 1, 2, 4 ) ), "from the band outwards" );
    }

    // Hoppe's fourth test: the edge is sharp and three or more sharp edges meet at both ends. Four materials meet in
    // threes at two vertices next to each other, and collapsing the edge between them would make one meeting of four
    [Test]
    public void sharpEdgeBetweenTwoMeetingsOfLinesDoesNotCollapse( ) {
        // left of i = 2: material 1 below j = 3 and material 3 above. Right of it: material 0 below j = 2 and 2 above
        var builder = boxWithTop( ( i, j ) => i < 2 ? ( j < 3 ? 1 : 3 ) : ( j < 2 ? 0 : 2 ) );

        using var counted = new Run( builder );

        Assert.AreEqual( 3, counted.mesh.sharpCounts[builder.at( 2, 2, 4 )], "sharp edges at the lower meeting" );
        Assert.AreEqual( 3, counted.mesh.sharpCounts[builder.at( 2, 3, 4 )], "sharp edges at the upper meeting" );

        Assert.AreEqual( Refusal.SharpCorners, checkMidpoint( builder, builder.at( 2, 2, 4 ), builder.at( 2, 3, 4 ) ), "between the two meetings" );

        // from a meeting along one of its lines to a vertex that the line passes straight through
        Assert.AreEqual( Refusal.None, checkMidpoint( builder, builder.at( 2, 2, 4 ), builder.at( 2, 1, 4 ) ), "from the lower meeting down its line" );
        Assert.AreEqual( Refusal.None, checkMidpoint( builder, builder.at( 2, 2, 4 ), builder.at( 3, 2, 4 ) ), "from the lower meeting along the line to its right" );
    }

    // Hoppe's last two tests: the edge is the only sharp edge of one end, and the other end is not on a line of two.
    // A line of sharp edges ends where an edge of four triangles ends. Here a line between two materials runs into the
    // middle of such an edge, which makes three sharp edges there, and the last segment of the edge would take the
    // end of the line into that meeting. In the first mesh the end of the line is the higher vertex of the edge, in the
    // second the lower, which are Hoppe's two tests
    [Test]
    public void lastSharpEdgeOfALineDoesNotCollapseIntoAMeetingOfLines( [Values( false, true )] bool endIsLower ) {
        // a quad of the first box that lies along the shared edge is of a second material: below the middle of the edge
        // when the line is to end at the top, above it otherwise
        var builder = boxesSharingAnEdge( 2, ( center, axis, side ) => axis == 0 && side == 1 && ( center.z < 1 ) != endIsLower ? 1 : 0 );

        var middle = builder.at( 1, 1, 1 );
        var end    = builder.at( 1, 1, endIsLower ? 0 : 2 );
        var other  = builder.at( 1, 1, endIsLower ? 2 : 0 );

        Assert.AreEqual( endIsLower, end < middle, "the end of the line is the lower vertex of the edge" );

        using var counted = new Run( builder );

        Assert.AreEqual( 1, counted.mesh.sharpCounts[end],    "sharp edges at the end of the line" );
        Assert.AreEqual( 3, counted.mesh.sharpCounts[middle], "sharp edges at the middle of the shared edge" );
        Assert.AreEqual( 2, counted.mesh.sharpCounts[other],  "sharp edges at the other end of the shared edge" );

        Assert.AreEqual( Refusal.SharpDart, checkMidpoint( builder, end, middle ), "the last segment of the line" );

        // the other segment runs from the meeting to where the shared edge ends. That end has two sharp edges, the
        // shared edge and a line between materials, which is enough for Hoppe's tests, and it is of order 2 while the
        // meeting is of order 1, which is enough for the link conditions. It is a point of its own all the same, and
        // so is the meeting
        Assert.AreEqual( 2, counted.mesh.orders[other],  "order of the other end of the shared edge" );
        Assert.AreEqual( 1, counted.mesh.orders[middle], "order of the middle of the shared edge" );

        Assert.AreEqual( Refusal.Points, checkMidpoint( builder, other, middle ), "the other segment" );
    }

    // a box whose top has three materials: material 1 where x < 3, and right of that material 0 below y = 2 and
    // material 2 above. The three meet at ( 3, 2 ), and the line between 1 and 2 runs on from there through ( 3, 3 ),
    // ( 3, 4 ) and ( 3, 5 ). A pyramid of material 0 stands on its tip on a vertex of the top, which pinches the mesh
    // there
    private static Builder boxWithPyramidOn( Vector3 tip ) {
        var builder = new Builder( ).box( cells( 0, 0, 0 ), cells( 6, 6, 2 ), ( center, axis, side ) => axis == 2 && side == 1 ? ( center.x < 3 ? 1 : center.y < 2 ? 0 : 2 ) : 0 );

        var corners = new[] { new Vector3( -0.5f, -0.5f, 1 ), new Vector3( 0.5f, -0.5f, 1 ), new Vector3( 0.5f, 0.5f, 1 ), new Vector3( -0.5f, 0.5f, 1 ) }.Select( ( corner ) => tip + corner ).ToArray( );

        for( var side = 0; side < 4; ++side ) {
            builder.triangle( tip, corners[( side + 1 ) % 4], corners[side] );
        }
        builder.quad( corners[0], corners[1], corners[2], corners[3] );

        return builder;
    }

    // neither paper has this case, since Hoppe's meshes are manifolds and Dey et al. have no materials: a vertex of
    // order 2 that is not on any sharp line of its own, such as one where two surfaces touch
    [Test]
    public void pinchedVertexStaysWhereItIsAmongTheSharpLines( ) {
        // the pyramid stands on the line between two materials, next to where three meet
        var onLine = boxWithPyramidOn( new Vector3( 3, 3, 2 ) );

        using var counted = new Run( onLine );

        Assert.AreEqual( 2, counted.mesh.orders[onLine.at( 3, 3, 2 )],      "order of the vertex the pyramid stands on" );
        Assert.AreEqual( 2, counted.mesh.sharpCounts[onLine.at( 3, 3, 2 )], "sharp edges of the vertex the pyramid stands on" );
        Assert.AreEqual( 3, counted.mesh.sharpCounts[onLine.at( 3, 2, 2 )], "sharp edges where the three materials meet" );
        Assert.AreEqual( 2, counted.mesh.sharpCounts[onLine.at( 3, 4, 2 )], "sharp edges further along the line" );

        // along the line into the meeting of three: the pinch would move onto the meeting
        Assert.AreEqual( Refusal.Points, checkMidpoint( onLine, onLine.at( 3, 3, 2 ), onLine.at( 3, 2, 2 ) ), "from the pinch to the meeting of three" );

        // along the line the other way: the vertex there is on the line and nothing more, and moves into the pinch
        Assert.AreEqual( Refusal.None, checkMidpoint( onLine, onLine.at( 3, 3, 2 ), onLine.at( 3, 4, 2 ) ), "from the pinch along the line" );
        Assert.AreEqual( Refusal.None, checkMidpoint( onLine, onLine.at( 3, 3, 2 ), onLine.at( 2, 3, 2 ) ), "from the pinch off the line" );

        // the pyramid stands inside one material, next to the line: the pinch would move onto the line
        var besideLine = boxWithPyramidOn( new Vector3( 2, 4, 2 ) );

        Assert.AreEqual( Refusal.Points, checkMidpoint( besideLine, besideLine.at( 2, 4, 2 ), besideLine.at( 3, 4, 2 ) ), "from the pinch onto the line" );
        Assert.AreEqual( Refusal.None,   checkMidpoint( besideLine, besideLine.at( 2, 4, 2 ), besideLine.at( 1, 4, 2 ) ), "from the pinch away from the line" );

        assertCollapsesKeepTheShape( "a pyramid on a line between materials", onLine,     0.4 );
        assertCollapsesKeepTheShape( "a pyramid beside a line between materials", besideLine, 0.4 );
    }

    // ---- triangles that turn over ----

    // Garland and Heckbert, section 6: a collapse that turns the normal of a neighbouring triangle over is not made
    [Test]
    public void collapseThatFoldsTheFanAroundAVertexIsRefused( ) {
        var builder = new Builder( ).patch( 5, 5 );

        var a = builder.at( 2, 2, 0 );
        var b = builder.at( 3, 2, 0 );

        using var run = new Run( builder, new[] {
            check( a, b, new Vector3( 2.5f, 2.0f, 0.0f ) ),  // halfway
            check( a, b, new Vector3( 2.5f, 2.3f, 0.4f ) ),  // off the plane, and still above the same triangles
            check( a, b, new Vector3( 9.0f, 2.0f, 0.0f ) ),  // far past the vertices around b
            check( a, b, new Vector3( 2.5f, 7.0f, 0.0f ) ),  // far past the vertices to one side
            check( a, b, new Vector3( 1.0f, 2.0f, 0.0f ) ),  // on a vertex next to a, which leaves two triangles without area
            check( a, b, new Vector3( 2.5f, 3.0f, 0.0f ) )   // on the edge between two vertices next to both
        } );

        Assert.AreEqual( Refusal.None,      run.refusals[0], "halfway" );
        Assert.AreEqual( Refusal.None,      run.refusals[1], "off the plane" );
        Assert.AreEqual( Refusal.Inversion, run.refusals[2], "far past the vertices around b" );
        Assert.AreEqual( Refusal.Inversion, run.refusals[3], "far to one side" );
        Assert.AreEqual( Refusal.Inversion, run.refusals[4], "on a vertex next to a" );
        Assert.AreEqual( Refusal.Inversion, run.refusals[5], "on an edge of the ring around both" );
    }


    // ---- quadrics ----

    // every vertex of a flat patch has the plane of the patch, weighted by a third of the area of its triangles. A
    // vertex of the rim also has, for each of its two open edges, the plane through the edge that stands on the patch,
    // weighted by a third of the area of the edge's triangle, in a quadric of its own. A triangle of the patch is half
    // a unit quad
    [Test]
    public void quadricsHoldThePlanesOfTheTrianglesAndOfTheOpenEdges( ) {
        var builder = new Builder( ).patch( 4, 4 );

        using var run = new Run( builder );

        // 0.5 up from the patch, 0.3 along x and 0.2 against y
        var offset = new double3( 0.3, -0.2, 0.5 );

        double3 at( int vertex ) {
            var position = builder.positions[vertex];

            return new double3( position.x, position.y, position.z ) + offset;
        }

        // inside: six triangles and no sharp edge
        var inside = builder.at( 2, 2, 0 );

        Assert.AreEqual( 1.0,  run.mesh.areas[inside],                         1e-12, "area of a vertex inside" );
        Assert.AreEqual( 0.25, run.mesh.planes[inside].evaluate( at( inside ) ), 1e-12, "error of a vertex inside" );
        Assert.AreEqual( 0.0,  run.mesh.sharpAreas[inside],                           "area of the sharp edges of a vertex inside" );
        Assert.AreEqual( 0.0,  run.mesh.sharpPlanes[inside].evaluate( at( inside ) ),   "error of the sharp edges of a vertex inside" );

        // on the rim along x: three triangles, and two open edges with a triangle each, both along x
        var rim = builder.at( 2, 0, 0 );

        Assert.AreEqual( 0.5,                run.mesh.areas[rim],                           1e-12, "area of a vertex of the rim" );
        Assert.AreEqual( 0.5 * 0.25,         run.mesh.planes[rim].evaluate( at( rim ) ),      1e-12, "error of a vertex of the rim" );
        Assert.AreEqual( 1.0 / 3.0,          run.mesh.sharpAreas[rim],                      1e-12, "area of the sharp edges of a vertex of the rim" );
        Assert.AreEqual( ( 1.0 / 3.0 ) * 0.04, run.mesh.sharpPlanes[rim].evaluate( at( rim ) ), 1e-12, "error of the sharp edges of a vertex of the rim" );

        // the corner at the origin: two triangles, an open edge along x and one along y
        var corner = builder.at( 0, 0, 0 );

        Assert.AreEqual( 1.0 / 3.0,                        run.mesh.areas[corner],                              1e-12, "area of a corner" );
        Assert.AreEqual( 0.25 / 3.0,                       run.mesh.planes[corner].evaluate( at( corner ) ),      1e-12, "error of a corner" );
        Assert.AreEqual( 1.0 / 3.0,                        run.mesh.sharpAreas[corner],                         1e-12, "area of the sharp edges of a corner" );
        Assert.AreEqual( ( 0.5 / 3.0 ) * ( 0.09 + 0.04 ),  run.mesh.sharpPlanes[corner].evaluate( at( corner ) ), 1e-12, "error of the sharp edges of a corner" );
    }

    // an edge between two materials has two triangles, and the plane that stands on each of them through the edge. On
    // a flat patch with the line between the materials along y those are four times the same plane for a vertex of
    // the line
    [Test]
    public void quadricsHoldThePlanesOfTheEdgesBetweenMaterials( ) {
        var builder = new Builder( ).patch( 4, 4, ( i, j ) => i < 2 ? 1 : 0 );

        using var run = new Run( builder );

        var vertex   = builder.at( 2, 2, 0 );
        var position = new double3( 2.0, 2.0, 0.0 );

        Assert.AreEqual( 2, run.mesh.sharpCounts[vertex], "sharp edges of a vertex of the line" );

        Assert.AreEqual( 1.0 * 0.25,                 run.mesh.planes[vertex].evaluate( position + new double3( 0.3, -0.2, 0.5 ) ),      1e-12, "error of a vertex of the line" );
        Assert.AreEqual( 4.0 * ( 0.5 / 3.0 ),        run.mesh.sharpAreas[vertex],                                                      1e-12, "area of the sharp edges of a vertex of the line" );
        Assert.AreEqual( 4.0 * ( 0.5 / 3.0 ) * 0.09, run.mesh.sharpPlanes[vertex].evaluate( position + new double3( 0.3, -0.2, 0.5 ) ), 1e-12, "error of the sharp edges of a vertex of the line" );

        // along the line it costs nothing
        Assert.AreEqual( 0.0, run.mesh.sharpPlanes[vertex].evaluate( position + new double3( 0.0, 0.7, 0.0 ) ), 1e-12, "error along the line" );
    }

    private static double3 positionOf( Builder builder, int vertex ) {
        var position = builder.positions[vertex];

        return new double3( position.x, position.y, position.z );
    }

    // the sum of the normals of the triangles of a vertex, each times twice the area of its triangle
    private static double3 normalOfTrianglesOf( Builder builder, int vertex ) {
        var normal = double3.zero;

        foreach( var triangle in builder.triangles ) {
            if( triangle.a == vertex || triangle.b == vertex || triangle.c == vertex ) {
                normal += math.cross( positionOf( builder, triangle.b ) - positionOf( builder, triangle.a ), positionOf( builder, triangle.c ) - positionOf( builder, triangle.a ) );
            }
        }
        return normal;
    }

    private static readonly double3[] offsets = { new( 0.3, -0.2, 0.5 ), new( -1.5, 0.25, 2.0 ), new( 0.0, 3.0, -0.75 ), new( 4.0, 4.0, 4.0 ) };

    // Liu et al., equations 12 to 16: every vertex has the squared distance to the line through it along the normal of
    // its triangles, which is the sum of their normals weighted by their areas, times the line weight and the area of
    // the vertex. On a box that line is square to the face for a vertex inside a face, and leans towards the faces
    // that the vertex has more of at an edge or a corner
    [Test]
    public void linesHoldEveryVertexToTheLineAlongTheNormalOfItsTriangles( [Values( 1e-5, 1e-2 )] double weight ) {
        var box = new Builder( ).box( cells( 0, 0, 0 ), cells( 3, 4, 5 ) );

        // a tetrahedron with faces of four different areas, so that a normal weighted by area is not a plain mean
        var tetrahedron = new Builder( );
        var origin      = Vector3.zero;
        var x           = new Vector3( 4, 0, 0 );
        var y           = new Vector3( 0, 2, 0 );
        var z           = new Vector3( 0, 0, 1 );

        tetrahedron.triangle( origin, y, x );
        tetrahedron.triangle( origin, x, z );
        tetrahedron.triangle( origin, z, y );
        tetrahedron.triangle( x, y, z );

        foreach( var builder in new[] { box, tetrahedron } ) {
            using var run = new Run( builder, lineWeight: weight );

            var areas = new HashSet<double>( );

            for( var vertex = 0; vertex < run.mesh.vertexCount; ++vertex ) {
                var position = positionOf( builder, vertex );
                var normal   = math.normalize( normalOfTrianglesOf( builder, vertex ) );

                areas.Add( Math.Round( run.mesh.areas[vertex], 6 ) );

                foreach( var offset in offsets ) {
                    var expected = weight * run.mesh.areas[vertex] * math.lengthsq( math.cross( offset, normal ) );

                    Assert.AreEqual( expected, run.mesh.lines[vertex].evaluate( position + offset ), 1e-9 * expected, $"error of the line of the vertex at {position}, {offset} from it" );
                }

                Assert.AreEqual( 0.0, run.mesh.lines[vertex].evaluate( position + ( normal * 2.5 ) ), 1e-15, $"error along the line of the vertex at {position}" );
            }

            // the vertices stand for different areas, so the area is part of what is tested
            Assert.Greater( areas.Count, 1, "different areas among the vertices" );
        }

        // inside the face of the box at z = 0 a vertex has six triangles and an area of 1, and its line runs along z
        using var boxRun = new Run( box, lineWeight: weight );

        Assert.AreEqual( weight * 0.25, boxRun.mesh.lines[box.at( 1, 2, 0 )].evaluate( new double3( 1.3, 1.6, 7.0 ) ), 1e-9 * weight, "error of the line of a vertex inside a face" );
    }

    // at a line weight of zero no vertex has a line, and the simplifier is Garland and Heckbert's
    [Test]
    public void withoutALineWeightNoVertexHasALine( ) {
        var builder = new Builder( ).box( cells( 0, 0, 0 ), cells( 3, 4, 5 ) );

        builder.triangle( new Vector3( 7, 7, 7 ), new Vector3( 8, 7, 7 ), new Vector3( 7, 8, 7 ) );
        builder.triangle( new Vector3( 7, 7, 7 ), new Vector3( 7, 8, 7 ), new Vector3( 8, 7, 7 ) );

        using var run = new Run( builder );

        for( var vertex = 0; vertex < run.mesh.vertexCount; ++vertex ) {
            Assert.AreEqual( 0.0, run.mesh.lines[vertex].trace,                                                  $"size of the line of vertex {vertex}" );
            Assert.AreEqual( 0.0, run.mesh.lines[vertex].evaluate( positionOf( builder, vertex ) + offsets[0] ), $"error of the line of vertex {vertex}" );
        }
    }

    // Liu et al., section 4.2: the normals of two triangles that lie back to back cancel, and their vertices have no
    // normal to run a line along. Such a vertex is held to itself instead, by a millionth of its area. A vertex that
    // no triangle uses stands for nothing and is held by nothing
    [Test]
    public void vertexWhoseTrianglesCancelIsHeldToItselfInstead( [Values( 1e-5, 1e-2 )] double weight ) {
        var builder = new Builder( );

        // of an area of 6
        builder.triangle( new Vector3( 1, 1, 1 ), new Vector3( 4, 1, 1 ), new Vector3( 1, 5, 1 ) );
        builder.triangle( new Vector3( 1, 1, 1 ), new Vector3( 1, 5, 1 ), new Vector3( 4, 1, 1 ) );

        var unused = builder.vertex( new Vector3( 9, 9, 9 ) );

        using var run = new Run( builder, lineWeight: weight );

        for( var vertex = 0; vertex < 3; ++vertex ) {
            Assert.AreEqual( 4.0, run.mesh.areas[vertex], 1e-12, $"area of vertex {vertex}" );

            foreach( var offset in offsets ) {
                var expected = BurstMeshSimplifier.PointWeight * 4.0 * math.lengthsq( offset );

                Assert.AreEqual( expected, run.mesh.lines[vertex].evaluate( positionOf( builder, vertex ) + offset ), 1e-9 * expected, $"error of the point of vertex {vertex}, {offset} from it" );
            }
        }

        Assert.AreEqual( 1e-6, BurstMeshSimplifier.PointWeight, "weight of a point" );

        Assert.AreEqual( 0.0, run.mesh.lines[unused].trace, "size of the line of the vertex without triangles" );
    }

    // an edge as the loop sees it, from the quadrics of a run
    private static bool evaluate( Run run, int a, int b, double limit, out double3 target, out double cost, out bool within ) {
        return BurstMeshSimplifier.evaluate(
            run.mesh.planes[a] + run.mesh.planes[b], run.mesh.areas[a] + run.mesh.areas[b], run.mesh.sharpPlanes[a] + run.mesh.sharpPlanes[b], run.mesh.sharpAreas[a] + run.mesh.sharpAreas[b],
            run.mesh.lines[a] + run.mesh.lines[b], run.mesh.positions[a], run.mesh.positions[b], limit, out target, out cost, out within
        );
    }

    // Liu et al., lemma 4.1, with the quadrics the simplifier builds and adds up: on a flat patch the vertex of a
    // collapsed edge goes to the centre of the vertices it stands for, each by its area, collapse after collapse. What
    // the collapse costs is the lines alone: the weight times the squared distances of those vertices from the centre,
    // each by its area. The surface is no further from its planes for it, so the collapse is within any error bound,
    // even one of nothing: the bound is on the planes and not on the lines
    [Test]
    public void onAFlatPatchACollapsedEdgeGoesToTheCentreOfItsVertices( [Values( 1e-5, 1e-3, 1e-1 )] double weight ) {
        var builder = new Builder( ).patch( 8, 8 );

        // three vertices inside the patch, each of an area of 1, with an edge between every two of them
        var a = builder.at( 3, 3, 0 );
        var b = builder.at( 4, 3, 0 );
        var c = builder.at( 4, 4, 0 );

        using var before = new Run( builder, lineWeight: weight );

        Assert.IsTrue( evaluate( before, a, b, 0.0, out var middle, out var first, out var firstWithin ), "the first edge has a best point" );

        Assert.LessOrEqual( math.length( middle - new double3( 3.5, 3.0, 0.0 ) ), 1e-9, $"where the first edge goes: {middle}" );
        Assert.AreEqual( weight * 0.5, first, 1e-9 * weight, "cost of the first edge" );
        Assert.IsTrue( firstWithin, "the first edge is within an error of nothing" );

        using var run = new Run( builder, new[] { collapse( a, b, new Vector3( 3.5f, 3.0f, 0.0f ) ) }, lineWeight: weight );

        Assert.AreEqual( Refusal.None, run.refusals[0], "the collapse is allowed" );

        var kept   = run.kept[0];
        var centre = new double3( 11.0 / 3.0, 10.0 / 3.0, 0.0 );

        Assert.AreEqual( 2.0, run.mesh.areas[kept], 1e-12, "area of the kept vertex" );

        Assert.IsTrue( evaluate( run, kept, c, 0.0, out var target, out var second, out var secondWithin ), "the second edge has a best point" );

        Assert.LessOrEqual( math.length( target - centre ), 1e-9, $"where the second edge goes: {target}" );

        var distances = math.distancesq( centre, new double3( 3, 3, 0 ) ) + math.distancesq( centre, new double3( 4, 3, 0 ) ) + math.distancesq( centre, new double3( 4, 4, 0 ) );

        Assert.AreEqual( weight * distances, second, 1e-9 * weight, "cost of the second edge" );
        Assert.IsTrue( secondWithin, "the second edge is within an error of nothing" );

        // without lines the plane of the patch says nothing about where on it the vertex goes
        using var plain = new Run( builder );

        Assert.IsFalse( evaluate( plain, a, b, 0.0, out _, out var free, out _ ), "the first edge has a best point without lines" );
        Assert.AreEqual( 0.0, free, "cost of the first edge without lines" );
    }

    // ---- a collapse ----

    // the mesh that the connectivity of a run wrote, built again from nothing
    private static Run rebuilt( Run run ) {
        return new Run( run.writtenPositions, run.writtenNormals, run.writtenTriangles );
    }

    // what a collapse keeps up to date has to be what building the collapsed mesh from nothing gives: for every vertex
    // its order, how many sharp edges it has and whether it is locked, and for every edge its triangles and why it is
    // sharp
    private static void assertMatchesRebuilt( Run run, string what ) {
        using var fresh = rebuilt( run );

        var vertices = run.verticesWithTriangles( );
        var written  = new Dictionary<int, int>( );

        for( var index = 0; index < vertices.Length; ++index ) {
            written.Add( vertices[index], index );
        }

        Assert.AreEqual( fresh.mesh.vertexCount, vertices.Length, $"{what}: vertices" );

        foreach( var vertex in vertices ) {
            Assert.AreEqual( fresh.mesh.orders[written[vertex]],      run.mesh.orders[vertex],      $"{what}: order of vertex {vertex} at {run.mesh.positions[vertex]}" );
            Assert.AreEqual( fresh.mesh.sharpCounts[written[vertex]], run.mesh.sharpCounts[vertex], $"{what}: sharp edges of vertex {vertex} at {run.mesh.positions[vertex]}" );
            Assert.AreEqual( fresh.mesh.locked[written[vertex]],      run.mesh.locked[vertex],      $"{what}: vertex {vertex} is locked" );
        }

        var expected = fresh.edges( );
        var actual   = run.edges( ).ToDictionary(
            ( edge ) => ( Math.Min( written[edge.Key.Item1], written[edge.Key.Item2] ), Math.Max( written[edge.Key.Item1], written[edge.Key.Item2] ) ),
            ( edge ) => edge.Value
        );

        Assert.AreEqual( expected.Count, actual.Count, $"{what}: edges" );

        foreach( var edge in expected ) {
            Assert.IsTrue( actual.ContainsKey( edge.Key ), $"{what}: the edge {edge.Key} is there" );
            Assert.AreEqual( edge.Value, actual[edge.Key], $"{what}: triangles and sharpness of the edge {edge.Key}" );
        }
    }

    [Test]
    public void collapseGivesTheKeptVertexWhatTheRemovedOneHad( ) {
        // the half of the box left of x = 1 is of a second material, and the edge runs along the line between the two
        var builder = new Builder( ).box( cells( 0, 0, 0 ), cells( 3, 3, 3 ), ( center, axis, side ) => center.x < 1 ? 1 : 0 );

        var a        = builder.at( 1, 1, 3 );
        var b        = builder.at( 1, 2, 3 );
        var kept     = Math.Min( a, b );
        var removed  = Math.Max( a, b );
        var position = new Vector3( 1.0f, 1.25f, 3.125f );

        using var before = new Run( builder, lineWeight: 1e-3 );
        using var run    = new Run( builder, new[] { collapse( a, b, position ) }, lineWeight: 1e-3 );

        Assert.AreEqual( Refusal.None, run.refusals[0], "the collapse is allowed" );
        Assert.AreEqual( kept,         run.kept[0],     "the vertex kept is the lower of the two" );

        // one vertex, three edges and two triangles fewer
        Assert.AreEqual( before.verticesWithTriangles( ).Length - 1, run.verticesWithTriangles( ).Length, "vertices" );
        Assert.AreEqual( before.edges( ).Count - 3,                  run.edges( ).Count,                  "edges" );
        Assert.AreEqual( before.triangleCount - 2,                   run.triangleCount,                   "triangles" );

        Assert.AreEqual( 3, run.mesh.removedEdges.Length, "edges that the collapse says it removed" );

        for( var index = 0; index < run.mesh.removedEdges.Length; ++index ) {
            var ends = before.mesh.edges[run.mesh.removedEdges[index]];

            Assert.IsTrue( ends.x == removed || ends.y == removed, "a removed edge was an edge of the removed vertex" );
            Assert.AreEqual( -1, run.mesh.edges[run.mesh.removedEdges[index]].x, "a removed edge has no vertices" );
        }

        Assert.AreEqual( -1, run.mesh.firstCorners[removed], "first triangle of the removed vertex" );
        Assert.AreEqual( -1, run.mesh.firstEdges[removed],   "first edge of the removed vertex" );

        // the triangles are those of before, without the two of the edge, with the kept vertex in place of the removed
        var expected = new List<BurstMesh.Triangle>( );

        foreach( var triangle in builder.triangles ) {
            var corners = new[] { triangle.a, triangle.b, triangle.c };

            if( corners.Contains( kept ) && corners.Contains( removed ) ) {
                continue;
            }

            int now( int vertex ) {
                // the vertices after the removed one move up by one in the mesh that is written
                var stays = vertex == removed ? kept : vertex;

                return stays > removed ? stays - 1 : stays;
            }

            expected.Add( new BurstMesh.Triangle( now( triangle.a ), now( triangle.b ), now( triangle.c ), triangle.subMeshIndex ) );
        }

        Assert.AreEqual( expected.Count, run.writtenTriangles.Length, "triangles written" );

        for( var index = 0; index < expected.Count; ++index ) {
            var actual = run.writtenTriangles[index];

            Assert.AreEqual(
                ( expected[index].a, expected[index].b, expected[index].c, expected[index].subMeshIndex ),
                ( actual.a, actual.b, actual.c, actual.subMeshIndex ),
                $"triangle {index}"
            );
        }

        // the kept vertex is at the position, with the areas and the quadrics of both added up
        Assert.AreEqual( position, ( Vector3 )( float3 )run.mesh.positions[kept], "position of the kept vertex" );
        Assert.AreEqual( position, run.writtenPositions[kept],                    "position written" );

        Assert.AreEqual( before.mesh.areas[a] + before.mesh.areas[b], run.mesh.areas[kept], 1e-12, "area of the kept vertex" );

        Assert.Greater( before.mesh.sharpAreas[a], 0.0, "area of the sharp edges of an end before" );

        Assert.AreEqual( before.mesh.sharpAreas[a] + before.mesh.sharpAreas[b], run.mesh.sharpAreas[kept], 1e-12, "area of the sharp edges of the kept vertex" );

        Assert.Greater( before.mesh.lines[a].trace, 0.0, "size of the line of an end before" );

        foreach( var point in new[] { new double3( 1.0, 2.0, 3.0 ), new double3( 0.3, -0.7, 4.1 ), new double3( 1.25, 1.0, 3.125 ) } ) {
            Assert.AreEqual( before.mesh.planes[a].evaluate( point ) + before.mesh.planes[b].evaluate( point ), run.mesh.planes[kept].evaluate( point ), 1e-9, $"error of the kept vertex at {point}" );

            Assert.AreEqual(
                before.mesh.sharpPlanes[a].evaluate( point ) + before.mesh.sharpPlanes[b].evaluate( point ), run.mesh.sharpPlanes[kept].evaluate( point ), 1e-9, $"error of the sharp edges of the kept vertex at {point}"
            );

            Assert.AreEqual( before.mesh.lines[a].evaluate( point ) + before.mesh.lines[b].evaluate( point ), run.mesh.lines[kept].evaluate( point ), 1e-12, $"error of the lines of the kept vertex at {point}" );
        }

        // and its normal is the sum of the two normals, each weighted by the area of its vertex
        var normal = math.normalize( ( before.mesh.normals[a] * before.mesh.areas[a] ) + ( before.mesh.normals[b] * before.mesh.areas[b] ) );

        Assert.LessOrEqual( math.length( normal - run.mesh.normals[kept] ), 1e-12, "normal of the kept vertex" );

        assertMatchesRebuilt( run, "after the collapse" );
    }

    // a normal for each vertex that leans a different way, so that the sum of two is not either of them
    [Test]
    public void collapseWeighsTheNormalsByTheAreasOfTheirVertices( ) {
        var builder = new Builder( ).patch( 4, 4 );
        var normals = builder.positions.Select( ( position ) => new Vector3( 0.1f * position.x, 0.2f * position.y, 1.0f ).normalized ).ToArray( );

        // a vertex inside, of six triangles, and one of the rim, of three
        var a = builder.at( 2, 1, 0 );
        var b = builder.at( 2, 0, 0 );

        using var run = new Run( builder.positions.ToArray( ), normals, builder.triangles.ToArray( ), new[] { collapse( a, b, new Vector3( 2.0f, 0.0f, 0.0f ) ) } );

        Assert.AreEqual( Refusal.None, run.refusals[0], "the collapse is allowed" );

        var expected = ( ( normals[a] * 1.0f ) + ( normals[b] * 0.5f ) ).normalized;

        Assert.LessOrEqual( ( expected - run.writtenNormals[Math.Min( a, b )] ).magnitude, 1e-6f, "normal of the kept vertex" );
    }

    // ---- many collapses ----

    // what no allowed collapse changes about a mesh
    private struct Shape {

        public int   components;
        public int   eulerCharacteristic;
        public int[] subMeshComponents;
        public int[] subMeshEulerCharacteristics;
        public int   verticesOfOrderTwo;
        public int   endsOfSharpLines;     // vertices with one sharp edge
        public int   meetingsOfSharpLines; // vertices with three or more
        public int   openLines;            // pieces that the open edges fall into, going from edge to edge at vertices
        public int   nonManifoldLines;
        public int   materialLines;
        public int   lockedVertices;

        public override string ToString( ) {
            return
                $"{this.components} components, euler characteristic {this.eulerCharacteristic}, " +
                $"components of the sub-meshes {string.Join( " ", this.subMeshComponents )}, their euler characteristics {string.Join( " ", this.subMeshEulerCharacteristics )}, " +
                $"{this.verticesOfOrderTwo} vertices of order two, {this.endsOfSharpLines} ends and {this.meetingsOfSharpLines} meetings of sharp lines, " +
                $"{this.openLines} open lines, {this.nonManifoldLines} lines of edges of more than two triangles, {this.materialLines} lines between materials, {this.lockedVertices} locked vertices";
        }

    }

    // the pieces that the edges of one kind of sharpness fall into
    private static int linesOf( Run run, Sharp kind ) {
        var parents = Enumerable.Range( 0, run.mesh.vertexCount ).ToArray( );

        int find( int vertex ) {
            while( parents[vertex] != vertex ) {
                vertex = parents[vertex] = parents[parents[vertex]];
            }
            return vertex;
        }

        var edges = run.edges( ).Where( ( edge ) => ( edge.Value.sharp & kind ) != 0 ).Select( ( edge ) => edge.Key ).ToArray( );

        foreach( var ( a, b ) in edges ) {
            parents[find( a )] = find( b );
        }

        return edges.Select( ( edge ) => find( edge.Item1 ) ).Distinct( ).Count( );
    }

    // the shape of the mesh a run wrote, from a connectivity built from nothing and from MeshTopology
    private static Shape shapeOf( Run run ) {
        using var fresh = rebuilt( run );

        var subMeshCount = run.writtenTriangles.Length == 0 ? 0 : run.writtenTriangles.Max( ( triangle ) => triangle.subMeshIndex ) + 1;

        var mesh = SurfaceExtractor.createMesh(
            run.writtenPositions,
            run.writtenNormals,
            run.writtenTriangles.Select( ( triangle ) => ( triangle.subMeshIndex, new[] { triangle.a, triangle.b, triangle.c } ) )
        );

        try {
            var whole     = MeshTopology.of( mesh );
            var subMeshes = Enumerable.Range( 0, subMeshCount ).Select( ( subMesh ) => MeshTopology.of( mesh, subMesh ) ).ToArray( );

            Assert.AreEqual( 0, whole.degenerateTriangles, "triangles that use a vertex twice" );

            var vertices = Enumerable.Range( 0, fresh.mesh.vertexCount );

            return new Shape {
                components                  = whole.components,
                eulerCharacteristic         = whole.eulerCharacteristic,
                subMeshComponents           = subMeshes.Select( ( subMesh ) => subMesh.components ).ToArray( ),
                subMeshEulerCharacteristics = subMeshes.Select( ( subMesh ) => subMesh.eulerCharacteristic ).ToArray( ),
                verticesOfOrderTwo          = vertices.Count( ( vertex ) => fresh.mesh.orders[vertex] == 2 ),
                endsOfSharpLines            = vertices.Count( ( vertex ) => fresh.mesh.sharpCounts[vertex] == 1 ),
                meetingsOfSharpLines        = vertices.Count( ( vertex ) => fresh.mesh.sharpCounts[vertex] >= 3 ),
                openLines                   = linesOf( fresh, Sharp.Open ),
                nonManifoldLines            = linesOf( fresh, Sharp.NonManifold ),
                materialLines               = linesOf( fresh, Sharp.Materials ),
                lockedVertices              = vertices.Count( ( vertex ) => fresh.mesh.locked[vertex] )
            };
        }
        finally {
            Object.DestroyImmediate( mesh );
        }
    }

    // what no allowed collapse changes about a mesh, as text
    internal static string shapeOf( Mesh mesh ) {
        using var run = new Run( mesh.vertices, mesh.normals, trianglesOf( mesh ) );

        return shapeOf( run ).ToString( );
    }

    // collapses every edge that may be collapsed into its midpoint, going through the edges in their order a few
    // times, and holds the mesh that is left to the shape of the one it started as
    private static void assertCollapsesKeepTheShape( string what, Vector3[] positions, Vector3[] normals, BurstMesh.Triangle[] triangles, double leastCollapsed ) {
        using var before = new Run( positions, normals, triangles );
        using var after  = new Run( positions, normals, triangles, passes: 4 );

        var refusals = string.Join( ", ", Enum.GetValues( typeof( Refusal ) ).Cast<Refusal>( ).Where( ( refusal ) => after.refused[( int )refusal] > 0 ).Select( ( refusal ) => $"{refusal} {after.refused[( int )refusal]}" ) );
        var summary  = $"{what}: {before.verticesWithTriangles( ).Length} vertices, {after.collapsed} collapses, refused: {refusals}";

        Debug.Log( summary );

        Assert.AreEqual( before.verticesWithTriangles( ).Length - after.collapsed, after.verticesWithTriangles( ).Length, $"{summary}: every collapse removes one vertex" );

        Assert.GreaterOrEqual( after.collapsed, leastCollapsed * before.verticesWithTriangles( ).Length, $"{summary}: collapses" );

        assertMatchesRebuilt( after, what );

        Assert.AreEqual( shapeOf( before ).ToString( ), shapeOf( after ).ToString( ), $"{summary}: shape" );
    }

    private static void assertCollapsesKeepTheShape( string what, Builder builder, double leastCollapsed ) {
        assertCollapsesKeepTheShape( what, builder.positions.ToArray( ), builder.normals( ), builder.triangles.ToArray( ), leastCollapsed );
    }

    [Test]
    public void collapsesKeepTheShapeOfMeshesBuiltByHand( ) {
        assertCollapsesKeepTheShape( "a box", new Builder( ).box( cells( 0, 0, 0 ), cells( 6, 5, 4 ) ), 0.5 );

        assertCollapsesKeepTheShape( "a box of two materials", new Builder( ).box( cells( 0, 0, 0 ), cells( 6, 5, 4 ), ( center, axis, side ) => center.z < 2 ? 1 : 0 ), 0.5 );

        assertCollapsesKeepTheShape( "a box with a band and a patch of other materials", new Builder( ).box(
            cells( 0, 0, 0 ), cells( 6, 6, 6 ), ( center, axis, side ) => center.x > 2 && center.x < 4 ? 1 : axis == 2 && side == 1 && center.x < 2 && center.y < 2 ? 2 : 0
        ), 0.4 );

        assertCollapsesKeepTheShape( "four materials that meet in threes", boxWithTop( ( i, j ) => i < 2 ? ( j < 3 ? 1 : 3 ) : ( j < 2 ? 0 : 2 ) ), 0.3 );

        assertCollapsesKeepTheShape( "two boxes that share an edge", new Builder( ).box( cells( 0, 0, 0 ), cells( 3, 3, 6 ) ).box( cells( 3, 3, 0 ), cells( 3, 3, 6 ) ), 0.5 );

        assertCollapsesKeepTheShape( "two boxes that share an edge, with a line between materials into it", new Builder( ).box(
            cells( 0, 0, 0 ), cells( 3, 3, 6 ), ( center, axis, side ) => center.z < 3 ? 1 : 0
        ).box( cells( 3, 3, 0 ), cells( 3, 3, 6 ) ), 0.4 );

        assertCollapsesKeepTheShape( "two boxes that touch in a vertex", new Builder( ).box( cells( 0, 0, 0 ), cells( 3, 3, 3 ) ).box( cells( 3, 3, 3 ), cells( 3, 3, 3 ) ), 0.5 );

        assertCollapsesKeepTheShape( "a patch", new Builder( ).patch( 8, 8 ), 0.5 );

        assertCollapsesKeepTheShape( "a patch of two materials", new Builder( ).patch( 8, 8, ( i, j ) => i < 3 ? 1 : 0 ), 0.5 );

        assertCollapsesKeepTheShape( "a strip", new Builder( ).patch( 8, 1 ), 0.3 );

        // a box with a fin, two triangles back to back on a corner, and a second box that touches the first in a vertex
        var odd = new Builder( ).box( cells( 0, 0, 0 ), cells( 4, 4, 4 ) ).box( cells( 4, 4, 4 ), cells( 2, 2, 2 ) );

        odd.triangle( new Vector3( 1, 1, 4 ), new Vector3( 2, 1, 4 ), new Vector3( 1.5f, 1.0f, 5.0f ) );
        odd.triangle( new Vector3( 0, 0, 0 ), new Vector3( -1, 0, -1 ), new Vector3( 0, -1, -1 ) );
        odd.triangle( new Vector3( 0, -1, -1 ), new Vector3( -1, 0, -1 ), new Vector3( 0, 0, 0 ) );

        assertCollapsesKeepTheShape( "a box with a fin and two triangles back to back", odd, 0.4 );
    }

    // surfaces that leave the grid, which spans -0.5 to 0.5: a ball through one side, and a plate with a band of a second
    // material that leaves with it, so that the lines between the materials run into the open edges
    private static readonly ContouringTest.Shape[] ballAcrossBounds = { sphere( new Vector3( 0.3f, 0.07f, -0.04f ), 0.35f ) };

    private static readonly ContouringTest.Shape[] plateAcrossBounds = {
        cuboid( Vector3.zero,                     new Vector3( 0.7f,  0.1f, 0.7f ) ),
        cuboid( new Vector3( 0.02f, 0.0f, 0.0f ), new Vector3( 0.15f, 0.1f, 0.7f ), Combination.Union, MaterialIndex.Material1 )
    };

    public enum Contour {
        Uniform,
        Adaptive,
        Manifold,
        ManifoldClustered,
        Occupancy
    }

    public enum Surface {
        ThreeBalls,
        SphereCutaway,
        TurnedShapes,
        NewShapes,
        BallAcrossBounds,
        PlateAcrossBounds
    }

    private Mesh contourOf( Contour contour, Surface surface, int cellsExponent ) {
        var shapes = surface switch {
            Surface.ThreeBalls        => shapesOf( Model.ThreeBalls ),
            Surface.SphereCutaway     => shapesOf( Model.SphereCutaway ),
            Surface.TurnedShapes      => turnedShapes,
            Surface.NewShapes         => newShapes,
            Surface.BallAcrossBounds  => ballAcrossBounds,
            Surface.PlateAcrossBounds => plateAcrossBounds,
            _                         => throw new Exception( "Unknown surface specified" )
        };

        this.implementation = ImplementationType.Burst;

        Voxelizer voxelizer;

        switch( contour ) {
            case Contour.Uniform: {
                voxelizer = this.createUniformVoxelizer( cellsExponent, ImplementationType.Burst, QEFSolverType.SVD, shapes );
                break;
            }
            case Contour.Adaptive: {
                var adaptive = this.createVoxelizer<AdaptiveDualContouring>( cellsExponent + 1, QEFSolverType.SVD, shapes );
                adaptive.simplification = true;
                adaptive.errorThreshold = 2e-4f;
                adaptive.topologySafe   = true;
                voxelizer = adaptive;
                break;
            }
            case Contour.Manifold:
            case Contour.ManifoldClustered: {
                var manifold = this.createVoxelizer<ManifoldDualContouring>( cellsExponent + 1, QEFSolverType.SVD, shapes );
                manifold.errorThreshold  = contour == Contour.ManifoldClustered ? 2e-4f : 0.0f;
                manifold.enforceManifold = true;
                voxelizer = manifold;
                break;
            }
            default: {
                voxelizer = this.createVoxelizer<OccupancyDualContouring>( cellsExponent, QEFSolverType.SVD, shapes );
                break;
            }
        }

        voxelizer.vertexNormals        = VertexNormals.Shared;
        voxelizer.keepDebugInformation = false;

        return this.voxelize( voxelizer ).mesh;
    }

    internal static BurstMesh.Triangle[] trianglesOf( Mesh mesh ) {
        var triangles = new List<BurstMesh.Triangle>( );

        for( var subMesh = 0; subMesh < mesh.subMeshCount; ++subMesh ) {
            var indices = mesh.GetTriangles( subMesh );

            for( var index = 0; index < indices.Length; index += 3 ) {
                triangles.Add( new BurstMesh.Triangle( indices[index], indices[index + 1], indices[index + 2], subMesh ) );
            }
        }
        return triangles.ToArray( );
    }

    // the edges of a contour as the connectivity has them, against MeshTopology
    [Test]
    public void connectivityOfContoursIsWhatMeshTopologyCounts( [Values] Contour contour, [Values] Surface surface ) {
        var mesh = this.contourOf( contour, surface, 5 );

        using var run = new Run( mesh.vertices, mesh.normals, trianglesOf( mesh ) );

        var topology = MeshTopology.of( mesh );
        var edges    = run.edges( );

        Assert.AreEqual( topology.triangles,        run.triangleCount,                                                           "triangles" );
        Assert.AreEqual( topology.vertices,         run.verticesWithTriangles( ).Length,                                         "vertices that triangles use" );
        Assert.AreEqual( topology.edges,            edges.Count,                                                                 "edges" );
        Assert.AreEqual( topology.openEdges,        edges.Values.Count( ( edge ) => edge.triangles == 1 ),                       "open edges" );
        Assert.AreEqual( topology.nonManifoldEdges, edges.Values.Count( ( edge ) => edge.triangles > 2 ),                        "edges of more than two triangles" );
        Assert.AreEqual( topology.openEdges,        edges.Values.Count( ( edge ) => ( edge.sharp & Sharp.Open ) != 0 ),          "edges marked open" );
        Assert.AreEqual( topology.nonManifoldEdges, edges.Values.Count( ( edge ) => ( edge.sharp & Sharp.NonManifold ) != 0 ),   "edges marked as of more than two triangles" );

        // a closed manifold has no first boundary
        if( topology.openEdges == 0 && topology.nonManifoldEdges == 0 && topology.pinchedVertices == 0 ) {
            Assert.IsTrue( Enumerable.Range( 0, run.mesh.vertexCount ).All( ( vertex ) => run.mesh.orders[vertex] == 0 ), "every vertex is of order 0" );
        }
    }

    [Test]
    public void collapsesKeepTheShapeOfContours( [Values] Contour contour, [Values] Surface surface ) {
        var mesh = this.contourOf( contour, surface, 5 );

        assertCollapsesKeepTheShape( $"{contour} {surface}", mesh.vertices, mesh.normals, trianglesOf( mesh ), 0.2 );
    }


    // ---- the loop ----

    // counts the edges that are left to collapse: within the error bound, and allowed
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    private struct LeftToCollapseJob : IJob {

        public double       limit;
        public Connectivity mesh;

        public NativeArray<int> left;

        public void Execute( ) {
            for( var edge = 0; edge < this.mesh.edgeCount; ++edge ) {
                var ends = this.mesh.edges[edge];

                if( ends.x == -1 ) {
                    continue;
                }

                BurstMeshSimplifier.evaluate(
                    this.mesh.planes[ends.x] + this.mesh.planes[ends.y], this.mesh.areas[ends.x] + this.mesh.areas[ends.y],
                    this.mesh.sharpPlanes[ends.x] + this.mesh.sharpPlanes[ends.y], this.mesh.sharpAreas[ends.x] + this.mesh.sharpAreas[ends.y],
                    this.mesh.lines[ends.x] + this.mesh.lines[ends.y], this.mesh.positions[ends.x], this.mesh.positions[ends.y], this.limit, out var target, out _, out var within
                );

                if( within && this.mesh.check( edge, target ) == Refusal.None ) {
                    ++this.left[0];
                }
            }
        }

    }

    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    private struct WriteJob : IJob {

        public Connectivity mesh;

        public NativeList<float3>             positions;
        public NativeList<float3>             normals;
        public NativeList<BurstMesh.Triangle> triangles;

        public void Execute( ) {
            this.mesh.write( this.positions, this.normals, this.triangles );
        }

    }

    // the jobs of the simplifier run one after the other, as BurstMeshSimplifier.record runs them, with the loop
    // stopped after a number of collapses. The connectivity is left as the loop left it
    private sealed class Loop : IDisposable {

        public NativeArray<float3>             positions;
        public NativeArray<float3>             normals;
        public NativeArray<BurstMesh.Triangle> triangles;

        public Connectivity connectivity;

        public NativeList<BurstMeshSimplifier.Collapse> record;

        public readonly BurstMeshSimplifier.Statistics statistics;

        public Loop( Vector3[] positions, Vector3[] normals, BurstMesh.Triangle[] triangles, double error, double lineWeight, int maximumCollapses = int.MaxValue ) {
            this.positions    = new NativeArray<float3>( positions.Select( ( position ) => ( float3 )position ).ToArray( ), Allocator.TempJob );
            this.normals      = new NativeArray<float3>( normals.Select( ( normal ) => ( float3 )normal ).ToArray( ), Allocator.TempJob );
            this.triangles    = new NativeArray<BurstMesh.Triangle>( triangles, Allocator.TempJob );
            this.connectivity = new Connectivity( positions.Length, triangles.Length, Allocator.TempJob );
            this.record       = new NativeList<BurstMeshSimplifier.Collapse>( Allocator.TempJob );

            new BurstMeshSimplifier.BuildJob { positions = this.positions, normals = this.normals, triangles = this.triangles, mesh = this.connectivity }.Run( );

            this.connectivity.quadricsJob( lineWeight ).Schedule( this.connectivity.vertexCount, 16 ).Complete( );

            using var targets    = new NativeArray<double3>( this.connectivity.edgeCount, Allocator.TempJob );
            using var costs      = new NativeArray<double>( this.connectivity.edgeCount, Allocator.TempJob );
            using var within     = new NativeArray<bool>( this.connectivity.edgeCount, Allocator.TempJob );
            using var heap       = new NativeArray<int>( this.connectivity.edgeCount, Allocator.TempJob );
            using var places     = new NativeArray<int>( this.connectivity.edgeCount, Allocator.TempJob );
            using var statistics = new NativeArray<BurstMeshSimplifier.Statistics>( 1, Allocator.TempJob );
            using var refusals   = new NativeArray<int>( Enum.GetValues( typeof( Refusal ) ).Length, Allocator.TempJob );

            new BurstMeshSimplifier.CostsJob {
                limit       = error * error,
                positions   = this.connectivity.positions,
                planes      = this.connectivity.planes,
                areas       = this.connectivity.areas,
                sharpPlanes = this.connectivity.sharpPlanes,
                sharpAreas  = this.connectivity.sharpAreas,
                lines       = this.connectivity.lines,
                edges       = this.connectivity.edges.AsArray( ),
                targets     = targets,
                costs       = costs,
                within      = within
            }.Schedule( this.connectivity.edgeCount, 16 ).Complete( );

            new BurstMeshSimplifier.CollapseJob {
                limit            = error * error,
                maximumCollapses = maximumCollapses,
                mesh             = this.connectivity,
                targets          = targets,
                costs            = costs,
                within           = within,
                heap             = heap,
                places           = places,
                record           = this.record,
                statistics       = statistics,
                refusals         = refusals
            }.Run( );

            this.statistics = statistics[0];
        }

        public void Dispose( ) {
            this.positions.Dispose( );
            this.normals.Dispose( );
            this.triangles.Dispose( );
            this.connectivity.Dispose( );
            this.record.Dispose( );
        }

        // how many edges the loop left that it could have collapsed
        public int left( double error ) {
            using var left = new NativeArray<int>( 1, Allocator.TempJob );

            new LeftToCollapseJob { limit = error * error, mesh = this.connectivity, left = left }.Run( );

            return left[0];
        }

        // the mesh the connectivity holds
        public ( float3[] positions, float3[] normals, BurstMesh.Triangle[] triangles ) written( ) {
            using var positions = new NativeList<float3>( Allocator.TempJob );
            using var normals   = new NativeList<float3>( Allocator.TempJob );
            using var triangles = new NativeList<BurstMesh.Triangle>( Allocator.TempJob );

            new WriteJob { mesh = this.connectivity, positions = positions, normals = normals, triangles = triangles }.Run( );

            return ( positions.AsArray( ).ToArray( ), normals.AsArray( ).ToArray( ), triangles.AsArray( ).ToArray( ) );
        }

        // the mesh the record gives after its first so many collapses, from the mesh as it came
        public ( float3[] positions, float3[] normals, BurstMesh.Triangle[] triangles ) level( int level ) {
            using var positions = new NativeList<float3>( Allocator.TempJob );
            using var normals   = new NativeList<float3>( Allocator.TempJob );
            using var triangles = new NativeList<BurstMesh.Triangle>( Allocator.TempJob );

            new BurstMeshSimplifier.LevelJob {
                positions      = this.positions,
                normals        = this.normals,
                triangles      = this.triangles,
                collapses      = this.record.AsArray( ),
                level          = level,
                levelPositions = positions,
                levelNormals   = normals,
                levelTriangles = triangles
            }.Run( );

            return ( positions.AsArray( ).ToArray( ), normals.AsArray( ).ToArray( ), triangles.AsArray( ).ToArray( ) );
        }

    }

    // runs the simplifier and counts what the loop left that it could have collapsed
    private static ( int collapses, int left ) simplifyAndCountWhatIsLeft( Vector3[] positions, Vector3[] normals, BurstMesh.Triangle[] triangles, double error, double lineWeight ) {
        using var loop = new Loop( positions, normals, triangles, error, lineWeight );

        return ( loop.statistics.collapses, loop.left( error ) );
    }

    private static ( int, int, int, int )[] cornersOf( BurstMesh.Triangle[] triangles ) {
        return triangles.Select( ( triangle ) => ( triangle.a, triangle.b, triangle.c, triangle.subMeshIndex ) ).ToArray( );
    }

    // Hoppe, section 3: the record of the collapses stands for every mesh between the one that came and the one the
    // loop ends with. The mesh it gives after its first k collapses, from the mesh that came and the ancestor of
    // every vertex (section 3.2), is the mesh the connectivity holds when the loop is stopped after k: the same
    // vertices in the same order, at the same positions with the same normals, and the same triangles. And the loop
    // that is stopped after k has made the first k collapses of the loop that ran to its end
    [Test]
    public void recordGivesTheMeshTheLoopHoldsAfterAnyNumberOfCollapses( [Values( 0.0, 1e-5 )] double lineWeight ) {
        void assertLevels( string what, Vector3[] positions, Vector3[] normals, BurstMesh.Triangle[] triangles, double error ) {
            using var whole = new Loop( positions, normals, triangles, error, lineWeight );

            var count = whole.record.Length;

            Assert.Greater( count, 4, $"{what}: collapses" );

            foreach( var level in new[] { 0, 1, 2, count / 3, count / 2, count - 1, count }.Distinct( ) ) {
                using var stopped = new Loop( positions, normals, triangles, error, lineWeight, level );

                Assert.AreEqual( level, stopped.record.Length, $"{what}: collapses of the loop stopped after {level}" );

                for( var index = 0; index < level; ++index ) {
                    var expected = whole.record[index];
                    var actual   = stopped.record[index];

                    Assert.AreEqual( ( expected.kept, expected.removed, expected.position, expected.normal ), ( actual.kept, actual.removed, actual.position, actual.normal ), $"{what}: collapse {index} of the loop stopped after {level}" );
                }

                var held  = stopped.written( );
                var given = whole.level( level );

                CollectionAssert.AreEqual( held.positions,              given.positions,              $"{what}: positions after {level} of {count} collapses" );
                CollectionAssert.AreEqual( held.normals,                given.normals,                $"{what}: normals after {level} of {count} collapses" );
                CollectionAssert.AreEqual( cornersOf( held.triangles ), cornersOf( given.triangles ), $"{what}: triangles after {level} of {count} collapses" );
            }
        }

        // boxes of two materials, alone and sharing an edge of four triangles, and a box with two triangles back to
        // back on a corner, a triangle that uses a vertex twice and a vertex that no triangle uses
        var box = new Builder( ).box( cells( 0, 0, 0 ), cells( 6, 5, 4 ), ( center, axis, side ) => center.z < 2 ? 1 : 0 );
        var two = new Builder( ).box( cells( 0, 0, 0 ), cells( 3, 3, 6 ), ( center, axis, side ) => center.z < 3 ? 1 : 0 ).box( cells( 3, 3, 0 ), cells( 3, 3, 6 ) );
        var odd = new Builder( ).box( cells( 0, 0, 0 ), cells( 4, 4, 4 ) );

        odd.triangle( new Vector3( 4, 4, 4 ), new Vector3( 5, 4, 5 ), new Vector3( 4, 5, 5 ) );
        odd.triangle( new Vector3( 4, 5, 5 ), new Vector3( 5, 4, 5 ), new Vector3( 4, 4, 4 ) );
        odd.triangles.Add( new BurstMesh.Triangle( 0, 0, 1, 0 ) );
        odd.vertex( new Vector3( 9, 9, 9 ) );

        assertLevels( "a box of two materials",                box.positions.ToArray( ), box.normals( ), box.triangles.ToArray( ), 0.1 );
        assertLevels( "two boxes that share an edge",          two.positions.ToArray( ), two.normals( ), two.triangles.ToArray( ), 0.1 );
        assertLevels( "a box with triangles that are not its", odd.positions.ToArray( ), odd.normals( ), odd.triangles.ToArray( ), 0.1 );

        foreach( var ( contour, surface ) in new[] { ( Contour.Uniform, Surface.TurnedShapes ), ( Contour.Occupancy, Surface.NewShapes ), ( Contour.ManifoldClustered, Surface.TurnedShapes ), ( Contour.Manifold, Surface.PlateAcrossBounds ) } ) {
            var mesh = this.contourOf( contour, surface, 5 );

            assertLevels( $"{contour} {surface}", mesh.vertices, mesh.normals, trianglesOf( mesh ), 0.1 / 32.0 );
        }
    }

    // the costs the loop starts from are those of every edge under the three quadrics of its two ends: the job that
    // works them out on all cores gives what evaluate gives for each edge here. The contour is of a sphere with a flat
    // cut, so some of its edges are within the error and some are not, and without lines some have no best point
    [Test]
    public void firstCostsAreThoseOfEveryEdgeUnderItsThreeQuadrics( [Values( 0.0, 1e-3 )] double lineWeight ) {
        var mesh  = this.contourOf( Contour.Uniform, Surface.SphereCutaway, 5 );
        var limit = ( 0.02 / 32.0 ) * ( 0.02 / 32.0 );

        using var positions    = new NativeArray<float3>( mesh.vertices.Select( ( position ) => ( float3 )position ).ToArray( ), Allocator.TempJob );
        using var normals      = new NativeArray<float3>( mesh.normals.Select( ( normal ) => ( float3 )normal ).ToArray( ), Allocator.TempJob );
        using var triangles    = new NativeArray<BurstMesh.Triangle>( trianglesOf( mesh ), Allocator.TempJob );
        using var connectivity = new Connectivity( positions.Length, triangles.Length, Allocator.TempJob );

        new BurstMeshSimplifier.BuildJob { positions = positions, normals = normals, triangles = triangles, mesh = connectivity }.Run( );

        connectivity.quadricsJob( lineWeight ).Schedule( connectivity.vertexCount, 16 ).Complete( );

        using var targets = new NativeArray<double3>( connectivity.edgeCount, Allocator.TempJob );
        using var costs   = new NativeArray<double>( connectivity.edgeCount, Allocator.TempJob );
        using var within  = new NativeArray<bool>( connectivity.edgeCount, Allocator.TempJob );

        new BurstMeshSimplifier.CostsJob {
            limit       = limit,
            positions   = connectivity.positions,
            planes      = connectivity.planes,
            areas       = connectivity.areas,
            sharpPlanes = connectivity.sharpPlanes,
            sharpAreas  = connectivity.sharpAreas,
            lines       = connectivity.lines,
            edges       = connectivity.edges.AsArray( ),
            targets     = targets,
            costs       = costs,
            within      = within
        }.Schedule( connectivity.edgeCount, 16 ).Complete( );

        var inside  = 0;
        var solved  = 0;
        var costing = 0;

        for( var edge = 0; edge < connectivity.edgeCount; ++edge ) {
            var a = connectivity.edges[edge].x;
            var b = connectivity.edges[edge].y;

            var best = BurstMeshSimplifier.evaluate(
                connectivity.planes[a] + connectivity.planes[b], connectivity.areas[a] + connectivity.areas[b], connectivity.sharpPlanes[a] + connectivity.sharpPlanes[b], connectivity.sharpAreas[a] + connectivity.sharpAreas[b],
                connectivity.lines[a] + connectivity.lines[b], connectivity.positions[a], connectivity.positions[b], limit, out var target, out var cost, out var isWithin
            );

            Assert.LessOrEqual( math.length( target - targets[edge] ), 1e-9, $"where edge {edge} goes" );
            Assert.AreEqual( cost, costs[edge], ( 1e-9 * cost ) + 1e-18, $"cost of edge {edge}" );
            Assert.AreEqual( isWithin, within[edge], $"edge {edge} is within the error" );

            inside  += isWithin ? 1 : 0;
            solved  += best ? 1 : 0;
            costing += cost > 0.0 ? 1 : 0;
        }

        Assert.Greater( inside, connectivity.edgeCount / 10,                         "edges within the error" );
        Assert.Greater( connectivity.edgeCount - inside, connectivity.edgeCount / 10, "edges over the error" );

        if( lineWeight > 0.0 ) {
            Assert.AreEqual( connectivity.edgeCount, solved,  "edges with a best point, with lines" );
            Assert.AreEqual( connectivity.edgeCount, costing, "edges that cost something, with lines" );
        }
        else {
            // the edges of the flat cut have none
            Assert.Less( solved, connectivity.edgeCount - 100, "edges with a best point, without lines" );
        }
    }

    // the loop ends when its heap is empty. An edge that was refused is not in the heap, and has to come back when a
    // collapse next to it changes what refused it, or it would be left behind while it could collapse
    [Test]
    public void loopLeavesNoEdgeThatCouldCollapse( [Values( 0.02, 0.1, 0.5 )] double error, [Values( 0.0, 1e-5, 1e-2 )] double lineWeight ) {
        // the errors are in cells of a grid of 32^3, which is what the contours are of. The boxes are 4 to 6 across
        var cell = 1.0 / 32.0;

        void assertNothingLeft( string what, Vector3[] positions, Vector3[] normals, BurstMesh.Triangle[] triangles, double unit ) {
            var ( collapses, left ) = simplifyAndCountWhatIsLeft( positions, normals, triangles, error * unit, lineWeight );

            Assert.Greater( collapses, 0, $"{what}: collapses" );
            Assert.AreEqual( 0, left, $"{what}: edges left that could collapse, after {collapses} collapses" );
        }

        var box = new Builder( ).box( cells( 0, 0, 0 ), cells( 6, 5, 4 ), ( center, axis, side ) => center.z < 2 ? 1 : 0 );
        var two = new Builder( ).box( cells( 0, 0, 0 ), cells( 3, 3, 6 ), ( center, axis, side ) => center.z < 3 ? 1 : 0 ).box( cells( 3, 3, 0 ), cells( 3, 3, 6 ) );

        assertNothingLeft( "a box of two materials", box.positions.ToArray( ), box.normals( ), box.triangles.ToArray( ), 1.0 );
        assertNothingLeft( "two boxes that share an edge", two.positions.ToArray( ), two.normals( ), two.triangles.ToArray( ), 1.0 );

        foreach( var ( contour, surface ) in new[] { ( Contour.Uniform, Surface.ThreeBalls ), ( Contour.Uniform, Surface.TurnedShapes ), ( Contour.Occupancy, Surface.NewShapes ), ( Contour.Manifold, Surface.PlateAcrossBounds ) } ) {
            var mesh = this.contourOf( contour, surface, 5 );

            assertNothingLeft( $"{contour} {surface}", mesh.vertices, mesh.normals, trianglesOf( mesh ), cell );
        }
    }

}
