using System;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

// the remeshing pass, which rebuilds the mesh of a contour after mesh simplification and before its normals are creased
// or split (Documents/Plans/Remeshing.md), and what a voxelizer keeps of the mesh the pass was last given. From that mesh
// Voxelizer.remesh shows the contour again under other remeshing settings, without contouring or simplifying. It is all
// in managed arrays, as MeshSimplification is, so that a script reload leaves nothing behind
public class Remeshing {

    // how a mesh is remeshed. The techniques of the plan come with their steps; until the first of them, the frame is
    // tested with a stand-in, which step 5 takes out again
    public enum Technique {
        Off = 0,

        // pairs of triangles that lie flat together become quads, and nothing moves
        [InspectorName( "Flat Pairs (Stand-In)" )]
        FlatPairs = 99
    }

    // what a remesh did
    public readonly struct Statistics {

        public readonly Technique technique;
        public readonly double    milliseconds;

        // the mesh it was given
        public readonly int inputVertexCount;
        public readonly int inputTriangleCount;

        // the length of the edges it made, in the cells of the contour (decision 9)
        public readonly double edgeLength;

        // its faces by the number of their sides
        public readonly int triangles;
        public readonly int quads;
        public readonly int largerFaces;

        // edges of one triangle, and of more than two, that it has beyond those of the mesh it was given (finding 7)
        public readonly int newOpenEdges;
        public readonly int newNonManifoldEdges;

        // why it gave the mesh back as it came, or null when it did not
        public readonly string gaveUp;

        public Statistics(
            Technique technique, double milliseconds, int inputVertexCount, int inputTriangleCount, double edgeLength, int triangles, int quads, int largerFaces, int newOpenEdges,
            int newNonManifoldEdges, string gaveUp
        ) {
            this.technique           = technique;
            this.milliseconds        = milliseconds;
            this.inputVertexCount    = inputVertexCount;
            this.inputTriangleCount  = inputTriangleCount;
            this.edgeLength          = edgeLength;
            this.triangles           = triangles;
            this.quads               = quads;
            this.largerFaces         = largerFaces;
            this.newOpenEdges        = newOpenEdges;
            this.newNonManifoldEdges = newNonManifoldEdges;
            this.gaveUp              = gaveUp;
        }

    }

    // the mesh the pass was given: vertices its triangles share, their normals, and its triangles with the sub-mesh of
    // each, and the size of the cells of its contour
    public readonly float3[]             positions;
    public readonly float3[]             normals;
    public readonly BurstMesh.Triangle[] triangles;
    public readonly float                cellSize;

    // its area, the vertices its triangles use, and its open and non-manifold edges, worked out when they are first
    // wanted: keeping the mesh is all that a contour does while remeshing is off
    private bool                    measured;
    private double                  area;
    private int                     usedVertexCount;
    private BurstRemeshing.Topology topology;

    private Remeshing( float3[] positions, float3[] normals, BurstMesh.Triangle[] triangles, float cellSize ) {
        this.positions = positions;
        this.normals   = normals;
        this.triangles = triangles;
        this.cellSize  = cellSize;
    }

    // keeps a mesh of shared vertices, which is left as it is
    public static Remeshing of( Mesh mesh, float cellSize ) {
        BurstMeshSimplifier.read( mesh, Allocator.TempJob, out var positions, out var normals, out var triangles );

        try {
            return new Remeshing( positions.ToArray( ), normals.ToArray( ), triangles.ToArray( ), cellSize );
        }
        finally {
            positions.Dispose( );
            normals.Dispose( );
            triangles.Dispose( );
        }
    }

    // the faces a technique makes, which its edge length is worked out for
    public static BurstRemeshing.Faces facesOf( Technique technique ) {
        return technique == Technique.FlatPairs ? BurstRemeshing.Faces.Quads : BurstRemeshing.Faces.Triangles;
    }

    // the length of the edges a technique makes at an edge length, in the units of the mesh: the setting times the length
    // at which its faces would be as many vertices as the mesh has (decision 9)
    public double lengthOf( Technique technique, float edgeLength ) {
        using var positions = new NativeArray<float3>( this.positions, Allocator.TempJob );
        using var triangles = new NativeArray<BurstMesh.Triangle>( this.triangles, Allocator.TempJob );

        this.measure( positions, triangles );

        return edgeLength * BurstRemeshing.lengthOf( this.area, this.usedVertexCount, facesOf( technique ) );
    }

    private void measure( NativeArray<float3> positions, NativeArray<BurstMesh.Triangle> triangles ) {
        if( this.measured ) {
            return;
        }

        using var area        = new NativeArray<double>( 1, Allocator.TempJob );
        using var vertexCount = new NativeArray<int>( 1, Allocator.TempJob );
        using var none        = new NativeArray<byte>( 0, Allocator.TempJob );

        new BurstRemeshing.AreaJob { positions = positions, triangles = triangles, area = area, vertexCount = vertexCount }.Run( );

        BurstRemeshing.count( triangles, none, out this.topology, out _ );

        this.area            = area[0];
        this.usedVertexCount = vertexCount[0];
        this.measured        = true;
    }

    // the mesh as the pass was given it
    public Mesh mesh( ) {
        using var positions = new NativeArray<float3>( this.positions, Allocator.TempJob );
        using var normals   = new NativeArray<float3>( this.normals, Allocator.TempJob );
        using var triangles = new NativeArray<BurstMesh.Triangle>( this.triangles, Allocator.TempJob );

        return BurstMesh.createMesh( positions, normals, triangles );
    }

    // the mesh remeshed: a new mesh of shared vertices, and for each of its triangles in the order of its index buffer,
    // which of its edges are edges of its faces (BurstRemeshing.allEdges). A technique that cannot do its work gives the
    // mesh as it came, without flags, and says why in the statistics
    public Mesh remesh( Technique technique, float edgeLength, out byte[] flags, out Statistics statistics ) {
        var stopwatch = System.Diagnostics.Stopwatch.StartNew( );

        using var positions = new NativeArray<float3>( this.positions, Allocator.TempJob );
        using var normals   = new NativeArray<float3>( this.normals, Allocator.TempJob );
        using var triangles = new NativeArray<BurstMesh.Triangle>( this.triangles, Allocator.TempJob );

        this.measure( positions, triangles );

        var length = edgeLength * BurstRemeshing.lengthOf( this.area, this.usedVertexCount, facesOf( technique ) );

        Mesh                      mesh;
        string                    gaveUp = null;
        BurstRemeshing.Topology   topology;
        BurstRemeshing.FaceCounts faces;

        switch( technique ) {
            case Technique.FlatPairs: {
                using var sides     = new NativeArray<BurstRemeshing.Side>( triangles.Length * 3, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
                using var pairFlags = new NativeArray<byte>( triangles.Length, Allocator.TempJob );
                using var pairs     = new NativeArray<int>( 1, Allocator.TempJob );
                using var none      = new NativeArray<byte>( 0, Allocator.TempJob );

                new BurstRemeshing.SidesJob { triangles = triangles, sides = sides }.Run( );

                new BurstRemeshing.FlatPairsJob { positions = positions, triangles = triangles, sides = sides, flags = pairFlags, pairs = pairs }.Run( );

                if( pairs[0] == 0 ) {
                    gaveUp = "no two triangles lie flat together";
                }

                // the triangles stay as they are, so their edges do too
                using var counts  = new NativeArray<BurstRemeshing.Topology>( 1, Allocator.TempJob );
                using var counted = new NativeArray<BurstRemeshing.FaceCounts>( 1, Allocator.TempJob );

                new BurstRemeshing.CountsJob { sides = sides, flags = gaveUp == null ? pairFlags : none, topology = counts, faces = counted }.Run( );

                topology = counts[0];
                faces    = counted[0];
                mesh     = BurstMesh.createMesh( positions, normals, triangles );
                flags    = gaveUp == null ? inIndexOrder( triangles, pairFlags ) : null;

                break;
            }
            default: {
                throw new ArgumentException( $"{technique} is not a technique that remeshes", nameof( technique ) );
            }
        }

        statistics = new Statistics(
            technique,
            stopwatch.Elapsed.TotalMilliseconds,
            this.positions.Length,
            this.triangles.Length,
            length / this.cellSize,
            faces.triangles,
            faces.quads,
            faces.larger,
            Math.Max( 0, topology.openEdges - this.topology.openEdges ),
            Math.Max( 0, topology.nonManifoldEdges - this.topology.nonManifoldEdges ),
            gaveUp
        );

        return mesh;
    }

    // the flags of the triangles in the order of the index buffer a mesh of them has: by sub-mesh, and in the order of
    // the triangles within each, as BurstMesh.WriteMeshJob writes them
    public static byte[] inIndexOrder( NativeArray<BurstMesh.Triangle> triangles, NativeArray<byte> flags ) {
        var subMeshCount = 0;
        for( var triangle = 0; triangle < triangles.Length; ++triangle ) {
            subMeshCount = Math.Max( subMeshCount, triangles[triangle].subMeshIndex + 1 );
        }

        var next = new int[subMeshCount + 1];
        for( var triangle = 0; triangle < triangles.Length; ++triangle ) {
            ++next[triangles[triangle].subMeshIndex + 1];
        }
        for( var subMesh = 0; subMesh < subMeshCount; ++subMesh ) {
            next[subMesh + 1] += next[subMesh];
        }

        var result = new byte[triangles.Length];
        for( var triangle = 0; triangle < triangles.Length; ++triangle ) {
            result[next[triangles[triangle].subMeshIndex]++] = flags[triangle];
        }

        return result;
    }

}
