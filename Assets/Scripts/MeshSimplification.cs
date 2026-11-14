using System;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

// a progressive mesh (Hoppe, Progressive Meshes, section 3), in the form that suits a mesh that is all there from the
// start: the mesh of a contour as it came, and the collapses that simplify it, in the order BurstMeshSimplifier made
// them. The mesh after any number of the collapses is had from the two without simplifying again, which is what the
// detail of a voxelizer picks: every vertex of the contour is replaced by the vertex it has gone into by then
// (BurstMeshSimplifier.LevelJob).
//
// A voxelizer keeps this of its last contour. It is all in managed arrays, so that a script reload, which forgets
// the voxelizer's hold on it, leaves nothing behind
public class MeshSimplification {

    // the contour: its vertices, their normals, and its triangles with the sub-mesh of each
    public readonly float3[]             positions;
    public readonly float3[]             normals;
    public readonly BurstMesh.Triangle[] triangles;

    // the collapses, and the error and the line weight they were made under (BurstMeshSimplifier.simplify)
    public readonly BurstMeshSimplifier.Collapse[] collapses;
    public readonly double                         error;
    public readonly double                         lineWeight;

    public readonly BurstMeshSimplifier.Statistics statistics;

    private MeshSimplification(
        float3[] positions, float3[] normals, BurstMesh.Triangle[] triangles, BurstMeshSimplifier.Collapse[] collapses, double error, double lineWeight, BurstMeshSimplifier.Statistics statistics
    ) {
        this.positions  = positions;
        this.normals    = normals;
        this.triangles  = triangles;
        this.collapses  = collapses;
        this.error      = error;
        this.lineWeight = lineWeight;
        this.statistics = statistics;
    }

    // simplifies the mesh of a contour, which is left as it is
    public static MeshSimplification of( Mesh contour, double error, double lineWeight ) {
        BurstMeshSimplifier.read( contour, Allocator.TempJob, out var positions, out var normals, out var triangles );

        try {
            return of( positions, normals, triangles, null, error, lineWeight );
        }
        finally {
            positions.Dispose( );
            normals.Dispose( );
            triangles.Dispose( );
        }
    }

    // the same contour, simplified under another error or line weight
    public MeshSimplification under( double error, double lineWeight ) {
        using var positions = new NativeArray<float3>( this.positions, Allocator.TempJob );
        using var normals   = new NativeArray<float3>( this.normals, Allocator.TempJob );
        using var triangles = new NativeArray<BurstMesh.Triangle>( this.triangles, Allocator.TempJob );

        return of( positions, normals, triangles, this, error, lineWeight );
    }

    // the arrays of the contour are those of the simplification it is taken from, where there is one
    private static MeshSimplification of(
        NativeArray<float3> positions, NativeArray<float3> normals, NativeArray<BurstMesh.Triangle> triangles, MeshSimplification from, double error, double lineWeight
    ) {
        using var collapses = new NativeList<BurstMeshSimplifier.Collapse>( positions.Length, Allocator.TempJob );

        BurstMeshSimplifier.record( positions, normals, triangles, error, lineWeight, collapses, out var statistics );

        return new MeshSimplification(
            from?.positions ?? positions.ToArray( ), from?.normals ?? normals.ToArray( ), from?.triangles ?? triangles.ToArray( ), collapses.AsArray( ).ToArray( ), error, lineWeight, statistics
        );
    }

    // how many collapses a detail stands for: all of them at 0, which gives the simplest mesh the error allowed, and
    // none at 1, which gives the contour
    public int levelOf( float detail ) {
        return ( int )Math.Round( ( 1.0 - Mathf.Clamp01( detail ) ) * this.collapses.Length );
    }

    // the mesh after the first so many collapses: a new mesh with the sub-meshes of the contour, and vertices with
    // normals that its triangles share
    public Mesh meshAt( int level ) {
        using var positions = new NativeArray<float3>( this.positions, Allocator.TempJob );
        using var normals   = new NativeArray<float3>( this.normals, Allocator.TempJob );
        using var triangles = new NativeArray<BurstMesh.Triangle>( this.triangles, Allocator.TempJob );
        using var collapses = new NativeArray<BurstMeshSimplifier.Collapse>( this.collapses, Allocator.TempJob );

        return BurstMeshSimplifier.meshAt( positions, normals, triangles, collapses, level );
    }

}
