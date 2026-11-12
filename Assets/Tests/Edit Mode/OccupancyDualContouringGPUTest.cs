using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using UnityEngine;

using ImplementationType = SurfaceExtractor.Implementation.Type;
using VertexNormals      = SurfaceExtractor.VertexNormals;
using QEFSolverType      = QEFSolver.Type;

// the compute shader of Occupancy-Based Dual Contouring against its Burst implementation. Its kernels do what the jobs
// do, so both should find the same points on the edges, put the vertices of a cell in the same places and split every
// polygon the same way. Distances are in cells of the grid of 16^3
public class OccupancyDualContouringGPUTest : TopologyTest {

    private const int resolution = 4;

    // the vertices of a cell in the order both implementations list them, by where the cell is in the grid
    private static Dictionary<Vector3Int, SurfaceExtractor.Voxel[]> byCell( IEnumerable<SurfaceExtractor.Voxel> voxels ) {
        return voxels.GroupBy(
            ( voxel ) => Vector3Int.FloorToInt( ( voxel.center + ( Vector3.one * 0.5f ) ) * ( 1 << resolution ) )
        ).ToDictionary( ( cell ) => cell.Key, ( cell ) => cell.ToArray( ) );
    }

    private void assertMatchesBurst( Shape[] shapes, bool keepVerticesInCells = true ) {
        var voxelizer = this.createVoxelizer<OccupancyDualContouring>( resolution, QEFSolverType.SVD, shapes );

        voxelizer.keepVerticesInCells = keepVerticesInCells;
        voxelizer.implementationType  = ImplementationType.Burst;

        var burst = this.voxelize( voxelizer );

        voxelizer.implementationType = ImplementationType.GPU;

        var gpu = this.voxelize( voxelizer );

        // the same cells have the same number of vertices
        var expected = byCell( burst.voxels );
        var actual   = byCell( gpu.voxels );

        Assert.Greater( expected.Count, 0, "no cell has a vertex" );

        CollectionAssert.AreEquivalent( expected.Keys, actual.Keys, "cells with vertices" );

        foreach( var cell in expected.Keys ) {
            Assert.AreEqual( expected[cell].Length, actual[cell].Length, $"vertices of the cell at {cell}" );
        }

        // the 1D points: a search on the GPU can come out a step from the one in Burst where a sample falls within
        // rounding of the surface
        var expectedEdges = burst.edges.Where( ( edge ) => edge.intersectsContour( ) ).ToDictionary( ( edge ) => ( edge.corners[0].position, edge.corners[1].position ) );
        var actualEdges   = gpu.edges.Where( ( edge ) => edge.intersectsContour( ) ).ToArray( );

        Assert.AreEqual( expectedEdges.Count, actualEdges.Length, "edges that cross the contour" );

        var intersections = actualEdges.Select(
            ( edge ) => Vector3.Distance( expectedEdges[( edge.corners[0].position, edge.corners[1].position )].intersection, edge.intersection ) * ( 1 << resolution )
        ).ToArray( );

        Assert.LessOrEqual( intersections.Max( ), intersectionTolerance, "how far the farthest 1D point is from where Burst has it, in cells" );

        // the vertices and their normals
        var vertices = expected.Keys.SelectMany(
            ( cell ) => expected[cell].Zip( actual[cell], ( one, other ) => Vector3.Distance( one.vertex, other.vertex ) * ( 1 << resolution ) )
        ).ToArray( );

        var normals = expected.Keys.SelectMany(
            ( cell ) => expected[cell].Zip( actual[cell], ( one, other ) => Vector3.Angle( one.normal, other.normal ) )
        ).ToArray( );

        var apart = vertices.Count( ( deviation ) => !( deviation <= vertexTolerance ) );

        Assert.AreEqual( 0, apart, $"{apart} of {vertices.Length} vertices are more than {vertexTolerance} cells from where Burst puts them, the worst by {vertices.Max( )}" );

        var turned = normals.Count( ( angle ) => !( angle <= normalTolerance ) );

        Assert.AreEqual( 0, turned, $"{turned} of {normals.Length} normals are more than {normalTolerance} degrees from those of Burst, the worst by {normals.Max( )}" );

        // the same polygons, split the same way
        Assert.AreEqual( burst.mesh.triangles.Length, gpu.mesh.triangles.Length, "indices of the mesh" );
        Assert.AreEqual( burst.mesh.vertexCount,      gpu.mesh.vertexCount,      "vertices of the mesh" );
        Assert.AreEqual( burst.mesh.subMeshCount,     gpu.mesh.subMeshCount,     "sub-meshes" );

        for( var subMesh = 0; subMesh < burst.mesh.subMeshCount; ++subMesh ) {
            Assert.AreEqual( burst.mesh.GetIndexCount( subMesh ), gpu.mesh.GetIndexCount( subMesh ), $"indices of sub-mesh {subMesh}" );
        }
    }

    // - a search of fifteen halvings steps by 2^-15 of a cell, 0.0000305, and the farthest 1D point is one step off;
    // - the farthest vertex is 0.0048 of a cell off, on the new shapes, and most are within 0.0001;
    // - the farthest normal is 0.13 degrees off
    private const float intersectionTolerance = 3.1e-5f;
    private const float vertexTolerance       = 0.02f;
    private const float normalTolerance       = 0.5f;

    [Test]
    public void contourIsTheBurstOne( [Values] Model model ) {
        this.assertMatchesBurst( shapesOf( model ) );
    }

    [Test]
    public void contourIsTheBurstOneWhereACellHoldsTwoPiecesOfSurface( ) {
        this.assertMatchesBurst( twoSpheres );
    }

    [Test]
    public void contourIsTheBurstOneWhereTwoCellsHaveAJoinedFaceInCommon( ) {
        this.assertMatchesBurst( hollowsAcrossFaces );
    }

    [Test]
    public void contourIsTheBurstOneOnTurnedShapes( ) {
        this.assertMatchesBurst( turnedShapes );
    }

    [Test]
    public void contourIsTheBurstOneOnTheNewShapes( ) {
        this.assertMatchesBurst( newShapes );
    }

    [Test]
    public void contourIsTheBurstOneWithoutKeepingVerticesInTheirCells( ) {
        this.assertMatchesBurst( turnedShapes, false );
    }

    // Voxelizer.contour can draw the mesh straight from the buffers of the shader, without reading it back. It draws as
    // many indices as the argument buffer says: three for every triangle of the mesh
    [Test]
    public void contourCanBeDrawnStraightFromTheGPU( [Values] VertexNormals normals ) {
        var voxelizer = this.createVoxelizer<OccupancyDualContouring>( resolution, QEFSolverType.SVD, shapesOf( Model.SphereCutaway ) );

        voxelizer.implementationType   = ImplementationType.GPU;
        voxelizer.vertexNormals        = normals;
        voxelizer.keepDebugInformation = false;

        // a renderer starts with one material that is not set, which contour would read a colour from
        voxelizer.GetComponent<MeshRenderer>( ).sharedMaterials = new Material[0];

        var statistics = voxelizer.contour( drawFromGPU: true );

        Assert.IsNull( statistics.vertexCount, "the size of a mesh that is not read back" );

        voxelizer.voxelize( voxelizer.GetComponentsInChildren<Volume>( ) ).visit(
            ( cpu ) => { Assert.Fail( "the GPU implementation gave a CPU voxelization" ); return 0; },
            ( gpu ) => {
                var arguments = new int[4];

                gpu.arguments.GetData( arguments );

                var mesh = voxelizer.readMesh( gpu );

                try {
                    Assert.AreEqual( mesh.triangles.Length, arguments[0], "indices to draw" );
                    Assert.AreEqual( 1,                     arguments[1], "instances to draw" );
                }
                finally {
                    Object.DestroyImmediate( mesh );
                }
                return 0;
            }
        );
    }

    // with split normals the shader gives every triangle three vertices of its own, with the normal of its face: the
    // triangles of the mesh with shared normals, each by itself
    [Test]
    public void splitNormalsGiveEveryTriangleItsOwnVertices( ) {
        var voxelizer = this.createVoxelizer<OccupancyDualContouring>( resolution, QEFSolverType.SVD, shapesOf( Model.SphereCutaway ) );

        voxelizer.implementationType = ImplementationType.GPU;

        var shared = this.voxelize( voxelizer ).mesh;

        voxelizer.vertexNormals = VertexNormals.Split;

        var split = this.voxelize( voxelizer ).mesh;

        Assert.AreEqual( shared.triangles.Length, split.triangles.Length, "indices" );
        Assert.AreEqual( split.triangles.Length,  split.vertexCount,      "vertices, three for every triangle" );

        var vertices  = split.vertices;
        var normals   = split.normals;
        var triangles = split.triangles;

        for( var triangle = 0; triangle < triangles.Length; triangle += 3 ) {
            var a = vertices[triangles[triangle]];
            var b = vertices[triangles[triangle + 1]];
            var c = vertices[triangles[triangle + 2]];

            var face = Vector3.Cross( b - a, c - a ).normalized;

            for( var corner = 0; corner < 3; ++corner ) {
                Assert.LessOrEqual( Vector3.Angle( face, normals[triangles[triangle + corner]] ), 0.1f, $"the angle between a normal of the triangle at {a} and its face, in degrees" );
            }
        }

        // the same triangles, corner for corner, in whatever order
        static List<string> trianglesOf( Mesh mesh ) {
            var positions = mesh.vertices;
            var indices   = mesh.triangles;
            var list      = new List<string>( );

            for( var index = 0; index < indices.Length; index += 3 ) {
                list.Add( $"{positions[indices[index]]:R} {positions[indices[index + 1]]:R} {positions[indices[index + 2]]:R}" );
            }

            list.Sort( System.StringComparer.Ordinal );

            return list;
        }

        CollectionAssert.AreEqual( trianglesOf( shared ), trianglesOf( split ), "the triangles with split normals against those with shared normals" );
    }

}
