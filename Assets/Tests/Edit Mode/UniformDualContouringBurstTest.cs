using System;
using System.Linq;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.Rendering;

using ImplementationType = SurfaceExtractor.Implementation.Type;
using CPUVoxelization    = SurfaceExtractor.Implementation.CPU.Voxelization;
using QEFSolverType      = QEFSolver.Type;

// the Burst implementation of uniform dual contouring against the managed one. Both go through the cells in the same
// order and create the vertices of the mesh in the order quads first use them, so their meshes are the same triangle
// for triangle. Their vertices differ in the last bits: Mono keeps the intermediate results of an expression at a
// higher precision than Burst
public class UniformDualContouringBurstTest : TopologyTest {

    // in cells, for vertices and intersections. A vertex goes through the inverse of the eigenvalues of its QEF, which
    // magnifies the last bits of the hermite data more than the hermite data itself
    private const float vertexTolerance       = 1e-3f;
    private const float intersectionTolerance = 1e-4f;
    private const float normalTolerance       = 1e-3f;

    private ( CPUVoxelization managed, CPUVoxelization burst ) contourBoth( int resolution, QEFSolverType solver, Shape[] shapes ) {
        var voxelizer = this.createUniformVoxelizer( resolution, ImplementationType.Managed, solver, shapes );

        var managed = this.voxelize( voxelizer );

        voxelizer.implementationType = ImplementationType.Burst;

        var burst = this.voxelize( voxelizer );

        return ( managed, burst );
    }

    // how many of the deviations are past the tolerance, and the largest of them
    private static void assertWithin( string what, float[] deviations, float tolerance ) {
        var past = deviations.Count( ( deviation ) => !( deviation <= tolerance ) );

        Assert.AreEqual( 0, past, $"{past} of {deviations.Length} {what} are more than {tolerance} apart, the farthest by {( deviations.Length > 0 ? deviations.Max( ) : 0.0f )}" );
    }

    private void assertSameContour( int resolution, QEFSolverType solver, Shape[] shapes ) {
        var ( managed, burst ) = this.contourBoth( resolution, solver, shapes );

        var cell = 1.0f / ( 1 << resolution );

        // the cells, in the same order, with a vertex in the same ones and in the same place
        var managedVoxels = managed.voxels.ToArray( );
        var burstVoxels   = burst.voxels.ToArray( );

        Assert.AreEqual( managedVoxels.Length, burstVoxels.Length, "cells" );

        var misplaced = Enumerable.Range( 0, managedVoxels.Length ).Where(
            ( index ) => managedVoxels[index].center != burstVoxels[index].center || managedVoxels[index].size != burstVoxels[index].size
        ).ToArray( );

        Assert.IsEmpty( misplaced, "cells that are not in the same order" );

        var disagreeing = Enumerable.Range( 0, managedVoxels.Length ).Where(
            ( index ) => managedVoxels[index].hasFeaturePoint != burstVoxels[index].hasFeaturePoint
        ).Select(
            ( index ) => managedVoxels[index].center
        ).ToArray( );

        Assert.IsEmpty( disagreeing, "cells that have a vertex in one implementation only" );

        var withVertex = Enumerable.Range( 0, managedVoxels.Length ).Where( ( index ) => managedVoxels[index].hasFeaturePoint ).ToArray( );

        Assert.IsNotEmpty( withVertex, "no cell has a vertex" );

        assertWithin( "vertices of cells, in cells", withVertex.Select( ( index ) => Vector3.Distance( managedVoxels[index].vertex, burstVoxels[index].vertex ) / cell ).ToArray( ), vertexTolerance );
        assertWithin( "normals of cells",            withVertex.Select( ( index ) => Vector3.Distance( managedVoxels[index].normal, burstVoxels[index].normal ) ).ToArray( ),        normalTolerance );

        // the hermite data of the contour, on every edge of the grid. Edges across a material boundary are left out: each
        // managed voxel has edges of its own, and the managed implementation shows the edge of the first voxel it is in,
        // which has the hermite data of the boundary only if the contour passes through that voxel
        var managedEdges = managed.edges.ToDictionary( ( edge ) => ( edge.corners[0].position, edge.corners[1].position ) );
        var burstEdges   = burst.edges.ToArray( );

        Assert.AreEqual( managedEdges.Count, burstEdges.Length, "edges" );

        var pairs = burstEdges.Select( ( edge ) => ( expected: managedEdges[( edge.corners[0].position, edge.corners[1].position )], actual: edge ) ).ToArray( );

        Assert.IsEmpty(
            pairs.Where( ( pair ) => pair.expected.intersectsContour( ) != pair.actual.intersectsContour( ) ).Select( ( pair ) => pair.actual.corners[0].position ).ToArray( ),
            "edges that cross the contour in one implementation only"
        );

        var crossing = pairs.Where( ( pair ) => pair.expected.intersectsContour( ) ).ToArray( );

        Assert.IsNotEmpty( crossing, "no edge crosses the contour" );

        assertWithin( "intersections, in cells", crossing.Select( ( pair ) => Vector3.Distance( pair.expected.intersection, pair.actual.intersection ) / cell ).ToArray( ), intersectionTolerance );
        assertWithin( "hermite normals",         crossing.Select( ( pair ) => Vector3.Distance( pair.expected.normal,       pair.actual.normal ) ).ToArray( ),              normalTolerance );

        // the mesh, triangle for triangle
        Assert.AreEqual( managed.mesh.subMeshCount, burst.mesh.subMeshCount, "sub-meshes" );
        Assert.AreEqual( managed.mesh.vertexCount,  burst.mesh.vertexCount,  "vertices" );

        for( var subMesh = 0; subMesh < managed.mesh.subMeshCount; ++subMesh ) {
            CollectionAssert.AreEqual( managed.mesh.GetTriangles( subMesh ), burst.mesh.GetTriangles( subMesh ), $"triangles of sub-mesh {subMesh}" );
        }

        assertWithin( "vertices of the mesh, in cells", managed.mesh.vertices.Zip( burst.mesh.vertices, ( expected, actual ) => Vector3.Distance( expected, actual ) / cell ).ToArray( ), vertexTolerance );
        assertWithin( "normals of the mesh",            managed.mesh.normals.Zip( burst.mesh.normals,   ( expected, actual ) => Vector3.Distance( expected, actual ) ).ToArray( ),        normalTolerance );
    }

    [Test]
    public void contourIsTheManagedOne( [Values] Model model, [Values] QEFSolverType solver ) {
        this.assertSameContour( 4, solver, shapesOf( model ) ); // 16^3 cells
    }

    [Test]
    public void contourIsTheManagedOneOnEveryGridSize( [Values( 2, 3, 5 )] int resolution ) {
        this.assertSameContour( resolution, QEFSolverType.SVD, shapesOf( Model.ThreeBalls ) );
    }

    // where a cell holds two pieces of surface its vertex is the mass point of its QEF
    [Test]
    public void contourIsTheManagedOneWhereACellHoldsTwoPiecesOfSurface( [Values] QEFSolverType solver ) {
        this.assertSameContour( 4, solver, twoSpheres );
    }

    // where the boundary between two materials passes through a cell it has a say in where the vertex goes
    [Test]
    public void contourIsTheManagedOneAlongAMaterialBoundary( [Values] QEFSolverType solver ) {
        this.assertSameContour( 4, solver, slabWithBand );
    }

    [Test]
    public void contourIsTheManagedOneWithAVertexAtTheOrigin( ) {
        this.assertSameContour( 3, QEFSolverType.SVD, cubeWithCornerAtOrigin ); // 8^3 cells
    }

    // the grids the managed implementation cannot contour any more: 256^3 cells, with more vertices than 16 bit indices
    // reach
    [Test]
    public void largestGridIsClosedAndManifold( ) {
        var voxelizer = this.createUniformVoxelizer( 8, ImplementationType.Burst, QEFSolverType.SVD, shapesOf( Model.Sphere ) );

        // copying out 16 million voxels would take longer than contouring them
        voxelizer.keepDebugInformation = false;

        var voxelization = this.voxelize( voxelizer );

        Assert.Greater( voxelization.mesh.vertexCount, ushort.MaxValue, "vertices" );
        Assert.AreEqual( IndexFormat.UInt32, voxelization.mesh.indexFormat );

        // MeshTopology takes far longer than the contour on 400,000 triangles, so the surface is checked by sorting its
        // edges instead: it is closed and every edge is shared by two triangles that agree on which side faces out when
        // every edge runs once in each direction
        var triangles = voxelization.mesh.triangles;

        var directed = new long[triangles.Length];
        for( var triangle = 0; triangle < triangles.Length; triangle += 3 ) {
            for( var corner = 0; corner < 3; ++corner ) {
                long start = triangles[triangle + corner];
                long end   = triangles[triangle + ( ( corner + 1 ) % 3 )];

                directed[triangle + corner] = ( start << 32 ) | end;
            }
        }

        var reversed = directed.Select( ( edge ) => ( ( edge & 0xFFFFFFFFL ) << 32 ) | ( edge >> 32 ) ).ToArray( );

        Array.Sort( directed );
        Array.Sort( reversed );

        var repeated = 0;
        var unpaired = 0;
        for( var edge = 0; edge < directed.Length; ++edge ) {
            if( edge > 0 && directed[edge] == directed[edge - 1] ) {
                ++repeated;
            }
            if( directed[edge] != reversed[edge] ) {
                ++unpaired;
            }
        }

        Assert.AreEqual( 0, repeated, "edges that two triangles run along in the same direction" );
        Assert.AreEqual( 0, unpaired, "edges that no triangle runs along in the other direction" );
    }

    // the corners, edges and voxels are copied out of the native buffers only when debug drawing is on or someone asked
    // for them, and then there are as many as the managed implementation has
    [Test]
    public void debugInformationIsOnlyCopiedWhenWanted( ) {
        const int resolution = 3; // 8^3 cells

        var voxelizer = this.createUniformVoxelizer( resolution, ImplementationType.Burst, QEFSolverType.SVD, shapesOf( Model.Sphere ) );

        voxelizer.keepDebugInformation = false;
        voxelizer.debugFlags           = Voxelizer.Debug.Off;

        var without = this.voxelize( voxelizer );

        Assert.IsEmpty( without.corners, "corners" );
        Assert.IsEmpty( without.edges,   "edges" );
        Assert.IsEmpty( without.voxels,  "voxels" );
        Assert.Greater( without.mesh.vertexCount, 0, "the mesh is there either way" );

        voxelizer.debugFlags = Voxelizer.Debug.Cells;

        var with = this.voxelize( voxelizer );

        voxelizer.implementationType = ImplementationType.Managed;

        var managed = this.voxelize( voxelizer );

        Assert.AreEqual( managed.corners.Count( ), with.corners.Count( ), "corners" );
        Assert.AreEqual( managed.edges.Count( ),   with.edges.Count( ),   "edges" );
        Assert.AreEqual( managed.voxels.Count( ),  with.voxels.Count( ),  "voxels" );
    }

}
