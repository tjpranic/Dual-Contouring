using System.Linq;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.Rendering;

using ImplementationType = SurfaceExtractor.Implementation.Type;
using CPUVoxelization    = SurfaceExtractor.Implementation.CPU.Voxelization;
using QEFSolverType      = QEFSolver.Type;

// the Burst implementation of uniform dual contouring against the managed one, as BurstParity holds them to each other
public class UniformDualContouringBurstTest : TopologyTest {

    private ( CPUVoxelization managed, CPUVoxelization burst ) contourBoth( int resolution, QEFSolverType solver, Shape[] shapes ) {
        var voxelizer = this.createUniformVoxelizer( resolution, ImplementationType.Managed, solver, shapes );

        var managed = this.voxelize( voxelizer );

        voxelizer.implementationType = ImplementationType.Burst;

        var burst = this.voxelize( voxelizer );

        return ( managed, burst );
    }

    private void assertSameContour( int resolution, QEFSolverType solver, Shape[] shapes ) {
        var ( managed, burst ) = this.contourBoth( resolution, solver, shapes );

        BurstParity.assertSameContour( managed, burst, 1.0f / ( 1 << resolution ) );
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
    public void contourIsTheManagedOneOnTurnedShapes( [Values] QEFSolverType solver ) {
        this.assertSameContour( 4, solver, turnedShapes );
    }

    [Test]
    public void contourIsTheManagedOneOnTheNewShapes( [Values] QEFSolverType solver ) {
        this.assertSameContour( 4, solver, newShapes );
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

        BurstParity.assertClosedAndOriented( voxelization.mesh );
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
