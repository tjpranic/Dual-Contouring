using System;
using System.Linq;
using NUnit.Framework;
using UnityEngine;

using ImplementationType = SurfaceExtractor.Implementation.Type;
using CPUVoxelization    = SurfaceExtractor.Implementation.CPU.Voxelization;
using QEFSolverType      = QEFSolver.Type;

// the Burst implementation of adaptive dual contouring against the managed one, as BurstParity holds them to each other.
// That includes the octree simplification leaves them with: the same nodes collapse, into the same vertices
public class AdaptiveDualContouringBurstTest : TopologyTest {

    // none, about half of the cells above the leaves, most of those and part of the level above them
    public enum Simplification {
        None,
        Light,
        Heavy
    }

    private static float thresholdOf( Simplification simplification ) {
        return simplification switch {
            Simplification.Light => 2e-4f,
            Simplification.Heavy => 2e-3f,
            _                    => 0.0f
        };
    }

    private AdaptiveDualContouring create( int resolution, QEFSolverType solver, Shape[] shapes, Simplification simplification, bool topologySafe ) {
        var voxelizer = this.createVoxelizer<AdaptiveDualContouring>( resolution, solver, shapes );

        voxelizer.simplification = simplification != Simplification.None;
        voxelizer.errorThreshold = thresholdOf( simplification );
        voxelizer.topologySafe   = topologySafe;

        return voxelizer;
    }

    private ( CPUVoxelization managed, CPUVoxelization burst ) contourBoth( AdaptiveDualContouring voxelizer ) {
        voxelizer.implementationType = ImplementationType.Managed;

        var managed = this.voxelize( voxelizer );

        voxelizer.implementationType = ImplementationType.Burst;

        var burst = this.voxelize( voxelizer );

        return ( managed, burst );
    }

    // an octree of the given depth has 2^(depth - 1) leaves across
    private void assertSameContour( int resolution, QEFSolverType solver, Shape[] shapes, Simplification simplification, bool topologySafe = true ) {
        var ( managed, burst ) = this.contourBoth( this.create( resolution, solver, shapes, simplification, topologySafe ) );

        BurstParity.assertSameContour( managed, burst, 1.0f / ( 1 << ( resolution - 1 ) ) );
    }

    [Test]
    public void contourIsTheManagedOne( [Values] Model model, [Values] QEFSolverType solver, [Values] Simplification simplification ) {
        this.assertSameContour( 5, solver, shapesOf( model ), simplification ); // 16^3 leaves
    }

    // simplification without the topology test lets thin features and pieces of surface collapse together
    [Test]
    public void contourIsTheManagedOneWithoutTheTopologyTest( [Values] Model model, [Values] QEFSolverType solver ) {
        this.assertSameContour( 5, solver, shapesOf( model ), Simplification.Heavy, false );
    }

    [Test]
    public void contourIsTheManagedOneOnEveryGridSize( [Values( 2, 3, 4, 6 )] int resolution ) {
        this.assertSameContour( resolution, QEFSolverType.SVD, shapesOf( Model.ThreeBalls ), Simplification.Light );
    }

    // where a cell holds two pieces of surface its vertex is the mass point of its QEF
    [Test]
    public void contourIsTheManagedOneWhereACellHoldsTwoPiecesOfSurface( [Values] QEFSolverType solver, [Values] Simplification simplification ) {
        this.assertSameContour( 5, solver, twoSpheres, simplification );
    }

    // the boundary between two materials is part of the QEFs that simplification adds up
    [Test]
    public void contourIsTheManagedOneAlongAMaterialBoundary( [Values] QEFSolverType solver, [Values] Simplification simplification ) {
        this.assertSameContour( 5, solver, slabWithBand, simplification );
    }

    [Test]
    public void contourIsTheManagedOneOnTurnedShapes( [Values] QEFSolverType solver, [Values] Simplification simplification ) {
        this.assertSameContour( 5, solver, turnedShapes, simplification );
    }

    [Test]
    public void contourIsTheManagedOneOnTheNewShapes( [Values] QEFSolverType solver, [Values] Simplification simplification ) {
        this.assertSameContour( 5, solver, newShapes, simplification );
    }

    [Test]
    public void contourIsTheManagedOneWithAVertexAtTheOrigin( ) {
        this.assertSameContour( 4, QEFSolverType.SVD, cubeWithCornerAtOrigin, Simplification.None ); // 8^3 leaves
    }

    // the octree the managed implementation cannot build any more in reasonable time: 128^3 leaves. On a sphere, because
    // dual contouring with one vertex for a cell gives the three balls edges with more than two triangles from 64^3
    // leaves on, where the balls meet in creases that put two sheets of surface in one cell, managed and Burst alike
    [Test]
    public void largestOctreeIsClosed( [Values] Simplification simplification ) {
        var voxelizer = this.create( 8, QEFSolverType.SVD, shapesOf( Model.Sphere ), simplification, true );

        voxelizer.implementationType = ImplementationType.Burst;

        // copying out millions of voxels would take longer than contouring them
        voxelizer.keepDebugInformation = false;

        BurstParity.assertClosedAndOriented( this.voxelize( voxelizer ).mesh );
    }

    // the corners, edges and voxels are copied out only when debug drawing is on or someone asked for them
    [Test]
    public void debugInformationIsOnlyCopiedWhenWanted( ) {
        var voxelizer = this.create( 4, QEFSolverType.SVD, shapesOf( Model.Sphere ), Simplification.Light, true );

        voxelizer.implementationType   = ImplementationType.Burst;
        voxelizer.keepDebugInformation = false;
        voxelizer.debugFlags           = Voxelizer.Debug.Off;

        var without = this.voxelize( voxelizer );

        Assert.IsEmpty( without.corners, "corners" );
        Assert.IsEmpty( without.edges,   "edges" );
        Assert.IsEmpty( without.voxels,  "voxels" );
        Assert.Greater( without.mesh.vertexCount, 0, "the mesh is there either way" );

        voxelizer.debugFlags = Voxelizer.Debug.Cells;

        Assert.IsNotEmpty( this.voxelize( voxelizer ).voxels, "voxels with debug drawing on" );
    }

}
