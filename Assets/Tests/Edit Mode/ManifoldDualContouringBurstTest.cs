using System;
using System.Linq;
using NUnit.Framework;
using UnityEngine;

using ImplementationType = SurfaceExtractor.Implementation.Type;
using CPUVoxelization    = SurfaceExtractor.Implementation.CPU.Voxelization;
using QEFSolverType      = QEFSolver.Type;

// the Burst implementation of manifold dual contouring against the managed one, as BurstParity holds them to each other.
// That includes what clustering leaves them with: the same vertices stand in for the same clusters, in the same cells
public class ManifoldDualContouringBurstTest : TopologyTest {

    // none, the threshold of the scenes that cluster, ten times that, and as far as the manifold criterion allows
    public enum Clustering {
        None,
        Light,
        Heavy,
        Full
    }

    private static float thresholdOf( Clustering clustering ) {
        return clustering switch {
            Clustering.Light => 2e-4f,
            Clustering.Heavy => 2e-3f,
            Clustering.Full  => float.MaxValue,
            _                => 0.0f
        };
    }

    private ManifoldDualContouring create( int resolution, QEFSolverType solver, Shape[] shapes, Clustering clustering, bool enforceManifold ) {
        var voxelizer = this.createVoxelizer<ManifoldDualContouring>( resolution, solver, shapes );

        voxelizer.errorThreshold  = thresholdOf( clustering );
        voxelizer.enforceManifold = enforceManifold;

        return voxelizer;
    }

    private ( CPUVoxelization managed, CPUVoxelization burst ) contourBoth( ManifoldDualContouring voxelizer ) {
        voxelizer.implementationType = ImplementationType.Managed;

        var managed = this.voxelize( voxelizer );

        voxelizer.implementationType = ImplementationType.Burst;

        var burst = this.voxelize( voxelizer );

        return ( managed, burst );
    }

    // an octree of the given depth has 2^(depth - 1) leaves across
    private void assertSameContour( int resolution, QEFSolverType solver, Shape[] shapes, Clustering clustering, bool enforceManifold = true ) {
        var ( managed, burst ) = this.contourBoth( this.create( resolution, solver, shapes, clustering, enforceManifold ) );

        BurstParity.assertSameContour( managed, burst, 1.0f / ( 1 << ( resolution - 1 ) ) );
    }

    [Test]
    public void contourIsTheManagedOne( [Values] Model model, [Values] QEFSolverType solver, [Values] Clustering clustering ) {
        this.assertSameContour( 5, solver, shapesOf( model ), clustering ); // 16^3 leaves
    }

    // without the manifold criterion the error threshold alone decides, and tubes and sheets may collapse together
    [Test]
    public void contourIsTheManagedOneWithoutTheManifoldCriterion( [Values] Model model, [Values] QEFSolverType solver ) {
        this.assertSameContour( 5, solver, shapesOf( model ), Clustering.Heavy, false );
    }

    [Test]
    public void contourIsTheManagedOneOnEveryGridSize( [Values( 2, 3, 4, 6 )] int resolution ) {
        this.assertSameContour( resolution, QEFSolverType.SVD, shapesOf( Model.ThreeBalls ), Clustering.Light );
    }

    // a leaf that holds two pieces of surface has a vertex for each, and they go up the vertex tree separately
    [Test]
    public void contourIsTheManagedOneWhereALeafHoldsTwoPiecesOfSurface( [Values] QEFSolverType solver, [Values] Clustering clustering ) {
        this.assertSameContour( 5, solver, twoSpheres, clustering );
    }

    // two leaves that have a joined face in common take their vertices from their complements
    [Test]
    public void contourIsTheManagedOneWhereTwoLeavesHaveAJoinedFaceInCommon( [Values] QEFSolverType solver, [Values] Clustering clustering ) {
        this.assertSameContour( 5, solver, hollowsAcrossFaces, clustering );
    }

    // the boundary between two materials is part of the QEFs that clustering adds up
    [Test]
    public void contourIsTheManagedOneAlongAMaterialBoundary( [Values] QEFSolverType solver, [Values] Clustering clustering ) {
        this.assertSameContour( 5, solver, slabWithBand, clustering );
    }

    [Test]
    public void contourIsTheManagedOneOnTurnedShapes( [Values] QEFSolverType solver, [Values] Clustering clustering ) {
        this.assertSameContour( 5, solver, turnedShapes, clustering );
    }

    [Test]
    public void contourIsTheManagedOneOnTheNewShapes( [Values] QEFSolverType solver, [Values] Clustering clustering ) {
        this.assertSameContour( 5, solver, newShapes, clustering );
    }

    // two surfaces that share the cells above the leaves and cluster independently
    [Test]
    public void contourIsTheManagedOneWhereSeparateSurfacesShareACell( [Values] QEFSolverType solver, [Values] Clustering clustering ) {
        this.assertSameContour( 5, solver, ManifoldDualContouringTopologyTest.twoBoxes, clustering );
    }

    // clusters that the manifold criterion keeps from standing in for their vertices, below ones that it lets
    [Test]
    public void contourIsTheManagedOneAboveUnsafeClusters( [Values] QEFSolverType solver, [Values] Clustering clustering ) {
        this.assertSameContour( 5, solver, ManifoldDualContouringTopologyTest.boxWithFinger, clustering );
    }

    // a tube through every cell it passes, which is never clustered
    [Test]
    public void contourIsTheManagedOneAlongATube( [Values] QEFSolverType solver ) {
        this.assertSameContour( 5, solver, ManifoldDualContouringTopologyTest.beam, Clustering.Full );
    }

    // the scenes move the vertices of the leaves onto the surface after solving them, which the other tests leave out
    [Test]
    public void contourIsTheManagedOneWithSurfaceCorrection( [Values] QEFSolverType solver, [Values( Clustering.None, Clustering.Light )] Clustering clustering ) {
        var voxelizer = this.create( 5, solver, shapesOf( Model.ThreeBalls ), clustering, true );

        voxelizer.surfaceCorrectionIterations = 6;

        var ( managed, burst ) = this.contourBoth( voxelizer );

        BurstParity.assertSameContour( managed, burst, 1.0f / 16 );
    }

    [Test]
    public void contourIsTheManagedOneWithAVertexAtTheOrigin( ) {
        this.assertSameContour( 4, QEFSolverType.SVD, cubeWithCornerAtOrigin, Clustering.None ); // 8^3 leaves
    }

    // the octree the managed implementation cannot build any more in reasonable time: 128^3 leaves. The three balls meet
    // in creases that put two sheets of surface in one cell, which gives dual contouring with one vertex for a cell edges
    // with more than two triangles. Here each sheet has a vertex of its own
    [Test]
    public void largestOctreeIsClosedAndManifold( [Values( Clustering.None, Clustering.Light, Clustering.Full )] Clustering clustering ) {
        var voxelizer = this.create( 8, QEFSolverType.SVD, shapesOf( Model.ThreeBalls ), clustering, true );

        voxelizer.implementationType = ImplementationType.Burst;

        // copying out the voxels would take longer than contouring them
        voxelizer.keepDebugInformation = false;

        BurstParity.assertClosedAndOriented( this.voxelize( voxelizer ).mesh );
    }

    // the corners, edges and voxels are copied out only when debug drawing is on or someone asked for them
    [Test]
    public void debugInformationIsOnlyCopiedWhenWanted( ) {
        var voxelizer = this.create( 4, QEFSolverType.SVD, shapesOf( Model.Sphere ), Clustering.Light, true );

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
