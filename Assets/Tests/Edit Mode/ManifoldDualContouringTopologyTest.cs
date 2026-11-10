using NUnit.Framework;
using UnityEngine;

using QEFSolverType = QEFSolver.Type;

public class ManifoldDualContouringTopologyTest : TopologyTest {

    private const int resolution = 5; // 16^3 leaves

    // two spheres 0.04 apart along the diagonal of the leaf [0, 0.0625]^2 in x and y. Each sphere covers one of two
    // diagonally opposite corners of that leaf and neither covers the other two, so the leaf holds two separate pieces
    // of surface
    private static readonly Shape[] twoSpheres = {
        sphere( new Vector3( 0.03125f - 0.0849f, 0.03125f - 0.0849f, 0.003f ), 0.1f ),
        sphere( new Vector3( 0.03125f + 0.0849f, 0.03125f + 0.0849f, 0.003f ), 0.1f )
    };

    // a 0.08 thick beam that runs through the whole grid along x, centered in one row of the 1 / 8 wide cells above the
    // leaves. Every cell it passes through, at any level of the octree, holds a tube that is open at both ends
    private static readonly Shape[] beam = {
        cuboid( new Vector3( 0.0f, 0.0625f, 0.0625f ), new Vector3( 1.0f, 0.04f, 0.04f ) )
    };

    // an error threshold that lets about half of the clusters above the leaves collapse on every model. The solvers
    // measure error differently: the SVD solver reports the residual of the QEF, the simple one how far the vertex is
    // from the surface
    private static float thresholdOf( QEFSolverType solver ) {
        return solver == QEFSolverType.SVD ? 2e-4f : 0.014f;
    }

    private ManifoldDualContouring create( int resolution, QEFSolverType solver, Shape[] shapes, float errorThreshold ) {
        var voxelizer = this.createVoxelizer<ManifoldDualContouring>( resolution, solver, shapes );

        voxelizer.errorThreshold  = errorThreshold;
        voxelizer.enforceManifold = true;

        return voxelizer;
    }

    // with a threshold of zero no vertex is collapsible and the octree contours like a uniform grid
    [Test]
    public void surfaceIsClosedAndManifoldWithoutClustering( [Values] Model model, [Values] QEFSolverType solver ) {
        var voxelizer = this.create( resolution, solver, shapesOf( model ), 0.0f );

        var topology = this.contour( voxelizer );

        assertSingleSphere( topology );

        Assert.AreEqual( 0, topology.degenerateTriangles, $"degenerate triangles in: {topology}" );
    }

    [Test]
    public void surfaceWithVertexAtOriginIsClosed( ) {
        var voxelizer = this.create( 4, QEFSolverType.SVD, cubeWithCornerAtOrigin, 0.0f ); // 8^3 leaves

        assertClosed( this.contour( voxelizer ) );
    }

    // Manifold Dual Contouring, section III: with a vertex for each piece of surface in a cell, the surface contoured
    // from a uniform grid "is always a manifold"
    [Test]
    public void surfaceIsManifoldWhereALeafHoldsTwoPiecesOfSurface( ) {
        var voxelizer = this.create( resolution, QEFSolverType.SVD, twoSpheres, 0.0f );

        assertManifold( this.contour( voxelizer ) );
    }

    // proposition 1: clustering under the manifold criterion generates a closed 2-manifold, and preserves the genus of
    // the surface
    [Test]
    public void clusteredSurfaceIsClosedAndManifold( [Values] Model model, [Values] QEFSolverType solver ) {
        var voxelizer = this.create( resolution, solver, shapesOf( model ), thresholdOf( solver ) );

        assertSingleSphere( this.contour( voxelizer ) );
    }

    // section IV.B: "If the resulting polygon collapses to an edge or a vertex, then we discard that polygon"
    [Test]
    public void clusteredSurfaceHasNoDegenerateTriangles( [Values] Model model, [Values] QEFSolverType solver ) {
        var voxelizer = this.create( resolution, solver, shapesOf( model ), thresholdOf( solver ) );

        var topology = this.contour( voxelizer );

        Assert.AreEqual( 0, topology.degenerateTriangles, $"degenerate triangles in: {topology}" );
    }

    // proposition 1 only lets a vertex stand in for a piece of surface with an euler characteristic of 1, a disk. A
    // tube has 0 (figure 5b), so the beam must come through clustering untouched whatever the error threshold is
    [Test]
    public void tubeIsNeverClustered( [Values] QEFSolverType solver ) {
        var voxelizer = this.create( resolution, solver, beam, 0.0f );

        var unclustered = this.contour( voxelizer );

        voxelizer.errorThreshold = float.MaxValue;

        var clustered = this.contour( voxelizer );

        Assert.Greater( unclustered.triangles, 0, "the mesh is empty" );

        Assert.AreEqual( unclustered.vertices,  clustered.vertices,  $"clustering turned {unclustered} into {clustered}" );
        Assert.AreEqual( unclustered.triangles, clustered.triangles, $"clustering turned {unclustered} into {clustered}" );
    }

}
