using NUnit.Framework;

using ImplementationType = SurfaceExtractor.Implementation.Type;
using QEFSolverType      = QEFSolver.Type;

public class UniformDualContouringTopologyTest : TopologyTest {

    // Dual Contouring of Hermite Data, section 2.2: a vertex for every cell with a sign change and a quad for every
    // edge with a sign change, connecting the vertices of the four cells around it. That closes the surface, and none
    // of the models puts two pieces of surface in one cell, so it is manifold as well
    [Test]
    public void surfaceIsClosedAndManifold( [Values] Model model, [Values] QEFSolverType solver, [Values( ImplementationType.Managed, ImplementationType.Burst )] ImplementationType implementation ) {
        var voxelizer = this.createUniformVoxelizer( 4, implementation, solver, shapesOf( model ) ); // 16^3 cells

        var topology = this.contour( voxelizer );

        assertSingleSphere( topology );

        Assert.AreEqual( 0, topology.degenerateTriangles, $"degenerate triangles in: {topology}" );
    }

    // the compute shader only has the SVD solver
    [Test]
    public void surfaceIsClosedAndManifoldOnGPU( [Values] Model model ) {
        var voxelizer = this.createUniformVoxelizer( 4, ImplementationType.GPU, QEFSolverType.SVD, shapesOf( model ) ); // 16^3 cells

        var topology = this.contour( voxelizer );

        assertSingleSphere( topology );

        Assert.AreEqual( 0, topology.degenerateTriangles, $"degenerate triangles in: {topology}" );
    }

    // edges of turned shapes can cross a cell so that its corners show two pieces of surface, which one vertex joins:
    // the surface stays closed, but need not be manifold there (RotatedCubeTest)
    [Test]
    public void surfaceOfTurnedShapesIsClosed( [Values] ImplementationType implementation ) {
        var voxelizer = this.createUniformVoxelizer( 4, implementation, QEFSolverType.SVD, turnedShapes ); // 16^3 cells

        assertClosed( this.contour( voxelizer ) );
    }

    // the same of the shapes of the article other than the box and the ellipsoid, all turned
    [Test]
    public void surfaceOfTheNewShapesIsClosed( [Values] ImplementationType implementation ) {
        var voxelizer = this.createUniformVoxelizer( 4, implementation, QEFSolverType.SVD, newShapes ); // 16^3 cells

        assertClosed( this.contour( voxelizer ) );
    }

    [Test]
    public void materialBoundaryIsFollowed( [Values] ImplementationType implementation ) {
        var voxelizer = this.createUniformVoxelizer( 4, implementation, QEFSolverType.SVD, slabWithBand ); // 16^3 cells

        this.assertBandFollowsMaterialBoundary( voxelizer );
    }

    [Test]
    public void surfaceWithVertexAtOriginIsClosed( [Values] ImplementationType implementation ) {
        var voxelizer = this.createUniformVoxelizer( 3, implementation, QEFSolverType.SVD, cubeWithCornerAtOrigin ); // 8^3 cells

        assertClosed( this.contour( voxelizer ) );
    }

}
