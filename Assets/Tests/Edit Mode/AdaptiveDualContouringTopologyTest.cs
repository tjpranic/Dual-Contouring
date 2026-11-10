using System;
using NUnit.Framework;
using UnityEngine;

using QEFSolverType = QEFSolver.Type;

public class AdaptiveDualContouringTopologyTest : TopologyTest {

    // both are thinner than the 1 / 8 wide cells that simplification collapses first, and lie inside one row of them
    public enum ThinFeature {
        Rod,
        Plate
    }

    private const int resolution = 5; // 16^3 leaves

    private static Shape[] shapesOf( ThinFeature feature ) {
        return feature switch {
            ThinFeature.Rod   => new[] { ellipsoid ( new Vector3( 0.013f, 0.0625f,  0.0625f ), new Vector3( 0.35f, 0.04f, 0.04f ) ) },
            ThinFeature.Plate => new[] { cuboid    ( new Vector3( 0.013f, 0.0625f, -0.011f  ), new Vector3( 0.3f,  0.02f, 0.3f  ) ) },
            _                 => throw new Exception( "Unknown thin feature specified" )
        };
    }

    // an error threshold that collapses about half of the cells above the leaves on every model, so that collapsed
    // cells end up next to leaves. The solvers measure error differently: the SVD solver reports the residual of the
    // QEF, the simple one how far the vertex is from the surface
    private static float thresholdOf( QEFSolverType solver ) {
        return solver == QEFSolverType.SVD ? 2e-4f : 0.007f;
    }

    private AdaptiveDualContouring createSimplifying( QEFSolverType solver, Shape[] shapes, float errorThreshold ) {
        var voxelizer = this.createVoxelizer<AdaptiveDualContouring>( resolution, solver, shapes );

        voxelizer.simplification = true;
        voxelizer.errorThreshold = errorThreshold;

        return voxelizer;
    }

    // without simplification every leaf is the same size and the octree contours like a uniform grid
    [Test]
    public void surfaceIsClosedAndManifoldWithoutSimplification( [Values] Model model, [Values] QEFSolverType solver ) {
        var voxelizer = this.createVoxelizer<AdaptiveDualContouring>( resolution, solver, shapesOf( model ) );

        var topology = this.contour( voxelizer );

        assertSingleSphere( topology );

        Assert.AreEqual( 0, topology.degenerateTriangles, $"degenerate triangles in: {topology}" );
    }

    [Test]
    public void surfaceWithVertexAtOriginIsClosed( ) {
        var voxelizer = this.createVoxelizer<AdaptiveDualContouring>( 4, QEFSolverType.SVD, cubeWithCornerAtOrigin ); // 8^3 leaves

        assertClosed( this.contour( voxelizer ) );
    }

    // Dual Contouring of Hermite Data, section 3.2: generating a polygon for each minimal edge that exhibits a sign
    // change "always produces a closed polygonal mesh for any simplified octree. In particular, every edge in the mesh
    // is contained by an even number of polygons"
    [Test]
    public void simplifiedSurfaceIsClosed( [Values] Model model, [Values] QEFSolverType solver ) {
        var voxelizer = this.createSimplifying( solver, shapesOf( model ), thresholdOf( solver ) );

        assertClosed( this.contour( voxelizer ) );
    }

    // section 3.2: where a coarse cell is face-adjacent to finer ones, a minimal edge is contained by only three cells
    // and generates a triangle rather than a quad that uses the coarse cell's vertex twice
    [Test]
    public void simplifiedSurfaceHasNoDegenerateTriangles( [Values] Model model, [Values] QEFSolverType solver ) {
        var voxelizer = this.createSimplifying( solver, shapesOf( model ), thresholdOf( solver ) );

        var topology = this.contour( voxelizer );

        Assert.AreEqual( 0, topology.degenerateTriangles, $"degenerate triangles in: {topology}" );
    }

    // section 4: a cell may only collapse when that preserves the topology of the contour. With the error threshold
    // out of the way that test is all that keeps a thin feature from merging into a single sheet or vanishing
    [Test]
    public void simplificationKeepsThinFeatures( [Values] ThinFeature feature, [Values] QEFSolverType solver ) {
        var voxelizer = this.createVoxelizer<AdaptiveDualContouring>( resolution, solver, shapesOf( feature ) );

        var unsimplified = this.contour( voxelizer );

        voxelizer.simplification = true;
        voxelizer.errorThreshold = float.MaxValue;

        var simplified = this.contour( voxelizer );

        assertClosed( simplified );

        Assert.AreEqual( unsimplified.components,          simplified.components,          $"simplification turned {unsimplified} into {simplified}" );
        Assert.AreEqual( unsimplified.eulerCharacteristic, simplified.eulerCharacteristic, $"simplification turned {unsimplified} into {simplified}" );
    }

}
