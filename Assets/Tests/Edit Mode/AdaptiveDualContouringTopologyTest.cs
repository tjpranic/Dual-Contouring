using System;
using NUnit.Framework;
using UnityEngine;

using Combination   = DensityFunction.Combination;
using MaterialIndex = SurfaceExtractor.MaterialIndex;
using QEFSolverType = QEFSolver.Type;

public class AdaptiveDualContouringTopologyTest : TopologyTest {

    // both are thinner than the 1 / 8 wide cells that simplification collapses first, and lie inside one row of them
    public enum ThinFeature {
        Rod,
        Plate
    }

    // how far simplification goes: light collapses about half of the cells directly above the leaves, heavy collapses
    // most of those and part of the level above them. Either way the octree ends up with leaves of several sizes
    // next to each other
    public enum Simplification {
        Light,
        Heavy
    }

    private const int resolution = 5; // 16^3 leaves

    // a slab with a strip of a second material set into its top face. The strip is 0.04 wide, a third of the 1 / 8 wide
    // cells that simplification collapses first, and lies between the corners of a row of them: none of those corners
    // has the material of the strip
    private static readonly Shape[] slabWithStrip = {
        cuboid( new Vector3( 0.0f,    -0.185f, 0.0f ), new Vector3( 0.4f,  0.215f, 0.4f ) ),
        cuboid( new Vector3( 0.0625f, -0.035f, 0.0f ), new Vector3( 0.02f, 0.065f, 0.3f ), Combination.Union, MaterialIndex.Material1 )
    };

    private static Shape[] shapesOf( ThinFeature feature ) {
        return feature switch {
            ThinFeature.Rod   => new[] { ellipsoid ( new Vector3( 0.013f, 0.0625f,  0.0625f ), new Vector3( 0.35f, 0.04f, 0.04f ) ) },
            ThinFeature.Plate => new[] { cuboid    ( new Vector3( 0.013f, 0.0625f, -0.011f  ), new Vector3( 0.3f,  0.02f, 0.3f  ) ) },
            _                 => throw new Exception( "Unknown thin feature specified" )
        };
    }

    // the solvers measure error differently: the SVD solver reports the residual of the QEF, the simple one how far the
    // vertex is from the surface
    private static float thresholdOf( QEFSolverType solver, Simplification simplification ) {
        return ( solver, simplification ) switch {
            ( QEFSolverType.SVD,    Simplification.Light ) => 2e-4f,
            ( QEFSolverType.SVD,    Simplification.Heavy ) => 2e-3f,
            ( QEFSolverType.Simple, Simplification.Light ) => 0.007f,
            ( QEFSolverType.Simple, Simplification.Heavy ) => 0.015f,
            _                                              => throw new Exception( "Unknown solver or simplification specified" )
        };
    }

    private AdaptiveDualContouring createSimplifying( QEFSolverType solver, Shape[] shapes, float errorThreshold, bool topologySafe ) {
        var voxelizer = this.createVoxelizer<AdaptiveDualContouring>( resolution, solver, shapes );

        voxelizer.simplification = true;
        voxelizer.errorThreshold = errorThreshold;
        voxelizer.topologySafe   = topologySafe;

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
    // is contained by an even number of polygons". Any octree includes the ones that topology safety would not allow
    [Test]
    public void simplifiedSurfaceIsClosed( [Values] Model model, [Values] QEFSolverType solver, [Values] Simplification simplification ) {
        var voxelizer = this.createSimplifying( solver, shapesOf( model ), thresholdOf( solver, simplification ), false );

        assertClosed( this.contour( voxelizer ) );
    }

    // section 3.2: where a coarse cell is face-adjacent to finer ones, a minimal edge is contained by only three cells
    // and generates a triangle rather than a quad that uses the coarse cell's vertex twice
    [Test]
    public void simplifiedSurfaceHasNoDegenerateTriangles( [Values] Model model, [Values] QEFSolverType solver, [Values] Simplification simplification ) {
        var voxelizer = this.createSimplifying( solver, shapesOf( model ), thresholdOf( solver, simplification ), false );

        var topology = this.contour( voxelizer );

        Assert.AreEqual( 0, topology.degenerateTriangles, $"degenerate triangles in: {topology}" );
    }

    // section 3.1: the QEFs of the leaves are added up at the interior nodes, and every node whose residual stays under
    // the tolerance collapses, whatever its level. A cube centered in the grid has an octant of the octree around each
    // of its corners, and the three faces in an octant meet in that corner without any residual: all that should
    // remain of the octree are those eight octants, with the corners of the cube as their vertices
    [Test]
    public void cubeSimplifiesToItsCorners( ) {
        var voxelizer = this.createSimplifying( QEFSolverType.SVD, new[] { cube( Vector3.zero, 0.35f ) }, 1e-4f, true );

        var topology = this.contour( voxelizer );

        assertSingleSphere( topology );

        Assert.AreEqual( 8,  topology.vertices,  $"vertices of: {topology}" );
        Assert.AreEqual( 12, topology.triangles, $"triangles of: {topology}" );
    }

    // section 4: a cell may only collapse when that preserves the topology of the contour. With the error threshold
    // out of the way that test is all that keeps a thin feature from merging into a single sheet or vanishing
    [Test]
    public void safeSimplificationKeepsThinFeatures( [Values] ThinFeature feature, [Values] QEFSolverType solver ) {
        var voxelizer = this.createVoxelizer<AdaptiveDualContouring>( resolution, solver, shapesOf( feature ) );

        var unsimplified = this.contour( voxelizer );

        voxelizer.simplification = true;
        voxelizer.errorThreshold = float.MaxValue;
        voxelizer.topologySafe   = true;

        var simplified = this.contour( voxelizer );

        assertClosed( simplified );

        Assert.AreEqual( unsimplified.components,          simplified.components,          $"simplification turned {unsimplified} into {simplified}" );
        Assert.AreEqual( unsimplified.eulerCharacteristic, simplified.eulerCharacteristic, $"simplification turned {unsimplified} into {simplified}" );
    }

    // section 4.1: the checks limit collapsing to manifold contours and keep the collapsed contour equivalent to the
    // one before. However far simplification is allowed to go, each model has to remain the single closed surface
    // without handles that it started as
    [Test]
    public void safeSimplificationKeepsSurfaceManifold( [Values] Model model, [Values] QEFSolverType solver ) {
        var voxelizer = this.createSimplifying( solver, shapesOf( model ), float.MaxValue, true );

        var topology = this.contour( voxelizer );

        assertSingleSphere( topology );

        Assert.AreEqual( 0, topology.degenerateTriangles, $"degenerate triangles in: {topology}" );
    }

    // section 4.2: the regions that the materials cover on the contour keep their topology as well. To the test for
    // empty and solid alone the top of the slab is a flat face that can collapse all the way, which would wipe out
    // the strip
    [Test]
    public void safeSimplificationKeepsMaterialRegions( [Values] QEFSolverType solver ) {
        var voxelizer = this.createVoxelizer<AdaptiveDualContouring>( resolution, solver, slabWithStrip );

        var unsimplified = this.contourRegions( voxelizer );

        voxelizer.simplification = true;
        voxelizer.errorThreshold = float.MaxValue;
        voxelizer.topologySafe   = true;

        var simplified = this.contourRegions( voxelizer );

        Assert.AreEqual( new[] { 1, 1 }, unsimplified, "the slab and the strip should each be a single region to begin with" );
        Assert.AreEqual( unsimplified,   simplified,   "pieces per material after simplification" );
    }

}
