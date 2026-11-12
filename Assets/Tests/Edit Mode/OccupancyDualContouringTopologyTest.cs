using System;
using System.Linq;
using NUnit.Framework;
using UnityEngine;

using Combination        = DensityFunction.Combination;
using MaterialIndex      = SurfaceExtractor.MaterialIndex;
using ImplementationType = SurfaceExtractor.Implementation.Type;
using QEFSolverType      = QEFSolver.Type;

// how the triangles of the contours of Occupancy-Based Dual Contouring connect, and what its debug view is given, for
// each implementation in the fixtures at the end
public abstract class OccupancyDualContouringTopologyTest : TopologyTest {

    private const int resolution = 4; // 16^3 cells

    // a ring lying at an angle, its hole more than three cells of 1 / 16 across and its tube as thick
    private static readonly Shape[] torus = {
        new Shape( DensityFunction.Type.Torus, new Vector3( 0.013f, -0.021f, 0.017f ), Quaternion.Euler( 20.0f, 0.0f, 30.0f ), new Vector3( 0.3f, 0.1f, 0.3f ), Combination.Union, MaterialIndex.Material0 )
    };

    private OccupancyDualContouring create( int resolution, Shape[] shapes ) {
        return this.createVoxelizer<OccupancyDualContouring>( resolution, QEFSolverType.SVD, shapes );
    }

    // Manifold Dual Contouring, section III: with a vertex for each piece of surface in a cell, the surface contoured
    // from a uniform grid is a manifold
    [Test]
    public void surfaceIsClosedAndManifold( [Values] Model model ) {
        var topology = this.contour( this.create( resolution, shapesOf( model ) ) );

        assertSingleSphere( topology );

        Assert.AreEqual( 0, topology.degenerateTriangles, $"degenerate triangles in: {topology}" );
    }

    // the three balls meet in creases, which put two sheets of surface in one cell on the finer grids
    [Test]
    public void surfaceIsManifoldOnEveryGridSize( [Values( 2, 3, 5, 6, 7 )] int resolution ) {
        assertManifold( this.contour( this.create( resolution, shapesOf( Model.ThreeBalls ) ) ) );
    }

    [Test]
    public void surfaceIsManifoldWhereACellHoldsTwoPiecesOfSurface( ) {
        assertTwoSpheres( this.contour( this.create( resolution, twoSpheres ) ) );
    }

    // Occupancy-Based Dual Contouring, appendix A.1.1: where two cells have a joined face in common, each takes its
    // vertices from its complement. The two hollows of each pair would otherwise hang together along an edge that four
    // polygons run along
    [Test]
    public void surfaceIsManifoldWhereTwoCellsHaveAJoinedFaceInCommon( ) {
        assertBlockWithSixHollows( this.contour( this.create( resolution, hollowsAcrossFaces ) ) );
    }

    [Test]
    public void surfaceOfTurnedShapesIsManifold( ) {
        assertManifold( this.contour( this.create( resolution, turnedShapes ) ) );
        assertManifold( this.contour( this.create( resolution, newShapes ) ) );
    }

    [Test]
    public void aTorusKeepsItsHandle( ) {
        var topology = this.contour( this.create( resolution, torus ) );

        assertManifold( topology );

        Assert.AreEqual( 1, topology.components,          $"separate pieces of surface in: {topology}" );
        Assert.AreEqual( 0, topology.eulerCharacteristic, $"euler characteristic of: {topology}" );
    }

    // the intersection-free polygons of section 5.4 are only that when every vertex is inside its own cell
    [Test]
    public void everyVertexLiesInItsCell( [Values] Model model ) {
        var voxels = this.voxelize( this.create( resolution, shapesOf( model ) ) ).voxels.ToArray( );

        Assert.IsNotEmpty( voxels, "no voxel has a vertex" );

        foreach( var voxel in voxels ) {
            for( var axis = 0; axis < 3; ++axis ) {
                Assert.GreaterOrEqual( voxel.vertex[axis], voxel.minimum[axis], $"vertex {voxel.vertex} of the cell at {voxel.center} along axis {axis}" );
                Assert.LessOrEqual(    voxel.vertex[axis], voxel.maximum[axis], $"vertex {voxel.vertex} of the cell at {voxel.center} along axis {axis}" );
            }
        }
    }

    // section 5.1: the point where the contour crosses an edge is found by halving the edge, and is the end of what is
    // left that is inside. The debug view shows it as the intersection of the edge
    [Test]
    public void intersectionsAreOnTheSurfaceToWithinTheLastHalving( [Values( 3, 15 )] int iterations ) {
        const float cell = 1.0f / 16;

        var shapes = shapesOf( Model.Sphere );
        var center = shapes[0].origin;
        var radius = shapes[0].extents.x;

        var voxelizer = this.create( resolution, shapes );

        voxelizer.edgeSearchIterations = iterations;

        var crossing = this.voxelize( voxelizer ).edges.Where( ( edge ) => edge.intersectsContour( ) ).ToArray( );

        Assert.IsNotEmpty( crossing, "no edge crosses the contour" );

        // an edge meets the sphere at an angle, so a step along the edge is at most that far toward the sphere
        var step = cell * Mathf.Pow( 2.0f, -iterations );

        foreach( var edge in crossing ) {
            var start = edge.corners[0].position;
            var end   = edge.corners[1].position;

            Assert.AreEqual( 0.0f, Vector3.Cross( edge.intersection - start, end - start ).magnitude, 1e-9f, $"the intersection {edge.intersection} is on its edge from {start}" );
            Assert.GreaterOrEqual( Vector3.Dot( edge.intersection - start, end - start ), 0.0f,                           $"the intersection {edge.intersection} is past the start of its edge" );
            Assert.LessOrEqual(    Vector3.Dot( edge.intersection - start, end - start ), ( end - start ).sqrMagnitude, $"the intersection {edge.intersection} is before the end of its edge" );

            var outside = Vector3.Distance( edge.intersection, center ) - radius;

            Assert.LessOrEqual(    outside, 1e-6f,           $"how far outside the sphere the intersection {edge.intersection} is" );
            Assert.GreaterOrEqual( outside, -step - 1e-6f, $"how far outside the sphere the intersection {edge.intersection} is" );
        }
    }

    // section 5.3: the normal of a plane is turned to point from the solid end of its edge to the empty one, and the
    // normal of a vertex is the sum of those of its planes. On a sphere that is away from its middle
    [Test]
    public void normalsPointOutOfTheShape( ) {
        var shapes = shapesOf( Model.Sphere );
        var voxels = this.voxelize( this.create( resolution, shapes ) ).voxels.ToArray( );

        Assert.IsNotEmpty( voxels, "no voxel has a vertex" );

        var worst = voxels.Min( ( voxel ) => Vector3.Dot( voxel.normal, ( voxel.vertex - shapes[0].origin ).normalized ) );

        // the cosine of five degrees
        Assert.GreaterOrEqual( worst, 0.996f, "the normal that points least away from the middle of the sphere, as the cosine of the angle between the two" );
    }

    // sections 5.2 and 5.3: with a single sample and a single halving a search across a face finds the surface to within
    // 0.4 of a cell instead of a ten-thousandth, and the vertices placed from it are farther from the sphere
    [Test]
    public void coarserFaceSearchesPlaceVerticesWorse( ) {
        var shapes = shapesOf( Model.Sphere );

        float farthestOf( int samples, int iterations ) {
            var voxelizer = this.create( resolution, shapes );

            voxelizer.faceSearchSamples    = samples;
            voxelizer.faceSearchIterations = iterations;

            return this.voxelize( voxelizer ).mesh.vertices.Max( ( vertex ) => Mathf.Abs( Vector3.Distance( vertex, shapes[0].origin ) - shapes[0].extents.x ) ) * 16.0f;
        }

        var fine   = farthestOf( 4, 11 );
        var coarse = farthestOf( 1, 1 );

        Assert.LessOrEqual( fine, 0.03f, "how far the farthest vertex is from the sphere with the searches of the paper, in cells" );
        Assert.Greater( coarse, 2.0f * fine, "how far the farthest vertex is from the sphere with coarse searches, in cells" );
    }

    // the debug view is given a voxel for every vertex of the contour: its cell, with that vertex in it. A cell that
    // holds two pieces of the contour is there twice
    [Test]
    public void everyVertexOfTheContourIsShownInAVoxelOfItsOwn( ) {
        var voxelization = this.voxelize( this.create( resolution, twoSpheres ) );

        var voxels = voxelization.voxels.ToList( );

        Assert.IsTrue( voxels.All( ( voxel ) => voxel.hasFeaturePoint ), "a voxel without a vertex" );

        CollectionAssert.AreEquivalent( voxelization.mesh.vertices, voxels.Select( ( voxel ) => voxel.vertex ).ToArray( ) );

        Assert.Less( voxels.Select( ( voxel ) => ( voxel.center, voxel.size ) ).Distinct( ).Count( ), voxels.Count, "no cell is shown twice" );
    }

    // a corner has no density here, only a side: -1 inside and 1 outside
    [Test]
    public void cornersShowWhichSideTheyAreOn( ) {
        var corners = this.voxelize( this.create( resolution, shapesOf( Model.Sphere ) ) ).corners.ToArray( );

        Assert.IsNotEmpty( corners, "no corners" );

        foreach( var corner in corners ) {
            Assert.AreEqual( corner.materialIndex != MaterialIndex.Void ? -1.0f : 1.0f, corner.density, $"density of the corner at {corner.position}" );
        }

        Assert.IsTrue( corners.Any( ( corner ) => corner.density < 0.0f ) && corners.Any( ( corner ) => corner.density > 0.0f ), "corners on both sides" );
    }

    // the corners, edges and voxels are copied out of the native buffers only when debug drawing is on or someone asked
    // for them
    [Test]
    public void debugInformationIsOnlyCopiedWhenWanted( ) {
        var voxelizer = this.create( resolution, shapesOf( Model.Sphere ) );

        voxelizer.keepDebugInformation = false;
        voxelizer.debugFlags           = Voxelizer.Debug.Off;

        var without = this.voxelize( voxelizer );

        Assert.IsEmpty( without.corners, "corners" );
        Assert.IsEmpty( without.edges,   "edges" );
        Assert.IsEmpty( without.voxels,  "voxels" );
        Assert.Greater( without.mesh.vertexCount, 0, "the mesh is there either way" );

        voxelizer.debugFlags = Voxelizer.Debug.Cells;

        var with = this.voxelize( voxelizer );

        Assert.IsNotEmpty( with.corners, "corners" );
        Assert.IsNotEmpty( with.edges,   "edges" );
        Assert.AreEqual( with.mesh.vertexCount, with.voxels.Count( ), "voxels" );

        CollectionAssert.AreEqual( without.mesh.vertices,  with.mesh.vertices,  "the mesh is the same either way" );
        CollectionAssert.AreEqual( without.mesh.triangles, with.mesh.triangles, "the mesh is the same either way" );
    }

}

public class BurstOccupancyDualContouringTopologyTest : OccupancyDualContouringTopologyTest {

    public BurstOccupancyDualContouringTopologyTest( ) {
        this.implementation = ImplementationType.Burst;
    }

}

public class OccupancyDualContouringImplementationTest : ContouringTest {

    // the paper's algorithm is written for Burst and the GPU alone
    [Test]
    public void thereIsNoManagedImplementation( ) {
        this.implementation = ImplementationType.Managed;

        var voxelizer = this.createVoxelizer<OccupancyDualContouring>( 3, QEFSolverType.SVD, new[] { sphere( Vector3.zero, 0.3f ) } );

        Assert.Throws<NotSupportedException>( ( ) => voxelizer.voxelize( voxelizer.GetComponentsInChildren<Volume>( ) ) );
    }

}
