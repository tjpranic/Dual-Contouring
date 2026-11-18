using System.Linq;
using NUnit.Framework;
using UnityEngine;

using ImplementationType = SurfaceExtractor.Implementation.Type;
using QEFSolverType      = QEFSolver.Type;

// the tests of the vertex clustering, which run for each implementation in the fixtures at the end
public abstract class ManifoldDualContouringTopologyTest : TopologyTest {

    private const int resolution = 5; // 16^3 leaves

    // a 0.08 thick beam that runs through the whole grid along x, centered in one row of the 1 / 8 wide cells above the
    // leaves. Every cell it passes through, at any level of the octree, holds a tube that is open at both ends
    internal static readonly Shape[] beam = {
        cuboid( new Vector3( 0.0f, 0.0625f, 0.0625f ), new Vector3( 1.0f, 0.04f, 0.04f ) )
    };

    // two boxes with a gap of 0.03 between them, around the plane x = 0.0625 where two leaves meet. No leaf holds more
    // than one piece of surface, but every larger cell along the gap holds a face of either box
    internal static readonly Shape[] twoBoxes = {
        cuboid( new Vector3( -0.175f, 0.0f, 0.0f ), new Vector3( 0.225f, 0.4f, 0.4f ) ), // -0.4  to 0.05 along x
        cuboid( new Vector3(  0.24f,  0.0f, 0.0f ), new Vector3( 0.16f,  0.3f, 0.3f ) )  //  0.08 to 0.4  along x
    };

    // a box with a thin finger standing on it. On its way up the finger passes through the 1 / 8 wide cells above the
    // leaves as a tube, which no vertex may stand in for. Every octant of the grid holds a disk: a corner of the box,
    // and in one of them the finger on top of it
    internal static readonly Shape[] boxWithFinger = {
        cuboid( new Vector3( 0.0f,    0.0f,    -0.2f ), new Vector3( 0.4f,  0.4f,  0.23f ) ),
        cuboid( new Vector3( 0.1875f, 0.1875f,  0.2f ), new Vector3( 0.04f, 0.04f, 0.2f  ) )
    };

    // an error threshold at which part of the vertices is clustered on every model, with either solver: both report
    // the value of the QEF at the vertex they found
    private const float clusteringThreshold = 2e-4f;

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

        assertTwoSpheres( this.contour( voxelizer ) );
    }

    // the vertices of the two pieces go up the vertex tree separately, and the spheres stay apart
    [Test]
    public void clusteredSurfaceIsManifoldWhereALeafHoldsTwoPiecesOfSurface( [Values] QEFSolverType solver ) {
        var voxelizer = this.create( resolution, solver, twoSpheres, clusteringThreshold );

        assertTwoSpheres( this.contour( voxelizer ) );
    }

    // Occupancy-Based Dual Contouring, appendix A.1.1: the table of dual marching cubes by itself leaves an edge that
    // four polygons run along between two cells that have a joined face in common. The two hollows of each pair would
    // hang together along such an edge
    [Test]
    public void surfaceIsManifoldWhereTwoLeavesHaveAJoinedFaceInCommon( ) {
        var voxelizer = this.create( resolution, QEFSolverType.SVD, hollowsAcrossFaces, 0.0f );

        assertBlockWithSixHollows( this.contour( voxelizer ) );
    }

    [Test]
    public void clusteredSurfaceIsManifoldWhereTwoLeavesHaveAJoinedFaceInCommon( [Values] QEFSolverType solver ) {
        var voxelizer = this.create( resolution, solver, hollowsAcrossFaces, clusteringThreshold );

        assertManifold( this.contour( voxelizer ) );
    }

    [Test]
    public void materialBoundaryIsFollowed( ) {
        this.assertBandFollowsMaterialBoundary( this.create( resolution, QEFSolverType.SVD, slabWithBand, 0.0f ) );
    }

    // the boundary is part of the QEFs that are combined during clustering, so a vertex only stands in for others
    // when it can stay on the boundary as well as on the surface
    [Test]
    public void clusteredMaterialBoundaryIsFollowed( ) {
        this.assertBandFollowsMaterialBoundary( this.create( resolution, QEFSolverType.SVD, slabWithBand, clusteringThreshold ) );
    }

    // the debug visualization is given a voxel for every vertex on the contour: the cell the vertex was created in,
    // with that vertex in it. A cell that holds two pieces of the contour is there twice
    [Test]
    public void everyVertexOfTheContourIsShownInAVoxelOfItsOwn( ) {
        var voxelization = this.voxelize( this.create( resolution, QEFSolverType.SVD, twoSpheres, 0.0f ) );

        var voxels = voxelization.voxels.ToList( );

        Assert.IsTrue( voxels.All( ( voxel ) => voxel.hasFeaturePoint ), "a voxel without a vertex" );

        CollectionAssert.AreEquivalent( voxelization.mesh.vertices, voxels.Select( ( voxel ) => voxel.vertex ).ToArray( ) );

        Assert.Less( voxels.Select( ( voxel ) => ( voxel.center, voxel.size ) ).Distinct( ).Count( ), voxels.Count, "no cell is shown twice" );
    }

    // section VI: "multiple contour components within a same octree cell simplify in an independent manner, hence
    // allowing flat regions to maximally collapse even if in the vicinity of other geometry". The larger box collapses
    // to its corners, one in each octant of the grid, as if the other box was not there. The smaller box reaches from
    // one side of its octants to the other, with opposite faces that no single vertex fits, and stops a level below:
    // one vertex in each of the 2 x 4 x 4 cells of 1 / 4 that it passes through
    [Test]
    public void separateSurfacesInACellClusterIndependently( ) {
        var voxelizer = this.create( resolution, QEFSolverType.SVD, twoBoxes, 1e-4f );

        var topology = this.contour( voxelizer );

        assertTwoSpheres( topology );

        Assert.AreEqual( 8 + 32, topology.vertices, $"vertices of: {topology}" );
    }

    // proposition 1: clustering under the manifold criterion generates a closed 2-manifold, and preserves the genus of
    // the surface
    [Test]
    public void clusteredSurfaceIsClosedAndManifold( [Values] Model model, [Values] QEFSolverType solver ) {
        var voxelizer = this.create( resolution, solver, shapesOf( model ), clusteringThreshold );

        assertSingleSphere( this.contour( voxelizer ) );
    }

    // section IV.B: "If the resulting polygon collapses to an edge or a vertex, then we discard that polygon"
    [Test]
    public void clusteredSurfaceHasNoDegenerateTriangles( [Values] Model model, [Values] QEFSolverType solver ) {
        var voxelizer = this.create( resolution, solver, shapesOf( model ), clusteringThreshold );

        var topology = this.contour( voxelizer );

        Assert.AreEqual( 0, topology.degenerateTriangles, $"degenerate triangles in: {topology}" );
    }

    // section IV.A: a clustered vertex is collapsible "if the error for this vertex is less than the threshold", at
    // every level of the octree. A cube centered in the grid has an octant around each of its corners, and the three
    // faces in an octant meet in that corner without any error: the corners should be all that is left
    [Test]
    public void cubeClustersToItsCorners( ) {
        var voxelizer = this.create( resolution, QEFSolverType.SVD, new[] { cube( Vector3.zero, 0.35f ) }, 1e-4f );

        var topology = this.contour( voxelizer );

        assertSingleSphere( topology );

        Assert.AreEqual( 8,  topology.vertices,  $"vertices of: {topology}" );
        Assert.AreEqual( 12, topology.triangles, $"triangles of: {topology}" );
    }

    // the octants of a cube of 0.6 have thousands of samples at a resolution of 7. Their error is zero, which the sums in
    // floats rounded to 3e-5, so that the threshold of the interactive editor's scene, 1e-5, left 56 vertices
    [Test]
    public void cubeOfAnySizeClustersToItsCorners( ) {
        var voxelizer = this.create( 7, QEFSolverType.SVD, new[] { cube( Vector3.zero, 0.3f ) }, 1e-5f );

        var topology = this.contour( voxelizer );

        assertSingleSphere( topology );

        Assert.AreEqual( 8, topology.vertices, $"vertices of: {topology}" );
    }

    // a cube turned off every axis, whose samples are not exact: under the sum the octants refuse it at 1e-5, which left
    // 246 vertices, and the root mean square of their distances is a few hundredths of a cell
    [Test]
    public void rotatedCubeClustersToItsCornersUnderTheDistance( ) {
        var rotation  = Quaternion.Euler( 20.0f, 30.0f, 10.0f );
        var voxelizer = this.create( 7, QEFSolverType.SVD, new[] { cube( Vector3.zero, 0.25f ).rotated( rotation ) }, 0.0f );

        voxelizer.errorMeasure  = ManifoldDualContouring.ErrorMeasure.Distance;
        voxelizer.errorDistance = 0.1f;

        var mesh     = this.voxelize( voxelizer ).mesh;
        var topology = MeshTopology.of( mesh );

        assertSingleSphere( topology );

        Assert.AreEqual( 8, topology.vertices, $"vertices of: {topology}" );

        // each of them in a corner, to within a tenth of a cell
        var corners = Enumerable.Range( 0, 8 ).Select(
            ( index ) => rotation * new Vector3( ( index & 1 ) == 0 ? -0.25f : 0.25f, ( index & 2 ) == 0 ? -0.25f : 0.25f, ( index & 4 ) == 0 ? -0.25f : 0.25f )
        ).ToArray( );

        foreach( var vertex in mesh.vertices ) {
            var nearest = corners.Min( ( corner ) => Vector3.Distance( corner, vertex ) );

            Assert.Less( nearest, 0.1f * voxelizer.cellSize, $"{vertex} is {nearest / voxelizer.cellSize} cells from the nearest corner" );
        }
    }

    // with the error threshold out of the way only the manifold criterion stops clustering. Every Model is a closed
    // surface around the center of the grid: the root holds all of it, which is not a disk, and each of the eight
    // octants holds a part that is
    [Test]
    public void clusteringGoesAsFarAsTheManifoldCriterionAllows( [Values] Model model, [Values] QEFSolverType solver ) {
        var voxelizer = this.create( resolution, solver, shapesOf( model ), float.MaxValue );

        var topology = this.contour( voxelizer );

        assertSingleSphere( topology );

        Assert.AreEqual( 8, topology.vertices, $"vertices of: {topology}" );
    }

    // section VI: "our manifold criterion may be able to determine that a safe simplification occurs later in vertex
    // clustering even if unsafe collapses occurred previously"
    [Test]
    public void clusteringContinuesAboveUnsafeClusters( [Values] QEFSolverType solver ) {
        var voxelizer = this.create( resolution, solver, boxWithFinger, float.MaxValue );

        var topology = this.contour( voxelizer );

        assertSingleSphere( topology );

        Assert.AreEqual( 8, topology.vertices, $"vertices of: {topology}" );
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

public class ManagedManifoldDualContouringTopologyTest : ManifoldDualContouringTopologyTest {

    public ManagedManifoldDualContouringTopologyTest( ) {
        this.implementation = ImplementationType.Managed;
    }

    // under the distance a cluster comes to its own samples' distance or more, no cluster comes to less than the
    // clusters among its members, and the leaves come to nothing. The vertex trees are the managed implementation's,
    // which the Burst one is held to, so it runs for that one alone
    [Test]
    public void clusterDistancesNeverShrinkUpTheVertexTree( [Values] Model model ) {
        // at a distance of zero no cluster stands in for anything, so the vertices on the contour are those of the
        // leaves, and every cluster is on the way up from one of them
        var voxelizer = this.createVoxelizer<ManifoldDualContouring>( 5, QEFSolverType.SVD, shapesOf( model ) );

        voxelizer.enforceManifold = true;

        voxelizer.errorMeasure  = ManifoldDualContouring.ErrorMeasure.Distance;
        voxelizer.errorDistance = 0.0f;

        var leaves = this.voxelize( voxelizer ).voxels.Cast<ManifoldDualContouring.Vertex>( ).ToList( );

        Assert.Greater( leaves.Count, 0 );

        var clusters = 0;

        foreach( var leaf in leaves ) {
            Assert.AreEqual( SurfaceExtractor.Voxel.Type.Leaf, leaf.voxel.type, "a vertex on the contour is not a leaf's" );
            Assert.AreEqual( 0.0f, leaf.distance, "a leaf has no distance" );

            for( var vertex = leaf.parent; vertex != null; vertex = vertex.parent ) {
                ++clusters;

                Assert.GreaterOrEqual( vertex.distance, Mathf.Sqrt( vertex.error / vertex.qef.intersectionCount ), "a cluster comes to less than its own samples" );

                if( vertex.parent != null ) {
                    Assert.GreaterOrEqual( vertex.parent.distance, vertex.distance, "a cluster comes to less than one of its members" );
                }
            }
        }

        Assert.Greater( clusters, 0 );
    }

}

public class BurstManifoldDualContouringTopologyTest : ManifoldDualContouringTopologyTest {

    public BurstManifoldDualContouringTopologyTest( ) {
        this.implementation = ImplementationType.Burst;
    }

}
