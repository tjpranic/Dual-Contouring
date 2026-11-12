using System;
using System.Collections.Generic;
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

    // every triangle goes around the way its polygon does, which faces the empty end of the polygon's edge: out of the
    // shape. That holds for the triangles of both diagonals and for the four around a point on an edge, which the sphere
    // cutaway has
    [Test]
    public void trianglesFaceOutOfTheShape( ) {
        var sphere = shapesOf( Model.Sphere );
        var mesh   = this.voxelize( this.create( resolution, sphere ) ).mesh;

        var vertices  = mesh.vertices;
        var triangles = mesh.triangles;

        for( var triangle = 0; triangle < triangles.Length; triangle += 3 ) {
            var a = vertices[triangles[triangle]];
            var b = vertices[triangles[triangle + 1]];
            var c = vertices[triangles[triangle + 2]];

            Assert.Greater( Vector3.Dot( Vector3.Cross( b - a, c - a ).normalized, ( ( ( a + b + c ) / 3.0f ) - sphere[0].origin ).normalized ), 0.8f, $"how far the triangle at {a} faces away from the middle of the sphere" );
        }

        // the cutaway is closed, so with every triangle facing out the volume its triangles enclose is positive
        var cutaway = this.voxelize( this.create( resolution, shapesOf( Model.SphereCutaway ) ) ).mesh;

        vertices  = cutaway.vertices;
        triangles = cutaway.triangles;

        var volume = 0.0f;

        for( var triangle = 0; triangle < triangles.Length; triangle += 3 ) {
            volume += Vector3.Dot( vertices[triangles[triangle]], Vector3.Cross( vertices[triangles[triangle + 1]], vertices[triangles[triangle + 2]] ) ) / 6.0f;
        }

        // half a ball of radius 0.5 with a ball of 0.4 in it
        Assert.AreEqual( ( 2.0f / 3.0f * Mathf.PI * 0.125f ) + ( 2.0f / 3.0f * Mathf.PI * 0.064f ), volume, 0.02f, "the volume the triangles of the cutaway enclose" );
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

    // section 5.4: with every vertex inside its cell and every polygon split so that its triangles stay in the envelope
    // around its edge, no two triangles pass through each other
    [Test]
    public void noTwoTrianglesCross( [Values] Model model, [Values( 4, 5 )] int resolution ) {
        Assert.AreEqual( 0, MeshIntersections.count( this.voxelize( this.create( resolution, shapesOf( model ) ) ).mesh ), "pairs of triangles that pass through each other" );
    }

    // the paper's argument is for cells with one vertex. Shapes that meet and cut into each other leave cells with
    // several, whose vertices are kept on their own sides of the planes between their pieces of the contour
    // (DualMarchingCubes.lookupSeparation). Without those planes the turned shapes have 32 crossings on the finer of
    // these grids and the new shapes 7, every one between triangles with vertices in the same cell
    [Test]
    public void noTwoTrianglesCrossWhereCellsHaveSeveralVertices( [Values( 4, 5 )] int resolution ) {
        Assert.AreEqual( 0, MeshIntersections.count( this.voxelize( this.create( resolution, turnedShapes ) ).mesh ),       "pairs of triangles that pass through each other on the turned shapes" );
        Assert.AreEqual( 0, MeshIntersections.count( this.voxelize( this.create( resolution, newShapes ) ).mesh ),          "pairs of triangles that pass through each other on the new shapes" );
        Assert.AreEqual( 0, MeshIntersections.count( this.voxelize( this.create( resolution, twoSpheres ) ).mesh ),         "pairs of triangles that pass through each other on the two spheres" );
        Assert.AreEqual( 0, MeshIntersections.count( this.voxelize( this.create( resolution, hollowsAcrossFaces ) ).mesh ), "pairs of triangles that pass through each other on the block with hollows" );
    }

    // each vertex of a cell with several is on its own side of the plane between it and each of the others, by half the
    // margin at least
    [Test]
    public void theVerticesOfACellStayOnTheirOwnSides( ) {
        var voxels  = this.voxelize( this.create( 5, turnedShapes ) ).voxels.ToArray( );
        var several = voxels.GroupBy( ( voxel ) => voxel.center ).Where( ( group ) => group.Count( ) > 1 ).ToArray( );

        Assert.Greater( several.Length, 10, "cells with several vertices" );

        foreach( var cell in several ) {
            var vertices = cell.ToArray( );

            // the debug view lists the vertices of a cell in the order the table numbers them
            var solidCorners = DualMarchingCubes.solidCornersOf( vertices[0].corners.Select( ( corner ) => corner.materialIndex ).ToArray( ) );

            Assert.AreEqual( DualMarchingCubes.lookupVertexEdges( solidCorners ).Length, vertices.Length, $"vertices of the cell at {cell.Key}" );

            for( var vertex = 0; vertex < vertices.Length; ++vertex ) {
                for( var other = 0; other < vertices.Length; ++other ) {
                    if( !DualMarchingCubes.lookupSeparation( solidCorners, vertex, other, out var normal, out var twiceOffset ) ) {
                        continue;
                    }

                    var inCell = ( vertices[vertex].vertex - vertices[vertex].minimum ).divide( vertices[vertex].size );
                    var along  = Vector3.Dot( new Vector3( normal.x, normal.y, normal.z ), inCell );

                    Assert.LessOrEqual( along, ( twiceOffset * 0.5f ) - ( 0.5f * BurstOccupancy.margin ), $"vertex {vertex} of the cell at {cell.Key} against the plane between it and vertex {other}" );
                }
            }
        }
    }

    // appendix A.1.5: where neither diagonal keeps the triangles of a polygon in its envelope, it becomes four
    // triangles around the 1D point of its edge, which is then a vertex of the mesh without a cell of its own. The
    // surface stays closed and manifold, as surfaceIsClosedAndManifold holds for this model
    [Test]
    public void aPolygonNoDiagonalFitsBecomesFourTrianglesAroundThePointOnItsEdge( ) {
        const float cell = 1.0f / 16;

        var voxelizer    = this.create( resolution, shapesOf( Model.SphereCutaway ) );
        var voxelization = this.voxelize( voxelizer );

        var ofCells  = new HashSet<Vector3>( voxelization.voxels.Select( ( voxel ) => voxel.vertex ) );
        var onEdges  = voxelization.mesh.vertices.Where( ( vertex ) => !ofCells.Contains( vertex ) ).ToArray( );
        var crossing = new HashSet<Vector3>( voxelization.edges.Where( ( edge ) => edge.intersectsContour( ) ).Select( ( edge ) => edge.intersection ) );

        Assert.Greater( onEdges.Length, 0, "vertices of the mesh that are not the vertex of a cell" );

        foreach( var vertex in onEdges ) {
            Assert.IsTrue( crossing.Contains( vertex ), $"the vertex at {vertex / cell} cells is the 1D point of an edge" );
        }

        // four triangles in place of two for each
        var triangles = voxelization.mesh.triangles;

        foreach( var vertex in onEdges ) {
            var index = Array.IndexOf( voxelization.mesh.vertices, vertex );

            Assert.AreEqual( 4, Enumerable.Range( 0, triangles.Length / 3 ).Count( ( triangle ) => triangles[triangle * 3] == index || triangles[( triangle * 3 ) + 1] == index || triangles[( triangle * 3 ) + 2] == index ), $"triangles around the vertex at {vertex / cell} cells" );
        }

        // with the split off, every polygon is two triangles between the vertices of cells
        voxelizer.splitQuads = false;

        var plain = this.voxelize( voxelizer );

        Assert.AreEqual( plain.voxels.Count( ), plain.mesh.vertexCount, "vertices of the mesh without the split" );
        Assert.AreEqual( plain.mesh.triangles.Length + ( 6 * onEdges.Length ), triangles.Length, "indices of the mesh with the split, against without" );
        Assert.Greater( MeshIntersections.count( plain.mesh ), 0, "pairs of triangles that pass through each other without the split" );
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
