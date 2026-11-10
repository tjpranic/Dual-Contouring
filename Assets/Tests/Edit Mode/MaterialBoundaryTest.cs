using System.Linq;
using NUnit.Framework;
using UnityEngine;

using Combination               = DensityFunction.Combination;
using MaterialIndex             = SurfaceExtractor.MaterialIndex;
using IntersectionApproximation = SurfaceExtractor.IntersectionApproximation;
using VoxelType                 = SurfaceExtractor.Voxel.Type;
using QEFSolverType             = QEFSolver.Type;

// Dual Contouring of Hermite Data, section 2.3: "edges that exhibit index changes are also tagged by exact intersection
// points and normals". Between two solid corners of different materials that is the point where one material turns
// into the other, and the normal of the boundary between them
public class MaterialBoundaryTest : ContouringTest {

    private const MaterialIndex _ = MaterialIndex.Void;
    private const MaterialIndex A = MaterialIndex.Material0;
    private const MaterialIndex B = MaterialIndex.Material1;

    private const float cell = 0.0625f;

    // an edge along x that starts at the given position and is as long as a cell of a grid of 16^3
    private static SurfaceExtractor.Edge edgeFrom( Vector3 start ) {
        return new AdaptiveDualContouring.Voxel( VoxelType.Leaf, 0, start + ( Vector3.one * cell / 2 ), Vector3.one * cell ).edges[0];
    }

    private static SurfaceExtractor.Edge edgeBetween( MaterialIndex materialIndex, MaterialIndex other ) {
        var edge = edgeFrom( Vector3.zero );

        edge.corners[0].materialIndex = materialIndex;
        edge.corners[1].materialIndex = other;

        return edge;
    }

    // the boundary that the edge from the given position crosses in the given shapes
    private ( Vector3 intersection, Vector3 normal ) boundaryOf( Shape[] shapes, Vector3 start, IntersectionApproximation intersectionApproximation ) {
        var densityFunctions = this.createVoxelizer<AdaptiveDualContouring>( 5, QEFSolverType.SVD, shapes ).GetComponentsInChildren<Volume>( );

        var edge = edgeFrom( start );

        foreach( var corner in edge.corners ) {
            corner.materialIndex = SurfaceExtractor.calculateMaterial( corner.position, densityFunctions );
        }

        Assert.IsTrue( SurfaceExtractor.crossesMaterialBoundary( edge ), "the edge does not cross a material boundary" );

        return SurfaceExtractor.calculateMaterialBoundary( edge, densityFunctions, intersectionApproximation, 12 );
    }

    [Test]
    public void edgeBetweenSolidsOfDifferentMaterialsCrossesABoundary( ) {
        Assert.IsTrue(  SurfaceExtractor.crossesMaterialBoundary( edgeBetween( A, B ) ), "two materials" );
        Assert.IsFalse( SurfaceExtractor.crossesMaterialBoundary( edgeBetween( A, A ) ), "the same material" );
        Assert.IsFalse( SurfaceExtractor.crossesMaterialBoundary( edgeBetween( _, _ ) ), "empty space" );

        // that is an edge the contour itself intersects
        Assert.IsFalse( SurfaceExtractor.crossesMaterialBoundary( edgeBetween( _, B ) ), "empty and solid" );
    }

    // the highest material of a corner is the one that is drawn
    [Test]
    public void materialOfACornerIsItsHighest( ) {
        Assert.IsTrue(  SurfaceExtractor.crossesMaterialBoundary( edgeBetween( A, A | B ) ), "the higher material is added" );
        Assert.IsFalse( SurfaceExtractor.crossesMaterialBoundary( edgeBetween( B, A | B ) ), "a lower material is added" );
    }

    // a band of a second material that ends at x = -0.13, inside a slab
    [Test]
    public void boundaryOfABoxIsItsFace( [Values] IntersectionApproximation intersectionApproximation ) {
        var shapes = new[] {
            cuboid( Vector3.zero,                     new Vector3( 0.4f,  0.1f, 0.4f ) ),
            cuboid( new Vector3( 0.02f, 0.0f, 0.0f ), new Vector3( 0.15f, 0.1f, 0.4f ), Combination.Union, B )
        };

        var ( intersection, normal ) = this.boundaryOf( shapes, new Vector3( -0.1875f, 0.0f, 0.0f ), intersectionApproximation );

        Assert.Less( Vector3.Distance( new Vector3( -0.13f, 0.0f, 0.0f ), intersection ), 1e-4f, $"the intersection is {intersection:F5}" );

        Assert.AreEqual( 1.0f, Mathf.Abs( Vector3.Dot( Vector3.right, normal ) ), 1e-4f, $"the normal is {normal:F5}" );
    }

    // a ball of a second material inside a cube
    [Test]
    public void boundaryOfABallIsItsSurface( ) {
        var center = new Vector3( 0.01f, 0.02f, 0.03f );
        var radius = 0.2f;

        var shapes = new[] {
            cube   ( Vector3.zero, 0.4f ),
            sphere ( center, radius, Combination.Union, B )
        };

        var ( intersection, normal ) = this.boundaryOf( shapes, new Vector3( 0.125f, 0.125f, 0.0625f ), IntersectionApproximation.BinarySearch );

        Assert.AreEqual( radius, Vector3.Distance( center, intersection ), 1e-4f, $"the intersection is {intersection:F5}" );

        Assert.AreEqual( 1.0f, Mathf.Abs( Vector3.Dot( Vector3.Normalize( intersection - center ), normal ) ), 1e-4f, $"the normal is {normal:F5}" );
    }

    // the cube is cut away again on one side of the ball: the boundary between the two materials is still the surface
    // of the ball, and the volumes of the cube have no part in it
    [Test]
    public void boundaryIsThatOfTheHigherMaterialAlone( ) {
        var center = new Vector3( 0.01f, 0.02f, 0.03f );
        var radius = 0.2f;

        var shapes = new[] {
            cube   ( Vector3.zero, 0.4f ),
            sphere ( center, radius, Combination.Union, B ),
            cube   ( new Vector3( 0.0f, 0.6f, 0.0f ), 0.4f, Combination.Subtraction )
        };

        var ( intersection, _ ) = this.boundaryOf( shapes, new Vector3( 0.125f, 0.125f, 0.0625f ), IntersectionApproximation.BinarySearch );

        Assert.AreEqual( radius, Vector3.Distance( center, intersection ), 1e-4f, $"the intersection is {intersection:F5}" );
    }

}
