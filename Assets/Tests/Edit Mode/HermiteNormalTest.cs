using System;
using System.Linq;
using NUnit.Framework;
using UnityEngine;

using ImplementationType = SurfaceExtractor.Implementation.Type;
using QEFSolverType      = QEFSolver.Type;

// Dual Contouring of Hermite Data, section 2: the edges with a sign change are "tagged by Hermite data consisting of
// exact intersection points and normals from the contour". These tests hold the normals to that on shapes whose
// normals are known in closed form
public class HermiteNormalTest : ContouringTest {

    private const float tolerance = 0.1f; // degrees

    // the angle between two directions. Going through the cross product keeps the precision that the arc cosine of a
    // dot product close to one would lose
    private static float angle( Vector3 direction, Vector3 other ) {
        return Mathf.Atan2( Vector3.Cross( direction, other ).magnitude, Vector3.Dot( direction, other ) ) * Mathf.Rad2Deg;
    }

    // contours the shape on 16^3 cells and compares the normal of every hermite sample with the exact one
    private void assertNormals( ImplementationType implementation, Shape shape, Func<Vector3, Vector3> exactNormal ) {
        var voxelizer = this.createUniformVoxelizer( 4, implementation, QEFSolverType.SVD, new[] { shape } );

        var deviations = this.voxelize( voxelizer ).edges.Where(
            ( edge ) => edge.intersectsContour( )
        ).Select(
            ( edge ) => angle( edge.normal, exactNormal( edge.intersection ) )
        ).ToArray( );

        Assert.Greater( deviations.Length, 0, "the shape has no hermite samples" );

        var inexact = deviations.Count( ( deviation ) => !( deviation <= tolerance ) );

        Assert.AreEqual( 0, inexact, $"{inexact} of {deviations.Length} normals deviate by more than {tolerance} degrees, the worst by {deviations.Max( )}" );
    }

    [Test]
    public void normalsOfSphereAreRadial( [Values( ImplementationType.Managed, ImplementationType.GPU )] ImplementationType implementation ) {
        var center = new Vector3( 0.013f, -0.021f, 0.017f );

        this.assertNormals( implementation, sphere( center, 0.3f ), ( intersection ) => intersection - center );
    }

    // an edge of the grid can only cross a face of this cube at least 0.0375 away from the edges of the cube, the grid
    // lines closest to them are at 0.3125. Every normal should therefore be exactly the normal of a face
    [Test]
    public void normalsOfCubeAreAxisAligned( [Values( ImplementationType.Managed, ImplementationType.GPU )] ImplementationType implementation ) {
        var extent = 0.35f;

        this.assertNormals(
            implementation,
            cube( Vector3.zero, extent ),
            ( intersection ) => {
                // the intersection lies on the face it is closest to
                var distances = new Vector3( Mathf.Abs( intersection.x ), Mathf.Abs( intersection.y ), Mathf.Abs( intersection.z ) ) - ( Vector3.one * extent );

                if( distances.x >= distances.y && distances.x >= distances.z ) {
                    return new Vector3( Mathf.Sign( intersection.x ), 0.0f, 0.0f );
                }
                if( distances.y >= distances.z ) {
                    return new Vector3( 0.0f, Mathf.Sign( intersection.y ), 0.0f );
                }
                return new Vector3( 0.0f, 0.0f, Mathf.Sign( intersection.z ) );
            }
        );
    }

}
