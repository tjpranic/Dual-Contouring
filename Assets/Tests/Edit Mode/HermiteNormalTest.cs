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

    // contours the shape on 16^3 cells and compares the normal of every hermite sample with the exact one. Where the
    // exact normal is null, the sample is left out
    private void assertNormals( ImplementationType implementation, Shape shape, Func<Vector3, Vector3?> exactNormal ) {
        var voxelizer = this.createUniformVoxelizer( 4, implementation, QEFSolverType.SVD, new[] { shape } );

        var deviations = this.voxelize( voxelizer ).edges.Where(
            ( edge ) => edge.intersectsContour( ) && exactNormal( edge.intersection ).HasValue
        ).Select(
            ( edge ) => angle( edge.normal, exactNormal( edge.intersection ).Value )
        ).ToArray( );

        Assert.Greater( deviations.Length, 0, "the shape has no hermite samples" );

        var inexact = deviations.Count( ( deviation ) => !( deviation <= tolerance ) );

        Assert.AreEqual( 0, inexact, $"{inexact} of {deviations.Length} normals deviate by more than {tolerance} degrees, the worst by {deviations.Max( )}" );
    }

    [Test]
    public void normalsOfSphereAreRadial( [Values] ImplementationType implementation ) {
        var center = new Vector3( 0.013f, -0.021f, 0.017f );

        this.assertNormals( implementation, sphere( center, 0.3f ), ( intersection ) => intersection - center );
    }

    // an edge of the grid can only cross a face of this cube at least 0.0375 away from the edges of the cube, the grid
    // lines closest to them are at 0.3125. Every normal should therefore be exactly the normal of a face
    [Test]
    public void normalsOfCubeAreAxisAligned( [Values] ImplementationType implementation ) {
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

    // a turned cube's faces cross the grid anywhere, so some intersections fall close to its edges. A normal is a central
    // difference over 1e-4 along each axis. An intersection is only found to within 1 / 1024 of the surface, on either
    // side of it. Inside the cube the density is the distance to the nearest face, outside it near a face the distance
    // to that face. So an intersection's normal is that of one face only while, by more than twice the step, that face
    // is the nearest one inside the cube, or the intersection lies within the face outside it. The others are left out,
    // and every normal left should be exactly the turned normal of its face
    [Test]
    public void normalsOfRotatedCubeAreTheNormalsOfItsFaces( [Values] ImplementationType implementation ) {
        // turned, it reaches at most sqrt(3) * 0.25 = 0.433 from its center, inside the grid
        var extent   = 0.25f;
        var rotation = Quaternion.Euler( 30.0f, -60.0f, 75.0f );

        this.assertNormals(
            implementation,
            cube( Vector3.zero, extent ).rotated( rotation ),
            ( intersection ) => {
                // how far outside each pair of faces the intersection lies, negative inside them
                var local     = Quaternion.Inverse( rotation ) * intersection;
                var distances = new[] { Mathf.Abs( local.x ), Mathf.Abs( local.y ), Mathf.Abs( local.z ) }.Select( ( coordinate ) => coordinate - extent ).ToArray( );
                var axes      = Enumerable.Range( 0, 3 ).OrderByDescending( ( axis ) => distances[axis] ).ToArray( );

                if( distances[axes[1]] > Mathf.Min( distances[axes[0]], 0.0f ) - 2e-4f ) {
                    return null;
                }

                var normal = Vector3.zero;

                normal[axes[0]] = Mathf.Sign( local[axes[0]] );

                return rotation * normal;
            }
        );
    }

    // a turned pyramid and octahedron, whose faces lean every way. Inside a shape with flat faces the density is the
    // distance to the nearest face's plane, outside it near a face, where the point lies over the face, the distance to
    // that face. So as with the turned cube, an intersection's normal is that of one face only while, by more than twice
    // the step, that face's plane is the nearest inside, or the point lies over the face outside. The others are left out
    [Test]
    public void normalsOfFlatFacedShapesAreTheNormalsOfTheirFaces( [Values] ImplementationType implementation, [Values( DensityFunction.Type.Pyramid, DensityFunction.Type.Octahedron )] DensityFunction.Type type ) {
        // turned, they reach at most 0.41 from their center, inside the grid
        var extents  = new Vector3( 0.25f, 0.2f, 0.25f );
        var rotation = Quaternion.Euler( 30.0f, -60.0f, 75.0f );
        var margin   = 2e-4f;

        var ( corners, faces ) = ShapeDistanceTest.polyhedronOf( type, extents );

        // each face's outward normal and corners
        var planes = faces.Select( ( face ) => {
            var points = face.Select( ( index ) => corners[index] ).ToArray( );
            var normal = Vector3.Cross( points[1] - points[0], points[2] - points[0] ).normalized;

            return ( normal: Vector3.Dot( normal, points[0] ) < 0.0f ? -normal : normal, points );
        } ).ToArray( );

        // how far inside the face a point of its plane lies, from the nearest of its edges
        float insideFace( Vector3 normal, Vector3[] points, Vector3 point ) {
            var nearest = float.PositiveInfinity;

            for( var index = 0; index < points.Length; ++index ) {
                var start  = points[index];
                var along  = ( points[( index + 1 ) % points.Length] - start ).normalized;
                var inward = Vector3.Cross( normal, along );

                // inward is the way to the face's other corners
                if( Vector3.Dot( inward, points[( index + 2 ) % points.Length] - start ) < 0.0f ) {
                    inward = -inward;
                }

                nearest = Mathf.Min( nearest, Vector3.Dot( inward, point - start ) );
            }

            return nearest;
        }

        this.assertNormals(
            implementation,
            new Shape( type, Vector3.zero, rotation, extents, DensityFunction.Combination.Union, SurfaceExtractor.MaterialIndex.Material0 ),
            ( intersection ) => {
                var local   = Quaternion.Inverse( rotation ) * intersection;
                var heights = planes.Select( ( plane ) => Vector3.Dot( plane.normal, local - plane.points[0] ) ).ToArray( );
                var order   = Enumerable.Range( 0, planes.Length ).OrderByDescending( ( face ) => heights[face] ).ToArray( );

                if( heights[order[0]] <= 0.0f ) {
                    return heights[order[0]] - heights[order[1]] > margin ? rotation * planes[order[0]].normal : null;
                }

                var nearest = planes[order[0]];

                return insideFace( nearest.normal, nearest.points, local - ( nearest.normal * heights[order[0]] ) ) > margin ? rotation * nearest.normal : null;
            }
        );
    }

}
