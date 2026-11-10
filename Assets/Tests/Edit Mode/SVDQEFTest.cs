using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using UnityEngine;

using VoxelType = SurfaceExtractor.Voxel.Type;

// Dual Contouring: "The Secret Sauce", section 2.1: the vertex of a cell is the minimizer of its QEF that lies closest
// to the mass point of the hermite samples, x = p + (AtA)^+ (AtB - AtA p), where the pseudo inverse truncates singular
// values with an absolute magnitude under 0.1
public class SVDQEFTest {

    public enum Orientation {
        AxisAligned,
        Rotated
    }

    private const float epsilon = 1e-4f;

    private static readonly Vector3 corner = new( 0.1f, 0.2f, 0.3f );

    private static readonly Vector3[] axes = { Vector3.right, Vector3.up, Vector3.forward };

    // where the hermite samples of a face lie within its plane, relative to the corner
    private static readonly Vector2[] offsets = { new( 0.02f, 0.01f ), new( 0.03f, -0.02f ), new( -0.01f, 0.025f ) };

    // large enough that no solution is rejected for lying outside of it
    private static readonly SurfaceExtractor.Voxel unbounded = new AdaptiveDualContouring.Voxel( VoxelType.Leaf, 0, Vector3.zero, Vector3.one * 100.0f );

    private static Quaternion rotationOf( Orientation orientation ) {
        return orientation == Orientation.Rotated ? Quaternion.AngleAxis( 40.0f, new Vector3( 1.0f, 2.0f, 3.0f ) ) : Quaternion.identity;
    }

    // hermite samples of mutually perpendicular faces that all pass through the corner, one face for every count
    private static List<( Vector3 intersection, Vector3 normal )> faces( Quaternion rotation, params int[] counts ) {
        var samples = new List<( Vector3 intersection, Vector3 normal )>( );

        for( var face = 0; face < counts.Length; ++face ) {
            foreach( var offset in offsets.Take( counts[face] ) ) {
                var inPlane = ( axes[( face + 1 ) % 3] * offset.x ) + ( axes[( face + 2 ) % 3] * offset.y );

                samples.Add( ( corner + ( rotation * inPlane ), rotation * axes[face] ) );
            }
        }

        return samples;
    }

    private static Vector3 massPointOf( List<( Vector3 intersection, Vector3 normal )> samples ) {
        return samples.Aggregate( Vector3.zero, ( sum, sample ) => sum + sample.intersection ) / samples.Count;
    }

    // equation 1 of both dual contouring papers: E[x] = sum of ( n . ( x - p ) )^2 over the hermite samples
    private static float residualOf( List<( Vector3 intersection, Vector3 normal )> samples, Vector3 point ) {
        return samples.Sum( ( sample ) => Mathf.Pow( Vector3.Dot( sample.normal, point - sample.intersection ), 2.0f ) );
    }

    private static ( Vector3 vertex, float error ) solve( List<( Vector3 intersection, Vector3 normal )> samples, SurfaceExtractor.Voxel voxel ) {
        var qef = new SVDQEF( 6 );

        foreach( var ( intersection, normal ) in samples ) {
            qef.add( intersection, normal );
        }

        return solve( qef, voxel );
    }

    private static ( Vector3 vertex, float error ) solve( QEFSolver qef, SurfaceExtractor.Voxel voxel ) {
        return qef.solve( voxel );
    }

    // three faces pin the minimizer down completely
    [Test]
    public void cornerIsReproduced( [Values] Orientation orientation ) {
        var ( vertex, _ ) = solve( faces( rotationOf( orientation ), 3, 2, 1 ), unbounded );

        Assert.Less( Vector3.Distance( corner, vertex ), epsilon, $"the vertex is {vertex:F5}, the corner {corner:F5}" );
    }

    // adding QEFs together during simplification makes the singular values grow with the number of samples, a large
    // singular value is as usable as a small one
    [Test]
    public void cornerIsReproducedFromManySamples( ) {
        var samples = Enumerable.Repeat( faces( rotationOf( Orientation.Rotated ), 3, 2, 1 ), 10 ).SelectMany( ( repeated ) => repeated ).ToList( );

        var ( vertex, _ ) = solve( samples, unbounded );

        Assert.Less( Vector3.Distance( corner, vertex ), epsilon, $"the vertex is {vertex:F5}, the corner {corner:F5}" );
    }

    // two faces leave the minimizer free to slide along the edge they share, and it settles next to the mass point
    [Test]
    public void edgeIsReproduced( [Values] Orientation orientation ) {
        var rotation = rotationOf( orientation );
        var samples  = faces( rotation, 2, 2 );

        var direction = rotation * axes[2];
        var expected  = corner + ( direction * Vector3.Dot( massPointOf( samples ) - corner, direction ) );

        var ( vertex, _ ) = solve( samples, unbounded );

        Assert.Less( Vector3.Distance( expected, vertex ), epsilon, $"the vertex is {vertex:F5}, the closest point of the edge to the mass point {expected:F5}" );
    }

    // four samples of a flat surface whose normals are off by about half a degree. Followed exactly, they meet almost
    // a voxel away from the samples; truncation discards those two directions and leaves the vertex with the samples
    [Test]
    public void noiseOnFlatSurfaceIsTruncated( ) {
        var samples = new List<( Vector3 intersection, Vector3 normal )> {
            ( new Vector3(  0.02f,  0.0005f,  0.02f ), Vector3.Normalize( new Vector3(  0.010f, 1.0f,  0.000f ) ) ),
            ( new Vector3( -0.02f, -0.0004f,  0.02f ), Vector3.Normalize( new Vector3( -0.008f, 1.0f,  0.006f ) ) ),
            ( new Vector3(  0.02f,  0.0003f, -0.02f ), Vector3.Normalize( new Vector3(  0.004f, 1.0f, -0.012f ) ) ),
            ( new Vector3( -0.02f, -0.0006f, -0.02f ), Vector3.Normalize( new Vector3( -0.006f, 1.0f,  0.009f ) ) )
        };

        var massPoint = massPointOf( samples );

        var ( vertex, _ ) = solve( samples, unbounded );

        Assert.Less( Vector3.Distance( massPoint, vertex ), 1e-3f, $"the vertex is {vertex:F5}, the mass point {massPoint:F5}" );
    }

    // Dual Contouring of Hermite Data, section 3.1: "the residual associated with the minimizer of this QEF estimates
    // how well the minimizing vertex approximates the original geometry". The top and bottom face of a slab 0.02 thick
    // cannot both be met: the best a single vertex can do is to sit 0.01 from each of the eight samples
    [Test]
    public void errorOfSlabIsDistanceToBothFaces( ) {
        var samples = new List<( Vector3 intersection, Vector3 normal )>( );

        foreach( var x in new[] { -0.02f, 0.02f } ) {
            foreach( var z in new[] { -0.02f, 0.02f } ) {
                samples.Add( ( new Vector3( x,  0.01f, z ), Vector3.up   ) );
                samples.Add( ( new Vector3( x, -0.01f, z ), Vector3.down ) );
            }
        }

        var ( _, error ) = solve( samples, unbounded );

        Assert.AreEqual( 8.0f * 0.01f * 0.01f, error, 1e-6f );
    }

    // four samples of a sphere do not meet in a point either, the residual of the vertex is whatever is left over
    [Test]
    public void errorIsResidualAtVertex( ) {
        var samples = new[] { new Vector3( 1.0f, 2.0f, 3.0f ), new Vector3( 1.3f, 2.0f, 3.0f ), new Vector3( 1.0f, 2.4f, 3.0f ), new Vector3( 1.3f, 2.4f, 2.8f ) }.Select(
            ( direction ) => ( intersection: Vector3.Normalize( direction ) * 0.3f, normal: Vector3.Normalize( direction ) )
        ).ToList( );

        var ( vertex, error ) = solve( samples, unbounded );

        Assert.AreEqual( residualOf( samples, vertex ), error, 1e-6f );
    }

    // the minimizer is searched from the mass point, which keeps it near the samples by itself. Nothing ties it to the
    // voxel the samples came from: a corner just outside of it is still where the vertex belongs
    [Test]
    public void vertexMayLieOutsideItsVoxel( ) {
        var samples   = faces( rotationOf( Orientation.Rotated ), 3, 2, 1 );
        var massPoint = massPointOf( samples );

        // the mass point lies 0.014 from the corner
        var voxel = new AdaptiveDualContouring.Voxel( VoxelType.Leaf, 0, massPoint, Vector3.one * 0.01f );

        var ( vertex, error ) = solve( samples, voxel );

        Assert.Less( Vector3.Distance( corner, vertex ), epsilon, $"the vertex is {vertex:F5}, the corner {corner:F5}" );

        Assert.AreEqual( 0.0f, error, 1e-6f );
    }

    // a QEF that has been solved, which is what tells the dimension of the feature its samples lie on
    private static SVDQEF solved( List<( Vector3 intersection, Vector3 normal )> samples ) {
        var qef = new SVDQEF( 6 );

        foreach( var ( intersection, normal ) in samples ) {
            qef.add( intersection, normal );
        }

        solve( qef, unbounded );

        return qef;
    }

    private static List<( Vector3 intersection, Vector3 normal )> moved( List<( Vector3 intersection, Vector3 normal )> samples, Vector3 offset ) {
        return samples.Select( ( sample ) => ( sample.intersection + offset, sample.normal ) ).ToList( );
    }

    // the point of the edge that the first two faces share, the line through the corner along the third axis, that
    // lies closest to the given point
    private static Vector3 closestOnEdge( Vector3 point ) {
        return corner + ( axes[2] * Vector3.Dot( point - corner, axes[2] ) );
    }

    // section 2.1, on combining QEFs during simplification: "If the mass points are of differing dimensions, set the
    // combined mass point equal to the mass point of the highest dimension". A stretch of an edge and, half a unit
    // further along it, a stretch of one of its faces by itself: the vertex stays with the samples of the edge
    [Test]
    public void massPointOfCombinedQEFsIsThatOfTheSharpestFeature( ) {
        var edge = faces( Quaternion.identity, 2, 2 );
        var face = moved( faces( Quaternion.identity, 3 ), axes[2] * 0.5f );

        var combined = new SVDQEF( 6 );

        combined.combine( solved( face ) );
        combined.combine( solved( edge ) );

        var ( vertex, _ ) = solve( combined, unbounded );

        var expected = closestOnEdge( massPointOf( edge ) );

        Assert.Less( Vector3.Distance( expected, vertex ), epsilon, $"the vertex is {vertex:F5}, the closest point of the edge to the mass point of its own samples {expected:F5}" );
    }

    // "If the mass points are of the same dimension, merge the two mass points like before by summing their
    // components": two stretches of the same edge put the vertex in between
    [Test]
    public void massPointsOfTheSameDimensionAreCombined( ) {
        var edge  = faces( Quaternion.identity, 2, 2 );
        var other = moved( faces( Quaternion.identity, 2, 1 ), axes[2] * 0.5f );

        var combined = new SVDQEF( 6 );

        combined.combine( solved( edge ) );
        combined.combine( solved( other ) );

        var ( vertex, _ ) = solve( combined, unbounded );

        var expected = closestOnEdge( massPointOf( edge.Concat( other ).ToList( ) ) );

        Assert.Less( Vector3.Distance( expected, vertex ), epsilon, $"the vertex is {vertex:F5}, the closest point of the edge to the mass point of all samples {expected:F5}" );
    }

}
