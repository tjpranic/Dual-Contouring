using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using UnityEngine;

using VoxelType = SurfaceExtractor.Voxel.Type;

// Fast and Robust QEF Minimization using Probabilistic Quadrics, section 3.1: with Gaussian noise of deviation s in each
// component of the normals, the vertex of a voxel minimizes the expected QEF, x = p + ( AtA + N s^2 I )^-1 ( AtB - AtA p ),
// for N samples with mass point p. For samples of faces square to the axes AtA is diagonal, and the vertex is known on
// each axis by itself
public class ProbabilisticQEFTest {

    public enum Orientation {
        AxisAligned,
        Rotated
    }

    public enum Feature {
        Face,
        Edge,
        Corner
    }

    public enum Degenerate {
        OneSample,
        NoNormals,
        ParallelPlanes,
        SamplesOnALine
    }

    private const float epsilon = 1e-4f;

    private static readonly Vector3 corner = new( 0.1f, 0.2f, 0.3f );

    private static readonly Vector3[] axes = { Vector3.right, Vector3.up, Vector3.forward };

    // where the hermite samples of a face lie within its plane, relative to the corner
    private static readonly Vector2[] offsets = { new( 0.02f, 0.01f ), new( 0.03f, -0.02f ), new( -0.01f, 0.025f ) };

    // the solver does not look at the voxel
    private static readonly SurfaceExtractor.Voxel voxel = new AdaptiveDualContouring.Voxel( VoxelType.Leaf, 0, corner, Vector3.one * 0.0625f );

    private static Quaternion rotationOf( Orientation orientation ) {
        return orientation == Orientation.Rotated ? Quaternion.AngleAxis( 40.0f, new Vector3( 1.0f, 2.0f, 3.0f ) ) : Quaternion.identity;
    }

    // how many samples each face of the feature has, one face for every count
    private static int[] countsOf( Feature feature ) {
        return feature switch {
            Feature.Face => new[] { 3 },
            Feature.Edge => new[] { 3, 2 },
            _            => new[] { 3, 2, 1 }
        };
    }

    // where the samples of mutually perpendicular faces through the corner lie within their faces, relative to the
    // corner and before the faces are turned
    private static List<Vector3> inPlaneOffsets( int[] counts ) {
        var positions = new List<Vector3>( );

        for( var face = 0; face < counts.Length; ++face ) {
            foreach( var offset in offsets.Take( counts[face] ) ) {
                positions.Add( ( axes[( face + 1 ) % 3] * offset.x ) + ( axes[( face + 2 ) % 3] * offset.y ) );
            }
        }

        return positions;
    }

    // hermite samples of mutually perpendicular faces that all pass through the corner, one face for every count
    private static List<( Vector3 intersection, Vector3 normal )> faces( Quaternion rotation, int[] counts ) {
        var samples   = new List<( Vector3 intersection, Vector3 normal )>( );
        var positions = inPlaneOffsets( counts );
        var index     = 0;

        for( var face = 0; face < counts.Length; ++face ) {
            for( var sample = 0; sample < counts[face]; ++sample ) {
                samples.Add( ( corner + ( rotation * positions[index++] ), rotation * axes[face] ) );
            }
        }

        return samples;
    }

    // the vertex for those faces in closed form, in doubles. Before the faces are turned AtA is the diagonal of the
    // numbers of samples of each face, N_k, and AtB - AtA p is N_k times how far the corner is from the mass point along
    // the axis of face k. So the vertex is p_k + ( c_k - p_k ) N_k / ( N_k + N s^2 ) on each axis, with no face along an
    // axis leaving the mass point there
    private static Vector3 closedForm( Quaternion rotation, int[] counts, float deviation ) {
        var positions = inPlaneOffsets( counts );
        var total     = positions.Count;
        var pull      = total * ( double )deviation * deviation;

        var vertex = new double[3];

        for( var axis = 0; axis < 3; ++axis ) {
            // the mass point relative to the corner, along the axis
            var massPoint = positions.Sum( ( position ) => ( double )position[axis] ) / total;
            var samples   = axis < counts.Length ? counts[axis] : 0;

            vertex[axis] = massPoint - ( massPoint * samples / ( samples + pull ) );
        }

        return corner + ( rotation * new Vector3( ( float )vertex[0], ( float )vertex[1], ( float )vertex[2] ) );
    }

    private static Vector3 massPointOf( List<( Vector3 intersection, Vector3 normal )> samples ) {
        return samples.Aggregate( Vector3.zero, ( sum, sample ) => sum + sample.intersection ) / samples.Count;
    }

    // equation 1 of both dual contouring papers: E[x] = sum of ( n . ( x - p ) )^2 over the hermite samples
    private static float residualOf( List<( Vector3 intersection, Vector3 normal )> samples, Vector3 point ) {
        return samples.Sum( ( sample ) => Mathf.Pow( Vector3.Dot( sample.normal, point - sample.intersection ), 2.0f ) );
    }

    private static ProbabilisticQEF qefOf( IEnumerable<( Vector3 intersection, Vector3 normal )> samples, float deviation ) {
        var qef = new ProbabilisticQEF( deviation );

        foreach( var ( intersection, normal ) in samples ) {
            qef.add( intersection, normal );
        }

        return qef;
    }

    private static ( Vector3 vertex, float error ) solve( List<( Vector3 intersection, Vector3 normal )> samples, float deviation ) {
        return qefOf( samples, deviation ).solve( voxel );
    }

    private static string describe( Vector3 vertex, Vector3 expected ) {
        return $"the vertex is {vertex:F6}, expected {expected:F6}, {Vector3.Distance( vertex, expected ):E2} apart";
    }

    // a face, an edge and a corner, square to the axes or turned, at the two ends of the setting's range, its default and
    // one value between. A face gives the mass point at any deviation; an edge keeps the mass point's place along it and
    // is drawn toward the line across it; a corner is drawn toward the corner, all the way as the deviation goes to zero
    // and hardly at all at one
    [Test]
    public void vertexIsTheClosedForm( [Values] Feature feature, [Values] Orientation orientation, [Values( 0.02f, 0.05f, 0.25f, 1.0f )] float deviation ) {
        var rotation = rotationOf( orientation );
        var counts   = countsOf( feature );

        var ( vertex, _ ) = solve( faces( rotation, counts ), deviation );
        var expected      = closedForm( rotation, counts, deviation );

        Assert.Less( Vector3.Distance( expected, vertex ), epsilon, describe( vertex, expected ) );
    }

    // samples that the classical QEF has no single minimizer for, or none at all, still give a finite vertex: the mass
    // point, since every one of them lies in the planes of the others or has no plane. It is off by what rounding leaves
    // of AtB - AtA p, divided by N s^2: 3e-5 for a single sample at 0.02
    [Test]
    public void degenerateSamplesGiveTheMassPoint( [Values] Degenerate degenerate, [Values( 0.02f, 1.0f )] float deviation ) {
        var samples = new List<( Vector3 intersection, Vector3 normal )>( );

        switch( degenerate ) {
            case Degenerate.OneSample: {
                samples.Add( ( corner, Vector3.Normalize( new Vector3( 1.0f, 2.0f, 3.0f ) ) ) );
                break;
            }
            case Degenerate.NoNormals: {
                foreach( var offset in offsets ) {
                    samples.Add( ( corner + new Vector3( offset.x, offset.y, offset.x - offset.y ), Vector3.zero ) );
                }
                break;
            }
            case Degenerate.ParallelPlanes: {
                // the top and bottom of a slab
                foreach( var offset in offsets ) {
                    samples.Add( ( corner + new Vector3( offset.x,  0.01f, offset.y ), Vector3.up   ) );
                    samples.Add( ( corner + new Vector3( offset.y, -0.01f, offset.x ), Vector3.down ) );
                }
                break;
            }
            case Degenerate.SamplesOnALine: {
                // the planes all hold the line, and turn about it
                for( var sample = 0; sample < 5; ++sample ) {
                    var angle = sample * 0.7f;

                    samples.Add( ( corner + ( Vector3.forward * ( 0.01f * sample ) ), new Vector3( Mathf.Cos( angle ), Mathf.Sin( angle ), 0.0f ) ) );
                }
                break;
            }
        }

        var massPoint     = massPointOf( samples );
        var ( vertex, _ ) = solve( samples, deviation );

        Assert.IsFalse( float.IsNaN( vertex.x ) || float.IsNaN( vertex.y ) || float.IsNaN( vertex.z ), $"the vertex is {vertex}" );
        Assert.IsFalse( float.IsInfinity( vertex.x ) || float.IsInfinity( vertex.y ) || float.IsInfinity( vertex.z ), $"the vertex is {vertex}" );

        Assert.Less( Vector3.Distance( massPoint, vertex ), epsilon, describe( vertex, massPoint ) );
    }

    // a ridge whose faces are 10 degrees apart, with more samples on one side of it than the other, in a voxel of a grid
    // of 64^3. Across the ridge AtA is about 0.06, under the SVD solver's truncation of 0.1, so that solver leaves the
    // vertex at the mass point across it. The probabilistic solver keeps most of the way to the ridge
    [Test]
    public void shallowRidgeIsKeptNearerThanBySVD( ) {
        var ridge    = new Vector3( 0.1f, 0.2f, 0.3f );
        var halfTurn = 5.0f * Mathf.Deg2Rad;
        var samples  = new List<( Vector3 intersection, Vector3 normal )>( );

        foreach( var ( side, distances ) in new[] { ( 1.0f, new[] { 0.002f, 0.004f } ), ( -1.0f, new[] { 0.008f, 0.012f } ) } ) {
            var normal = new Vector3( side * Mathf.Sin( halfTurn ), Mathf.Cos( halfTurn ), 0.0f );
            var down   = new Vector3( side * Mathf.Cos( halfTurn ), -Mathf.Sin( halfTurn ), 0.0f );

            foreach( var distance in distances ) {
                foreach( var along in new[] { -0.004f, 0.004f } ) {
                    samples.Add( ( ridge + ( down * distance ) + ( Vector3.forward * along ), normal ) );
                }
            }
        }

        float fromRidge( Vector3 point ) {
            var offset = point - ridge;

            return new Vector2( offset.x, offset.y ).magnitude;
        }

        var svd = new SVDQEF( 6 );

        foreach( var ( intersection, normal ) in samples ) {
            svd.add( intersection, normal );
        }

        var ( svdVertex, _ )           = svd.solve( voxel );
        var ( probabilisticVertex, _ ) = solve( samples, 0.05f );

        // the test is of a ridge the SVD solver misses, by a fifth of a voxel
        Assert.Greater( fromRidge( svdVertex ), 0.003f, $"the SVD solver's vertex is {svdVertex:F6}" );

        Assert.Less( fromRidge( probabilisticVertex ), 0.4f * fromRidge( svdVertex ), $"the vertex is {fromRidge( probabilisticVertex ):E2} from the ridge, the SVD solver's {fromRidge( svdVertex ):E2}" );
    }

    // the pull is toward the mass point of the samples, not toward the origin: the same samples elsewhere give the same
    // vertex relative to them
    [Test]
    public void vertexMovesWithItsSamples( ) {
        var samples = faces( rotationOf( Orientation.Rotated ), countsOf( Feature.Corner ) );
        var offset  = new Vector3( 0.3f, -0.25f, 0.15f );

        var ( vertex, _ )      = solve( samples, 0.05f );
        var ( movedVertex, _ ) = solve( samples.Select( ( sample ) => ( sample.intersection + offset, sample.normal ) ).ToList( ), 0.05f );

        Assert.Less( Vector3.Distance( vertex + offset, movedVertex ), epsilon, describe( movedVertex, vertex + offset ) );
    }

    // the error is that of the classical QEF at the vertex the solver returns, as for the other solvers. A corner with one
    // more sample of its first face, 0.006 off that face, cannot be met by any vertex, so the error is not zero, and the
    // vertex is not the mass point
    [Test]
    public void errorIsTheResidualAtTheVertex( [Values( 0.02f, 0.05f, 1.0f )] float deviation ) {
        var rotation = rotationOf( Orientation.Rotated );
        var samples  = faces( rotation, countsOf( Feature.Corner ) );

        samples.Add( ( corner + ( rotation * new Vector3( 0.006f, 0.015f, -0.01f ) ), rotation * axes[0] ) );

        var ( vertex, error ) = solve( samples, deviation );

        var residual = residualOf( samples, vertex );

        // the error is what is left when terms of about one cancel, which floats keep to a few parts in ten million
        Assert.Greater( residual, 1e-5f, "the samples are met" );
        Assert.AreEqual( residual, error, 1e-6f );
    }

    // combined QEFs are the QEF of all their samples, with the mass point of all of them whether solved before or not
    [Test]
    public void combinedQEFsSolveAsTheQEFOfAllTheirSamples( ) {
        var samples = faces( rotationOf( Orientation.Rotated ), countsOf( Feature.Corner ) );

        var parts = new[] { samples.Take( 3 ), samples.Skip( 3 ).Take( 2 ), samples.Skip( 5 ) }.Select( ( part ) => qefOf( part, 0.05f ) ).ToArray( );

        // a QEF that has been solved is combined as it was
        parts[0].solve( voxel );

        var combined = new ProbabilisticQEF( 0.05f );

        foreach( var part in parts ) {
            combined.combine( part );
        }

        var whole = qefOf( samples, 0.05f );

        var ( vertex, error )        = combined.solve( voxel );
        var ( expected, wholeError ) = whole.solve( voxel );

        Assert.AreEqual( whole.intersectionCount, combined.intersectionCount );
        Assert.Less( Vector3.Distance( expected, vertex ), 1e-6f, describe( vertex, expected ) );
        Assert.AreEqual( wholeError, error, 1e-6f );
        Assert.Less( Vector3.Distance( massPointOf( samples ), combined.solveMassPoint( ).vertex ), 1e-6f );
    }

}
