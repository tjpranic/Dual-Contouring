using NUnit.Framework;
using Unity.Mathematics;
using UnityEngine;

using QEFSolverType = QEFSolver.Type;

// the error of a QEF, which every solver works out from its sums in doubles (QEFError). Close to the surface the three
// terms of the error cancel, and in floats their rounding is larger than the error itself
public class QEFErrorTest {

    // a corner that floats do not hold exactly, away from the origin, and seen by many samples on each of its three
    // faces: as many as the octant of a cube has at a resolution of 8
    private static readonly Vector3 corner = new( 0.3f, -0.2f, 0.45f );

    private const int samplesPerFace = 1000;

    // the samples lie on the three planes through the corner, which floats hold exactly: their points have the
    // corner's coordinate along the normal. So the QEF is zero at the corner, exactly
    private static ( Vector3[] intersections, Vector3[] normals ) cornerSamples( ) {
        var random        = new System.Random( 5 );
        var intersections = new Vector3[3 * samplesPerFace];
        var normals       = new Vector3[3 * samplesPerFace];

        for( var face = 0; face < 3; ++face ) {
            for( var sample = 0; sample < samplesPerFace; ++sample ) {
                var point = corner;

                point[( face + 1 ) % 3] -= ( float )random.NextDouble( ) * 0.3f;
                point[( face + 2 ) % 3] -= ( float )random.NextDouble( ) * 0.3f;

                var normal = Vector3.zero;

                normal[face] = 1.0f;

                intersections[( face * samplesPerFace ) + sample] = point;
                normals[( face * samplesPerFace ) + sample]       = normal;
            }
        }

        return ( intersections, normals );
    }

    // what the error came to when it was worked out from the float sums: far above the bound the tests hold it to,
    // which is what makes them tell the two apart
    [Test]
    public void floatSumsRoundToMoreThanTheError( ) {
        var ( intersections, normals ) = cornerSamples( );

        var ata = new SVDQEF.SymmetricMatrix3x3( );
        var atb = Vector3.zero;
        var btb = 0.0f;

        for( var index = 0; index < intersections.Length; ++index ) {
            var normal = normals[index];
            var dot    = Vector3.Dot( intersections[index], normal );

            ata.m00 += normal.x * normal.x;
            ata.m01 += normal.x * normal.y;
            ata.m02 += normal.x * normal.z;
            ata.m11 += normal.y * normal.y;
            ata.m12 += normal.y * normal.z;
            ata.m22 += normal.z * normal.z;

            atb += dot * normal;
            btb += dot * dot;
        }

        Assert.Greater( SVDQEF.SVD.calculateError( ata, atb, btb, corner ), 1e-6f );
    }

    [Test]
    public void errorIsZeroAtTheCorner( ) {
        var ( intersections, normals ) = cornerSamples( );

        var error = new QEFError( );

        for( var index = 0; index < intersections.Length; ++index ) {
            error.add( intersections[index].x, intersections[index].y, intersections[index].z, normals[index].x, normals[index].y, normals[index].z );
        }

        Assert.Less( error.valueAt( corner.x, corner.y, corner.z ), 1e-10f );
    }

    // combined from many QEFs, as the levels of an octree combine them, it is the same
    [Test]
    public void combinedErrorIsZeroAtTheCorner( ) {
        var ( intersections, normals ) = cornerSamples( );

        var error = new QEFError( );

        for( var index = 0; index < intersections.Length; index += 10 ) {
            var part = new QEFError( );

            for( var sample = index; sample < index + 10; ++sample ) {
                part.add( intersections[sample].x, intersections[sample].y, intersections[sample].z, normals[sample].x, normals[sample].y, normals[sample].z );
            }

            error.combine( part );
        }

        Assert.Less( error.valueAt( corner.x, corner.y, corner.z ), 1e-10f );
    }

    // the Burst QEF of every solver, at the corner itself. The probabilistic solver's vertex is pulled towards the mass
    // point, where the error is real
    [Test]
    public void burstErrorIsZeroAtTheCorner( [Values] QEFSolverType type ) {
        var ( intersections, normals ) = cornerSamples( );

        var qef = BurstQEF.create( type, 6, 0.05f );

        for( var index = 0; index < intersections.Length; ++index ) {
            qef.add( intersections[index], normals[index] );
        }

        Assert.Less( qef.calculateError( corner ), 1e-10f );
    }

    // the QEF at a point summed sample by sample, in doubles
    private static double errorAt( Vector3 point, Vector3[] intersections, Vector3[] normals ) {
        var error = 0.0;

        for( var index = 0; index < intersections.Length; ++index ) {
            var distance = ( ( double )normals[index].x * ( point.x - ( double )intersections[index].x ) ) +
                           ( ( double )normals[index].y * ( point.y - ( double )intersections[index].y ) ) +
                           ( ( double )normals[index].z * ( point.z - ( double )intersections[index].z ) );

            error += distance * distance;
        }

        return error;
    }

    // the solvers that find the corner report the error at what they found: the solve is in floats, which leaves the
    // vertex a few millionths from the corner, and an error of some 2e-8 there, which the float sums rounded to 1e-6
    // and more
    [Test]
    public void solversReportTheErrorAtTheirVertex( [Values( QEFSolverType.Simple, QEFSolverType.SVD )] QEFSolverType type ) {
        var ( intersections, normals ) = cornerSamples( );

        QEFSolver managed = type == QEFSolverType.SVD ? new SVDQEF( 6 ) : new SimpleQEF( 64 );

        var burst = BurstQEF.create( type, type == QEFSolverType.SVD ? 6 : 64, 0.05f );

        for( var index = 0; index < intersections.Length; ++index ) {
            managed.add( intersections[index], normals[index] );
            burst.add( intersections[index], normals[index] );
        }

        // the simple solver steps from the center of the voxel towards the planes, the SVD solver does not look at it
        var voxel = new UniformDualContouring.Voxel( corner, Vector3.one * 0.5f );

        var ( managedVertex, managedError ) = managed.solve( voxel );

        burst.solve( corner, out var burstVertex, out var burstError );

        Assert.Less( Vector3.Distance( managedVertex, corner ), 1e-5f, "the managed solver found the corner" );
        Assert.Less( math.distance( burstVertex, corner ), 1e-5f, "the Burst solver found the corner" );

        // the managed simple solver sums the squares of the distances to its planes in floats, which rounds by a little
        // more than the others
        var managedExpected = errorAt( managedVertex, intersections, normals );
        var burstExpected   = errorAt( burstVertex,   intersections, normals );

        Assert.AreEqual( managedExpected, managedError, ( managedExpected * 1e-3 ) + 1e-12, "managed" );
        Assert.AreEqual( burstExpected,   burstError,   ( burstExpected   * 1e-3 ) + 1e-12, "Burst" );
    }

}
