using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;

using VoxelType     = SurfaceExtractor.Voxel.Type;
using QEFSolverType = QEFSolver.Type;

// solveQEF of QEF.hlsl against the managed solvers, on the sums of hermite samples as the compute shaders keep them:
// the probabilistic solver against ProbabilisticQEF at the two ends of its range, its default and a value between, and
// the SVD solver against SVDQEF, which solveQEF also chooses between
public class ProbabilisticTest : ImplementationTest {

    protected override string shader { get; } = "Tests/ProbabilisticTest";

    // struct Settings of ProbabilisticTest.compute, field for field
    public struct Settings {

        public uint  qefSolver;
        public uint  minimizerIterations;
        public float normalDeviation;

    }

    private const int   cases               = 200;
    private const int   minimizerIterations = 6;
    private const float cellSize            = 1.0f / 16.0f;

    // as BurstQEFTest has them. The probabilistic solver divides what is left of AtB - AtA p by N s^2, which magnifies the
    // last bits of the sums by one over the square of the deviation: on these samples the GPU's vertices are about
    // 2e-8 / s^2 from the managed ones, the most 4.9e-5 at 0.02. The SVD solver's are within 1.6e-7
    private const float vertexTolerance = 1e-5f;
    private const float errorTolerance  = 1e-5f;

    private static float probabilisticVertexTolerance( float normalDeviation ) {
        return vertexTolerance + ( 5e-8f / ( normalDeviation * normalDeviation ) );
    }

    // the solvers do not look at the voxel
    private static readonly SurfaceExtractor.Voxel voxel = new AdaptiveDualContouring.Voxel( VoxelType.Leaf, 0, Vector3.zero, Vector3.one * cellSize );

    private static float range( System.Random random, float minimum, float maximum ) {
        return minimum + ( ( float )random.NextDouble( ) * ( maximum - minimum ) );
    }

    private static Vector3 direction( System.Random random ) {
        return new Vector3( range( random, -1.0f, 1.0f ), range( random, -1.0f, 1.0f ), range( random, -1.0f, 1.0f ) ).normalized;
    }

    // hermite samples in a cell of a grid of 16^3: on a plane, an edge or a corner with a little noise in the normals, or
    // with normals in every direction
    private static List<( Vector3 intersection, Vector3 normal )> samplesOf( System.Random random ) {
        var center   = new Vector3( range( random, -0.4f, 0.4f ), range( random, -0.4f, 0.4f ), range( random, -0.4f, 0.4f ) );
        var count    = 1 + random.Next( 12 );
        var kind     = random.Next( 4 );
        var features = new[] { direction( random ), direction( random ), direction( random ) };
        var samples  = new List<( Vector3 intersection, Vector3 normal )>( );

        for( var sample = 0; sample < count; ++sample ) {
            var offset = new Vector3( range( random, -0.5f, 0.5f ), range( random, -0.5f, 0.5f ), range( random, -0.5f, 0.5f ) ) * cellSize;
            var normal = kind == 0 ? direction( random ) : features[sample % kind] + ( direction( random ) * 1e-3f );

            samples.Add( ( center + offset, normal.normalized ) );
        }

        return samples;
    }

    // the vertex and error of the managed solver, and those of solveQEF on the same samples
    private ( Vector4 expected, Vector4 actual ) solveBoth( List<( Vector3 intersection, Vector3 normal )> samples, QEFSolverType type, float normalDeviation ) {
        var sums = new SVDQEF( minimizerIterations );

        QEFSolver managed = type == QEFSolverType.Probabilistic ? new ProbabilisticQEF( normalDeviation ) : new SVDQEF( minimizerIterations );

        foreach( var ( intersection, normal ) in samples ) {
            sums.add( intersection, normal );
            managed.add( intersection, normal );
        }

        var ( vertex, error ) = managed.solve( voxel );

        var actual = this.testKernel(
            "solveQEFTest",
            new SVDQEF.Data( sums ),
            new Settings { qefSolver = ( uint )QEFSolver.shaderSolverOf( type ), minimizerIterations = minimizerIterations, normalDeviation = normalDeviation },
            Vector4.zero
        );

        return ( new Vector4( vertex.x, vertex.y, vertex.z, error ), actual );
    }

    // relative to the size of the value, and never tighter than the tolerance itself
    private static float apart( float expected, float actual ) {
        return Mathf.Abs( expected - actual ) / Mathf.Max( 1.0f, Mathf.Abs( expected ) );
    }

    private void assertSolvesAsManaged( QEFSolverType type, float normalDeviation, float vertexTolerance, float errorTolerance ) {
        var random      = new System.Random( 7 );
        var worstVertex = 0.0f;
        var worstError  = 0.0f;
        var worstCase   = "";

        for( var index = 0; index < cases; ++index ) {
            var ( expected, actual ) = this.solveBoth( samplesOf( random ), type, normalDeviation );

            var vertex = Mathf.Max( apart( expected.x, actual.x ), Mathf.Max( apart( expected.y, actual.y ), apart( expected.z, actual.z ) ) );

            if( !( vertex <= worstVertex ) ) {
                worstVertex = vertex;
                worstCase   = $"managed {expected:F7}, GPU {actual:F7}";
            }

            worstError = Mathf.Max( worstError, apart( expected.w, actual.w ) );
        }

        Assert.That( worstVertex, Is.LessThanOrEqualTo( vertexTolerance ), $"the vertex farthest from the managed one, of {cases}: {worstCase}" );
        Assert.That( worstError,  Is.LessThanOrEqualTo( errorTolerance ),  $"the error farthest from the managed one, of {cases}" );
    }

    [Test]
    public void probabilisticSolveIsTheManagedOne( [Values( 0.02f, 0.05f, 0.25f, 1.0f )] float normalDeviation ) {
        this.assertSolvesAsManaged( QEFSolverType.Probabilistic, normalDeviation, probabilisticVertexTolerance( normalDeviation ), errorTolerance );
    }

    [Test]
    public void svdSolveIsTheManagedOne( ) {
        this.assertSolvesAsManaged( QEFSolverType.SVD, 0.05f, vertexTolerance, errorTolerance );
    }

}
