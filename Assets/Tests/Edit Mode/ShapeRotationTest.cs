using System;
using System.Linq;
using NUnit.Framework;
using UnityEngine;

using Combination          = DensityFunction.Combination;
using MaterialIndex        = SurfaceExtractor.MaterialIndex;
using QEFSolverType        = QEFSolver.Type;
using FixedDensityFunction = BurstTestSupport.FixedDensityFunction;

// sampling a rotated shape: the position is taken into the shape's own frame, by undoing the rotation about its origin,
// and the shape's formula is applied there. A shape without rotation samples exactly as it did before shapes could rotate
public class ShapeRotationTest : ContouringTest {

    private static readonly Vector3 origin = new( 0.013f, -0.021f, 0.017f );

    // about one axis, about all three in turn, and about a diagonal
    private static readonly Quaternion[] rotations = {
        Quaternion.Euler     ( 0.0f,  45.0f, 0.0f  ),
        Quaternion.Euler     ( 30.0f, -60.0f, 75.0f ),
        Quaternion.AngleAxis ( 45.0f, Vector3.one  )
    };

    private static readonly int[] rotationIndices = Enumerable.Range( 0, rotations.Length ).ToArray( );

    // points all over the grid and a little past it
    private static Vector3[] randomPoints( int seed ) {
        var random = new System.Random( seed );

        return Enumerable.Range( 0, 2000 ).Select(
            ( _ ) => BurstTestSupport.range( random, -Vector3.one, Vector3.one )
        ).ToArray( );
    }

    // within a tolerance relative to the size of the density, and never tighter than the tolerance itself
    private static void assertClose( float expected, float actual, float tolerance, string message ) {
        Assert.That( Math.Abs( expected - actual ), Is.LessThanOrEqualTo( tolerance * Math.Max( 1.0f, Math.Abs( expected ) ) ), message );
    }

    private static FixedDensityFunction shape( DensityFunction.Type type, Vector3 extents, Quaternion rotation ) {
        return new( type, origin, rotation, extents, Combination.Union, MaterialIndex.Material0 );
    }

    // Unity's rotation, the way a transform turns its children: the turned shape at a point is the unturned shape at that
    // point turned back about the origin. The two turn by different arithmetic, so they agree to rounding
    [Test]
    public void aRotatedShapeIsTheShapeTurnedAboutItsOrigin( [Values] DensityFunction.Type type, [ValueSource( nameof( rotationIndices ) )] int rotationIndex ) {
        var rotation = rotations[rotationIndex];
        var extents  = new Vector3( 0.1f, 0.2f, 0.3f );
        var rotated  = shape( type, extents, rotation );
        var upright  = shape( type, extents, Quaternion.identity );

        foreach( var point in randomPoints( 1 ) ) {
            var turnedBack = origin + ( Quaternion.Inverse( rotation ) * ( point - origin ) );

            assertClose( upright.sample( turnedBack ), rotated.sample( point ), 1e-5f, $"density at {point}" );
        }
    }

    // a quarter turn about one axis lays the shape's extents along the other two axes the other way round. That makes the
    // same shape only of a shape that treats its three axes alike: a cone turned onto its side is not a cone
    [Test]
    public void aQuarterTurnSwapsTwoExtents( [Values( DensityFunction.Type.Cuboid, DensityFunction.Type.Ellipsoid, DensityFunction.Type.Octahedron )] DensityFunction.Type type ) {
        var extents = new Vector3( 0.1f, 0.2f, 0.3f );

        var turns = new[] {
            ( Quaternion.AngleAxis( 90.0f, Vector3.right   ), new Vector3( 0.1f, 0.3f, 0.2f ) ),
            ( Quaternion.AngleAxis( 90.0f, Vector3.up      ), new Vector3( 0.3f, 0.2f, 0.1f ) ),
            ( Quaternion.AngleAxis( 90.0f, Vector3.forward ), new Vector3( 0.2f, 0.1f, 0.3f ) )
        };

        foreach( var ( rotation, swapped ) in turns ) {
            var rotated = shape( type, extents, rotation );
            var upright = shape( type, swapped, Quaternion.identity );

            foreach( var point in randomPoints( 2 ) ) {
                assertClose( upright.sample( point ), rotated.sample( point ), 1e-6f, $"density at {point} turned by {rotation.eulerAngles}" );
            }
        }
    }

    [Test]
    public void aSphereIsUnchangedByAnyRotation( ) {
        var random  = new System.Random( 3 );
        var upright = shape( DensityFunction.Type.Ellipsoid, Vector3.one * 0.3f, Quaternion.identity );

        for( var turn = 0; turn < 20; ++turn ) {
            var rotation = Quaternion.AngleAxis( BurstTestSupport.range( random, -180.0f, 180.0f ), BurstTestSupport.direction( random ) );
            var rotated  = upright.rotated( rotation );

            foreach( var point in randomPoints( 4 + turn ) ) {
                assertClose( upright.sample( point ), rotated.sample( point ), 1e-6f, $"density at {point} turned by {rotation.eulerAngles}" );
            }
        }
    }

    // the identity either way round, and a zero quaternion, which a density function that never sets a rotation would
    // give, turn nothing. Any angle at all does, however small
    [Test]
    public void onlyARotationThatTurnsSomethingCountsAsRotated( ) {
        Assert.IsFalse( new DensityFunction.Frame( origin, Quaternion.identity                ).rotated, "identity" );
        Assert.IsFalse( new DensityFunction.Frame( origin, new Quaternion( 0.0f, 0.0f, 0.0f, -1.0f ) ).rotated, "identity, negated" );
        Assert.IsFalse( new DensityFunction.Frame( origin, default                            ).rotated, "zero quaternion" );
        Assert.IsTrue ( new DensityFunction.Frame( origin, Quaternion.Euler( 0.0f, 0.0f, 0.001f ) ).rotated, "a thousandth of a degree" );
        Assert.IsTrue ( new DensityFunction.Frame( origin, rotations[0]                       ).rotated, "45 degrees" );
    }

    // a quaternion of any length turns the shape like its unit quaternion, without scaling it
    [Test]
    public void theLengthOfTheQuaternionDoesNotMatter( [Values] DensityFunction.Type type ) {
        var rotation = rotations[1];
        var extents  = new Vector3( 0.1f, 0.2f, 0.3f );
        var unit     = shape( type, extents, rotation );
        var longer   = shape( type, extents, new Quaternion( rotation.x * 3.0f, rotation.y * 3.0f, rotation.z * 3.0f, rotation.w * 3.0f ) );

        foreach( var point in randomPoints( 5 ) ) {
            assertClose( unit.sample( point ), longer.sample( point ), 1e-6f, $"density at {point}" );
        }
    }

    // the formulas as they were before shapes could rotate
    private static float cuboidAsBefore( Vector3 position, Vector3 origin, Vector3 extents ) {
        var q = ( position - origin ).absolute( ) - extents;
        return Vector3.Magnitude( Vector3.Max( q, Vector3.zero ) ) + Mathf.Min( Mathf.Max( q.x, Mathf.Max( q.y, q.z ) ), 0.0f );
    }

    private static float ellipsoidAsBefore( Vector3 position, Vector3 origin, Vector3 extents ) {
        if( position - origin == Vector3.zero ) {
            return -1.0f;
        }

        var k0 = Vector3.Magnitude( ( position - origin ).divide( extents ) );
        var k1 = Vector3.Magnitude( ( position - origin ).divide( extents.multiply( extents ) ) );

        return k0 * ( k0 - 1.0f ) / k1;
    }

    // volumes, their snapshots and density functions that never set a rotation all sample bit for bit as before, at points
    // all over the grid and at their origins
    [Test]
    public void withoutRotationShapesSampleExactlyAsBefore( ) {
        var shapes = new[] {
            cuboid    ( new Vector3(  0.1f,  -0.2f, 0.05f ), new Vector3( 0.3f, 0.2f,  0.25f ) ),
            ellipsoid ( new Vector3( -0.15f,  0.1f, 0.2f  ), new Vector3( 0.2f, 0.35f, 0.15f ) ),
            sphere    ( new Vector3(  0.05f,  0.0f, -0.1f ), 0.4f )
        };

        var voxelizer = this.createVoxelizer<UniformDualContouring>( 2, QEFSolverType.SVD, shapes );

        // as in the scenes, where the voxelizer is scaled
        voxelizer.transform.localScale = Vector3.one * 5.0f;

        var volumes   = voxelizer.GetComponentsInChildren<Volume>( );
        var snapshots = DensityFunction.snapshot( volumes );

        for( var index = 0; index < volumes.Length; ++index ) {
            var volume = volumes[index];

            Func<Vector3, float> before = volume.type == DensityFunction.Type.Cuboid
                ? ( point ) => cuboidAsBefore    ( point, volume.origin, volume.extents )
                : ( point ) => ellipsoidAsBefore ( point, volume.origin, volume.extents );

            var unset = new FixedDensityFunction( volume.type, volume.origin, default, volume.extents, Combination.Union, MaterialIndex.Material0 );

            foreach( var point in randomPoints( 6 ).Append( volume.origin ) ) {
                var expected = before( point );

                Assert.AreEqual( expected, volume.sample( point ),           $"volume, {volume.type} at {point}" );
                Assert.AreEqual( expected, snapshots[index].sample( point ), $"snapshot, {volume.type} at {point}" );
                Assert.AreEqual( expected, unset.sample( point ),            $"zero quaternion, {volume.type} at {point}" );
            }
        }
    }

}
