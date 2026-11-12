using System;
using System.Linq;
using NUnit.Framework;
using Unity.Burst;
using Unity.Collections;
using UnityEngine;

using Combination   = DensityFunction.Combination;
using MaterialIndex = SurfaceExtractor.MaterialIndex;

// what the tests of the Burst code share: making sure a job really ran as Burst compiled code, and density functions
// that are given their origin and extents instead of reading them off a transform
public static class BurstTestSupport {

    // Burst leaves calls to this out of the code it compiles, so it only sets the flag in a job that runs as managed code
    [BurstDiscard]
    private static void markManaged( ref bool managed ) {
        managed = true;
    }

    public static bool runningManaged( ) {
        var managed = false;

        markManaged( ref managed );

        return managed;
    }

    // a test of Burst code that ran as managed code would only test that the managed code agrees with itself
    public static void assertRanUnderBurst( NativeArray<bool> managed ) {
        Assert.IsTrue( BurstCompiler.IsEnabled, "Burst compilation is off (Jobs > Burst > Enable Compilation)" );
        Assert.IsFalse( managed[0], "the job ran as managed code, not as Burst compiled code" );
    }

    public class FixedDensityFunction : DensityFunction {

        public DensityFunction.Type type          { get; }
        public Vector3              origin        { get; }
        public Quaternion           rotation      { get; }
        public Vector3              extents       { get; }
        public Combination          combination   { get; set; }
        public MaterialIndex        materialIndex { get; set; }

        private readonly DensityFunction.Frame frame;

        public FixedDensityFunction( DensityFunction.Type type, Vector3 origin, Vector3 extents, Combination combination, MaterialIndex materialIndex )
            : this( type, origin, Quaternion.identity, extents, combination, materialIndex ) {
        }

        public FixedDensityFunction( DensityFunction.Type type, Vector3 origin, Quaternion rotation, Vector3 extents, Combination combination, MaterialIndex materialIndex ) {
            this.type          = type;
            this.origin        = origin;
            this.rotation      = rotation;
            this.extents       = extents;
            this.combination   = combination;
            this.materialIndex = materialIndex;

            this.frame = new DensityFunction.Frame( origin, rotation );
        }

        // the same shape with the given rotation
        public FixedDensityFunction rotated( Quaternion rotation ) {
            return new( this.type, this.origin, rotation, this.extents, this.combination, this.materialIndex );
        }

        public float sample( Vector3 position ) {
            return DensityFunction.sample( this.type, this.frame.local( position ), this.extents );
        }

    }

    public static float range( System.Random random, float minimum, float maximum ) {
        return minimum + ( ( float )random.NextDouble( ) * ( maximum - minimum ) );
    }

    public static Vector3 range( System.Random random, Vector3 minimum, Vector3 maximum ) {
        return new Vector3( range( random, minimum.x, maximum.x ), range( random, minimum.y, maximum.y ), range( random, minimum.z, maximum.z ) );
    }

    public static Vector3 direction( System.Random random ) {
        return range( random, -Vector3.one, Vector3.one ).normalized;
    }

    // the materials volumes are given in the tests, including none and two at once
    public static readonly MaterialIndex[] materials = {
        MaterialIndex.Void,
        MaterialIndex.Material0,
        MaterialIndex.Material1,
        MaterialIndex.Material2,
        MaterialIndex.Material0 | MaterialIndex.Material1,
        MaterialIndex.Material3
    };

    // shapes of every type in the grid, combined in every way, of every material. The first one is a union, so that
    // there is something for the others to cut into or intersect with
    public static FixedDensityFunction[] randomShapes( System.Random random, int count ) {
        return Enumerable.Range( 0, count ).Select(
            ( index ) => new FixedDensityFunction(
                random.Next( 2 ) == 0 ? DensityFunction.Type.Cuboid : DensityFunction.Type.Ellipsoid,
                range( random, -Vector3.one * 0.4f, Vector3.one * 0.4f ),
                range( random, Vector3.one * 0.05f, Vector3.one * 0.4f ),
                index == 0 ? Combination.Union : ( Combination )random.Next( 3 ),
                materials[random.Next( materials.Length )]
            )
        ).ToArray( );
    }

    // a turn by any angle about any axis
    public static Quaternion randomRotation( System.Random random ) {
        return Quaternion.AngleAxis( range( random, -180.0f, 180.0f ), direction( random ) );
    }

    // the shapes, each turned by a rotation of its own. A test draws the rotations from a generator of their own, so that
    // everything else it draws stays the same as without them
    public static FixedDensityFunction[] randomlyRotated( FixedDensityFunction[] shapes, System.Random random ) {
        return shapes.Select( ( shape ) => shape.rotated( randomRotation( random ) ) ).ToArray( );
    }

    public static NativeArray<BurstDensityFunction> toNative( FixedDensityFunction[] shapes ) {
        return BurstDensityFunction.snapshot( shapes, Allocator.TempJob );
    }

    // Mono keeps the intermediate results of an expression at a higher precision than float and rounds them when it
    // stores them, Burst rounds every operation to float. The same operations in the same order therefore differ in the
    // last bits, and results are compared within a tolerance: relative to the size of the value, and never tighter than
    // the tolerance itself. NaN agrees with NaN
    private static bool close( float expected, float actual, float tolerance ) {
        if( float.IsNaN( expected ) || float.IsNaN( actual ) ) {
            return float.IsNaN( expected ) && float.IsNaN( actual );
        }
        if( float.IsInfinity( expected ) || float.IsInfinity( actual ) ) {
            return expected == actual;
        }
        return Math.Abs( expected - actual ) <= tolerance * Math.Max( 1.0f, Math.Abs( expected ) );
    }

    public static void assertClose( string what, float[] expected, float[] actual, float tolerance ) {
        Assert.AreEqual( expected.Length, actual.Length, $"number of {what}" );

        var far = Enumerable.Range( 0, expected.Length ).Where( ( index ) => !close( expected[index], actual[index], tolerance ) ).ToArray( );

        Assert.AreEqual(
            0,
            far.Length,
            $"{far.Length} of {expected.Length} {what} differ by more than {tolerance}:\n" + string.Join( "\n", far.Take( 5 ).Select( ( index ) => $"  [{index}] managed {expected[index]:R}, Burst {actual[index]:R}" ) )
        );
    }

    public static void assertClose( string what, Vector3[] expected, Vector3[] actual, float tolerance ) {
        Assert.AreEqual( expected.Length, actual.Length, $"number of {what}" );

        var far = Enumerable.Range( 0, expected.Length ).Where(
            ( index ) => !( close( expected[index].x, actual[index].x, tolerance ) && close( expected[index].y, actual[index].y, tolerance ) && close( expected[index].z, actual[index].z, tolerance ) )
        ).ToArray( );

        Assert.AreEqual(
            0,
            far.Length,
            $"{far.Length} of {expected.Length} {what} differ by more than {tolerance}:\n" + string.Join( "\n", far.Take( 5 ).Select(
                ( index ) => $"  [{index}] managed ({expected[index].x:R}, {expected[index].y:R}, {expected[index].z:R}), Burst ({actual[index].x:R}, {actual[index].y:R}, {actual[index].z:R})"
            ) )
        );
    }

    public static void assertSame<T>( string what, T[] expected, T[] actual ) {
        var description = describeMismatches( expected, actual, ( index ) => $"  [{index}] managed {expected[index]}, Burst {actual[index]}", out var mismatches );

        Assert.AreEqual( 0, mismatches, $"{mismatches} of {expected.Length} {what} differ:\n{description}" );
    }

    // how many of the pairs differ, and a description of the first few
    public static string describeMismatches<T>( T[] expected, T[] actual, Func<int, string> describe, out int mismatches ) {
        Assert.AreEqual( expected.Length, actual.Length, "lengths" );

        var first = Enumerable.Range( 0, expected.Length ).Where( ( index ) => !Equals( expected[index], actual[index] ) ).ToArray( );

        mismatches = first.Length;

        return string.Join( "\n", first.Take( 5 ).Select( describe ) );
    }

}
