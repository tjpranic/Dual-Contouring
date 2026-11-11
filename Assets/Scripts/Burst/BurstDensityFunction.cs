using System.Collections.Generic;
using System.Linq;
using Unity.Collections;
using Unity.Mathematics;

using Combination   = DensityFunction.Combination;
using MaterialIndex = SurfaceExtractor.MaterialIndex;

// a density function as Burst code sees it: what a volume is at the time it is contoured, in voxelizer object space.
// The shapes are those of Cuboid and Ellipsoid, written out operation for operation so that they give the same densities
public struct BurstDensityFunction {

    public DensityFunction.Type type;
    public Combination          combination;
    public MaterialIndex        materialIndex;
    public float3               origin;
    public float3               extents;

    public BurstDensityFunction( DensityFunction densityFunction ) {
        this.type          = densityFunction.type;
        this.combination   = densityFunction.combination;
        this.materialIndex = densityFunction.materialIndex;
        this.origin        = densityFunction.origin;
        this.extents       = densityFunction.extents;
    }

    // the density functions in the same order, for jobs to read. The caller disposes of the array
    public static NativeArray<BurstDensityFunction> snapshot( IEnumerable<DensityFunction> densityFunctions, Allocator allocator ) {
        return new NativeArray<BurstDensityFunction>( densityFunctions.Select( ( densityFunction ) => new BurstDensityFunction( densityFunction ) ).ToArray( ), allocator );
    }

    public readonly float sample( float3 position ) {
        return this.type switch {
            DensityFunction.Type.Cuboid    => sampleCuboid    ( position, this.origin, this.extents ),
            DensityFunction.Type.Ellipsoid => sampleEllipsoid ( position, this.origin, this.extents ),
            _                              => float.NaN
        };
    }

    // Cuboid.sample
    public static float sampleCuboid( float3 position, float3 origin, float3 extents ) {
        var q = math.abs( position - origin ) - extents;

        var outside = new float3(
            BurstSurfaceExtractor.max( q.x, 0.0f ),
            BurstSurfaceExtractor.max( q.y, 0.0f ),
            BurstSurfaceExtractor.max( q.z, 0.0f )
        );

        return BurstSurfaceExtractor.magnitude( outside ) + BurstSurfaceExtractor.min( BurstSurfaceExtractor.max( q.x, BurstSurfaceExtractor.max( q.y, q.z ) ), 0.0f );
    }

    // Ellipsoid.sample
    public static float sampleEllipsoid( float3 position, float3 origin, float3 extents ) {
        var offset = position - origin;

        // Unity's == for vectors: closer than 1e-5 counts as equal
        if( ( offset.x * offset.x ) + ( offset.y * offset.y ) + ( offset.z * offset.z ) < 1e-5f * 1e-5f ) {
            return -1.0f;
        }

        var k0 = BurstSurfaceExtractor.magnitude( offset / extents );
        var k1 = BurstSurfaceExtractor.magnitude( offset / ( extents * extents ) );

        return k0 * ( k0 - 1.0f ) / k1;
    }

}
