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

    // the shape's frame, copied from DensityFunction.Frame: the rows of the inverse rotation, and whether there is a
    // rotation to undo
    public float3 xAxis;
    public float3 yAxis;
    public float3 zAxis;
    public bool   rotated;

    public BurstDensityFunction( DensityFunction densityFunction ) {
        var frame = new DensityFunction.Frame( densityFunction );

        this.type          = densityFunction.type;
        this.combination   = densityFunction.combination;
        this.materialIndex = densityFunction.materialIndex;
        this.origin        = frame.origin;
        this.extents       = densityFunction.extents;
        this.xAxis         = frame.xAxis;
        this.yAxis         = frame.yAxis;
        this.zAxis         = frame.zAxis;
        this.rotated       = frame.rotated;
    }

    // the density functions in the same order, for jobs to read. The caller disposes of the array
    public static NativeArray<BurstDensityFunction> snapshot( IEnumerable<DensityFunction> densityFunctions, Allocator allocator ) {
        return new NativeArray<BurstDensityFunction>( densityFunctions.Select( ( densityFunction ) => new BurstDensityFunction( densityFunction ) ).ToArray( ), allocator );
    }

    // DensityFunction.Frame.local
    public readonly float3 local( float3 position ) {
        var offset = position - this.origin;

        if( !this.rotated ) {
            return offset;
        }

        return new float3(
            ( this.xAxis.x * offset.x ) + ( this.xAxis.y * offset.y ) + ( this.xAxis.z * offset.z ),
            ( this.yAxis.x * offset.x ) + ( this.yAxis.y * offset.y ) + ( this.yAxis.z * offset.z ),
            ( this.zAxis.x * offset.x ) + ( this.zAxis.y * offset.y ) + ( this.zAxis.z * offset.z )
        );
    }

    public readonly float sample( float3 position ) {
        var local = this.local( position );

        return this.type switch {
            DensityFunction.Type.Cuboid    => sampleCuboid    ( local, this.extents ),
            DensityFunction.Type.Ellipsoid => sampleEllipsoid ( local, this.extents ),
            _                              => float.NaN
        };
    }

    // Cuboid.sample
    public static float sampleCuboid( float3 position, float3 extents ) {
        var q = math.abs( position ) - extents;

        var outside = new float3(
            BurstSurfaceExtractor.max( q.x, 0.0f ),
            BurstSurfaceExtractor.max( q.y, 0.0f ),
            BurstSurfaceExtractor.max( q.z, 0.0f )
        );

        return BurstSurfaceExtractor.magnitude( outside ) + BurstSurfaceExtractor.min( BurstSurfaceExtractor.max( q.x, BurstSurfaceExtractor.max( q.y, q.z ) ), 0.0f );
    }

    // Ellipsoid.sample
    public static float sampleEllipsoid( float3 position, float3 extents ) {
        // Unity's == for vectors: closer than 1e-5 counts as equal
        if( ( position.x * position.x ) + ( position.y * position.y ) + ( position.z * position.z ) < 1e-5f * 1e-5f ) {
            return -1.0f;
        }

        var k0 = BurstSurfaceExtractor.magnitude( position / extents );
        var k1 = BurstSurfaceExtractor.magnitude( position / ( extents * extents ) );

        return k0 * ( k0 - 1.0f ) / k1;
    }

}
