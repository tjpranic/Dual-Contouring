using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Mathematics;

using Combination   = DensityFunction.Combination;
using MaterialIndex = SurfaceExtractor.MaterialIndex;

// a density function as Burst code sees it: what a volume is at the time it is contoured, in voxelizer object space.
// The shapes are those of the volumes, Cuboid, Ellipsoid and the others, written out operation for operation so that
// they give the same densities
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

    // with nine shapes to choose from, Burst no longer inlines this into the loops that sample, and contours of cuboids
    // and ellipsoids took a quarter longer, up to a half. Inlined, they take an eighth longer (New Shapes, step 2)
    [MethodImpl( MethodImplOptions.AggressiveInlining )]
    public readonly float sample( float3 position ) {
        var local = this.local( position );

        return this.type switch {
            DensityFunction.Type.Cuboid         => sampleCuboid         ( local, this.extents ),
            DensityFunction.Type.Ellipsoid      => sampleEllipsoid      ( local, this.extents ),
            DensityFunction.Type.Cylinder       => sampleCylinder       ( local, this.extents ),
            DensityFunction.Type.Cone           => sampleCone           ( local, this.extents ),
            DensityFunction.Type.Capsule        => sampleCapsule        ( local, this.extents ),
            DensityFunction.Type.Torus          => sampleTorus          ( local, this.extents ),
            DensityFunction.Type.Pyramid        => samplePyramid        ( local, this.extents ),
            DensityFunction.Type.Octahedron     => sampleOctahedron     ( local, this.extents ),
            DensityFunction.Type.HexagonalPrism => sampleHexagonalPrism ( local, this.extents ),
            _                                   => float.NaN
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

    // Cylinder.sample
    public static float sampleCylinder( float3 position, float3 extents ) {
        var radius = BurstSurfaceExtractor.min( extents.x, extents.z );

        var side = math.sqrt( ( position.x * position.x ) + ( position.z * position.z ) ) - radius;
        var caps = math.abs( position.y ) - extents.y;

        var outsideSide = BurstSurfaceExtractor.max( side, 0.0f );
        var outsideCaps = BurstSurfaceExtractor.max( caps, 0.0f );

        return BurstSurfaceExtractor.min( BurstSurfaceExtractor.max( side, caps ), 0.0f ) + math.sqrt( ( outsideSide * outsideSide ) + ( outsideCaps * outsideCaps ) );
    }

    // Cone.sample
    public static float sampleCone( float3 position, float3 extents ) {
        var radius = BurstSurfaceExtractor.min( extents.x, extents.z );

        var positionAcross = math.sqrt( ( position.x * position.x ) + ( position.z * position.z ) );
        var positionAlong  = position.y - extents.y;
        var rimAcross      = radius;
        var rimAlong       = -2.0f * extents.y;

        var t = BurstSurfaceExtractor.clamp( ( ( positionAcross * rimAcross ) + ( positionAlong * rimAlong ) ) / ( ( rimAcross * rimAcross ) + ( rimAlong * rimAlong ) ), 0.0f, 1.0f );

        var toSideAcross = positionAcross - ( rimAcross * t );
        var toSideAlong  = positionAlong  - ( rimAlong  * t );
        var toBaseAcross = positionAcross - ( rimAcross * BurstSurfaceExtractor.clamp( positionAcross / rimAcross, 0.0f, 1.0f ) );
        var toBaseAlong  = positionAlong  - rimAlong;

        var squared = BurstSurfaceExtractor.min(
            ( toSideAcross * toSideAcross ) + ( toSideAlong * toSideAlong ),
            ( toBaseAcross * toBaseAcross ) + ( toBaseAlong * toBaseAlong )
        );

        var outside = BurstSurfaceExtractor.max( ( positionAlong * rimAcross ) - ( positionAcross * rimAlong ), rimAlong - positionAlong );

        return outside < 0.0f ? -math.sqrt( squared ) : math.sqrt( squared );
    }

    // Capsule.sample
    public static float sampleCapsule( float3 position, float3 extents ) {
        var radius = BurstSurfaceExtractor.min( BurstSurfaceExtractor.min( extents.x, extents.z ), extents.y );
        var half   = extents.y - radius;

        var along = position.y - BurstSurfaceExtractor.clamp( position.y, -half, half );

        return math.sqrt( ( position.x * position.x ) + ( along * along ) + ( position.z * position.z ) ) - radius;
    }

    // Torus.sample
    public static float sampleTorus( float3 position, float3 extents ) {
        var across = BurstSurfaceExtractor.min( extents.x, extents.z );
        var tube   = BurstSurfaceExtractor.min( extents.y, across );
        var ring   = across - tube;

        var fromRing = math.sqrt( ( position.x * position.x ) + ( position.z * position.z ) ) - ring;

        return math.sqrt( ( fromRing * fromRing ) + ( position.y * position.y ) ) - tube;
    }

    // Pyramid.sample
    public static float samplePyramid( float3 position, float3 extents ) {
        var side   = 2.0f * BurstSurfaceExtractor.min( extents.x, extents.z );
        var height = 2.0f * extents.y / side;

        var x = position.x / side;
        var y = ( position.y + extents.y ) / side;
        var z = position.z / side;

        if( y <= 0.0f ) {
            var offBaseX = BurstSurfaceExtractor.max( math.abs( x ) - 0.5f, 0.0f );
            var offBaseZ = BurstSurfaceExtractor.max( math.abs( z ) - 0.5f, 0.0f );

            return math.sqrt( ( offBaseX * offBaseX ) + ( y * y ) + ( offBaseZ * offBaseZ ) ) * side;
        }

        var m2 = ( height * height ) + 0.25f;

        var px = math.abs( x );
        var pz = math.abs( z );

        if( pz > px ) {
            var swapped = px;

            px = pz;
            pz = swapped;
        }

        px -= 0.5f;
        pz -= 0.5f;

        var qx = pz;
        var qy = ( height * y ) - ( 0.5f * px );
        var qz = ( height * px ) + ( 0.5f * y );

        var s = BurstSurfaceExtractor.max( -qx, 0.0f );
        var t = BurstSurfaceExtractor.clamp( ( qy - ( 0.5f * pz ) ) / ( m2 + 0.25f ), 0.0f, 1.0f );

        var a = ( m2 * ( qx + s ) * ( qx + s ) ) + ( qy * qy );
        var b = ( m2 * ( qx + ( 0.5f * t ) ) * ( qx + ( 0.5f * t ) ) ) + ( ( qy - ( m2 * t ) ) * ( qy - ( m2 * t ) ) );

        var squared  = BurstSurfaceExtractor.min( qy, ( -qx * m2 ) - ( qy * 0.5f ) ) > 0.0f ? 0.0f : BurstSurfaceExtractor.min( a, b );
        var distance = math.sqrt( ( squared + ( qz * qz ) ) / m2 );

        if( qz < 0.0f ) {
            return BurstSurfaceExtractor.max( -distance, -y ) * side;
        }

        return distance * side;
    }

    // Octahedron.sample
    public static float sampleOctahedron( float3 position, float3 extents ) {
        var px = math.abs( position.x );
        var py = math.abs( position.y );
        var pz = math.abs( position.z );

        var nx = 1.0f / extents.x;
        var ny = 1.0f / extents.y;
        var nz = 1.0f / extents.z;

        var nn = ( nx * nx ) + ( ny * ny ) + ( nz * nz );
        var m  = ( px * nx ) + ( py * ny ) + ( pz * nz ) - 1.0f;

        var plane = m / math.sqrt( nn );

        if( m <= 0.0f ) {
            return plane;
        }

        var scale = m / nn;

        if( px - ( nx * scale ) < 0.0f ) {
            return segment( px, py, pz, 0.0f, extents.y, 0.0f, 0.0f, 0.0f, extents.z );
        }
        if( py - ( ny * scale ) < 0.0f ) {
            return segment( px, py, pz, 0.0f, 0.0f, extents.z, extents.x, 0.0f, 0.0f );
        }
        if( pz - ( nz * scale ) < 0.0f ) {
            return segment( px, py, pz, extents.x, 0.0f, 0.0f, 0.0f, extents.y, 0.0f );
        }

        return plane;
    }

    // Octahedron.segment: the distance from the point p to the segment from a to b
    private static float segment( float px, float py, float pz, float ax, float ay, float az, float bx, float by, float bz ) {
        var pax = px - ax;
        var pay = py - ay;
        var paz = pz - az;
        var bax = bx - ax;
        var bay = by - ay;
        var baz = bz - az;

        var t = BurstSurfaceExtractor.clamp( ( ( pax * bax ) + ( pay * bay ) + ( paz * baz ) ) / ( ( bax * bax ) + ( bay * bay ) + ( baz * baz ) ), 0.0f, 1.0f );

        var dx = pax - ( bax * t );
        var dy = pay - ( bay * t );
        var dz = paz - ( baz * t );

        return math.sqrt( ( dx * dx ) + ( dy * dy ) + ( dz * dz ) );
    }

    // HexagonalPrism.sample
    public static float sampleHexagonalPrism( float3 position, float3 extents ) {
        var inradius = BurstSurfaceExtractor.min( extents.z, extents.x * HexagonalPrism.cos30 );

        var across = math.abs( position.x );
        var up     = math.abs( position.z );
        var along  = math.abs( position.y );

        var reflection = 2.0f * BurstSurfaceExtractor.min( ( -HexagonalPrism.cos30 * across ) + ( 0.5f * up ), 0.0f );

        across -= reflection * -HexagonalPrism.cos30;
        up     -= reflection * 0.5f;

        var offSideAcross = across - BurstSurfaceExtractor.clamp( across, -HexagonalPrism.tan30 * inradius, HexagonalPrism.tan30 * inradius );
        var offSideUp     = up - inradius;

        var fromSide = math.sqrt( ( offSideAcross * offSideAcross ) + ( offSideUp * offSideUp ) );

        var side = offSideUp < 0.0f ? -fromSide : fromSide;
        var ends = along - extents.y;

        var outsideSide = BurstSurfaceExtractor.max( side, 0.0f );
        var outsideEnds = BurstSurfaceExtractor.max( ends, 0.0f );

        return BurstSurfaceExtractor.min( BurstSurfaceExtractor.max( side, ends ), 0.0f ) + math.sqrt( ( outsideSide * outsideSide ) + ( outsideEnds * outsideEnds ) );
    }

}
