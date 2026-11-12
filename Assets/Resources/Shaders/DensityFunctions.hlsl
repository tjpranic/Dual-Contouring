// the volumes as the compute shaders sample them: the layout a density function is uploaded in, and each shape as an
// exact signed distance. The shaders of Uniform and of Occupancy-Based Dual Contouring both include this

#define DENSITY_FUNCTION_TYPE_CUBOID 0
#define DENSITY_FUNCTION_TYPE_ELLIPSOID 1
#define DENSITY_FUNCTION_TYPE_CYLINDER 2
#define DENSITY_FUNCTION_TYPE_CONE 3
#define DENSITY_FUNCTION_TYPE_CAPSULE 4
#define DENSITY_FUNCTION_TYPE_TORUS 5
#define DENSITY_FUNCTION_TYPE_PYRAMID 6
#define DENSITY_FUNCTION_TYPE_OCTAHEDRON 7
#define DENSITY_FUNCTION_TYPE_HEXAGONAL_PRISM 8

#define DENSITY_FUNCTION_COMBINATION_UNION 0
#define DENSITY_FUNCTION_COMBINATION_INTERSECTION 1
#define DENSITY_FUNCTION_COMBINATION_SUBTRACTION 2

// DensityFunction.Data in C#, field for field. The axes are the rows of the inverse rotation
struct DensityFunction {
    uint   type;
    uint   combination;
    uint   materialIndex;
    float3 origin;
    float3 extents;
    float3 xAxis;
    float3 yAxis;
    float3 zAxis;
};

// element-wise multiply
float3 multiply( float3 v1, float3 v2 ) {
    return float3( v1.x * v2.x, v1.y * v2.y, v1.z * v2.z );
}

// element-wise divide
float3 divide( float3 v1, float3 v2 ) {
    return float3( v1.x / v2.x, v1.y / v2.y, v1.z / v2.z );
}

// element-wise absolute value
float3 absolute( float3 v ) {
    return float3( abs( v.x ), abs( v.y ), abs( v.z ) );
}

// DensityFunction.Frame.local in C#: the position relative to the origin, along the shape's own axes. Every shape is
// taken into its frame. The axes of an unrotated shape are exactly those of the grid, which leave the offset as it is
float3 toFrame( float3 position, in DensityFunction densityFunction ) {
    float3 offset = position - densityFunction.origin;

    return float3(
        ( densityFunction.xAxis.x * offset.x ) + ( densityFunction.xAxis.y * offset.y ) + ( densityFunction.xAxis.z * offset.z ),
        ( densityFunction.yAxis.x * offset.x ) + ( densityFunction.yAxis.y * offset.y ) + ( densityFunction.yAxis.z * offset.z ),
        ( densityFunction.zAxis.x * offset.x ) + ( densityFunction.zAxis.y * offset.y ) + ( densityFunction.zAxis.z * offset.z )
    );
}

// a cuboid centred on the origin of its own frame, at a position in that frame
float sampleCuboid( float3 position, float3 extents ) {
    float3 q = absolute( position ) - extents;
    return length( max( q, float3( 0.0f, 0.0f, 0.0f ) ) ) + min( max( q.x, max( q.y, q.z ) ), 0.0f );
}

// an ellipsoid centred on the origin of its own frame, at a position in that frame
float sampleEllipsoid( float3 position, float3 extents ) {
    // avoid division by zero
    if( length( position ) == 0.0f ) {
        return -1.0f;
    }
    else {
        float k0 = length( divide( position, extents ) );
        float k1 = length( divide( position, multiply( extents, extents ) ) );

        return k0 * ( k0 - 1.0f ) / k1;
    }
}

// Cylinder.sample in C#
float sampleCylinder( float3 position, float3 extents ) {
    float radius = min( extents.x, extents.z );

    float side = sqrt( ( position.x * position.x ) + ( position.z * position.z ) ) - radius;
    float caps = abs( position.y ) - extents.y;

    float outsideSide = max( side, 0.0f );
    float outsideCaps = max( caps, 0.0f );

    return min( max( side, caps ), 0.0f ) + sqrt( ( outsideSide * outsideSide ) + ( outsideCaps * outsideCaps ) );
}

// Cone.sample in C#
float sampleCone( float3 position, float3 extents ) {
    float radius = min( extents.x, extents.z );

    float positionAcross = sqrt( ( position.x * position.x ) + ( position.z * position.z ) );
    float positionAlong  = position.y - extents.y;
    float rimAcross      = radius;
    float rimAlong       = -2.0f * extents.y;

    float t = clamp( ( ( positionAcross * rimAcross ) + ( positionAlong * rimAlong ) ) / ( ( rimAcross * rimAcross ) + ( rimAlong * rimAlong ) ), 0.0f, 1.0f );

    float toSideAcross = positionAcross - ( rimAcross * t );
    float toSideAlong  = positionAlong  - ( rimAlong  * t );
    float toBaseAcross = positionAcross - ( rimAcross * clamp( positionAcross / rimAcross, 0.0f, 1.0f ) );
    float toBaseAlong  = positionAlong  - rimAlong;

    float squared = min(
        ( toSideAcross * toSideAcross ) + ( toSideAlong * toSideAlong ),
        ( toBaseAcross * toBaseAcross ) + ( toBaseAlong * toBaseAlong )
    );

    float outside = max( ( positionAlong * rimAcross ) - ( positionAcross * rimAlong ), rimAlong - positionAlong );

    return outside < 0.0f ? -sqrt( squared ) : sqrt( squared );
}

// Capsule.sample in C#
float sampleCapsule( float3 position, float3 extents ) {
    float radius = min( min( extents.x, extents.z ), extents.y );
    float straight = extents.y - radius;

    float along = position.y - clamp( position.y, -straight, straight );

    return sqrt( ( position.x * position.x ) + ( along * along ) + ( position.z * position.z ) ) - radius;
}

// Torus.sample in C#
float sampleTorus( float3 position, float3 extents ) {
    float across = min( extents.x, extents.z );
    float tube   = min( extents.y, across );
    float ring   = across - tube;

    float fromRing = sqrt( ( position.x * position.x ) + ( position.z * position.z ) ) - ring;

    return sqrt( ( fromRing * fromRing ) + ( position.y * position.y ) ) - tube;
}

// Pyramid.sample in C#
float samplePyramid( float3 position, float3 extents ) {
    float side   = 2.0f * min( extents.x, extents.z );
    float height = 2.0f * extents.y / side;

    float x = position.x / side;
    float y = ( position.y + extents.y ) / side;
    float z = position.z / side;

    if( y <= 0.0f ) {
        float offBaseX = max( abs( x ) - 0.5f, 0.0f );
        float offBaseZ = max( abs( z ) - 0.5f, 0.0f );

        return sqrt( ( offBaseX * offBaseX ) + ( y * y ) + ( offBaseZ * offBaseZ ) ) * side;
    }
    else {
        float m2 = ( height * height ) + 0.25f;

        float px = abs( x );
        float pz = abs( z );

        if( pz > px ) {
            float swapped = px;

            px = pz;
            pz = swapped;
        }

        px -= 0.5f;
        pz -= 0.5f;

        float qx = pz;
        float qy = ( height * y ) - ( 0.5f * px );
        float qz = ( height * px ) + ( 0.5f * y );

        float s = max( -qx, 0.0f );
        float t = clamp( ( qy - ( 0.5f * pz ) ) / ( m2 + 0.25f ), 0.0f, 1.0f );

        float a = ( m2 * ( qx + s ) * ( qx + s ) ) + ( qy * qy );
        float b = ( m2 * ( qx + ( 0.5f * t ) ) * ( qx + ( 0.5f * t ) ) ) + ( ( qy - ( m2 * t ) ) * ( qy - ( m2 * t ) ) );

        float squared   = min( qy, ( -qx * m2 ) - ( qy * 0.5f ) ) > 0.0f ? 0.0f : min( a, b );
        float fromSides = sqrt( ( squared + ( qz * qz ) ) / m2 );

        if( qz < 0.0f ) {
            return max( -fromSides, -y ) * side;
        }
        else {
            return fromSides * side;
        }
    }
}

// the distance from the point p to the segment from a to b, Octahedron.segment in C#
float distanceToSegment( float3 p, float3 a, float3 b ) {
    float3 pa = p - a;
    float3 ba = b - a;

    float t = clamp( ( ( pa.x * ba.x ) + ( pa.y * ba.y ) + ( pa.z * ba.z ) ) / ( ( ba.x * ba.x ) + ( ba.y * ba.y ) + ( ba.z * ba.z ) ), 0.0f, 1.0f );

    float3 d = pa - ( ba * t );

    return sqrt( ( d.x * d.x ) + ( d.y * d.y ) + ( d.z * d.z ) );
}

// Octahedron.sample in C#
float sampleOctahedron( float3 position, float3 extents ) {
    float3 p = absolute( position );

    float nx = 1.0f / extents.x;
    float ny = 1.0f / extents.y;
    float nz = 1.0f / extents.z;

    float nn = ( nx * nx ) + ( ny * ny ) + ( nz * nz );
    float m  = ( p.x * nx ) + ( p.y * ny ) + ( p.z * nz ) - 1.0f;

    float plane = m / sqrt( nn );
    float scale = m / nn;

    if( m <= 0.0f ) {
        return plane;
    }
    else if( p.x - ( nx * scale ) < 0.0f ) {
        return distanceToSegment( p, float3( 0.0f, extents.y, 0.0f ), float3( 0.0f, 0.0f, extents.z ) );
    }
    else if( p.y - ( ny * scale ) < 0.0f ) {
        return distanceToSegment( p, float3( 0.0f, 0.0f, extents.z ), float3( extents.x, 0.0f, 0.0f ) );
    }
    else if( p.z - ( nz * scale ) < 0.0f ) {
        return distanceToSegment( p, float3( extents.x, 0.0f, 0.0f ), float3( 0.0f, extents.y, 0.0f ) );
    }
    else {
        return plane;
    }
}

// cos 30 and tan 30, HexagonalPrism.cos30 and HexagonalPrism.tan30 in C#
#define COS30 0.8660254f
#define TAN30 0.57735027f

// HexagonalPrism.sample in C#
float sampleHexagonalPrism( float3 position, float3 extents ) {
    float inradius = min( extents.z, extents.x * COS30 );

    float across = abs( position.x );
    float up     = abs( position.z );
    float along  = abs( position.y );

    float reflection = 2.0f * min( ( -COS30 * across ) + ( 0.5f * up ), 0.0f );

    across -= reflection * -COS30;
    up     -= reflection * 0.5f;

    float offSideAcross = across - clamp( across, -TAN30 * inradius, TAN30 * inradius );
    float offSideUp     = up - inradius;

    float fromSide = sqrt( ( offSideAcross * offSideAcross ) + ( offSideUp * offSideUp ) );

    float side = offSideUp < 0.0f ? -fromSide : fromSide;
    float ends = along - extents.y;

    float outsideSide = max( side, 0.0f );
    float outsideEnds = max( ends, 0.0f );

    return min( max( side, ends ), 0.0f ) + sqrt( ( outsideSide * outsideSide ) + ( outsideEnds * outsideEnds ) );
}

// #include "Noise.hlsl"

// the shader compiler arranges float operations as it sees fit, so unrotated shapes come out a unit or two in the last
// place apart from the shader before shapes could rotate, which sampled them without the rotation
float sampleDensityFunction( float3 position, in DensityFunction densityFunction ) {
    float3 inFrame = toFrame( position, densityFunction );

    switch( densityFunction.type ) {
        case DENSITY_FUNCTION_TYPE_CUBOID:
            return sampleCuboid( inFrame, densityFunction.extents );
        case DENSITY_FUNCTION_TYPE_ELLIPSOID:
            return sampleEllipsoid( inFrame, densityFunction.extents )/*- ( ( ( perlinNoise( position * 1.0f ) + 1 ) / 2 ) * 0.25f )*/;
        case DENSITY_FUNCTION_TYPE_CYLINDER:
            return sampleCylinder( inFrame, densityFunction.extents );
        case DENSITY_FUNCTION_TYPE_CONE:
            return sampleCone( inFrame, densityFunction.extents );
        case DENSITY_FUNCTION_TYPE_CAPSULE:
            return sampleCapsule( inFrame, densityFunction.extents );
        case DENSITY_FUNCTION_TYPE_TORUS:
            return sampleTorus( inFrame, densityFunction.extents );
        case DENSITY_FUNCTION_TYPE_PYRAMID:
            return samplePyramid( inFrame, densityFunction.extents );
        case DENSITY_FUNCTION_TYPE_OCTAHEDRON:
            return sampleOctahedron( inFrame, densityFunction.extents );
        case DENSITY_FUNCTION_TYPE_HEXAGONAL_PRISM:
            return sampleHexagonalPrism( inFrame, densityFunction.extents );
        default:
            return 0.0f;
    }
}
