// stolen from: https://github.com/ashima/webgl-noise
// and: https://gist.github.com/h3r/3a92295517b2bee8a82c1de1456431dc

// https://forum.unity.com/threads/translating-a-glsl-shader-noise-algorithm-to-hlsl-cg.485750/
#define mod( x, y ) ( x - y * floor( x / y ) )

#define INVERSE_OF_289 0.00346020761f
#define INVERSE_OF_7 0.14285714285f

float3 mod289( float3 x ) {
    return x - floor( x * ( INVERSE_OF_289 ) ) * 289.0f;
}

float4 mod289( float4 x ) {
    return x - floor( x * ( INVERSE_OF_289 ) ) * 289.0f;
}

float4 permute( float4 x ) {
    return mod289( ( ( x * 34.0f ) + 10.0f ) * x );
}

float4 taylorInvSqrt( float4 r ) {
    return 1.79284291400159f - 0.85373472095314f * r;
}

float3 fade( float3 t ) {
    return t * t * t * ( t * ( t * 6.0f - 15.0f ) + 10.0f );
}

// classic perlin noise
float perlinNoise( float3 p ) {
    float3 pi0 = floor( p );
    float3 pi1 = pi0 + float3( 1.0f, 1.0f, 1.0f );

    pi0 = mod289( pi0 );
    pi1 = mod289( pi1 );

    float3 pf0 = frac( p );
    float3 pf1 = pf0 - float3( 1.0f, 1.0f, 1.0f );

    float4 ix  = float4( pi0.x, pi1.x, pi0.x, pi1.x );
    float4 iy  = float4( pi0.yy, pi1.yy );
    float4 iz0 = pi0.zzzz;
    float4 iz1 = pi1.zzzz;

    float4 ixy  = permute( permute( ix ) + iy );
    float4 ixy0 = permute( ixy + iz0 );
    float4 ixy1 = permute( ixy + iz1 );

    float4 gx0 = ixy0 * INVERSE_OF_7;
    float4 gy0 = frac( floor( gx0 ) * INVERSE_OF_7 ) - 0.5f;

    gx0 = frac( gx0 );

    float4 gz0 = float4( 0.5f, 0.5f, 0.5f, 0.5f ) - abs( gx0 ) - abs( gy0 );
    float4 sz0 = step( gz0, float4( 0.0f, 0.0f, 0.0f, 0.0f ) );

    gx0 -= sz0 * ( step( 0.0f, gx0 ) - 0.5f );
    gy0 -= sz0 * ( step( 0.0f, gy0 ) - 0.5f );

    float4 gx1 = ixy1 * INVERSE_OF_7;
    float4 gy1 = frac( floor( gx1 ) * INVERSE_OF_7 ) - 0.5f;

    gx1 = frac( gx1 );

    float4 gz1 = float4( 0.5f, 0.5f, 0.5f, 0.5f ) - abs( gx1 ) - abs( gy1 );
    float4 sz1 = step( gz1, float4( 0.0f, 0.0f, 0.0f, 0.0f ) );

    gx1 -= sz1 * ( step( 0.0f, gx1 ) - 0.5f );
    gy1 -= sz1 * ( step( 0.0f, gy1 ) - 0.5f );

    float3 g000 = float3( gx0.x, gy0.x, gz0.x );
    float3 g100 = float3( gx0.y, gy0.y, gz0.y );
    float3 g010 = float3( gx0.z, gy0.z, gz0.z );
    float3 g110 = float3( gx0.w, gy0.w, gz0.w );
    float3 g001 = float3( gx1.x, gy1.x, gz1.x );
    float3 g101 = float3( gx1.y, gy1.y, gz1.y );
    float3 g011 = float3( gx1.z, gy1.z, gz1.z );
    float3 g111 = float3( gx1.w, gy1.w, gz1.w );

    float4 norm0 = taylorInvSqrt( float4( dot( g000, g000 ), dot( g010, g010 ), dot( g100, g100 ), dot( g110, g110 ) ) );

    g000 *= norm0.x;
    g010 *= norm0.y;
    g100 *= norm0.z;
    g110 *= norm0.w;

    float4 norm1 = taylorInvSqrt( float4( dot( g001, g001 ), dot( g011, g011 ), dot( g101, g101 ), dot( g111, g111 ) ) );

    g001 *= norm1.x;
    g011 *= norm1.y;
    g101 *= norm1.z;
    g111 *= norm1.w;

    float n000 = dot( g000, pf0 );
    float n100 = dot( g100, float3( pf1.x,  pf0.yz ) );
    float n010 = dot( g010, float3( pf0.x,  pf1.y, pf0.z ) );
    float n110 = dot( g110, float3( pf1.xy, pf0.z ) );
    float n001 = dot( g001, float3( pf0.xy, pf1.z ) );
    float n101 = dot( g101, float3( pf1.x,  pf0.y, pf1.z ) );
    float n011 = dot( g011, float3( pf0.x,  pf1.yz ) );
    float n111 = dot( g111, pf1 );

    float3 fxyz = fade( pf0 );
    float4 nz   = lerp( float4( n000, n100, n010, n110 ), float4( n001, n101, n011, n111 ), fxyz.z );
    float2 nyz  = lerp( nz.xy, nz.zw, fxyz.y );
    float  nxyz = lerp( nyz.x, nyz.y, fxyz.x );

    return 2.2f * nxyz;
}