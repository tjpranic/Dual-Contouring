#define SVD_TOLERANCE 1e-6f
#define PSEUDO_INVERSE_TOLERANCE 1e-6f

struct SymmetricMatrix3x3 {
    float m00, m01, m02, m11, m12, m22;
};

SymmetricMatrix3x3 createSymmetricMatrix3x3( ) {
    SymmetricMatrix3x3 symmetricMatrix3x3;

    symmetricMatrix3x3.m00 = 0.0f;
    symmetricMatrix3x3.m01 = 0.0f;
    symmetricMatrix3x3.m02 = 0.0f;
    symmetricMatrix3x3.m11 = 0.0f;
    symmetricMatrix3x3.m12 = 0.0f;
    symmetricMatrix3x3.m22 = 0.0f;

    return symmetricMatrix3x3;
}

SymmetricMatrix3x3 createSymmetricMatrix3x3(
    float m00,
    float m01,
    float m02,
    float m11,
    float m12,
    float m22
) {
    SymmetricMatrix3x3 symmetricMatrix3x3;

    symmetricMatrix3x3.m00 = m00;
    symmetricMatrix3x3.m01 = m01;
    symmetricMatrix3x3.m02 = m02;
    symmetricMatrix3x3.m11 = m11;
    symmetricMatrix3x3.m12 = m12;
    symmetricMatrix3x3.m22 = m22;

    return symmetricMatrix3x3;
}

struct Matrix3x3 {
    float m00, m01, m02, m10, m11, m12, m20, m21, m22;
};

Matrix3x3 createMatrix3x3( ) {
    Matrix3x3 matrix3x3;

    matrix3x3.m00 = 0.0f;
    matrix3x3.m01 = 0.0f;
    matrix3x3.m02 = 0.0f;
    matrix3x3.m10 = 0.0f;
    matrix3x3.m11 = 0.0f;
    matrix3x3.m12 = 0.0f;
    matrix3x3.m20 = 0.0f;
    matrix3x3.m21 = 0.0f;
    matrix3x3.m22 = 0.0f;

    return matrix3x3;
}

Matrix3x3 createMatrix3x3(
    float m00,
    float m01,
    float m02,
    float m10,
    float m11,
    float m12,
    float m20,
    float m21,
    float m22
) {
    Matrix3x3 matrix3x3;

    matrix3x3.m00 = m00;
    matrix3x3.m01 = m01;
    matrix3x3.m02 = m02;
    matrix3x3.m10 = m10;
    matrix3x3.m11 = m11;
    matrix3x3.m12 = m12;
    matrix3x3.m20 = m20;
    matrix3x3.m21 = m21;
    matrix3x3.m22 = m22;

    return matrix3x3;
}

Matrix3x3 createMatrix3x3( SymmetricMatrix3x3 symmetricMatrix3x3 ) {
    Matrix3x3 matrix3x3;

    matrix3x3.m00 = symmetricMatrix3x3.m00;
    matrix3x3.m01 = symmetricMatrix3x3.m01;
    matrix3x3.m02 = symmetricMatrix3x3.m02;
    matrix3x3.m10 = 0.0f;
    matrix3x3.m11 = symmetricMatrix3x3.m11;
    matrix3x3.m12 = symmetricMatrix3x3.m12;
    matrix3x3.m20 = 0.0f;
    matrix3x3.m21 = 0.0f;
    matrix3x3.m22 = symmetricMatrix3x3.m22;

    return matrix3x3;
}

struct SVDQEF {
    SymmetricMatrix3x3 ATA;
    float3             ATB;
    float              BTB;
    float3             massPoint;
    uint               intersectionCount;
};

SVDQEF createSVDQEF( ) {
    SVDQEF svdqef;

    svdqef.ATA               = createSymmetricMatrix3x3( );
    svdqef.ATB               = float3( 0.0f, 0.0f, 0.0f );
    svdqef.BTB               = 0.0f;
    svdqef.massPoint         = float3( 0.0f, 0.0f, 0.0f );
    svdqef.intersectionCount = 0;

    return svdqef;
}

float3 multiplyVector( in Matrix3x3 m, float3 v ) {
    return float3(
        ( m.m00 * v.x ) + ( m.m01 * v.y ) + ( m.m02 * v.z ),
        ( m.m10 * v.x ) + ( m.m11 * v.y ) + ( m.m12 * v.z ),
        ( m.m20 * v.x ) + ( m.m21 * v.y ) + ( m.m22 * v.z )
    );
}

Matrix3x3 rotate01Post( in Matrix3x3 V, float c, float s ) {
    return createMatrix3x3(
        ( c * V.m00 ) - ( s * V.m01 ),
        ( s * V.m00 ) + ( c * V.m01 ),
        V.m02,
        ( c * V.m10 ) - ( s * V.m11 ),
        ( s * V.m10 ) + ( c * V.m11 ),
        V.m12,
        ( c * V.m20 ) - ( s * V.m21 ),
        ( s * V.m20 ) + ( c * V.m21 ),
        V.m22
    );
}

Matrix3x3 rotate02Post( in Matrix3x3 V, float c, float s ) {
    return createMatrix3x3(
        ( c * V.m00 ) - ( s * V.m02 ),
        V.m01,
        ( s * V.m00 ) + ( c * V.m02 ),
        ( c * V.m10 ) - ( s * V.m12 ),
        V.m11,
        ( s * V.m10 ) + ( c * V.m12 ),
        ( c * V.m20 ) - ( s * V.m22 ),
        V.m21,
        ( s * V.m20 ) + ( c * V.m22 )
    );
}

Matrix3x3 rotate12Post( in Matrix3x3 V, float c, float s ) {
    return createMatrix3x3(
        V.m00,
        ( c * V.m01 ) - ( s * V.m02 ),
        ( s * V.m01 ) + ( c * V.m02 ),
        V.m10,
        ( c * V.m11 ) - ( s * V.m12 ),
        ( s * V.m11 ) + ( c * V.m12 ),
        V.m20,
        ( c * V.m21 ) - ( s * V.m22 ),
        ( s * V.m21 ) + ( c * V.m22 )
    );
}

float3 multiplyVector( in SymmetricMatrix3x3 m, float3 v ) {
    return float3(
        ( m.m00 * v.x ) + ( m.m01 * v.y ) + ( m.m02 * v.z ),
        ( m.m01 * v.x ) + ( m.m11 * v.y ) + ( m.m12 * v.z ),
        ( m.m02 * v.x ) + ( m.m12 * v.y ) + ( m.m22 * v.z )
    );
}

float frobeniusNorm( in SymmetricMatrix3x3 VTAV ) {
    return sqrt(
        ( VTAV.m00 * VTAV.m00 ) + ( VTAV.m01 * VTAV.m01 ) + ( VTAV.m02 * VTAV.m02 ) +
        ( VTAV.m01 * VTAV.m01 ) + ( VTAV.m11 * VTAV.m11 ) + ( VTAV.m12 * VTAV.m12 ) +
        ( VTAV.m02 * VTAV.m02 ) + ( VTAV.m12 * VTAV.m12 ) + ( VTAV.m22 * VTAV.m22 )
    );
}

float off( in SymmetricMatrix3x3 VTAV ) {
    return sqrt( 2 * ( ( VTAV.m01 * VTAV.m01 ) + ( VTAV.m02 * VTAV.m02 ) + ( VTAV.m12 * VTAV.m12 ) ) );
}

SymmetricMatrix3x3 rotate01( in SymmetricMatrix3x3 VTAV, float c, float s ) {
    float cc  = c * c;
    float ss  = s * s;
    float mix = 2 * c * s * VTAV.m01;

    return createSymmetricMatrix3x3(
        ( cc * VTAV.m00 ) - mix + ( ss * VTAV.m11 ),
        0.0f,
        ( c  * VTAV.m02 ) -       ( s  * VTAV.m12 ),
        ( ss * VTAV.m00 ) + mix + ( cc * VTAV.m11 ),
        ( s  * VTAV.m02 ) +       ( c  * VTAV.m12 ),
        VTAV.m22
    );
}

SymmetricMatrix3x3 rotate02( in SymmetricMatrix3x3 VTAV, float c, float s ) {
    float cc  = c * c;
    float ss  = s * s;
    float mix = 2 * c * s * VTAV.m02;

    return createSymmetricMatrix3x3(
        ( cc * VTAV.m00 ) - mix + ( ss * VTAV.m22 ),
        ( c  * VTAV.m01 ) -       ( s  * VTAV.m12 ),
        0.0f,
        VTAV.m11,
        ( s  * VTAV.m01 ) +       ( c  * VTAV.m12 ),
        ( ss * VTAV.m00 ) + mix + ( cc * VTAV.m22 )
    );
}

SymmetricMatrix3x3 rotate12( in SymmetricMatrix3x3 VTAV, float c, float s ) {
    float cc  = c * c;
    float ss  = s * s;
    float mix = 2 * c * s * VTAV.m12;

    return createSymmetricMatrix3x3(
        VTAV.m00,
        ( c  * VTAV.m01 ) -       ( s  * VTAV.m02 ),
        ( s  * VTAV.m01 ) +       ( c  * VTAV.m02 ),
        ( cc * VTAV.m11 ) - mix + ( ss * VTAV.m22 ),
        0.0f,
        ( ss * VTAV.m11 ) + mix + ( cc * VTAV.m22 )
    );
}

void calculateSymmetricGivensCoefficients( float app, float apq, float aqq, out float c, out float s ) {
    if( apq != 0.0f ) {
        float tau = ( aqq - app ) / ( 2.0f * apq );
        float stt = sqrt( 1.0f + ( tau * tau ) );
        float tan = 1.0f / ( tau >= 0.0f ? tau + stt : tau - stt );

        c = 1.0f / sqrt( 1.0f + ( tan * tan ) );
        s = tan * c;
    }
}

void rotate01( in out SymmetricMatrix3x3 VTAV, in out Matrix3x3 V ) {
    if( VTAV.m01 != 0.0f ) {
        float c = 0.0f;
        float s = 0.0f;

        calculateSymmetricGivensCoefficients( VTAV.m00, VTAV.m01, VTAV.m11, c, s );

        VTAV = rotate01( VTAV, c, s );

        c = 0.0f;
        s = 0.0f;

        V = rotate01Post( V, c, s );
    }
}

void rotate02( in out SymmetricMatrix3x3 VTAV, in out Matrix3x3 V ) {
    if( VTAV.m02 != 0.0f ) {
        float c = 0.0f;
        float s = 0.0f;

        calculateSymmetricGivensCoefficients( VTAV.m00, VTAV.m02, VTAV.m22, c, s );

        VTAV = rotate02( VTAV, c, s );

        c = 0.0f;
        s = 0.0f;

        V = rotate02Post( V, c, s );
    }
}

void rotate12( in out SymmetricMatrix3x3 VTAV, in out Matrix3x3 V ) {
    if( VTAV.m12 != 0.0f ) {
        float c = 0.0f;
        float s = 0.0f;

        calculateSymmetricGivensCoefficients( VTAV.m11, VTAV.m12, VTAV.m22, c, s );

        VTAV = rotate12( VTAV, c, s );

        c = 0.0f;
        s = 0.0f;

        V = rotate12Post( V, c, s );
    }
}

void getSymmetricSVD( in SymmetricMatrix3x3 ATA, out SymmetricMatrix3x3 VTAV, out Matrix3x3 V, uint minimizerIterations ) {
    VTAV = createSymmetricMatrix3x3 ( ATA.m00, ATA.m01, ATA.m02, ATA.m11, ATA.m12, ATA.m22 );
    V    = createMatrix3x3          ( 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f );

    float delta = SVD_TOLERANCE * frobeniusNorm( VTAV );

    for( uint sweep = 0; sweep < minimizerIterations && off( VTAV ) > delta; ++sweep ) {
        rotate01( VTAV, V );
        rotate02( VTAV, V );
        rotate12( VTAV, V );
    }
}

float pseudoInverse( float x ) {
    if( abs( x ) < PSEUDO_INVERSE_TOLERANCE ) {
        return 0.0f;
    }
    else {
        return abs( 1.0f / x ) < PSEUDO_INVERSE_TOLERANCE ? 0.0f : ( 1.0f / x );
    }
}

Matrix3x3 pseudoInverse( in SymmetricMatrix3x3 VTAV, in Matrix3x3 V ) {
    float d0 = pseudoInverse( VTAV.m00 );
    float d1 = pseudoInverse( VTAV.m11 );
    float d2 = pseudoInverse( VTAV.m22 );

    return createMatrix3x3(
        ( V.m00 * d0 * V.m00 ) + ( V.m01 * d1 * V.m01 ) + ( V.m02 * d2 * V.m02 ),
        ( V.m00 * d0 * V.m10 ) + ( V.m01 * d1 * V.m11 ) + ( V.m02 * d2 * V.m12 ),
        ( V.m00 * d0 * V.m20 ) + ( V.m01 * d1 * V.m21 ) + ( V.m02 * d2 * V.m22 ),
        ( V.m10 * d0 * V.m00 ) + ( V.m11 * d1 * V.m01 ) + ( V.m12 * d2 * V.m02 ),
        ( V.m10 * d0 * V.m10 ) + ( V.m11 * d1 * V.m11 ) + ( V.m12 * d2 * V.m12 ),
        ( V.m10 * d0 * V.m20 ) + ( V.m11 * d1 * V.m21 ) + ( V.m12 * d2 * V.m22 ),
        ( V.m20 * d0 * V.m00 ) + ( V.m21 * d1 * V.m01 ) + ( V.m22 * d2 * V.m02 ),
        ( V.m20 * d0 * V.m10 ) + ( V.m21 * d1 * V.m11 ) + ( V.m22 * d2 * V.m12 ),
        ( V.m20 * d0 * V.m20 ) + ( V.m21 * d1 * V.m21 ) + ( V.m22 * d2 * V.m22 )
    );
}

float calculateError( in SymmetricMatrix3x3 ATA, float3 x, float3 ATB ) {
    Matrix3x3 A = createMatrix3x3( ATA );

    float3 V = ATB - multiplyVector( A, x );

    return dot( V, V );
}

void solveSVD( in SymmetricMatrix3x3 ATA, float3 ATB, out float3 vertex, out float error, uint minimizerIterations ) {
    SymmetricMatrix3x3 VTAV;
    Matrix3x3          V;
    getSymmetricSVD( ATA, VTAV, V, minimizerIterations );

    vertex = multiplyVector( pseudoInverse( VTAV, V ), ATB );
    error  = calculateError( ATA, vertex, ATB );
}

void solveQEF( in SVDQEF qef, out float3 vertex, out float error, uint minimizerIterations ) {
    if( qef.intersectionCount > 0 ) {
        qef.massPoint /= qef.intersectionCount;
        qef.ATB       -= multiplyVector( qef.ATA, qef.massPoint );

        solveSVD( qef.ATA, qef.ATB, vertex, error, minimizerIterations );

        vertex += qef.massPoint;
    }
    else {
        vertex = float3( 0.0f, 0.0f, 0.0f );
        error  = 0.0f;
    }
}

void addToQEF( in out SVDQEF qef, float3 intersection, float3 normal ) {
    qef.ATA.m00 += normal.x * normal.x;
    qef.ATA.m01 += normal.x * normal.y;
    qef.ATA.m02 += normal.x * normal.z;
    qef.ATA.m11 += normal.y * normal.y;
    qef.ATA.m12 += normal.y * normal.z;
    qef.ATA.m22 += normal.z * normal.z;

    float x = dot( intersection, normal );

    qef.ATB       += x * normal;
    qef.BTB       += x * x;
    qef.massPoint += intersection;

    ++qef.intersectionCount;
}