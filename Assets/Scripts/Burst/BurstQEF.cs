using Unity.Mathematics;

using QEFSolverType = QEFSolver.Type;

// a QEF that Burst code can keep in a struct, for either solver. The SVD solver is SVDQEF operation for operation. The
// simple solver steps from the center of the voxel towards the planes of the hermite samples like SimpleQEF does, but
// keeps the sums of AtA, AtB and BtB instead of the planes themselves: the step it takes, the sum over the planes of
// their distance to the vertex times their normal, is AtA x - AtB, and its error is the same function of x as the SVD
// solver's. That is the same math as SimpleQEF with different rounding, and it fits in a fixed size however many QEFs are
// combined
public struct BurstQEF {

    // SVDQEF.SVDTolerance and SVDQEF.PseudoInverseTolerance
    private const float SVDTolerance           = 1e-6f;
    private const float PseudoInverseTolerance = 0.1f;

    public QEFSolverType type;
    public int           minimizerIterations;

    // AtA, of which only the upper triangle is kept, AtB and BtB
    public float  ata00, ata01, ata02, ata11, ata12, ata22;
    public float3 atb;
    public float  btb;

    public int intersectionCount;

    // the sum of the intersections and how many there are, and the dimension of the feature they lie on: 1 for a plane,
    // 2 for an edge and 3 for a corner, 0 until the QEF has been solved. See SVDQEF
    public float3 massPoint;
    public int    massPointCount;
    public int    massPointDimension;

    public static BurstQEF create( QEFSolverType type, int minimizerIterations ) {
        return new BurstQEF {
            type                = type,
            minimizerIterations = minimizerIterations
        };
    }

    public void add( float3 intersection, float3 normal ) {
        // SimpleQEF keeps a Plane for every sample, which normalizes its normal again
        if( this.type == QEFSolverType.Simple ) {
            normal = BurstSurfaceExtractor.normalize( normal );
        }

        this.ata00 += normal.x * normal.x;
        this.ata01 += normal.x * normal.y;
        this.ata02 += normal.x * normal.z;
        this.ata11 += normal.y * normal.y;
        this.ata12 += normal.y * normal.z;
        this.ata22 += normal.z * normal.z;

        var dot = BurstSurfaceExtractor.dot( intersection, normal );

        this.atb       += dot * normal;
        this.btb       += dot * dot;
        this.massPoint += intersection;

        ++this.massPointCount;
        ++this.intersectionCount;
    }

    public void combine( in BurstQEF other ) {
        this.ata00             += other.ata00;
        this.ata01             += other.ata01;
        this.ata02             += other.ata02;
        this.ata11             += other.ata11;
        this.ata12             += other.ata12;
        this.ata22             += other.ata22;
        this.atb               += other.atb;
        this.btb               += other.btb;
        this.intersectionCount += other.intersectionCount;

        // the simple solver's mass point is that of every intersection. The SVD solver merges mass points of the same
        // dimension and keeps the one of the higher dimension of two that differ
        if( this.type == QEFSolverType.Simple || other.massPointDimension == this.massPointDimension ) {
            this.massPoint      += other.massPoint;
            this.massPointCount += other.massPointCount;
        }
        else if( other.massPointDimension > this.massPointDimension ) {
            this.massPoint          = other.massPoint;
            this.massPointCount     = other.massPointCount;
            this.massPointDimension = other.massPointDimension;
        }
    }

    // the point that minimizes the QEF and the value of the QEF there. The QEF needs at least one intersection
    public void solve( float3 center, out float3 vertex, out float error ) {
        if( this.type == QEFSolverType.Simple ) {
            vertex = center;

            for( var minimizerIteration = 0; minimizerIteration < this.minimizerIterations; ++minimizerIteration ) {
                vertex -= ( this.ata( ).multiplyVector( vertex ) - this.atb ) / this.intersectionCount;
            }
        }
        else {
            var massPoint = this.massPoint / this.massPointCount;

            var atb = this.atb - this.ata( ).multiplyVector( massPoint );

            // of the points that minimize the QEF this is the one closest to the mass point, x = p + (AtA)^+ (AtB - AtA p)
            getSymmetricSVD( this.ata( ), this.minimizerIterations, out var vtav, out var v );

            vertex = pseudoInverse( vtav, v ).multiplyVector( atb ) + massPoint;

            this.massPointDimension = calculateRank( vtav );
        }

        error = this.calculateError( vertex );
    }

    // the mass point of the intersections and the value of the QEF there
    public readonly void solveMassPoint( out float3 vertex, out float error ) {
        vertex = this.type == QEFSolverType.Simple ? this.massPoint / this.intersectionCount : this.massPoint / this.massPointCount;
        error  = this.calculateError( vertex );
    }

    // QEFSolver.placeVertex, given how many separate pieces of the contour pass through the voxel
    public void placeVertex( int pieces, float3 center, out float3 vertex, out float error ) {
        if( pieces > 1 ) {
            this.solveMassPoint( out vertex, out error );
        }
        else {
            this.solve( center, out vertex, out error );
        }
    }

    // E[x] = xT AtA x - 2 xT AtB + BtB, which rounding can leave just below zero close to the surface
    public readonly float calculateError( float3 x ) {
        var error = BurstSurfaceExtractor.dot( x, this.ata( ).multiplyVector( x ) ) - ( 2.0f * BurstSurfaceExtractor.dot( x, this.atb ) ) + this.btb;

        return BurstSurfaceExtractor.max( error, 0.0f );
    }

    private readonly SymmetricMatrix3x3 ata( ) {
        return new SymmetricMatrix3x3( this.ata00, this.ata01, this.ata02, this.ata11, this.ata12, this.ata22 );
    }

    // SVDQEF.Matrix3x3
    public struct Matrix3x3 {

        public float m00, m01, m02, m10, m11, m12, m20, m21, m22;

        public static Matrix3x3 identity( ) {
            return new Matrix3x3( 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f );
        }

        public Matrix3x3(
            float m00, float m01, float m02,
            float m10, float m11, float m12,
            float m20, float m21, float m22
        ) {
            this.m00 = m00;
            this.m01 = m01;
            this.m02 = m02;
            this.m10 = m10;
            this.m11 = m11;
            this.m12 = m12;
            this.m20 = m20;
            this.m21 = m21;
            this.m22 = m22;
        }

        public readonly float3 multiplyVector( float3 vector ) {
            return new float3(
                ( this.m00 * vector.x ) + ( this.m01 * vector.y ) + ( this.m02 * vector.z ),
                ( this.m10 * vector.x ) + ( this.m11 * vector.y ) + ( this.m12 * vector.z ),
                ( this.m20 * vector.x ) + ( this.m21 * vector.y ) + ( this.m22 * vector.z )
            );
        }

        public readonly Matrix3x3 rotate01Post( float c, float s ) {
            return new Matrix3x3(
                ( c * this.m00 ) - ( s * this.m01 ),
                ( s * this.m00 ) + ( c * this.m01 ),
                this.m02,
                ( c * this.m10 ) - ( s * this.m11 ),
                ( s * this.m10 ) + ( c * this.m11 ),
                this.m12,
                ( c * this.m20 ) - ( s * this.m21 ),
                ( s * this.m20 ) + ( c * this.m21 ),
                this.m22
            );
        }

        public readonly Matrix3x3 rotate02Post( float c, float s ) {
            return new Matrix3x3(
                ( c * this.m00 ) - ( s * this.m02 ),
                this.m01,
                ( s * this.m00 ) + ( c * this.m02 ),
                ( c * this.m10 ) - ( s * this.m12 ),
                this.m11,
                ( s * this.m10 ) + ( c * this.m12 ),
                ( c * this.m20 ) - ( s * this.m22 ),
                this.m21,
                ( s * this.m20 ) + ( c * this.m22 )
            );
        }

        public readonly Matrix3x3 rotate12Post( float c, float s ) {
            return new Matrix3x3(
                this.m00,
                ( c * this.m01 ) - ( s * this.m02 ),
                ( s * this.m01 ) + ( c * this.m02 ),
                this.m10,
                ( c * this.m11 ) - ( s * this.m12 ),
                ( s * this.m11 ) + ( c * this.m12 ),
                this.m20,
                ( c * this.m21 ) - ( s * this.m22 ),
                ( s * this.m21 ) + ( c * this.m22 )
            );
        }

    }

    // SVDQEF.SymmetricMatrix3x3
    public struct SymmetricMatrix3x3 {

        public float m00, m01, m02, m11, m12, m22;

        public SymmetricMatrix3x3( float m00, float m01, float m02, float m11, float m12, float m22 ) {
            this.m00 = m00;
            this.m01 = m01;
            this.m02 = m02;
            this.m11 = m11;
            this.m12 = m12;
            this.m22 = m22;
        }

        public readonly float frobeniusNorm( ) {
            return math.sqrt(
                ( this.m00 * this.m00 ) + ( this.m01 * this.m01 ) + ( this.m02 * this.m02 ) +
                ( this.m01 * this.m01 ) + ( this.m11 * this.m11 ) + ( this.m12 * this.m12 ) +
                ( this.m02 * this.m02 ) + ( this.m12 * this.m12 ) + ( this.m22 * this.m22 )
            );
        }

        public readonly float off( ) {
            return math.sqrt( 2 * ( ( this.m01 * this.m01 ) + ( this.m02 * this.m02 ) + ( this.m12 * this.m12 ) ) );
        }

        public readonly float3 multiplyVector( float3 vector ) {
            return new float3(
                ( this.m00 * vector.x ) + ( this.m01 * vector.y ) + ( this.m02 * vector.z ),
                ( this.m01 * vector.x ) + ( this.m11 * vector.y ) + ( this.m12 * vector.z ),
                ( this.m02 * vector.x ) + ( this.m12 * vector.y ) + ( this.m22 * vector.z )
            );
        }

        public readonly SymmetricMatrix3x3 rotate01( float c, float s ) {
            var cc  = c * c;
            var ss  = s * s;
            var mix = 2 * c * s * this.m01;

            return new SymmetricMatrix3x3(
                ( cc * this.m00 ) - mix + ( ss * this.m11 ),
                0.0f,
                ( c  * this.m02 ) -       ( s  * this.m12 ),
                ( ss * this.m00 ) + mix + ( cc * this.m11 ),
                ( s  * this.m02 ) +       ( c  * this.m12 ),
                this.m22
            );
        }

        public readonly SymmetricMatrix3x3 rotate02( float c, float s ) {
            var cc  = c * c;
            var ss  = s * s;
            var mix = 2 * c * s * this.m02;

            return new SymmetricMatrix3x3(
                ( cc * this.m00 ) - mix + ( ss * this.m22 ),
                ( c  * this.m01 ) -       ( s  * this.m12 ),
                0.0f,
                this.m11,
                ( s  * this.m01 ) +       ( c  * this.m12 ),
                ( ss * this.m00 ) + mix + ( cc * this.m22 )
            );
        }

        public readonly SymmetricMatrix3x3 rotate12( float c, float s ) {
            var cc  = c * c;
            var ss  = s * s;
            var mix = 2 * c * s * this.m12;

            return new SymmetricMatrix3x3(
                this.m00,
                ( c  * this.m01 ) -       ( s  * this.m02 ),
                ( s  * this.m01 ) +       ( c  * this.m02 ),
                ( cc * this.m11 ) - mix + ( ss * this.m22 ),
                0.0f,
                ( ss * this.m11 ) + mix + ( cc * this.m22 )
            );
        }

    }

    // SVDQEF.SVD

    private static void calculateSymmetricGivensCoefficients( float app, float apq, float aqq, out float c, out float s ) {
        c = 1.0f;
        s = 0.0f;
        if( apq != 0.0f ) {
            var tau = ( aqq - app ) / ( 2.0f * apq );
            var stt = math.sqrt( 1.0f + ( tau * tau ) );
            var tan = 1.0f / ( tau >= 0.0f ? tau + stt : tau - stt );

            c = 1.0f / math.sqrt( 1.0f + ( tan * tan ) );
            s = tan * c;
        }
    }

    private static void getSymmetricSVD( SymmetricMatrix3x3 ata, int sweeps, out SymmetricMatrix3x3 vtav, out Matrix3x3 v ) {
        vtav = ata;
        v    = Matrix3x3.identity( );

        var delta = SVDTolerance * vtav.frobeniusNorm( );

        for( var sweep = 0; sweep < sweeps && vtav.off( ) > delta; ++sweep ) {
            if( vtav.m01 != 0.0f ) {
                calculateSymmetricGivensCoefficients( vtav.m00, vtav.m01, vtav.m11, out var c, out var s );

                vtav = vtav.rotate01( c, s );
                v    = v.rotate01Post( c, s );
            }
            if( vtav.m02 != 0.0f ) {
                calculateSymmetricGivensCoefficients( vtav.m00, vtav.m02, vtav.m22, out var c, out var s );

                vtav = vtav.rotate02( c, s );
                v    = v.rotate02Post( c, s );
            }
            if( vtav.m12 != 0.0f ) {
                calculateSymmetricGivensCoefficients( vtav.m11, vtav.m12, vtav.m22, out var c, out var s );

                vtav = vtav.rotate12( c, s );
                v    = v.rotate12Post( c, s );
            }
        }
    }

    private static float pseudoInverse( float x ) {
        return math.abs( x ) < PseudoInverseTolerance ? 0.0f : ( 1.0f / x );
    }

    private static int calculateRank( SymmetricMatrix3x3 vtav ) {
        var rank = 0;

        if( pseudoInverse( vtav.m00 ) != 0.0f ) {
            ++rank;
        }
        if( pseudoInverse( vtav.m11 ) != 0.0f ) {
            ++rank;
        }
        if( pseudoInverse( vtav.m22 ) != 0.0f ) {
            ++rank;
        }

        return rank;
    }

    private static Matrix3x3 pseudoInverse( SymmetricMatrix3x3 vtav, Matrix3x3 v ) {
        var d0 = pseudoInverse( vtav.m00 );
        var d1 = pseudoInverse( vtav.m11 );
        var d2 = pseudoInverse( vtav.m22 );

        return new Matrix3x3(
            ( v.m00 * d0 * v.m00 ) + ( v.m01 * d1 * v.m01 ) + ( v.m02 * d2 * v.m02 ),
            ( v.m00 * d0 * v.m10 ) + ( v.m01 * d1 * v.m11 ) + ( v.m02 * d2 * v.m12 ),
            ( v.m00 * d0 * v.m20 ) + ( v.m01 * d1 * v.m21 ) + ( v.m02 * d2 * v.m22 ),
            ( v.m10 * d0 * v.m00 ) + ( v.m11 * d1 * v.m01 ) + ( v.m12 * d2 * v.m02 ),
            ( v.m10 * d0 * v.m10 ) + ( v.m11 * d1 * v.m11 ) + ( v.m12 * d2 * v.m12 ),
            ( v.m10 * d0 * v.m20 ) + ( v.m11 * d1 * v.m21 ) + ( v.m12 * d2 * v.m22 ),
            ( v.m20 * d0 * v.m00 ) + ( v.m21 * d1 * v.m01 ) + ( v.m22 * d2 * v.m02 ),
            ( v.m20 * d0 * v.m10 ) + ( v.m21 * d1 * v.m11 ) + ( v.m22 * d2 * v.m12 ),
            ( v.m20 * d0 * v.m20 ) + ( v.m21 * d1 * v.m21 ) + ( v.m22 * d2 * v.m22 )
        );
    }

}
