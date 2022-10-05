using System;
using System.Collections.Generic;
using UnityEngine;

public class SVDQEF : QEFSolver {

    public struct Data {

        public SymmetricMatrix3x3.Data ATA;
        public Vector3                 ATB;
        public float                   BTB;
        public Vector3                 massPoint;
        public int                     intersectionCount;

        public Data( SVDQEF qef ) {
            this.ATA               = new SymmetricMatrix3x3.Data( qef.ATA );
            this.ATB               = qef.ATB;
            this.BTB               = qef.BTB;
            this.massPoint         = qef.massPoint;
            this.intersectionCount = qef.intersectionCount;
        }

    }

    private const float SVDTolerance           = 1e-6f;
    private const float PseudoInverseTolerance = 1e-6f;

    public int minimizerIterations { get; set; }

    private int _intersectionCount;
    public int intersectionCount {
                get { return this._intersectionCount;  }
        private set { this._intersectionCount = value; }
    }

    private SymmetricMatrix3x3 ATA = new( );
    private Vector3            ATB;
    private float              BTB;
    private Vector3            massPoint;

    public SVDQEF( int minimizerIterations ) {
        this.minimizerIterations = minimizerIterations;
    }

    public SVDQEF( Data qefData ) {
        this.ATA               = new( qefData.ATA );
        this.ATB               = qefData.ATB;
        this.BTB               = qefData.BTB;
        this.massPoint         = qefData.massPoint;
        this.intersectionCount = qefData.intersectionCount;
    }

    public void add( Vector3 intersection, Vector3 normal ) {
        this.ATA.m00 += normal.x * normal.x;
        this.ATA.m01 += normal.x * normal.y;
        this.ATA.m02 += normal.x * normal.z;
        this.ATA.m11 += normal.y * normal.y;
        this.ATA.m12 += normal.y * normal.z;
        this.ATA.m22 += normal.z * normal.z;

        var dot = Vector3.Dot( intersection, normal );

        this.ATB       += dot * normal;
        this.BTB       += dot * dot;
        this.massPoint += intersection;

        ++this.intersectionCount;
    }

    public void combine( QEFSolver other ) {
        if( other is SVDQEF solver ) {
            this.ATA.m00         += solver.ATA.m00;
            this.ATA.m01         += solver.ATA.m01;
            this.ATA.m02         += solver.ATA.m02;
            this.ATA.m11         += solver.ATA.m11;
            this.ATA.m12         += solver.ATA.m12;
            this.ATA.m22         += solver.ATA.m22;
            this.ATB               += solver.ATB;
            this.BTB               += solver.BTB;
            this.massPoint         += solver.massPoint;
            this.intersectionCount += solver.intersectionCount;
        }
        else {
            throw new Exception( "Unable to combine different derived solvers" );
        }
    }

    public ( Vector3 vertex, float error ) solve( SurfaceExtractor.Voxel voxel, IEnumerable<DensityFunction> densityFunctions ) {
        if( this.intersectionCount == 0 ) {
            throw new Exception( "Unable to solve QEF, no intersections accumulated" );
        }

        // copy QEF data into temporaries so the SVD solve doesn't affect future QEF combinations
        var ATA       = this.ATA;
        var ATB       = this.ATB;
        var massPoint = this.massPoint / this.intersectionCount;

        ATB -= ATA.multiplyVector( massPoint );

        var ( minimizingVertex, error ) = SVD.solve( ATA, ATB, this.minimizerIterations );

        minimizingVertex += massPoint;

        // if the solved position is outside the voxel, use the mass point instead
        if(
            minimizingVertex.x < voxel.minimum.x || minimizingVertex.x > voxel.maximum.x ||
            minimizingVertex.y < voxel.minimum.y || minimizingVertex.y > voxel.maximum.y ||
            minimizingVertex.z < voxel.minimum.z || minimizingVertex.z > voxel.maximum.z
        ) {
            minimizingVertex = massPoint;
        }

        return ( minimizingVertex, error );
    }

    public class Matrix3x3 {

        public struct Data {

            public float m00, m01, m02, m10, m11, m12, m20, m21, m22;

            public Data( Matrix3x3 matrix ) {
                this.m00 = matrix.m00;
                this.m01 = matrix.m01;
                this.m02 = matrix.m02;
                this.m10 = matrix.m10;
                this.m11 = matrix.m11;
                this.m12 = matrix.m12;
                this.m20 = matrix.m20;
                this.m21 = matrix.m21;
                this.m22 = matrix.m22;
            }

        }

        public static Matrix3x3 identity = new( 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f );

        public float m00, m01, m02, m10, m11, m12, m20, m21, m22 = 0.0f;

        public Matrix3x3( ) { }

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

        public Matrix3x3( SymmetricMatrix3x3 other ) {
            this.m00 = other.m00;
            this.m01 = other.m01;
            this.m02 = other.m02;
            this.m10 = 0.0f;
            this.m11 = other.m11;
            this.m12 = other.m12;
            this.m20 = 0.0f;
            this.m21 = 0.0f;
            this.m22 = other.m22;
        }

        public Vector3 multiplyVector( Vector3 vector ) {
            return new Vector3 {
                x = ( this.m00 * vector.x ) + ( this.m01 * vector.y ) + ( this.m02 * vector.z ),
                y = ( this.m10 * vector.x ) + ( this.m11 * vector.y ) + ( this.m12 * vector.z ),
                z = ( this.m20 * vector.x ) + ( this.m21 * vector.y ) + ( this.m22 * vector.z )
            };
        }

        public Matrix3x3 rotate01Post( float c, float s ) {
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

        public Matrix3x3 rotate02Post( float c, float s ) {
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

        public Matrix3x3 rotate12Post( float c, float s ) {
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

    public class SymmetricMatrix3x3 {

        public struct Data {

            public float m00, m01, m02, m11, m12, m22;

            public Data( SymmetricMatrix3x3 matrix ) {
                this.m00 = matrix.m00;
                this.m01 = matrix.m01;
                this.m02 = matrix.m02;
                this.m11 = matrix.m11;
                this.m12 = matrix.m12;
                this.m22 = matrix.m22;
            }

            public Data( float m00, float m01, float m02, float m11, float m12, float m22 ) {
                this.m00 = m00;
                this.m01 = m01;
                this.m02 = m02;
                this.m11 = m11;
                this.m12 = m12;
                this.m22 = m22;
            }

        }

        public float m00, m01, m02, m11, m12, m22 = 0.0f;

        public SymmetricMatrix3x3( ) { }

        public SymmetricMatrix3x3( float m00, float m01, float m02, float m11, float m12, float m22 ) {
            this.m00 = m00;
            this.m01 = m01;
            this.m02 = m02;
            this.m11 = m11;
            this.m12 = m12;
            this.m22 = m22;
        }

        public SymmetricMatrix3x3( Data matrixData ) {
            this.m00 = matrixData.m00;
            this.m01 = matrixData.m01;
            this.m02 = matrixData.m02;
            this.m11 = matrixData.m11;
            this.m12 = matrixData.m12;
            this.m22 = matrixData.m22;
        }

        public float frobeniusNorm( ) {
            return Mathf.Sqrt(
                ( this.m00 * this.m00 ) + ( this.m01 * this.m01 ) + ( this.m02 * this.m02 ) +
                ( this.m01 * this.m01 ) + ( this.m11 * this.m11 ) + ( this.m12 * this.m12 ) +
                ( this.m02 * this.m02 ) + ( this.m12 * this.m12 ) + ( this.m22 * this.m22 )
            );
        }

        public float off( ) {
            return Mathf.Sqrt( 2 * ( ( this.m01 * this.m01 ) + ( this.m02 * this.m02 ) + ( this.m12 * this.m12 ) ) );
        }

        public Vector3 multiplyVector( Vector3 vector ) {
            return new Vector3 {
                x = ( this.m00 * vector.x ) + ( this.m01 * vector.y ) + ( this.m02 * vector.z ),
                y = ( this.m01 * vector.x ) + ( this.m11 * vector.y ) + ( this.m12 * vector.z ),
                z = ( this.m02 * vector.x ) + ( this.m12 * vector.y ) + ( this.m22 * vector.z )
            };
        }

        public SymmetricMatrix3x3 rotate01( float c, float s ) {
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

        public SymmetricMatrix3x3 rotate02( float c, float s ) {
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

        public SymmetricMatrix3x3 rotate12( float c, float s ) {
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

    public static class SVD {

        public static ( float, float ) calculateSymmetricGivensCoefficients( float app, float apq, float aqq ) {
            var c = 1.0f;
            var s = 0.0f;
            if( apq != 0.0f ) {
                var tau = ( aqq - app ) / ( 2.0f * apq );
                var stt = Mathf.Sqrt( 1.0f + ( tau * tau ) );
                var tan = 1.0f / ( tau >= 0.0f ? tau + stt : tau - stt );

                c = 1.0f / Mathf.Sqrt( 1.0f + ( tan * tan ) );
                s = tan * c;
            }
            return ( c, s );
        }

        public static ( SymmetricMatrix3x3, Matrix3x3 ) rotate01( SymmetricMatrix3x3 VTAV, Matrix3x3 V ) {
            if( VTAV.m01 != 0.0f ) {
                var ( c, s ) = calculateSymmetricGivensCoefficients( VTAV.m00, VTAV.m01, VTAV.m11 );

                VTAV = VTAV.rotate01( c, s );

                ( c, s ) = ( 0.0f, 0.0f );

                V = V.rotate01Post( c, s );
            }
            return ( VTAV, V );
        }

        public static ( SymmetricMatrix3x3, Matrix3x3 ) rotate02( SymmetricMatrix3x3 VTAV, Matrix3x3 V ) {
            if( VTAV.m02 != 0.0f ) {
                var ( c, s ) = calculateSymmetricGivensCoefficients( VTAV.m00, VTAV.m02, VTAV.m22 );

                VTAV = VTAV.rotate02( c, s );

                // not sure why these are needed
                // I think they keep minimizing vertices bounded to voxels
                ( c, s ) = ( 0.0f, 0.0f );

                V = V.rotate02Post( c, s );
            }
            return ( VTAV, V );
        }

        public static ( SymmetricMatrix3x3, Matrix3x3 ) rotate12( SymmetricMatrix3x3 VTAV, Matrix3x3 V ) {
            if( VTAV.m12 != 0.0f ) {
                var ( c, s ) = calculateSymmetricGivensCoefficients( VTAV.m11, VTAV.m12, VTAV.m22 );

                VTAV = VTAV.rotate12( c, s );

                ( c, s ) = ( 0.0f, 0.0f );

                V = V.rotate12Post( c, s );
            }
            return ( VTAV, V );
        }

        public static ( SymmetricMatrix3x3, Matrix3x3 ) getSymmetricSVD( SymmetricMatrix3x3 ATA, int sweeps ) {
            var VTAV = ATA;
            var V    = Matrix3x3.identity;

            var delta = SVDTolerance * VTAV.frobeniusNorm( );

            for( var sweep = 0; sweep < sweeps && VTAV.off( ) > delta; ++sweep ) {
                ( VTAV, V ) = rotate01( VTAV, V );
                ( VTAV, V ) = rotate02( VTAV, V );
                ( VTAV, V ) = rotate12( VTAV, V );
            }

            return ( VTAV, V );
        }

        public static float pseudoInverse( float x ) {
            return ( Mathf.Abs( x ) < PseudoInverseTolerance || Mathf.Abs( 1.0f / x ) < PseudoInverseTolerance ) ? 0.0f : ( 1.0f / x );
        }

        public static Matrix3x3 pseudoInverse( SymmetricMatrix3x3 VTAV, Matrix3x3 V ) {
            var d0 = pseudoInverse( VTAV.m00 );
            var d1 = pseudoInverse( VTAV.m11 );
            var d2 = pseudoInverse( VTAV.m22 );

            return new Matrix3x3(
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

        public static float calculateError( SymmetricMatrix3x3 ATA, Vector3 x, Vector3 ATB ) {
            var A = new Matrix3x3( ATA );

            var V = ATB - A.multiplyVector( x );

            return Vector3.Dot( V, V );
        }

        public static ( Vector3, float ) solve( SymmetricMatrix3x3 ATA, Vector3 ATB, int sweeps ) {
            var ( VTAV, V ) = getSymmetricSVD( ATA, sweeps );

            var x = pseudoInverse( VTAV, V ).multiplyVector( ATB );

            var error = calculateError( ATA, x, ATB );

            return ( x, error );
        }

    }

}