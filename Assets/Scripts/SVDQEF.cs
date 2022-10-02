using System;
using System.Collections.Generic;
using System.Linq;
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

    public int  minimizerIterations         { get; set; }
    public int  surfaceCorrectionIterations { get; set; }
    public bool empty {
        get { return this.intersectionCount == 0; }
    }

    private SymmetricMatrix3x3 ATA = new( );
    private Vector3            ATB;
    private float              BTB;
    private Vector3            massPoint;
    private int                intersectionCount;

    public SVDQEF( int minimizerIterations, int surfaceCorrectionIterations ) {
        this.minimizerIterations         = minimizerIterations;
        this.surfaceCorrectionIterations = surfaceCorrectionIterations;
    }

    public SVDQEF( Data qefData ) {
        this.ATA               = new( qefData.ATA );
        this.ATB               = qefData.ATB;
        this.BTB               = qefData.BTB;
        this.massPoint         = qefData.massPoint;
        this.intersectionCount = qefData.intersectionCount;
    }

    public void add( Vector3 intersection, Vector3 normal ) {
        this.ATA[0, 0] += normal.x * normal.x;
        this.ATA[0, 1] += normal.x * normal.y;
        this.ATA[0, 2] += normal.x * normal.z;
        this.ATA[1, 1] += normal.y * normal.y;
        this.ATA[1, 2] += normal.y * normal.z;
        this.ATA[2, 2] += normal.z * normal.z;

        var dot = Vector3.Dot( intersection, normal );

        this.ATB       += dot * normal;
        this.BTB       += dot * dot;
        this.massPoint += intersection;

        ++this.intersectionCount;
    }

    public void combine( QEFSolver other ) {
        if( other is SVDQEF solver ) {
            this.ATA[0, 0]         += solver.ATA[0, 0];
            this.ATA[0, 1]         += solver.ATA[0, 1];
            this.ATA[0, 2]         += solver.ATA[0, 2];
            this.ATA[1, 1]         += solver.ATA[1, 1];
            this.ATA[1, 2]         += solver.ATA[1, 2];
            this.ATA[2, 2]         += solver.ATA[2, 2];
            this.ATB               += solver.ATB;
            this.BTB               += solver.BTB;
            this.massPoint         += solver.massPoint;
            this.intersectionCount += solver.intersectionCount;
        }
        else {
            throw new Exception( "Unable to combine different derived solvers" );
        }
    }

    public ( Vector3 vertex, Vector3 normal, float error ) solve( SurfaceExtractor.Voxel voxel, IEnumerable<DensityFunction> densityFunctions ) {
        if( this.empty ) {
            throw new Exception( "Unable to solve QEF, no intersections accumulated" );
        }

        var surfaceNormal = voxel.edges.Aggregate(
            Vector3.zero,
            ( accumulator, edge ) => {
                if( edge.intersectsContour( ) ) {
                    accumulator += edge.normal;
                }
                return accumulator;
            }
        ) / this.intersectionCount;

        // copy QEF data into temporaries so the SVD solve doesn't affect future QEF combinations
        var ATA       = this.ATA;
        var ATB       = this.ATB;
        var massPoint = this.massPoint / this.intersectionCount;

        ATB -= ATA.multiplyVector( massPoint );

        var ( minimizingVertex, error ) = SVD.solve( ATA, ATB, this.minimizerIterations );

        minimizingVertex += massPoint;

        minimizingVertex = QEFSolver.surfaceCorrection( minimizingVertex, surfaceNormal, densityFunctions, this.surfaceCorrectionIterations );

        return ( minimizingVertex, surfaceNormal, error );
    }

    public class Matrix3x3 {

        public struct Data {

            public float m00, m01, m02, m10, m11, m12, m20, m21, m22;

            public Data( Matrix3x3 matrix ) {
                this.m00 = matrix[0, 0];
                this.m01 = matrix[0, 1];
                this.m02 = matrix[0, 2];
                this.m10 = matrix[1, 0];
                this.m11 = matrix[1, 1];
                this.m12 = matrix[1, 2];
                this.m20 = matrix[2, 0];
                this.m21 = matrix[2, 1];
                this.m22 = matrix[2, 2];
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
            this[0, 0] = m00;
            this[0, 1] = m01;
            this[0, 2] = m02;
            this[1, 0] = m10;
            this[1, 1] = m11;
            this[1, 2] = m12;
            this[2, 0] = m20;
            this[2, 1] = m21;
            this[2, 2] = m22;
        }

        public Matrix3x3( SymmetricMatrix3x3 other ) {
            this[0, 0] = other[0, 0];
            this[0, 1] = other[0, 1];
            this[0, 2] = other[0, 2];
            this[1, 0] = 0.0f;
            this[1, 1] = other[1, 1];
            this[1, 2] = other[1, 2];
            this[2, 0] = 0.0f;
            this[2, 1] = 0.0f;
            this[2, 2] = other[2, 2];
        }

        public float this[int row, int column] {
            get {
                return row switch {
                    0 => column switch {
                        0 => this.m00,
                        1 => this.m01,
                        2 => this.m02,
                        _ => throw new IndexOutOfRangeException( )
                    },
                    1 => column switch {
                        0 => this.m01,
                        1 => this.m11,
                        2 => this.m12,
                        _ => throw new IndexOutOfRangeException( )
                    },
                    2 => column switch {
                        0 => this.m02,
                        1 => this.m12,
                        2 => this.m22,
                        _ => throw new IndexOutOfRangeException( )
                    },
                    _ => throw new IndexOutOfRangeException( )
                };
            }
            set {
                switch( row ) {
                    case 0:
                        switch( column ) {
                            case 0: this.m00 = value; break;
                            case 1: this.m01 = value; break;
                            case 2: this.m02 = value; break;
                            default: throw new IndexOutOfRangeException( );
                        }
                        break;
                    case 1:
                        switch( column ) {
                            case 0: this.m01 = value; break;
                            case 1: this.m11 = value; break;
                            case 2: this.m12 = value; break;
                            default: throw new IndexOutOfRangeException( );
                        }
                        break;
                    case 2:
                        switch( column ) {
                            case 0: this.m02 = value; break;
                            case 1: this.m12 = value; break;
                            case 2: this.m22 = value; break;
                            default: throw new IndexOutOfRangeException( );
                        }
                        break;
                    default: throw new IndexOutOfRangeException( );
                }
            }
        }

        public Vector3 multiplyVector( Vector3 vector ) {
            return new Vector3 {
                x = ( this[0, 0] * vector.x ) + ( this[0, 1] * vector.y ) + ( this[0, 2] * vector.z ),
                y = ( this[1, 0] * vector.x ) + ( this[1, 1] * vector.y ) + ( this[1, 2] * vector.z ),
                z = ( this[2, 0] * vector.x ) + ( this[2, 1] * vector.y ) + ( this[2, 2] * vector.z )
            };
        }

        public Matrix3x3 rotate01Post( float c, float s ) {
            return new Matrix3x3(
                ( c * this[0, 0] ) - ( s * this[0, 1] ),
                ( s * this[0, 0] ) + ( c * this[0, 1] ),
                this[0, 2],
                ( c * this[1, 0] ) - ( s * this[1, 1] ),
                ( s * this[1, 0] ) + ( c * this[1, 1] ),
                this[1, 2],
                ( c * this[2, 0] ) - ( s * this[2, 1] ),
                ( s * this[2, 0] ) + ( c * this[2, 1] ),
                this[2, 2]
            );
        }

        public Matrix3x3 rotate02Post( float c, float s ) {
            return new Matrix3x3(
                ( c * this[0, 0] ) - ( s * this[0, 2] ),
                this[0, 1],
                ( s * this[0, 0] ) + ( c * this[0, 2] ),
                ( c * this[1, 0] ) - ( s * this[1, 2] ),
                this[1, 1],
                ( s * this[1, 0] ) + ( c * this[1, 2] ),
                ( c * this[2, 0] ) - ( s * this[2, 2] ),
                this[2, 1],
                ( s * this[2, 0] ) + ( c * this[2, 2] )
            );
        }

        public Matrix3x3 rotate12Post( float c, float s ) {
            return new Matrix3x3(
                this[0, 0],
                ( c * this[0, 1] ) - ( s * this[0, 2] ),
                ( s * this[0, 1] ) + ( c * this[0, 2] ),
                this[1, 0],
                ( c * this[1, 1] ) - ( s * this[1, 2] ),
                ( s * this[1, 1] ) + ( c * this[1, 2] ),
                this[2, 0],
                ( c * this[2, 1] ) - ( s * this[2, 2] ),
                ( s * this[2, 1] ) + ( c * this[2, 2] )
            );
        }

    }

    public class SymmetricMatrix3x3 {

        public struct Data {

            public float m00, m01, m02, m11, m12, m22;

            public Data( SymmetricMatrix3x3 matrix ) {
                this.m00 = matrix[0, 0];
                this.m01 = matrix[0, 1];
                this.m02 = matrix[0, 2];
                this.m11 = matrix[1, 1];
                this.m12 = matrix[1, 2];
                this.m22 = matrix[2, 2];
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
            this[0, 0] = m00;
            this[0, 1] = m01;
            this[0, 2] = m02;
            this[1, 1] = m11;
            this[1, 2] = m12;
            this[2, 2] = m22;
        }

        public SymmetricMatrix3x3( Data matrixData ) {
            this[0, 0] = matrixData.m00;
            this[0, 1] = matrixData.m01;
            this[0, 2] = matrixData.m02;
            this[1, 1] = matrixData.m11;
            this[1, 2] = matrixData.m12;
            this[2, 2] = matrixData.m22;
        }

        public float this[int row, int column] {
            get {
                return row switch {
                    0 => column switch {
                        0 => this.m00,
                        1 => this.m01,
                        2 => this.m02,
                        _ => throw new IndexOutOfRangeException( )
                    },
                    1 => column switch {
                        0 => 0.0f,
                        1 => this.m11,
                        2 => this.m12,
                        _ => throw new IndexOutOfRangeException( )
                    },
                    2 => column switch {
                        0 => 0.0f,
                        1 => 0.0f,
                        2 => this.m22,
                        _ => throw new IndexOutOfRangeException( )
                    },
                    _ => throw new IndexOutOfRangeException( )
                };
            }
            set {
                switch( row ) {
                    case 0:
                        switch( column ) {
                            case 0: this.m00 = value; break;
                            case 1: this.m01 = value; break;
                            case 2: this.m02 = value; break;
                            default: throw new IndexOutOfRangeException( );
                        }
                        break;
                    case 1:
                        switch( column ) {
                            case 0: break;
                            case 1: this.m11 = value; break;
                            case 2: this.m12 = value; break;
                            default: throw new IndexOutOfRangeException( );
                        }
                        break;
                    case 2:
                        switch( column ) {
                            case 0: break;
                            case 1: break;
                            case 2: this.m22 = value; break;
                            default: throw new IndexOutOfRangeException( );
                        }
                        break;
                    default: throw new IndexOutOfRangeException( );
                }
            }
        }

        public float frobeniusNorm( ) {
            return Mathf.Sqrt(
                ( this[0, 0] * this[0, 0] ) + ( this[0, 1] * this[0, 1] ) + ( this[0, 2] * this[0, 2] ) +
                ( this[0, 1] * this[0, 1] ) + ( this[1, 1] * this[1, 1] ) + ( this[1, 2] * this[1, 2] ) +
                ( this[0, 2] * this[0, 2] ) + ( this[1, 2] * this[1, 2] ) + ( this[2, 2] * this[2, 2] )
            );
        }

        public float off( ) {
            return Mathf.Sqrt( 2 * ( ( this[0, 1] * this[0, 1] ) + ( this[0, 2] * this[0, 2] ) + ( this[1, 2] * this[1, 2] ) ) );
        }

        public Vector3 multiplyVector( Vector3 vector ) {
            return new Vector3 {
                x = ( this[0, 0] * vector.x ) + ( this[0, 1] * vector.y ) + ( this[0, 2] * vector.z ),
                y = ( this[0, 1] * vector.x ) + ( this[1, 1] * vector.y ) + ( this[1, 2] * vector.z ),
                z = ( this[0, 2] * vector.x ) + ( this[1, 2] * vector.y ) + ( this[2, 2] * vector.z )
            };
        }

        public SymmetricMatrix3x3 rotate01( float c, float s ) {
            var cc  = c * c;
            var ss  = s * s;
            var mix = 2 * c * s * this[0, 1];

            return new SymmetricMatrix3x3(
                ( cc * this[0, 0] ) - mix + ( ss * this[1, 1] ),
                0.0f,
                ( c  * this[0, 2] ) -       ( s  * this[1, 2] ),
                ( ss * this[0, 0] ) + mix + ( cc * this[1, 1] ),
                ( s  * this[0, 2] ) +       ( c  * this[1, 2] ),
                this[2, 2]
            );
        }

        public SymmetricMatrix3x3 rotate02( float c, float s ) {
            var cc  = c * c;
            var ss  = s * s;
            var mix = 2 * c * s * this[0, 2];

            return new SymmetricMatrix3x3(
                ( cc * this[0, 0] ) - mix + ( ss * this[2, 2] ),
                ( c  * this[0, 1] ) -       ( s  * this[1, 2] ),
                0.0f,
                this[1, 1],
                ( s  * this[0, 1] ) +       ( c  * this[1, 2] ),
                ( ss * this[0, 0] ) + mix + ( cc * this[2, 2] )
            );
        }

        public SymmetricMatrix3x3 rotate12( float c, float s ) {
            var cc  = c * c;
            var ss  = s * s;
            var mix = 2 * c * s * this[1, 2];

            return new SymmetricMatrix3x3(
                this[0, 0],
                ( c  * this[0, 1] ) -       ( s  * this[0, 2] ),
                ( s  * this[0, 1] ) +       ( c  * this[0, 2] ),
                ( cc * this[1, 1] ) - mix + ( ss * this[2, 2] ),
                0.0f,
                ( ss * this[1, 1] ) + mix + ( cc * this[2, 2] )
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
            if( VTAV[0, 1] != 0.0f ) {
                var ( c, s ) = calculateSymmetricGivensCoefficients( VTAV[0, 0], VTAV[0, 1], VTAV[1, 1] );

                VTAV = VTAV.rotate01( c, s );

                ( c, s ) = ( 0.0f, 0.0f );

                V = V.rotate01Post( c, s );
            }
            return ( VTAV, V );
        }

        public static ( SymmetricMatrix3x3, Matrix3x3 ) rotate02( SymmetricMatrix3x3 VTAV, Matrix3x3 V ) {
            if( VTAV[0, 2] != 0.0f ) {
                var ( c, s ) = calculateSymmetricGivensCoefficients( VTAV[0, 0], VTAV[0, 2], VTAV[2, 2] );

                VTAV = VTAV.rotate02( c, s );

                // not sure why these are needed
                ( c, s ) = ( 0.0f, 0.0f );

                V = V.rotate02Post( c, s );
            }
            return ( VTAV, V );
        }

        public static ( SymmetricMatrix3x3, Matrix3x3 ) rotate12( SymmetricMatrix3x3 VTAV, Matrix3x3 V ) {
            if( VTAV[1, 2] != 0.0f ) {
                var ( c, s ) = calculateSymmetricGivensCoefficients( VTAV[1, 1], VTAV[1, 2], VTAV[2, 2] );

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
            var d0 = pseudoInverse( VTAV[0, 0] );
            var d1 = pseudoInverse( VTAV[1, 1] );
            var d2 = pseudoInverse( VTAV[2, 2] );

            return new Matrix3x3(
                ( V[0, 0] * d0 * V[0, 0] ) + ( V[0, 1] * d1 * V[0, 1] ) + ( V[0, 2] * d2 * V[0, 2] ),
                ( V[0, 0] * d0 * V[1, 0] ) + ( V[0, 1] * d1 * V[1, 1] ) + ( V[0, 2] * d2 * V[1, 2] ),
                ( V[0, 0] * d0 * V[2, 0] ) + ( V[0, 1] * d1 * V[2, 1] ) + ( V[0, 2] * d2 * V[2, 2] ),
                ( V[1, 0] * d0 * V[0, 0] ) + ( V[1, 1] * d1 * V[0, 1] ) + ( V[1, 2] * d2 * V[0, 2] ),
                ( V[1, 0] * d0 * V[1, 0] ) + ( V[1, 1] * d1 * V[1, 1] ) + ( V[1, 2] * d2 * V[1, 2] ),
                ( V[1, 0] * d0 * V[2, 0] ) + ( V[1, 1] * d1 * V[2, 1] ) + ( V[1, 2] * d2 * V[2, 2] ),
                ( V[2, 0] * d0 * V[0, 0] ) + ( V[2, 1] * d1 * V[0, 1] ) + ( V[2, 2] * d2 * V[0, 2] ),
                ( V[2, 0] * d0 * V[1, 0] ) + ( V[2, 1] * d1 * V[1, 1] ) + ( V[2, 2] * d2 * V[1, 2] ),
                ( V[2, 0] * d0 * V[2, 0] ) + ( V[2, 1] * d1 * V[2, 1] ) + ( V[2, 2] * d2 * V[2, 2] )
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