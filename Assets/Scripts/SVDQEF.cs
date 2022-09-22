using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class SVDQEF : QEFSolver {

    private const float SVDTolerance           = 1e-6f;
    private const float PseudoInverseTolerance = 1e-6f;

    public bool empty {
        get { return this.intersectionCount == 0; }
    }

    private QEF.SymmetricMatrix3x3 ATA = new( );
    private Vector3                ATB;
    private float                  BTB;
    private Vector3                massPoint;
    private int                    intersectionCount;

    private readonly int minimizerIterations;
    private readonly int surfaceCorrectionIterations;

    public SVDQEF( int minimizerIterations, int surfaceCorrectionIterations  ) {
        this.minimizerIterations         = minimizerIterations;
        this.surfaceCorrectionIterations = surfaceCorrectionIterations;
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

        this.massPoint /= this.intersectionCount;

        this.ATB -= this.ATA.multiplyVector( this.massPoint );

        var ( minimizingVertex, error ) = QEF.SVD.solve( this.ATA, this.ATB, SVDTolerance, this.minimizerIterations, PseudoInverseTolerance );

        minimizingVertex += this.massPoint;

        minimizingVertex = QEFSolver.surfaceCorrection( minimizingVertex, surfaceNormal, this.surfaceCorrectionIterations, densityFunctions );

        return ( minimizingVertex, surfaceNormal, error );
    }

}

namespace QEF {

    public class Matrix3x3 {

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
            this[0, 0] = other.m00;
            this[0, 1] = other.m01;
            this[0, 2] = other.m02;
            this[1, 0] = 0.0f;
            this[1, 1] = other.m11;
            this[1, 2] = other.m12;
            this[2, 0] = 0.0f;
            this[2, 1] = 0.0f;
            this[2, 2] = other.m22;
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

        public Vector3 multiplyVector( Vector3 v ) {
            return new Vector3 {
                x = ( this[0, 0] * v.x ) + ( this[0, 1] * v.y ) + ( this[0, 2] * v.z ),
                y = ( this[1, 0] * v.x ) + ( this[1, 1] * v.y ) + ( this[1, 2] * v.z ),
                z = ( this[2, 0] * v.x ) + ( this[2, 1] * v.y ) + ( this[2, 2] * v.z )
            };
        }

        public void rotate01( float c, float s ) {
            this[0, 0] = ( c * this[0, 0] ) - ( s * this[0, 1] );
            this[0, 1] = ( s * this[0, 0] ) + ( c * this[0, 1] );
            this[1, 0] = ( c * this[1, 0] ) - ( s * this[1, 1] );
            this[1, 1] = ( s * this[1, 0] ) + ( c * this[1, 1] );
            this[2, 0] = ( c * this[2, 0] ) - ( s * this[2, 1] );
            this[2, 1] = ( s * this[2, 0] ) + ( c * this[2, 1] );
        }

        public void rotate02( float c, float s ) {
            this[0, 0] = ( c * this[0, 0] ) - ( s * this[0, 2] );
            this[0, 2] = ( s * this[0, 0] ) + ( c * this[0, 2] );
            this[1, 0] = ( c * this[1, 0] ) - ( s * this[1, 2] );
            this[1, 2] = ( s * this[1, 0] ) + ( c * this[1, 2] );
            this[2, 0] = ( c * this[2, 0] ) - ( s * this[2, 2] );
            this[2, 2] = ( s * this[2, 0] ) + ( c * this[2, 2] );
        }

        public void rotate12( float c, float s ) {
            this[0, 1] = ( c * this[0, 1] ) - ( s * this[0, 2] );
            this[0, 2] = ( s * this[0, 1] ) + ( c * this[0, 2] );
            this[1, 1] = ( c * this[1, 1] ) - ( s * this[1, 2] );
            this[1, 2] = ( s * this[1, 1] ) + ( c * this[1, 2] );
            this[2, 1] = ( c * this[2, 1] ) - ( s * this[2, 2] );
            this[2, 2] = ( s * this[2, 1] ) + ( c * this[2, 2] );
        }

        public static void calculateSymmetricGivensCoefficients( float a_pp, float a_pq, float a_qq, out float c, out float s ) {
            if( a_pq == 0 ) {
                c = 1;
                s = 0;
            }
            else {
                var tau = ( a_qq - a_pp ) / ( 2.0f * a_pq );
                var stt = Mathf.Sqrt( 1.0f + ( tau * tau ) );
                var tan = 1.0f / ( tau >= 0 ? tau + stt : tau - stt );

                c = 1.0f / Mathf.Sqrt( 1.0f + ( tan * tan ) );
                s = tan * c;
            }
        }
    }

    public class SymmetricMatrix3x3 {

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

        public SymmetricMatrix3x3( SymmetricMatrix3x3 other ) {
            this[0, 0] = other.m00;
            this[0, 1] = other.m01;
            this[0, 2] = other.m02;
            this[1, 1] = other.m11;
            this[1, 2] = other.m12;
            this[2, 2] = other.m22;
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

        public void rotate01( ref float c, ref float s ) {
            Matrix3x3.calculateSymmetricGivensCoefficients( this.m00, this.m01, this.m11, out c, out s );

            var cc  = c * c;
            var ss  = s * s;
            var mix = 2 * c * s * this[0, 1];

            this[0, 0] = ( cc * this[0, 0] ) - mix + ( ss * this[1, 1] );
            this[0, 1] = 0.0f;
            this[0, 2] = ( c  * this[0, 2] ) -       ( s  * this[1, 2] );
            this[1, 1] = ( ss * this[0, 0] ) + mix + ( cc * this[1, 1] );
            this[1, 2] = ( s  * this[0, 2] ) +       ( c  * this[1, 2] );
        }

        public void rotate02( ref float c, ref float s ) {
            Matrix3x3.calculateSymmetricGivensCoefficients( this.m00, this.m02, this.m22, out c, out s );

            var cc  = c * c;
            var ss  = s * s;
            var mix = 2 * c * s * this[0, 2];

            this[0, 0] = ( cc * this[0, 0] ) - mix + ( ss * this[2, 2] );
            this[0, 1] = ( c  * this[0, 1] ) -       ( s  * this[1, 2] );
            this[0, 2] = 0.0f;
            this[1, 2] = ( s  * this[0, 1] ) +       ( c  * this[1, 2] );
            this[2, 2] = ( ss * this[0, 0] ) + mix + ( cc * this[2, 2] );
        }

        public void rotate12( ref float c, ref float s ) {
            Matrix3x3.calculateSymmetricGivensCoefficients( this.m11, this.m12, this.m22, out c, out s );

            var cc  = c * c;
            var ss  = s * s;
            var mix = 2 * c * s * this[1, 2];

            this[0, 1] = ( c  * this[0, 1] ) -       ( s  * this[0, 2] );
            this[0, 2] = ( s  * this[0, 1] ) +       ( c  * this[0, 2] );
            this[1, 1] = ( cc * this[1, 1] ) - mix + ( ss * this[2, 2] );
            this[1, 2] = 0.0f;
            this[2, 2] = ( ss * this[1, 1] ) + mix + ( cc * this[2, 2] );
        }

    }

    public static class SVD {

        public static void rotate01( ref SymmetricMatrix3x3 vtav, ref Matrix3x3 v ) {
            if( vtav[0, 1] == 0 ) {
                return;
            }

            var c = 0.0f;
            var s = 0.0f;
            vtav.rotate01( ref c, ref s );

            c = 0.0f;
            s = 0.0f;
            v.rotate01( c, s );
        }

        public static void rotate02( ref SymmetricMatrix3x3 vtav, ref Matrix3x3 v ) {
            if( vtav[0, 2] == 0 ) {
                return;
            }

            var c = 0.0f;
            var s = 0.0f;
            vtav.rotate02( ref c, ref s );

            c = 0.0f;
            s = 0.0f;
            v.rotate02( c, s );
        }

        public static void rotate12( ref SymmetricMatrix3x3 vtav, ref Matrix3x3 v ) {
            if( vtav[1, 2] == 0 ) {
                return;
            }

            var c = 0.0f;
            var s = 0.0f;
            vtav.rotate12( ref c, ref s );

            c = 0.0f;
            s = 0.0f;
            v.rotate12( c, s );
        }

        public static void getSymmetricSVD( ref SymmetricMatrix3x3 a, ref SymmetricMatrix3x3 vtav, ref Matrix3x3 v, float tolerance, int sweeps ) {
            vtav = new SymmetricMatrix3x3( a );
            v    = new Matrix3x3( 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f );

            var delta = tolerance * vtav.frobeniusNorm( );

            for( var sweep = 0; sweep < sweeps && vtav.off( ) > delta; ++sweep ) {
                rotate01( ref vtav, ref v );
                rotate02( ref vtav, ref v );
                rotate12( ref vtav, ref v );
            }
        }

        public static float calculateError( SymmetricMatrix3x3 a, Vector3 x, Vector3 b ) {
            var A = new Matrix3x3( a );

            var v = b - A.multiplyVector( x );

            return Vector3.Dot( v, v );
        }

        public static float pseudoInverse( float x, float tolerance ) {
            return ( Mathf.Abs( x ) < tolerance || Mathf.Abs( 1.0f / x ) < tolerance ) ? 0.0f : ( 1.0f / x );
        }

        public static Matrix3x3 pseudoInverse( SymmetricMatrix3x3 d, Matrix3x3 v, float tolerance ) {
            var d0 = pseudoInverse( d[0, 0], tolerance );
            var d1 = pseudoInverse( d[1, 1], tolerance );
            var d2 = pseudoInverse( d[2, 2], tolerance );

            return new Matrix3x3(
                ( v[0, 0] * d0 * v[0, 0] ) + ( v[0, 1] * d1 * v[0, 1] ) + ( v[0, 2] * d2 * v[0, 2] ),
                ( v[0, 0] * d0 * v[1, 0] ) + ( v[0, 1] * d1 * v[1, 1] ) + ( v[0, 2] * d2 * v[1, 2] ),
                ( v[0, 0] * d0 * v[2, 0] ) + ( v[0, 1] * d1 * v[2, 1] ) + ( v[0, 2] * d2 * v[2, 2] ),
                ( v[1, 0] * d0 * v[0, 0] ) + ( v[1, 1] * d1 * v[0, 1] ) + ( v[1, 2] * d2 * v[0, 2] ),
                ( v[1, 0] * d0 * v[1, 0] ) + ( v[1, 1] * d1 * v[1, 1] ) + ( v[1, 2] * d2 * v[1, 2] ),
                ( v[1, 0] * d0 * v[2, 0] ) + ( v[1, 1] * d1 * v[2, 1] ) + ( v[1, 2] * d2 * v[2, 2] ),
                ( v[2, 0] * d0 * v[0, 0] ) + ( v[2, 1] * d1 * v[0, 1] ) + ( v[2, 2] * d2 * v[0, 2] ),
                ( v[2, 0] * d0 * v[1, 0] ) + ( v[2, 1] * d1 * v[1, 1] ) + ( v[2, 2] * d2 * v[1, 2] ),
                ( v[2, 0] * d0 * v[2, 0] ) + ( v[2, 1] * d1 * v[2, 1] ) + ( v[2, 2] * d2 * v[2, 2] )
            );
        }

        public static ( Vector3, float ) solve( SymmetricMatrix3x3 ATA, Vector3 ATB, float tolerance, int sweeps, float pseudoInverseTolerance ) {
            var V    = new Matrix3x3( );
            var VTAV = new SymmetricMatrix3x3( );

            getSymmetricSVD( ref ATA, ref VTAV, ref V, tolerance, sweeps );

            var x = pseudoInverse( VTAV, V, pseudoInverseTolerance ).multiplyVector( ATB );

            var error = calculateError( ATA, x, ATB );

            return ( x, error );
        }

    }

}