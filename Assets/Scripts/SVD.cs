using UnityEngine;

// TODO: refactor

public class Mat3 {

    public float m00, m01, m02, m10, m11, m12, m20, m21, m22;

    public Mat3( ) {
        this.clear( );
    }

    public Mat3(
        float m00, float m01, float m02,
        float m10, float m11, float m12,
        float m20, float m21, float m22
    ) {
        this.set( m00, m01, m02, m10, m11, m12, m20, m21, m22 );
    }

    public void clear( ) {
        this.set( 0, 0, 0, 0, 0, 0, 0, 0, 0 );
    }

    public void set(
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

    public void set( Mat3 rhs ) {
        this.set( rhs.m00, rhs.m01, rhs.m02, rhs.m10, rhs.m11, rhs.m12, rhs.m20, rhs.m21, rhs.m22 );
    }

    public void setSymmetric( float a00, float a01, float a02, float a11, float a12, float a22 ) {
        this.set( a00, a01, a02, a01, a11, a12, a02, a12, a22 );
    }

    public void setSymmetric( SMat3 rhs ) {
        this.setSymmetric( rhs.m00, rhs.m01, rhs.m02, rhs.m11, rhs.m12, rhs.m22 );
    }

    public float fnorm( ) {
        return Mathf.Sqrt(
            ( m00 * m00 ) + ( m01 * m01 ) + ( m02 * m02 ) +
            ( m10 * m10 ) + ( m11 * m11 ) + ( m12 * m12 ) +
            ( m20 * m20 ) + ( m21 * m21 ) + ( m22 * m22 )
        );
    }

    public float off( ) {
        return Mathf.Sqrt(
            ( m01 * m01 ) + ( m02 * m02 ) + ( m10 * m10 ) +
            ( m12 * m12 ) + ( m20 * m20 ) + ( m21 * m21 )
        );
    }

    public static Mat3 operator*( Mat3 a, Mat3 b ) {
        var m = new Mat3();
        m.set(
            a.m00 * b.m00 + a.m01 * b.m10 + a.m02 * b.m20,
            a.m00 * b.m01 + a.m01 * b.m11 + a.m02 * b.m21,
            a.m00 * b.m02 + a.m01 * b.m12 + a.m02 * b.m22,
            a.m10 * b.m00 + a.m11 * b.m10 + a.m12 * b.m20,
            a.m10 * b.m01 + a.m11 * b.m11 + a.m12 * b.m21,
            a.m10 * b.m02 + a.m11 * b.m12 + a.m12 * b.m22,
            a.m20 * b.m00 + a.m21 * b.m10 + a.m22 * b.m20,
            a.m20 * b.m01 + a.m21 * b.m11 + a.m22 * b.m21,
            a.m20 * b.m02 + a.m21 * b.m12 + a.m22 * b.m22
        );
        return m;
    }

    public SMat3 mulATA( ) {
        var m = new SMat3();
        m.setSymmetric(
            m00 * m00 + m10 * m10 + m20 * m20,
            m00 * m01 + m10 * m11 + m20 * m21,
            m00 * m02 + m10 * m12 + m20 * m22,
            m01 * m01 + m11 * m11 + m21 * m21,
            m01 * m02 + m11 * m12 + m21 * m22,
            m02 * m02 + m12 * m12 + m22 * m22
        );
        return m;
    }

    public Mat3 transpose( ) {
        var m = new Mat3( );
        m.set( m00, m10, m20, m01, m11, m21, m02, m12, m22 );
        return m;
    }

    public Vector3 vmul( Vector3 v ) {
        var o = new Vector3( );
        o.x = ( m00 * v.x ) + ( m01 * v.y ) + ( m02 * v.z );
        o.y = ( m10 * v.x ) + ( m11 * v.y ) + ( m12 * v.z );
        o.z = ( m20 * v.x ) + ( m21 * v.y ) + ( m22 * v.z );
        return o;
    }

    public void rot01_post( float c, float s ) {
        float m00 = this.m00, m01 = this.m01, m10 = this.m10, m11 = this.m11, m20 = this.m20, m21 = this.m21;
        this.set(
            c * m00 - s * m01,
            s * m00 + c * m01,
            this.m02,
            c * m10 - s * m11,
            s * m10 + c * m11,
            this.m12,
            c * m20 - s * m21,
            s * m20 + c * m21,
            this.m22
        );
    }

    public void rot02_post( float c, float s ) {
        float m00 = this.m00, m02 = this.m02, m10 = this.m10, m12 = this.m12, m20 = this.m20, m22 = this.m22;
        this.set(
            c * m00 - s * m02,
            this.m01,
            s * m00 + c * m02,
            c * m10 - s * m12,
            this.m11,
            s * m10 + c * m12,
            c * m20 - s * m22,
            this.m21,
            s * m20 + c * m22
        );
    }

    public void rot12_post( float c, float s ) {
        float m01 = this.m01, m02 = this.m02, m11 = this.m11, m12 = this.m12, m21 = this.m21, m22 = this.m22;
        this.set(
            this.m00,
            c * m01 - s * m02,
            s * m01 + c * m02,
            this.m10,
            c * m11 - s * m12,
            s * m11 + c * m12,
            this.m20,
            c * m21 - s * m22,
            s * m21 + c * m22
        );
    }

    public static void calcSymmetricGivensCoefficients( float a_pp, float a_pq, float a_qq, out float c, out float s ) {
        if( a_pq == 0 ) {
            c = 1;
            s = 0;
            return;
        }

        var tau = ( a_qq - a_pp ) / ( 2.0f * a_pq );
        var stt = Mathf.Sqrt( 1.0f + tau * tau );
        var tan = 1.0f / ( ( tau >= 0 ) ? ( tau + stt ) : ( tau - stt ) );

        c = 1.0f / Mathf.Sqrt( 1.0f + tan * tan );
        s = tan * c;
    }
}

public class SMat3 {

    public float m00, m01, m02, m11, m12, m22;

    public SMat3( ) {
        this.clear( );
    }

    public SMat3(
        float m00, float m01, float m02,
        float m11, float m12, float m22
    ) {
        this.setSymmetric( m00, m01, m02, m11, m12, m22 );
    }

    public void clear( ) {
        this.setSymmetric( 0, 0, 0, 0, 0, 0 );
    }

    public void setSymmetric( float a00, float a01, float a02, float a11, float a12, float a22 ) {
        this.m00 = a00;
        this.m01 = a01;
        this.m02 = a02;
        this.m11 = a11;
        this.m12 = a12;
        this.m22 = a22;
    }

    public void setSymmetric( SMat3 rhs ) {
        this.setSymmetric( rhs.m00, rhs.m01, rhs.m02, rhs.m11, rhs.m12, rhs.m22 );
    }

    public float fnorm( ) {
        return Mathf.Sqrt(
            ( m00 * m00 ) + ( m01 * m01 ) + ( m02 * m02 ) +
            ( m01 * m01 ) + ( m11 * m11 ) + ( m12 * m12 ) +
            ( m02 * m02 ) + ( m12 * m12 ) + ( m22 * m22 )
        );
    }

    public float off( ) {
        return Mathf.Sqrt( 2 * ( ( m01 * m01 ) + ( m02 * m02 ) + ( m12 * m12 ) ) );
    }

    public SMat3 Mul_ata( Mat3 a ) {
        var m = new SMat3( );
        m.setSymmetric(
            a.m00 * a.m00 + a.m10 * a.m10 + a.m20 * a.m20,
            a.m00 * a.m01 + a.m10 * a.m11 + a.m20 * a.m21,
            a.m00 * a.m02 + a.m10 * a.m12 + a.m20 * a.m22,
            a.m01 * a.m01 + a.m11 * a.m11 + a.m21 * a.m21,
            a.m01 * a.m02 + a.m11 * a.m12 + a.m21 * a.m22,
            a.m02 * a.m02 + a.m12 * a.m12 + a.m22 * a.m22
        );
        return m;
    }

    public Vector3 vmul( Vector3 v ) {
        var o = new Vector3( );
        o.x = ( m00 * v.x ) + ( m01 * v.y ) + ( m02 * v.z );
        o.y = ( m01 * v.x ) + ( m11 * v.y ) + ( m12 * v.z );
        o.z = ( m02 * v.x ) + ( m12 * v.y ) + ( m22 * v.z );
        return o;
    }

    public void rot01( ref float c, ref float s ) {
        Mat3.calcSymmetricGivensCoefficients( m00, m01, m11, out c, out s );
        var cc  = c * c;
        var ss  = s * s;
        var mix = 2 * c * s * m01;
        this.setSymmetric(
            cc * m00 - mix + ss * m11,
            0,
            c  * m02 - s        * m12,
            ss * m00 + mix + cc * m11,
            s  * m02 + c        * m12,
            m22
        );
    }

    public void rot02( ref float c, ref float s ) {
        Mat3.calcSymmetricGivensCoefficients( m00, m02, m22, out c, out s );
        var cc  = c * c;
        var ss  = s * s;
        var mix = 2 * c * s * m02;
        this.setSymmetric(
            cc * m00 - mix + ss * m22,
            c  * m01 - s        * m12,
            0,
            m11,
            s  * m01 + c        * m12,
            ss * m00 + mix + cc * m22
        );
    }

    public void rot12( ref float c, ref float s) {
        Mat3.calcSymmetricGivensCoefficients( m11, m12, m22, out c, out s );
        var cc  = c * c;
        var ss  = s * s;
        var mix = 2 * c * s * m12;
        this.setSymmetric(
            m00,
            c  * m01 - s        * m02,
            s  * m01 + c        * m02,
            cc * m11 - mix + ss * m22,
            0,
            ss * m11 + mix + cc * m22
        );
    }


}

public class SVD {

    public static void rotate01( ref SMat3 vtav, ref Mat3 v ) {
        if( vtav.m01 == 0 ) {
            return;
        }

        float c = 0, s = 0;
        vtav.rot01( ref c, ref s );

        c = 0;
        s = 0;
        v.rot01_post( c, s );
    }

    public static void rotate02( ref SMat3 vtav, ref Mat3 v ) {
        if( vtav.m02 == 0 ) {
            return;
        }

        float c = 0, s = 0;
        vtav.rot02( ref c, ref s );

        c = 0;
        s = 0;
        v.rot02_post( c, s );
    }

    public static void rotate12( ref SMat3 vtav, ref Mat3 v ) {
        if( vtav.m12 == 0 ) {
            return;
        }

        float c = 0, s = 0;
        vtav.rot12( ref c, ref s );

        c = 0;
        s = 0;
        v.rot12_post( c, s );
    }

    public static void getSymmetricSVD( ref SMat3 a, ref SMat3 vtav, ref Mat3 v, float tol, int max_sweeps ) {
        vtav.setSymmetric( a );
        v.set( 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f );

        var delta = tol * vtav.fnorm( );

        for( var i = 0; i < max_sweeps && vtav.off( ) > delta; i++ ) {
            rotate01( ref vtav, ref v );
            rotate02( ref vtav, ref v );
            rotate12( ref vtav, ref v );
        }
    }

    public static float calculateError( Mat3 a, Vector3 x, Vector3 b ) {
        var vtmp = a.vmul( x );
        vtmp = b - vtmp;
        return ( vtmp.x * vtmp.x ) + ( vtmp.y * vtmp.y ) + ( vtmp.z * vtmp.z );
    }

    public static float calculateError( SMat3 origA, Vector3 x, Vector3 b ) {
        var A = new Mat3( );
        A.setSymmetric( origA );

        var vtmp = A.vmul( x );
        vtmp = b - vtmp;

        return ( vtmp.x * vtmp.x ) + ( vtmp.y * vtmp.y ) + ( vtmp.z * vtmp.z );
    }

    public static float pinv( float x, float tol ) {
        return ( Mathf.Abs( x ) < tol || Mathf.Abs( 1.0f / x ) < tol ) ? 0.0f : ( 1.0f / x );
    }

    public static Mat3 pseudoInverse( SMat3 d, Mat3 v, float tol ) {
        var m = new Mat3( );
        float d0 = pinv( d.m00, tol ),
              d1 = pinv( d.m11, tol ),
              d2 = pinv( d.m22, tol );

        m.set(
            v.m00 * d0 * v.m00 + v.m01 * d1 * v.m01 + v.m02 * d2 * v.m02,
            v.m00 * d0 * v.m10 + v.m01 * d1 * v.m11 + v.m02 * d2 * v.m12,
            v.m00 * d0 * v.m20 + v.m01 * d1 * v.m21 + v.m02 * d2 * v.m22,
            v.m10 * d0 * v.m00 + v.m11 * d1 * v.m01 + v.m12 * d2 * v.m02,
            v.m10 * d0 * v.m10 + v.m11 * d1 * v.m11 + v.m12 * d2 * v.m12,
            v.m10 * d0 * v.m20 + v.m11 * d1 * v.m21 + v.m12 * d2 * v.m22,
            v.m20 * d0 * v.m00 + v.m21 * d1 * v.m01 + v.m22 * d2 * v.m02,
            v.m20 * d0 * v.m10 + v.m21 * d1 * v.m11 + v.m22 * d2 * v.m12,
            v.m20 * d0 * v.m20 + v.m21 * d1 * v.m21 + v.m22 * d2 * v.m22
        );

        return m;
    }

    public static float solveSymmetric( SMat3 A, Vector3 b, ref Vector3 x, float svd_tol, int svd_sweeps, float pinv_tol ) {
        Mat3 mtmp = new Mat3( ), pinv = new Mat3( ), V = new Mat3( );

        var VTAV = new SMat3( );
        getSymmetricSVD( ref A, ref VTAV, ref V, svd_tol, svd_sweeps );

        pinv = pseudoInverse( VTAV, V, pinv_tol );
        x    = pinv.vmul( b );

        return calculateError( A, x, b );
    }

    float solveLeastSquares( Mat3 a, Vector3 b, ref Vector3 x, float svd_tol, int svd_sweeps, float pinv_tol ) {
        var at  = new Mat3( );
        var ata = new SMat3( );
        var atb = new Vector3( );

        at  = a.transpose( );
        ata = a.mulATA( );
        atb = at.vmul( b );

        return solveSymmetric( ata, atb, ref x, svd_tol, svd_sweeps, pinv_tol );
    }

}