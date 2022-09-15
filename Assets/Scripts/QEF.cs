using System;
using UnityEngine;

// TODO: refactor

/*
namespace QEF {

    public class Data {

        public float ata_00, ata_01, ata_02, ata_11, ata_12, ata_22;
        public float atb_x,  atb_y,  atb_z;
        public float btb;
        public float massPoint_x, massPoint_y, massPoint_z;
        public int   numPoints;

        public Data( ) {
            this.clear( );
        }

        public Data(
            float ata_00, float ata_01, float ata_02,
            float ata_11, float ata_12, float ata_22,
            float atb_x,  float atb_y,  float atb_z,
            float btb,
            float massPoint_x, float massPoint_y, float massPoint_z,
            int   numPoints
        ) {
            this.set(
                ata_00, ata_01, ata_02,
                ata_11, ata_12, ata_22,
                atb_x,  atb_y,  atb_z,
                btb,
                massPoint_x, massPoint_y, massPoint_z,
                numPoints
            );
        }

        public void add( ref Data rhs ) {
            ata_00      += rhs.ata_00;
            ata_01      += rhs.ata_01;
            ata_02      += rhs.ata_02;
            ata_11      += rhs.ata_11;
            ata_12      += rhs.ata_12;
            ata_22      += rhs.ata_22;
            atb_x       += rhs.atb_x;
            atb_y       += rhs.atb_y;
            atb_z       += rhs.atb_z;
            btb         += rhs.btb;
            massPoint_x += rhs.massPoint_x;
            massPoint_y += rhs.massPoint_y;
            massPoint_z += rhs.massPoint_z;
            numPoints   += rhs.numPoints;
        }

        public void clear( ) {
            set( 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0 );
        }

        public void set(
            float ata_00, float ata_01, float ata_02,
            float ata_11, float ata_12, float ata_22,
            float atb_x,  float atb_y,  float atb_z,
            float btb,
            float massPoint_x, float massPoint_y, float massPoint_z,
            int   numPoints
        ) {
            this.ata_00      = ata_00;
            this.ata_01      = ata_01;
            this.ata_02      = ata_02;
            this.ata_11      = ata_11;
            this.ata_12      = ata_12;
            this.ata_22      = ata_22;
            this.atb_x       = atb_x;
            this.atb_y       = atb_y;
            this.atb_z       = atb_z;
            this.btb         = btb;
            this.massPoint_x = massPoint_x;
            this.massPoint_y = massPoint_y;
            this.massPoint_z = massPoint_z;
            this.numPoints   = numPoints;
        }

        public void set( ref Data rhs ) {
            this.set(
                rhs.ata_00, rhs.ata_01, rhs.ata_02,
                rhs.ata_11, rhs.ata_12, rhs.ata_22,
                rhs.atb_x,  rhs.atb_y,  rhs.atb_z,
                rhs.btb,
                rhs.massPoint_x, rhs.massPoint_y, rhs.massPoint_z,
                rhs.numPoints
            );
        }

    }

    public class Solver {

        public Data    data;
        public SMat3   ata;
        public Vector3 atb;
        public Vector3 x;
        public Vector3 MassPoint;
        public bool    hasSolution;
        public float   last_error;

        public Solver( ) {
            data        = new Data( );
            ata         = new SMat3( );
            atb         = new Vector3( );
            x           = new Vector3( );
            MassPoint   = new Vector3( );
            hasSolution = false;
        }

        public void Add( Vector3 p, Vector3 n ) {
            this.hasSolution = false;

            n.Normalize( );

            this.data.ata_00 += n.x * n.x;
            this.data.ata_01 += n.x * n.y;
            this.data.ata_02 += n.x * n.z;
            this.data.ata_11 += n.y * n.y;
            this.data.ata_12 += n.y * n.z;
            this.data.ata_22 += n.z * n.z;

            var dot = ( n.x * p.x ) + ( n.y * p.y ) + ( n.z * p.z );

            this.data.atb_x       += dot * n.x;
            this.data.atb_y       += dot * n.y;
            this.data.atb_z       += dot * n.z;
            this.data.btb         += dot * dot;
            this.data.massPoint_x += p.x;
            this.data.massPoint_y += p.y;
            this.data.massPoint_z += p.z;

            ++this.data.numPoints;
        }

        public void add( ref Data rhs ) {
            this.hasSolution = false;
            this.data.add( ref rhs );
        }

        public float getError( ) {
            return this.getError( x );
        }

        public float getError( Vector3 pos ) {
            if( !this.hasSolution ) {
                this.setAta( );
                this.setAtb( );
            }

            Vector3 atax = this.ata.vmul( pos );
            this.last_error = Vector3.Dot( pos, atax ) - 2.0f * Vector3.Dot( pos, this.atb ) + this.data.btb;

            if( float.IsNaN( this.last_error ) ) {
                // last_error = 10000;
                throw new Exception( "Unable to calculate QEF error" );
            }

            return last_error;
        }

        public Vector3 solve( float svd_tol, int svd_sweeps, float pinv_tol ) {
            if( this.data.numPoints == 0 ) {
                throw new Exception( "Unable to solve, no data set" );
            }

            this.MassPoint  = new Vector3( this.data.massPoint_x, this.data.massPoint_y, this.data.massPoint_z );
            this.MassPoint /= this.data.numPoints;

            this.setAta( );
            this.setAtb( );

            this.atb -= ata.vmul( this.MassPoint );

            this.x = Vector3.zero;

            var result = SVD.solveSymmetric( this.ata, this.atb, ref this.x, svd_tol, svd_sweeps, pinv_tol );
            if( float.IsNaN( result ) ) {
                this.x = this.MassPoint;

            }
            else {
                this.x += this.MassPoint;
            }

            this.last_error = result;
            Debug.Assert( result >= 0.0f );

            this.setAtb( );

            this.hasSolution = true;

            return x;
        }

        private void setAta( ) {
            this.ata.setSymmetric(
                this.data.ata_00, this.data.ata_01, this.data.ata_02,
                this.data.ata_11, this.data.ata_12, this.data.ata_22
            );
        }

        private void setAtb( ) {
            this.atb = new Vector3( this.data.atb_x, this.data.atb_y, this.data.atb_z );
        }

    }

}
*/