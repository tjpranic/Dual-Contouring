using System;
using UnityEngine;

// Fast and Robust QEF Minimization using Probabilistic Quadrics (Trettner and Kobbelt), section 3.1. The plane of a
// hermite sample is taken as uncertain, with Gaussian noise of the given standard deviation in each component of its
// normal. What the vertex minimizes is then the expected QEF, whose quadric is E[A] = n nT + s^2 I and E[b] = n nT q + s^2 q:
// the plane's own quadric, and a pull towards the sample's point that the noise weighs. Summed over the samples of a
// voxel, with N of them and p their mass point,
//
//     x = p + ( AtA + N s^2 I )^-1 ( AtB - AtA p )
//
// The matrix has full rank for any deviation above zero, so the minimizer is the solution of a linear system: no SVD,
// no truncation and nothing to iterate. Section 5.1: at a deviation of zero this is the classical QEF, and as it grows the
// vertex goes to the mass point. The error is the classical QEF at the vertex, as for the other solvers, so that an
// error threshold means the same for all three
public class ProbabilisticQEF : QEFSolver {

    // the solve has nothing to iterate. The setting is kept for the interface
    public int minimizerIterations { get; set; }

    // the standard deviation of the noise in each component of a normal, as a fraction of its length. It has to be above
    // zero: at zero the matrix is singular wherever the samples lie on a plane or along an edge
    public float normalDeviation { get; set; }

    private int _intersectionCount;
    public int intersectionCount {
                get { return this._intersectionCount;  }
        private set { this._intersectionCount = value; }
    }

    private readonly SVDQEF.SymmetricMatrix3x3 ATA = new( );
    private Vector3                            ATB;
    private float                              BTB;

    // the sum of the intersections, the mass point is their average. Unlike the SVD solver's it is always that of every
    // sample, combined or not: it is the point every sample's pull goes to
    private Vector3 massPoint;

    public ProbabilisticQEF( float normalDeviation ) {
        this.normalDeviation = normalDeviation;
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
        if( other is ProbabilisticQEF solver ) {
            this.ATA.m00           += solver.ATA.m00;
            this.ATA.m01           += solver.ATA.m01;
            this.ATA.m02           += solver.ATA.m02;
            this.ATA.m11           += solver.ATA.m11;
            this.ATA.m12           += solver.ATA.m12;
            this.ATA.m22           += solver.ATA.m22;
            this.ATB               += solver.ATB;
            this.BTB               += solver.BTB;
            this.massPoint         += solver.massPoint;
            this.intersectionCount += solver.intersectionCount;
        }
        else {
            throw new Exception( "Unable to combine different derived solvers" );
        }
    }

    public ( Vector3 vertex, float error ) solve( SurfaceExtractor.Voxel voxel ) {
        if( this.intersectionCount == 0 ) {
            throw new Exception( "Unable to solve QEF, no intersections accumulated" );
        }

        var massPoint = this.massPoint / this.intersectionCount;

        // the move from the mass point, which keeps the numbers of the solve small
        var pull = this.intersectionCount * this.normalDeviation * this.normalDeviation;

        var vertex = minimize( this.ATA, this.ATB - this.ATA.multiplyVector( massPoint ), pull ) + massPoint;

        return ( vertex, SVDQEF.SVD.calculateError( this.ATA, this.ATB, this.BTB, vertex ) );
    }

    public ( Vector3 vertex, float error ) solveMassPoint( ) {
        if( this.intersectionCount == 0 ) {
            throw new Exception( "Unable to solve QEF, no intersections accumulated" );
        }

        var massPoint = this.massPoint / this.intersectionCount;

        return ( massPoint, SVDQEF.SVD.calculateError( this.ATA, this.ATB, this.BTB, massPoint ) );
    }

    // the solution d of ( AtA + pull I ) d = r. The matrix is symmetric and, with a pull above zero, positive definite, and
    // is solved by its LDLT decomposition, unrolled. This is minimizer() of the authors' probabilistic-quadrics.hh
    // (github.com/Philip-Trettner/probabilistic-quadrics, Copyright (c) 2020 Philip Trettner, MIT License), operation for
    // operation, without its fused multiply-adds: the Burst and GPU solvers do the same
    public static Vector3 minimize( SVDQEF.SymmetricMatrix3x3 ATA, Vector3 r, float pull ) {
        var a00 = ATA.m00 + pull;
        var a10 = ATA.m01;
        var a20 = ATA.m02;
        var a11 = ATA.m11 + pull;
        var a21 = ATA.m12;
        var a22 = ATA.m22 + pull;

        var x0 = r.x;
        var x1 = r.y;
        var x2 = r.z;

        var d0  = 1.0f / a00;
        var l10 = a10 * -d0;
        var l20 = a20 * -d0;

        a11 = ( a10 * l10 ) + a11;
        a21 = ( a20 * l10 ) + a21;
        a22 = ( a20 * l20 ) + a22;

        var d1  = 1.0f / a11;
        var l21 = a21 * -d1;

        a22 = ( a21 * l21 ) + a22;

        var d2 = 1.0f / a22;

        x1 = ( l10 * x0 ) + x1;
        x2 = ( l20 * x0 ) + x2;
        x2 = ( l21 * x1 ) + x2;

        x0 *= d0;
        x1 *= d1;
        x2 *= d2;

        x0 = ( l20 * x2 ) + x0;
        x1 = ( l21 * x2 ) + x1;
        x0 = ( l10 * x1 ) + x0;

        return new Vector3( x0, x1, x2 );
    }

}
