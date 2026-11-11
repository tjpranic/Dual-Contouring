using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class SimpleQEF : QEFSolver {

    public int minimizerIterations { get; set; }
    public int intersectionCount {
        get { return this.intersectionPlanes.Count; }
    }

    private readonly List<Plane> intersectionPlanes = new( );

    private Vector3 massPoint; // the sum of the intersections, the mass point is their average

    public SimpleQEF( int minimizerIterations ) {
        this.minimizerIterations = minimizerIterations;
    }

    public void add( Vector3 intersection, Vector3 normal ) {
        this.intersectionPlanes.Add( new( normal, intersection ) );

        this.massPoint += intersection;
    }

    public void combine( QEFSolver other ) {
        if( other is SimpleQEF solver ) {
            this.intersectionPlanes.AddRange( solver.intersectionPlanes );

            this.massPoint += solver.massPoint;
        }
        else {
            throw new Exception( "Unable to combine different derived solver" );
        }
    }

    public ( Vector3 vertex, float error ) solve( SurfaceExtractor.Voxel voxel ) {
        if( this.intersectionCount == 0 ) {
            throw new Exception( "Unable to solve QEF, no intersections accumulated" );
        }

        // calculate minimizing vertex
        // see https://gamedev.stackexchange.com/questions/83457/can-someone-explain-dual-contouring
        // and https://gamedev.stackexchange.com/questions/111387/dual-contouring-finding-the-feature-point-normals-off
        var minimizingVertex = voxel.center;

        for( var minimizingIteration = 0; minimizingIteration < this.minimizerIterations; ++minimizingIteration ) {
            minimizingVertex -= this.intersectionPlanes.Aggregate(
                Vector3.zero,
                ( accumulator, plane ) => {
                    accumulator += plane.GetDistanceToPoint( minimizingVertex ) * plane.normal;
                    return accumulator;
                }
            ) / this.intersectionPlanes.Count;
        }

        return ( minimizingVertex, this.calculateError( minimizingVertex ) );
    }

    public ( Vector3 vertex, float error ) solveMassPoint( ) {
        if( this.intersectionCount == 0 ) {
            throw new Exception( "Unable to solve QEF, no intersections accumulated" );
        }

        var massPoint = this.massPoint / this.intersectionCount;

        return ( massPoint, this.calculateError( massPoint ) );
    }

    // the value of the QEF at x: the sum over the hermite samples of the squared distance from x to the plane of the
    // sample
    private float calculateError( Vector3 x ) {
        return this.intersectionPlanes.Sum( ( plane ) => plane.GetDistanceToPoint( x ) * plane.GetDistanceToPoint( x ) );
    }

}