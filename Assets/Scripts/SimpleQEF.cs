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

    public SimpleQEF( int minimizerIterations ) {
        this.minimizerIterations = minimizerIterations;
    }

    public void add( Vector3 intersection, Vector3 normal ) {
        this.intersectionPlanes.Add( new( normal, intersection ) );
    }

    public void combine( QEFSolver other ) {
        if( other is SimpleQEF solver ) {
            this.intersectionPlanes.AddRange( solver.intersectionPlanes );
        }
        else {
            throw new Exception( "Unable to combine different derived solver" );
        }
    }

    public ( Vector3 vertex, float error ) solve( SurfaceExtractor.Voxel voxel, IEnumerable<DensityFunction> densityFunctions ) {
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

        // error value is simply how far the minimizing vertex is from the surface before correction
        var error = Mathf.Abs( SurfaceExtractor.calculateDensity( minimizingVertex, densityFunctions ) );

        return ( minimizingVertex, error );
    }

}