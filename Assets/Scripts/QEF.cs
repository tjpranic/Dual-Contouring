using System.Collections.Generic;
using System.Linq;
using UnityEngine;

// TODO: integrate this in somehow

public class QEF : QEFSolver {

    private Vector3 _vertex = Vector3.zero;
    public Vector3 vertex {
        get { return this._vertex; }
    }

    private Vector3 _normal = Vector3.zero;
    public Vector3 normal {
        get { return this._normal; }
    }

    private float _error = float.NaN;
    public float error {
        get { return this._error; }
    }

    private readonly int minimizerIterations;
    private readonly int surfaceCorrectionIterations;

    private readonly List<Plane> intersectionPlanes = new( );

    public QEF( int minimizerIterations, int surfaceCorrectionIterations ) {
        this.minimizerIterations         = minimizerIterations;
        this.surfaceCorrectionIterations = surfaceCorrectionIterations;
    }

    public void add( Vector3 intersection, Vector3 normal ) {
        this.intersectionPlanes.Add( new( normal, intersection ) );
    }

    public void solve( SurfaceExtractor.Voxel voxel, IEnumerable<DensityFunction> densityFunctions ) {
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

        // calculate surface normal
        var surfaceNormal = Vector3.Normalize(
            this.intersectionPlanes.Aggregate(
                Vector3.zero,
                ( accumulator, plane ) => {
                    accumulator += plane.normal;
                    return accumulator;
                }
            ) / this.intersectionPlanes.Count
        );

        // error value is simply how far the minimizing vertex is from the surface before correction
        this._error = Mathf.Abs( SurfaceExtractor.calculateDensity( minimizingVertex, densityFunctions ) );

        // correct surface by forcing the minimizing vertex towards the zero crossing
        // see https://www.reddit.com/r/VoxelGameDev/comments/mhiec0/how_are_people_getting_good_results_with_dual/gti0b8d/
        for( var surfaceCorrectionIteration = 0; surfaceCorrectionIteration < this.surfaceCorrectionIterations; ++surfaceCorrectionIteration ) {
            var density = SurfaceExtractor.calculateDensity( minimizingVertex, densityFunctions );
            if( density == 0.0f ) {
                // vertex is at the surface
                break;
            }
            minimizingVertex -= surfaceNormal * density;
        }

        this._vertex = minimizingVertex;
        this._normal = surfaceNormal;
    }

}