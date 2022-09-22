using System.Collections.Generic;
using UnityEngine;

public interface QEFSolver{

    public bool empty { get; }

    public void add( Vector3 intersection, Vector3 normal );

    public void combine( QEFSolver other );

    public ( Vector3 vertex, Vector3 normal, float error ) solve( SurfaceExtractor.Voxel voxel, IEnumerable<DensityFunction> densityFunctions );

    public static Vector3 surfaceCorrection( Vector3 vertex, Vector3 normal, int surfaceCorrectionIterations, IEnumerable<DensityFunction> densityFunctions ) {
        // correct surface by forcing the minimizing vertex towards the zero crossing
        // see https://www.reddit.com/r/VoxelGameDev/comments/mhiec0/how_are_people_getting_good_results_with_dual/gti0b8d/
        for( var surfaceCorrectionIteration = 0; surfaceCorrectionIteration < surfaceCorrectionIterations; ++surfaceCorrectionIteration ) {
            var density = SurfaceExtractor.calculateDensity( vertex, densityFunctions );
            if( density == 0.0f ) {
                // vertex is at the surface
                break;
            }
            vertex -= normal * density;
        }
        return vertex;
    }

}