using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public partial interface QEFSolver {

    public int  minimizerIterations { get; set; }
    public int  intersectionCount   { get; }

    public void add( Vector3 intersection, Vector3 normal );

    public void combine( QEFSolver other );

    // the point that the solver finds to minimize the QEF, and the value of the QEF there: the sum over the hermite
    // samples of the squared distance from the point to the plane of the sample
    public ( Vector3 vertex, float error ) solve( SurfaceExtractor.Voxel voxel );

    // the mass point of the hermite samples, and the value of the QEF there
    public ( Vector3 vertex, float error ) solveMassPoint( );

    // the vertex of a voxel that has a single vertex for everything inside it. Dual Contouring: The Secret Sauce,
    // section 2.1: where separate pieces of the contour pass through the voxel, which the corners of the voxel tell,
    // the minimizer of the QEF is a poor place for it and can lie far outside the voxel. The mass point is used instead
    public static ( Vector3 vertex, float error ) placeVertex( QEFSolver qef, SurfaceExtractor.Voxel voxel ) {
        var materialIndices = voxel.corners.Select( ( corner ) => corner.materialIndex ).ToArray( );

        return DualMarchingCubes.lookupVertexEdges( materialIndices ).Length > 1 ? qef.solveMassPoint( ) : qef.solve( voxel );
    }

    public static Vector3 surfaceCorrection( Vector3 vertex, Vector3 normal, IEnumerable<DensityFunction> densityFunctions, int surfaceCorrectionIterations ) {
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
