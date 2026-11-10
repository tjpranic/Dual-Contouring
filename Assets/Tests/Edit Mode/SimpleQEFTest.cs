using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using UnityEngine;

using VoxelType = SurfaceExtractor.Voxel.Type;

// the simple solver walks from the center of the voxel towards the planes of the hermite samples instead of solving for
// the minimizer of the QEF. Whatever point it ends up at, the error it reports is the value of the same QEF there, so
// that an error threshold means the same thing for either solver
public class SimpleQEFTest {

    private static readonly SurfaceExtractor.Voxel voxel = new AdaptiveDualContouring.Voxel( VoxelType.Leaf, 0, new Vector3( 0.1f, 0.15f, 0.2f ), Vector3.one * 0.1f );

    // equation 1 of both dual contouring papers: E[x] = sum of ( n . ( x - p ) )^2 over the hermite samples
    private static float residualOf( List<( Vector3 intersection, Vector3 normal )> samples, Vector3 point ) {
        return samples.Sum( ( sample ) => Mathf.Pow( Vector3.Dot( sample.normal, point - sample.intersection ), 2.0f ) );
    }

    private static ( Vector3 vertex, float error ) solve( List<( Vector3 intersection, Vector3 normal )> samples ) {
        var qef = new SimpleQEF( 6 );

        foreach( var ( intersection, normal ) in samples ) {
            qef.add( intersection, normal );
        }

        return qef.solve( voxel );
    }

    // four samples of a sphere do not meet in a point, the residual of the vertex is whatever is left over
    [Test]
    public void errorIsResidualAtVertex( ) {
        var samples = new[] { new Vector3( 1.0f, 2.0f, 3.0f ), new Vector3( 1.3f, 2.0f, 3.0f ), new Vector3( 1.0f, 2.4f, 3.0f ), new Vector3( 1.3f, 2.4f, 2.8f ) }.Select(
            ( direction ) => ( intersection: Vector3.Normalize( direction ) * 0.3f, normal: Vector3.Normalize( direction ) )
        ).ToList( );

        var ( vertex, error ) = solve( samples );

        Assert.Greater( error, 0.0f );

        Assert.AreEqual( residualOf( samples, vertex ), error, 1e-6f );
    }

    // the top and bottom face of a slab 0.02 thick: wherever the vertex ends up between them, the squares of its
    // distances to the eight samples add up to at least 8 * 0.01^2, and to exactly that in the middle
    [Test]
    public void errorOfSlabIsDistanceToBothFaces( ) {
        var samples = new List<( Vector3 intersection, Vector3 normal )>( );

        foreach( var x in new[] { 0.08f, 0.12f } ) {
            foreach( var z in new[] { 0.18f, 0.22f } ) {
                samples.Add( ( new Vector3( x, 0.16f, z ), Vector3.up   ) );
                samples.Add( ( new Vector3( x, 0.14f, z ), Vector3.down ) );
            }
        }

        var ( _, error ) = solve( samples );

        Assert.AreEqual( 8.0f * 0.01f * 0.01f, error, 1e-6f );
    }

}
