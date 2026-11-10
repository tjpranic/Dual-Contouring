using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using UnityEngine;

using MaterialIndex = SurfaceExtractor.MaterialIndex;
using VoxelType     = SurfaceExtractor.Voxel.Type;
using QEFSolverType = QEFSolver.Type;

// Dual Contouring: "The Secret Sauce", section 2.1, on where the one vertex of a cell goes: "In the case where multiple,
// separate pieces of a surface intersect the same cube, QEF's can do a poor job of positioning the minimizer [...]
// non-manifold configurations are simple to detect using the signs at the eight corners of a cube [...] In these
// cases, we use the position of the mass point to position the vertex inside of the cube"
public class QEFSolverTest {

    private const MaterialIndex _ = MaterialIndex.Void;
    private const MaterialIndex A = MaterialIndex.Material0;

    // three perpendicular faces that meet in a point outside of the voxel, each sampled inside of it
    private static readonly Vector3 corner = new( 0.3f, 0.2f, 0.25f );

    private static readonly List<( Vector3 intersection, Vector3 normal )> samples = new( ) {
        ( new Vector3( 0.3f,  0.02f, 0.03f ), Vector3.right   ),
        ( new Vector3( 0.3f,  0.04f, 0.01f ), Vector3.right   ),
        ( new Vector3( 0.03f, 0.2f,  0.02f ), Vector3.up      ),
        ( new Vector3( 0.01f, 0.2f,  0.04f ), Vector3.up      ),
        ( new Vector3( 0.02f, 0.03f, 0.25f ), Vector3.forward )
    };

    private static SurfaceExtractor.Voxel voxelWith( params MaterialIndex[] corners ) {
        var voxel = new AdaptiveDualContouring.Voxel( VoxelType.Leaf, 0, Vector3.one * 0.03f, Vector3.one * 0.06f );

        for( var index = 0; index < corners.Length; ++index ) {
            voxel.corners[index].materialIndex = corners[index];
        }

        return voxel;
    }

    private static QEFSolver create( QEFSolverType solver ) {
        QEFSolver qef = solver switch {
            QEFSolverType.Simple => new SimpleQEF ( 6 ),
            QEFSolverType.SVD    => new SVDQEF    ( 6 ),
            _                    => throw new Exception( "Unknown solver type specified" )
        };

        foreach( var ( intersection, normal ) in samples ) {
            qef.add( intersection, normal );
        }

        return qef;
    }

    private static Vector3 massPoint {
        get { return samples.Aggregate( Vector3.zero, ( sum, sample ) => sum + sample.intersection ) / samples.Count; }
    }

    // equation 1 of both dual contouring papers: E[x] = sum of ( n . ( x - p ) )^2 over the hermite samples
    private static float residualOf( Vector3 point ) {
        return samples.Sum( ( sample ) => Mathf.Pow( Vector3.Dot( sample.normal, point - sample.intersection ), 2.0f ) );
    }

    [Test]
    public void massPointIsTheAverageOfTheIntersections( [Values] QEFSolverType solver ) {
        var ( vertex, error ) = create( solver ).solveMassPoint( );

        Assert.Less( Vector3.Distance( massPoint, vertex ), 1e-6f, $"the vertex is {vertex:F5}, the mass point {massPoint:F5}" );

        Assert.AreEqual( residualOf( massPoint ), error, 1e-6f );
    }

    // a voxel with a single piece of contour gets what the solver finds, also when that lies outside of the voxel
    [Test]
    public void vertexOfVoxelWithOnePieceOfContourIsTheMinimizer( [Values] QEFSolverType solver ) {
        var voxel = voxelWith( A, _, _, _, _, _, _, _ );

        var ( expected, expectedError ) = create( solver ).solve( voxel );

        var ( vertex, error ) = QEFSolver.placeVertex( create( solver ), voxel );

        Assert.AreEqual( expected,      vertex );
        Assert.AreEqual( expectedError, error  );

        if( solver == QEFSolverType.SVD ) {
            Assert.Less( Vector3.Distance( corner, vertex ), 1e-4f, $"the vertex is {vertex:F5}, the corner {corner:F5}" );
        }
    }

    // two solid corners diagonally across a face are two pieces of contour
    [Test]
    public void vertexOfVoxelWithSeparatePiecesOfContourIsTheMassPoint( [Values] QEFSolverType solver ) {
        var voxel = voxelWith( A, _, A, _, _, _, _, _ );

        var ( vertex, error ) = QEFSolver.placeVertex( create( solver ), voxel );

        Assert.Less( Vector3.Distance( massPoint, vertex ), 1e-6f, $"the vertex is {vertex:F5}, the mass point {massPoint:F5}" );

        Assert.AreEqual( residualOf( massPoint ), error, 1e-6f );
    }

}
