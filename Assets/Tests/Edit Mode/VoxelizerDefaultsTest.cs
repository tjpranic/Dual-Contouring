using NUnit.Framework;
using UnityEngine;

using ImplementationType = SurfaceExtractor.Implementation.Type;
using QEFSolverType      = QEFSolver.Type;

// a voxelizer that is added to a scene starts out contouring the way the papers describe: the vertex of a cell is the
// minimizer of its QEF, and stays where the QEF puts it. It runs the managed implementation, which every algorithm has
public class VoxelizerDefaultsTest {

    private static void assertDefaults<T>( ) where T : Voxelizer {
        var gameObject = new GameObject( typeof( T ).Name ) { hideFlags = HideFlags.HideAndDontSave };

        try {
            var voxelizer = gameObject.AddComponent<T>( );

            Assert.AreEqual( ImplementationType.Managed, voxelizer.implementationType,          "implementation" );
            Assert.AreEqual( QEFSolverType.SVD,          voxelizer.qefSolverType,               "solver" );
            Assert.AreEqual( 0,                          voxelizer.surfaceCorrectionIterations, "surface correction iterations" );
        }
        finally {
            Object.DestroyImmediate( gameObject );
        }
    }

    [Test]
    public void uniformDualContouring( ) {
        assertDefaults<UniformDualContouring>( );
    }

    [Test]
    public void adaptiveDualContouring( ) {
        assertDefaults<AdaptiveDualContouring>( );
    }

    [Test]
    public void manifoldDualContouring( ) {
        assertDefaults<ManifoldDualContouring>( );
    }

}
