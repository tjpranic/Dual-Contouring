using NUnit.Framework;
using UnityEngine;

using ImplementationType = SurfaceExtractor.Implementation.Type;
using QEFSolverType      = QEFSolver.Type;

// a voxelizer that is added to a scene starts out contouring the way the papers describe: the vertex of a cell is the
// minimizer of its QEF, and stays where the QEF puts it. It runs the managed implementation where the algorithm has one,
// leaves the mesh of the contour as it is, and draws no wireframe until asked to
public class VoxelizerDefaultsTest {

    private static void assertDefaults<T>( ImplementationType implementation = ImplementationType.Managed ) where T : Voxelizer {
        var gameObject = new GameObject( typeof( T ).Name ) { hideFlags = HideFlags.HideAndDontSave };

        try {
            var voxelizer = gameObject.AddComponent<T>( );

            Assert.AreEqual( implementation,    voxelizer.implementationType,          "implementation" );
            Assert.AreEqual( QEFSolverType.SVD, voxelizer.qefSolverType,               "solver" );
            Assert.AreEqual( 0,                 voxelizer.surfaceCorrectionIterations, "surface correction iterations" );

            Assert.IsFalse( voxelizer.meshSimplification, "mesh simplification" );
            Assert.AreEqual( 0.05f, voxelizer.simplificationError, "simplification error" );

            Assert.AreEqual( Wireframe.Look.Off, voxelizer.wireframe,          "wireframe" );
            Assert.AreEqual( 1.0f,               voxelizer.wireframeLineWidth, "wireframe line width" );
            Assert.AreEqual( Color.black,        voxelizer.wireframeLineColor, "wireframe line colour" );
            Assert.AreEqual( Color.black,        voxelizer.wireframeFillColor, "wireframe fill colour" );
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

    // it has no managed implementation, and searches as the paper does
    [Test]
    public void occupancyDualContouring( ) {
        assertDefaults<OccupancyDualContouring>( ImplementationType.Burst );

        var gameObject = new GameObject( nameof( OccupancyDualContouring ) ) { hideFlags = HideFlags.HideAndDontSave };

        try {
            var voxelizer = gameObject.AddComponent<OccupancyDualContouring>( );

            Assert.AreEqual( 15, voxelizer.edgeSearchIterations, "edge search iterations" );
            Assert.AreEqual( 4,  voxelizer.faceSearchSamples,    "face search samples" );
            Assert.AreEqual( 11, voxelizer.faceSearchIterations, "face search iterations" );

            // and as its reference implementation does: vertices are kept in their cells and polygons split
            Assert.IsTrue( voxelizer.keepVerticesInCells, "keep vertices in cells" );
            Assert.IsTrue( voxelizer.splitQuads,          "split quads" );
        }
        finally {
            Object.DestroyImmediate( gameObject );
        }
    }

}
