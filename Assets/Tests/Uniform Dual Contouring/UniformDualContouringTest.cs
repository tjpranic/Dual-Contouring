using NUnit.Framework;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.TestTools;

public class UniformDualContouringTest {

    protected const float epsilon = 6e-6f;

    private struct ContourResult {

        public Mesh                                 mesh;
        public IEnumerable<SurfaceExtractor.Corner> corners;
        public IEnumerable<SurfaceExtractor.Edge>   edges;
        public IEnumerable<SurfaceExtractor.Voxel>  voxels;

        public ContourResult(
            (
                Mesh                                 mesh,
                IEnumerable<SurfaceExtractor.Corner> corners,
                IEnumerable<SurfaceExtractor.Edge>   edges,
                IEnumerable<SurfaceExtractor.Voxel>  voxels
            ) contour
        ) {
            this.mesh    = contour.mesh;
            this.corners = contour.corners;
            this.edges   = contour.edges;
            this.voxels  = contour.voxels;
        }

    }

    [UnityTest]
    public IEnumerator uniformDualContouringTest( ) {

        var voxelizer = new GameObject( "Uniform Dual Contouring" );

        voxelizer.transform.localScale = new Vector3( 5.0f, 5.0f, 5.0f );

        var uniformDualContouring = voxelizer.AddComponent<UniformDualContouring>( );

        uniformDualContouring.resolution                  = 2;
        uniformDualContouring.minimizerIterations         = 6;
        uniformDualContouring.binarySearchIterations      = 6;
        uniformDualContouring.surfaceCorrectionIterations = 0;
        uniformDualContouring.qefSolverType               = QEFSolver.Type.SVD;
        uniformDualContouring.intersectionApproximation   = SurfaceExtractor.IntersectionApproximation.BinarySearch;
        uniformDualContouring.implementationType          = SurfaceExtractor.Implementation.Type.CPU;
        uniformDualContouring.testing                     = true;

        var sphere = new GameObject( "Sphere" );

        sphere.transform.parent     = voxelizer.transform;
        sphere.transform.localScale = new Vector3( 5.0f, 5.0f, 5.0f );

        var volume = sphere.AddComponent<Ellipsoid>( );

        volume.materialIndex = SurfaceExtractor.MaterialIndex.Material1;

        var densityFunctions = new List<DensityFunction> { volume };

        // wait for lifecycle methods
        yield return new WaitForSeconds( 1.0f / 60.0f );

        var expectedResult = new ContourResult( uniformDualContouring.voxelize( densityFunctions ) );

        uniformDualContouring.implementationType = SurfaceExtractor.Implementation.Type.GPU;

        var actualResult = new ContourResult( uniformDualContouring.voxelize( densityFunctions ) );

        this.compareOutput( expectedResult.mesh.vertices, actualResult.mesh.vertices );
        this.compareOutput( expectedResult.mesh.normals,  actualResult.mesh.normals );

        // GPU indexes voxels out of order, meaning this comparison isn't possible:
        // this.compareOutput( expectedResult.mesh.triangles, actualResult.mesh.triangles );
        // so just compare triangle count instead
        Assert.AreEqual( expectedResult.mesh.triangles.Length, actualResult.mesh.triangles.Length );

        this.compareOutput( expectedResult.edges.ToArray( ),   actualResult.edges.ToArray( ) );
        this.compareOutput( expectedResult.corners.ToArray( ), actualResult.corners.ToArray( ) );
        this.compareOutput( expectedResult.voxels.ToArray( ),  actualResult.voxels.ToArray( ) );
    }

    protected void compareOutput<T>( T[] expected, T[] actual ) where T : IEquatable<T> {
        Assert.AreEqual( expected.Length, actual.Length );

        var indices = new Dictionary<int, int>( );
        for( var i = 0; i < expected.Length; ++i ) {
            for( var j = 0; j < actual.Length; ++j ) {
                if(
                    !indices.ContainsKey   ( i ) &&
                    !indices.ContainsValue ( j ) &&
                    expected[i].Equals( actual[j] )
                ) {
                    indices[i] = j;
                }
            }
        }

        Assert.AreEqual( expected.Length, indices.Count );

        foreach( var index in indices.Keys ) {
            var otherIndex = indices[index];

            Assert.AreEqual( expected[index], actual[otherIndex] );
        }
    }

}
