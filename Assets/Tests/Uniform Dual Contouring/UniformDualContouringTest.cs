using NUnit.Framework;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.TestTools;

using Corner                    = SurfaceExtractor.Corner;
using Edge                      = SurfaceExtractor.Edge;
using Voxel                     = SurfaceExtractor.Voxel;
using ImplementationType        = SurfaceExtractor.Implementation.Type;
using CPUVoxelization           = SurfaceExtractor.Implementation.CPU.Voxelization;
using GPUVoxelization           = SurfaceExtractor.Implementation.GPU.Voxelization;
using IntersectionApproximation = SurfaceExtractor.IntersectionApproximation;
using QEFSolverType             = QEFSolver.Type;

public class UniformDualContouringTest {

    protected const float epsilon = 3e-3f;

    private record ContourResult {
        public Mesh                mesh;
        public IEnumerable<Corner> corners;
        public IEnumerable<Edge>   edges;
        public IEnumerable<Voxel>  voxels;
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
        uniformDualContouring.qefSolverType               = QEFSolverType.SVD;
        uniformDualContouring.intersectionApproximation   = IntersectionApproximation.BinarySearch;
        uniformDualContouring.implementationType          = ImplementationType.CPU;
        uniformDualContouring.testing                     = true;

        var sphere = new GameObject( "Sphere" );

        sphere.transform.parent     = voxelizer.transform;
        sphere.transform.localScale = new Vector3( 5.0f, 5.0f, 5.0f );

        var volume = sphere.AddComponent<Ellipsoid>( );

        volume.materialIndex = SurfaceExtractor.MaterialIndex.Material1;

        var densityFunctions = new List<DensityFunction> { volume };

        // wait for lifecycle methods
        yield return new WaitForSeconds( 1.0f / 60.0f );

        ContourResult getCPUVoxelization( CPUVoxelization voxelization ) => new( ) {
            mesh    = voxelization.mesh,
            corners = voxelization.corners,
            edges   = voxelization.edges,
            voxels  = voxelization.voxels
        };
        ContourResult getGPUVoxelization( GPUVoxelization voxelization ) {
            var converted = uniformDualContouring.convert( voxelization );
            return new( ) {
                mesh    = converted.mesh,
                corners = converted.corners,
                edges   = converted.edges,
                voxels  = converted.voxels
            };
        }

        var expectedResult = uniformDualContouring.voxelize( densityFunctions ).visit( getCPUVoxelization, getGPUVoxelization );

        uniformDualContouring.implementationType = ImplementationType.GPU;

        var actualResult = uniformDualContouring.voxelize( densityFunctions ).visit( getCPUVoxelization, getGPUVoxelization );

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

    protected void compareOutput( Vector3[] expected, Vector3[] actual ) {
        Assert.AreEqual( expected.Length, actual.Length );

        var indices = new Dictionary<int, int>( );
        for( var i = 0; i < expected.Length; ++i ) {
            for( var j = 0; j < actual.Length; ++j ) {
                if(
                    !indices.ContainsKey   ( i ) &&
                    !indices.ContainsValue ( j ) &&
                    Vector3.Distance( expected[i], actual[j] ) <= epsilon
                ) {
                    indices[i] = j;
                }
            }
        }

        Assert.AreEqual( expected.Length, indices.Count );

        foreach( var index in indices.Keys ) {
            var otherIndex = indices[index];

            Assert.AreEqual( expected[index].x, actual[otherIndex].x, epsilon );
            Assert.AreEqual( expected[index].y, actual[otherIndex].y, epsilon );
            Assert.AreEqual( expected[index].z, actual[otherIndex].z, epsilon );
        }
    }

}
