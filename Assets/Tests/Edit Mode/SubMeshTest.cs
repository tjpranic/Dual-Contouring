using System;
using System.Linq;
using NUnit.Framework;
using UnityEngine;

using Combination        = DensityFunction.Combination;
using MaterialIndex      = SurfaceExtractor.MaterialIndex;
using ImplementationType = SurfaceExtractor.Implementation.Type;
using QEFSolverType      = QEFSolver.Type;

// the mesh renderer draws sub-mesh n with its n-th material, and the GPU colours the surface of material bit n with the
// n-th material. Every implementation therefore puts the surface of a material in the sub-mesh of its bit, whatever
// order the contour comes across the materials in
public class SubMeshTest : ContouringTest {

    public enum Algorithm {
        UniformManaged,
        UniformGPU,
        Adaptive,
        Manifold
    }

    // a cube of only the second material
    private static readonly Shape[] secondMaterialOnly = {
        cube( Vector3.zero, 0.3f, Combination.Union, MaterialIndex.Material1 )
    };

    // a cube of the second material on the negative side of x, which every implementation reaches first, and one of the
    // first material on the positive side. Neither has a face on a plane of the grid
    private static readonly Shape[] secondMaterialFirst = {
        cube( new Vector3( -0.25f, 0.013f, 0.017f ), 0.15f, Combination.Union, MaterialIndex.Material1 ),
        cube( new Vector3(  0.25f, 0.013f, 0.017f ), 0.15f, Combination.Union, MaterialIndex.Material0 )
    };

    // 16^3 cells or leaves either way
    private Voxelizer create( Algorithm algorithm, Shape[] shapes ) {
        return algorithm switch {
            Algorithm.UniformManaged => this.createUniformVoxelizer( 4, ImplementationType.Managed, QEFSolverType.SVD, shapes ),
            Algorithm.UniformGPU     => this.createUniformVoxelizer( 4, ImplementationType.GPU,     QEFSolverType.SVD, shapes ),
            Algorithm.Adaptive       => this.createVoxelizer<AdaptiveDualContouring>( 5, QEFSolverType.SVD, shapes ),
            Algorithm.Manifold       => this.createVoxelizer<ManifoldDualContouring>( 5, QEFSolverType.SVD, shapes ),
            _                        => throw new Exception( "Unknown algorithm specified" )
        };
    }

    [Test]
    public void surfaceOfTheSecondMaterialIsTheSecondSubMesh( [Values] Algorithm algorithm ) {
        var mesh = this.voxelize( this.create( algorithm, secondMaterialOnly ) ).mesh;

        Assert.AreEqual( 2, mesh.subMeshCount, "sub-meshes, up to the highest material on the surface" );

        Assert.AreEqual( 0, mesh.GetTriangles( 0 ).Length, "triangles of the first material, which is nowhere" );
        Assert.Greater( mesh.GetTriangles( 1 ).Length, 0, "triangles of the second material" );
    }

    [Test]
    public void subMeshesFollowTheMaterialsNotTheOrderTheyAreFoundIn( [Values] Algorithm algorithm ) {
        var mesh = this.voxelize( this.create( algorithm, secondMaterialFirst ) ).mesh;

        Assert.AreEqual( 2, mesh.subMeshCount, "sub-meshes, up to the highest material on the surface" );

        var vertices = mesh.vertices;

        float[] positionsAlongX( int subMesh ) {
            return mesh.GetTriangles( subMesh ).Select( ( index ) => vertices[index].x ).ToArray( );
        }

        var first  = positionsAlongX( 0 );
        var second = positionsAlongX( 1 );

        Assert.Greater( first.Length,  0, "triangles of the first material" );
        Assert.Greater( second.Length, 0, "triangles of the second material" );

        Assert.IsTrue( first.All  ( ( x ) => x > 0.0f ), "the first sub-mesh should be the cube of the first material, on the positive side" );
        Assert.IsTrue( second.All ( ( x ) => x < 0.0f ), "the second sub-mesh should be the cube of the second material, on the negative side" );
    }

}
