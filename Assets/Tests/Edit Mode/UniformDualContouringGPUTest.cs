using System.Linq;
using NUnit.Framework;
using UnityEngine;

using ImplementationType = SurfaceExtractor.Implementation.Type;
using QEFSolverType      = QEFSolver.Type;

public class UniformDualContouringGPUTest : TopologyTest {

    private const int   resolution = 4;     // 16^3 cells
    private const float tolerance  = 0.05f; // in cells

    // the index of a cell along each axis, which unlike its center is the same number on the CPU and the GPU
    private static Vector3Int indexOf( SurfaceExtractor.Voxel voxel ) {
        return Vector3Int.FloorToInt( ( voxel.center + ( Vector3.one * 0.5f ) ) * ( 1 << resolution ) );
    }

    // the compute shader is a port of the CPU implementation, so both should put the vertex of a cell in the same place
    [Test]
    public void verticesMatchCPU( [Values] Model model ) {
        this.assertVerticesMatchCPU( shapesOf( model ) );
    }

    // where a cell holds two pieces of surface its vertex is not the minimizer of its QEF, on either side
    [Test]
    public void verticesMatchCPUWhereACellHoldsTwoPiecesOfSurface( ) {
        this.assertVerticesMatchCPU( twoSpheres );
    }

    // where the boundary between two materials passes through a cell it has a say in where the vertex goes
    [Test]
    public void verticesMatchCPUAlongAMaterialBoundary( ) {
        this.assertVerticesMatchCPU( slabWithBand );
    }

    private void assertVerticesMatchCPU( Shape[] shapes ) {
        var voxelizer = this.createUniformVoxelizer( resolution, ImplementationType.Managed, QEFSolverType.SVD, shapes );

        var expected = this.voxelize( voxelizer ).voxels.Where( ( voxel ) => voxel.hasFeaturePoint ).ToDictionary( indexOf );

        voxelizer.implementationType = ImplementationType.GPU;

        var deviations = this.voxelize( voxelizer ).voxels.Where(
            ( voxel ) => voxel.hasFeaturePoint && expected.ContainsKey( indexOf( voxel ) )
        ).Select(
            ( voxel ) => Vector3.Distance( expected[indexOf( voxel )].vertex, voxel.vertex ) / voxel.size.x
        ).ToArray( );

        Assert.Greater( deviations.Length, 0, "no cell has a vertex on both the CPU and the GPU" );

        var apart = deviations.Count( ( deviation ) => !( deviation <= tolerance ) );

        Assert.AreEqual( 0, apart, $"{apart} of {deviations.Length} vertices are more than {tolerance} cells from where the CPU puts them, the worst by {deviations.Max( )}" );
    }

}
