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

    // the compute shader is a port of the CPU implementation, so both should put the vertex of a cell in the same place,
    // with either of the solvers it has
    [Test]
    public void verticesMatchCPU( [Values] Model model, [Values( QEFSolverType.SVD, QEFSolverType.Probabilistic )] QEFSolverType solver ) {
        this.assertVerticesMatchCPU( shapesOf( model ), solver );
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

    [Test]
    public void verticesMatchCPUOnTurnedShapes( [Values( QEFSolverType.SVD, QEFSolverType.Probabilistic )] QEFSolverType solver ) {
        this.assertVerticesMatchCPU( turnedShapes, solver );
    }

    // the compute shader has no simple solver, and solves with SVD in its place. The shader writes the vertices in the order its
    // threads come to them, which differs from one contour to the next, so they are compared in order of position
    [Test]
    public void simpleSolverContoursAsSVD( ) {
        var voxelizer = this.createUniformVoxelizer( resolution, ImplementationType.GPU, QEFSolverType.SVD, turnedShapes );

        var svd = inOrder( this.voxelize( voxelizer ).mesh.vertices );

        voxelizer.qefSolverType = QEFSolverType.Simple;

        var simple = inOrder( this.voxelize( voxelizer ).mesh.vertices );

        Assert.Greater( svd.Length, 0, "the mesh is empty" );
        CollectionAssert.AreEqual( svd, simple );
    }

    [Test]
    public void verticesMatchCPUOnTheNewShapes( ) {
        this.assertVerticesMatchCPU( newShapes );
    }

    // the compute shader reads density functions with the layout of DensityFunction.Data, its buffer steps through them
    // by the size Marshal gives, and SetData copies their bytes as they are in memory. All three have to agree, which a
    // bool, one byte in memory and four to Marshal, would break
    [Test]
    public void densityFunctionsAreUploadedWithTheShadersLayout( ) {
        // uint type, combination and material index, float3 origin, extents and three axes
        const int shaderSize = ( 3 * 4 ) + ( 5 * 12 );

        Assert.AreEqual( shaderSize, System.Runtime.InteropServices.Marshal.SizeOf( typeof( DensityFunction.Data ) ), "stride" );
        Assert.AreEqual( shaderSize, Unity.Collections.LowLevel.Unsafe.UnsafeUtility.SizeOf<DensityFunction.Data>( ), "bytes copied" );
    }

    private void assertVerticesMatchCPU( Shape[] shapes, QEFSolverType solver = QEFSolverType.SVD ) {
        var voxelizer = this.createUniformVoxelizer( resolution, ImplementationType.Managed, solver, shapes );

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
