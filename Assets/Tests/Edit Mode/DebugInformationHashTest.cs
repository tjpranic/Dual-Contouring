using System;
using System.Linq;
using NUnit.Framework;

using ImplementationType = SurfaceExtractor.Implementation.Type;
using QEFSolverType      = QEFSolver.Type;

// the managed implementations hand out the corners and edges of their voxels through Distinct( ), and Voxelizer goes
// through them when debug drawing is on. Distinct( ) is only as fast as the hashes of what it tells apart are different:
// corners and edges that share a hash have to be compared one by one. An edge along an axis hashed as the XOR of the
// hashes of its corners gave every edge between the same two planes of the grid the same hash, 31 hashes for the 104,544
// edges of a grid of 32^3, and telling them apart took half a minute
public class DebugInformationHashTest : TopologyTest {

    public enum Algorithm {
        UniformManaged,
        UniformBurst,
        Adaptive,
        AdaptiveBurst,
        Manifold
    }

    private static Voxelizer burst( Voxelizer voxelizer ) {
        voxelizer.implementationType = ImplementationType.Burst;
        return voxelizer;
    }

    // 16^3 cells or leaves either way
    private Voxelizer create( Algorithm algorithm ) {
        var shapes = shapesOf( Model.ThreeBalls );

        return algorithm switch {
            Algorithm.UniformManaged => this.createUniformVoxelizer( 4, ImplementationType.Managed, QEFSolverType.SVD, shapes ),
            Algorithm.UniformBurst   => this.createUniformVoxelizer( 4, ImplementationType.Burst,   QEFSolverType.SVD, shapes ),
            Algorithm.Adaptive       => this.createVoxelizer<AdaptiveDualContouring>( 5, QEFSolverType.SVD, shapes ),
            Algorithm.AdaptiveBurst  => burst( this.createVoxelizer<AdaptiveDualContouring>( 5, QEFSolverType.SVD, shapes ) ),
            Algorithm.Manifold       => this.createVoxelizer<ManifoldDualContouring>( 5, QEFSolverType.SVD, shapes ),
            _                        => throw new Exception( "Unknown algorithm specified" )
        };
    }

    // with a well mixed hash, a set of this size has next to no two items with the same hash
    private const float distinctHashes = 0.99f;

    [Test]
    public void cornersHaveHashesOfTheirOwn( [Values] Algorithm algorithm ) {
        var corners = this.voxelize( this.create( algorithm ) ).corners.ToArray( );

        var hashes = corners.Select( ( corner ) => corner.GetHashCode( ) ).Distinct( ).Count( );

        Assert.Greater( corners.Length, 1000, "corners" );
        Assert.GreaterOrEqual( hashes, corners.Length * distinctHashes, $"{hashes} hashes for {corners.Length} corners" );
    }

    [Test]
    public void edgesHaveHashesOfTheirOwn( [Values] Algorithm algorithm ) {
        var edges = this.voxelize( this.create( algorithm ) ).edges.ToArray( );

        var hashes = edges.Select( ( edge ) => edge.GetHashCode( ) ).Distinct( ).Count( );

        Assert.Greater( edges.Length, 1000, "edges" );
        Assert.GreaterOrEqual( hashes, edges.Length * distinctHashes, $"{hashes} hashes for {edges.Length} edges" );
    }

}
