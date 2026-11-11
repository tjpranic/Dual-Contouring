using System.Linq;
using NUnit.Framework;
using Unity.Collections;
using Unity.Jobs;

using MaterialIndex = SurfaceExtractor.MaterialIndex;

// what the Burst implementations work out for a whole grid in one go
public class BurstGridTest {

    // the job goes through the grid a row of cells at a time and carries the corners two cells share from one to the
    // next. Whatever way it gets there, the solid corners of a cell are the bits of its corners that are not empty
    [Test]
    public void solidCornersOfEveryCellAreItsCornersThatAreNotEmpty( [Values( 1, 2, 4, 8 )] int resolution, [Values( 0.1f, 0.5f, 0.9f )] float solid ) {
        var grid   = new BurstGrid( resolution );
        var random = new System.Random( resolution );

        var corners = Enumerable.Range( 0, grid.cornerCount ).Select(
            ( _ ) => random.NextDouble( ) < solid ? BurstTestSupport.materials[1 + random.Next( BurstTestSupport.materials.Length - 1 )] : MaterialIndex.Void
        ).ToArray( );

        Assert.AreEqual( MaterialIndex.Void, BurstTestSupport.materials[0], "the first of the materials should be the empty one" );

        using var materials    = new NativeArray<MaterialIndex>( corners, Allocator.TempJob );
        using var solidCorners = new NativeArray<byte>( grid.cellCount, Allocator.TempJob );

        new BurstSolidCornersJob { materials = materials, grid = grid, solidCorners = solidCorners }.Schedule( grid.rowCount, 2 ).Complete( );

        for( var index = 0; index < grid.cellCount; ++index ) {
            var cell     = grid.cellOf( index );
            var expected = Enumerable.Range( 0, 8 ).Sum( ( corner ) => corners[grid.cornerIndex( grid.cornerOfCell( cell, corner ) )] != MaterialIndex.Void ? 1 << corner : 0 );

            Assert.AreEqual( expected, solidCorners[index], $"cell {cell}" );
        }
    }

}
