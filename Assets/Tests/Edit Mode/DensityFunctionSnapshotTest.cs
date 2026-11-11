using System.Linq;
using NUnit.Framework;
using UnityEngine;

using Combination   = DensityFunction.Combination;
using MaterialIndex = SurfaceExtractor.MaterialIndex;
using QEFSolverType = QEFSolver.Type;

// the managed implementations contour snapshots of the volumes, which work out their origin, rotation and extents once
// instead of on every sample. A snapshot has to sample exactly like its volume, or the contour would change
public class DensityFunctionSnapshotTest : ContouringTest {

    // each shape unrotated and rotated, so that both ways of sampling are compared
    private static readonly Shape[] shapes = {
        cuboid    ( new Vector3(  0.1f,  -0.2f, 0.05f ), new Vector3( 0.3f, 0.2f,  0.25f ), Combination.Union,        MaterialIndex.Material0 ),
        ellipsoid ( new Vector3( -0.15f,  0.1f, 0.2f  ), new Vector3( 0.2f, 0.35f, 0.15f ), Combination.Subtraction,  MaterialIndex.Material1 ),
        sphere    ( new Vector3(  0.05f,  0.0f, -0.1f ), 0.4f,                              Combination.Intersection, MaterialIndex.Material2 ),
        cuboid    ( new Vector3(  0.1f,  -0.2f, 0.05f ), new Vector3( 0.3f, 0.2f,  0.25f ), Combination.Union,        MaterialIndex.Material0 ).rotated( Quaternion.Euler( 30.0f, -60.0f, 75.0f ) ),
        ellipsoid ( new Vector3( -0.15f,  0.1f, 0.2f  ), new Vector3( 0.2f, 0.35f, 0.15f ), Combination.Subtraction,  MaterialIndex.Material1 ).rotated( Quaternion.AngleAxis( 45.0f, Vector3.one ) )
    };

    [Test]
    public void snapshotsSampleExactlyLikeTheirVolumes( ) {
        var voxelizer = this.createVoxelizer<AdaptiveDualContouring>( 2, QEFSolverType.SVD, shapes );

        // a volume's origin and extents are relative to the scale of its voxelizer
        voxelizer.transform.localScale = new Vector3( 5.0f, 3.0f, 2.0f );

        var volumes   = voxelizer.GetComponentsInChildren<Volume>( );
        var snapshots = DensityFunction.snapshot( volumes );

        Assert.AreEqual( volumes.Length, snapshots.Count );

        var random = new System.Random( 1 );

        float coordinate( ) {
            return ( float )( ( random.NextDouble( ) * 2.0 ) - 1.0 );
        }

        // points all over the grid and a little past it, and the origin of every volume, where an ellipsoid takes care
        // not to divide by zero
        var points = Enumerable.Range( 0, 2000 ).Select(
            ( _ ) => new Vector3( coordinate( ), coordinate( ), coordinate( ) )
        ).Concat(
            volumes.Select( ( volume ) => volume.origin )
        ).ToArray( );

        for( var index = 0; index < volumes.Length; ++index ) {
            var volume   = volumes[index];
            var snapshot = snapshots[index];

            Assert.IsInstanceOf<DensityFunction.Snapshot>( snapshot );

            Assert.AreEqual( volume.type,          snapshot.type,          "type" );
            Assert.AreEqual( volume.origin,        snapshot.origin,        "origin" );
            Assert.AreEqual( volume.rotation,      snapshot.rotation,      "rotation" );
            Assert.AreEqual( volume.extents,       snapshot.extents,       "extents" );
            Assert.AreEqual( volume.combination,   snapshot.combination,   "combination" );
            Assert.AreEqual( volume.materialIndex, snapshot.materialIndex, "material" );

            foreach( var point in points ) {
                Assert.AreEqual( volume.sample( point ), snapshot.sample( point ), $"density of {volume.type} at {point}" );
            }
        }
    }

    // a density function that is not a volume does not depend on any transform
    [Test]
    public void otherDensityFunctionsAreKeptAsTheyAre( ) {
        var voxelizer = this.createVoxelizer<AdaptiveDualContouring>( 2, QEFSolverType.SVD, shapes );

        var snapshot = DensityFunction.snapshot( voxelizer.GetComponentsInChildren<Volume>( ) )[0];

        Assert.AreSame( snapshot, DensityFunction.snapshot( new[] { snapshot } )[0] );
    }

}
