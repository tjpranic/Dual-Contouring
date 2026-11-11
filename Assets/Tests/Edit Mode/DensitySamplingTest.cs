using System;
using System.Linq;
using NUnit.Framework;
using UnityEngine;

using MaterialIndex      = SurfaceExtractor.MaterialIndex;
using ImplementationType = SurfaceExtractor.Implementation.Type;
using QEFSolverType      = QEFSolver.Type;

// every stage of contouring samples the density functions together, through SurfaceExtractor. Copies of a volume add
// samples of the copies but no more samples of each one, so each of three copies of a sphere is sampled exactly as
// often as the sphere by itself. A stage that went over the density functions once for every function would sample each
// of n copies n times as often
public class DensitySamplingTest : ContouringTest {

    public enum Algorithm {
        Uniform,
        Adaptive,
        Manifold
    }

    // a sphere that counts how often it is sampled. It sits off the grid lines, so no corner lands on the surface
    private class CountingSphere : DensityFunction {

        private const float radius = 0.3f;

        public int samples { get; private set; } = 0;

        public DensityFunction.Type        type          { get { return DensityFunction.Type.Ellipsoid; } }
        public Vector3                     origin        { get { return new Vector3( 0.013f, -0.021f, 0.017f ); } }
        public Quaternion                  rotation      { get { return Quaternion.identity; } }
        public Vector3                     extents       { get { return Vector3.one * radius; } }
        public DensityFunction.Combination combination   { get; set; } = DensityFunction.Combination.Union;
        public MaterialIndex               materialIndex { get; set; } = MaterialIndex.Material0;

        public float sample( Vector3 position ) {
            ++this.samples;

            return Vector3.Distance( position, this.origin ) - radius;
        }

    }

    // 16^3 cells or leaves either way, without volumes of its own
    private Voxelizer create( Algorithm algorithm ) {
        return algorithm switch {
            Algorithm.Uniform  => this.createUniformVoxelizer( 4, ImplementationType.Managed, QEFSolverType.SVD, Array.Empty<Shape>( ) ),
            Algorithm.Adaptive => this.createVoxelizer<AdaptiveDualContouring>( 5, QEFSolverType.SVD, Array.Empty<Shape>( ) ),
            Algorithm.Manifold => this.createVoxelizer<ManifoldDualContouring>( 5, QEFSolverType.SVD, Array.Empty<Shape>( ) ),
            _                  => throw new Exception( "Unknown algorithm specified" )
        };
    }

    // how often each of the given number of copies of the sphere is sampled while contouring them together
    private int samplesOfEach( Algorithm algorithm, int copies ) {
        var spheres = Enumerable.Range( 0, copies ).Select( ( _ ) => new CountingSphere( ) ).ToArray( );

        this.voxelize( this.create( algorithm ), spheres );

        Assert.IsTrue( spheres.All( ( sphere ) => sphere.samples == spheres[0].samples ), "copies sampled unequally" );

        return spheres[0].samples;
    }

    [Test]
    public void copiesOfAVolumeAreSampledAsOftenAsOne( [Values] Algorithm algorithm ) {
        var alone    = this.samplesOfEach( algorithm, 1 );
        var together = this.samplesOfEach( algorithm, 3 );

        Assert.Greater( alone, 0, "the sphere is never sampled" );

        Assert.AreEqual( alone, together, "samples of each of three copies of the sphere, against those of the sphere by itself" );
    }

}
