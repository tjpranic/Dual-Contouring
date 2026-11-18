using NUnit.Framework;
using UnityEngine;

using ImplementationType = SurfaceExtractor.Implementation.Type;
using QEFSolverType      = QEFSolver.Type;

// the normal deviation reaches the probabilistic solver in every implementation on the CPU, and nothing else: with the
// probabilistic solver the contour moves when the deviation changes, and with either other solver it stays the same to
// the bit. Adaptive and Manifold Dual Contouring simplify, so that the QEFs of their larger nodes are solved with it too
public class NormalDeviationTest : TopologyTest {

    public enum Variant {
        UniformManaged,
        UniformBurst,
        AdaptiveManaged,
        AdaptiveBurst,
        ManifoldManaged,
        ManifoldBurst,
        OccupancyBurst
    }

    // a cube turned about a diagonal, whose edges and corners lie anywhere in their cells, where the deviation moves the
    // vertices
    private static readonly Shape[] turnedCube = { cube( new Vector3( 0.013f, -0.021f, 0.017f ), 0.25f ).rotated( Quaternion.AngleAxis( 45.0f, Vector3.one ) ) };

    // the error threshold of the "Simplification" scenes
    private const float errorThreshold = 2e-4f;

    private Voxelizer create( Variant variant, QEFSolverType solver ) {
        switch( variant ) {
            case Variant.UniformManaged:
            case Variant.UniformBurst: {
                return this.createUniformVoxelizer( 4, variant == Variant.UniformManaged ? ImplementationType.Managed : ImplementationType.Burst, solver, turnedCube );
            }
            case Variant.AdaptiveManaged:
            case Variant.AdaptiveBurst: {
                var voxelizer = this.createVoxelizer<AdaptiveDualContouring>( 5, solver, turnedCube );

                voxelizer.implementationType = variant == Variant.AdaptiveManaged ? ImplementationType.Managed : ImplementationType.Burst;
                voxelizer.simplification     = true;
                voxelizer.errorThreshold     = errorThreshold;

                return voxelizer;
            }
            case Variant.ManifoldManaged:
            case Variant.ManifoldBurst: {
                var voxelizer = this.createVoxelizer<ManifoldDualContouring>( 5, solver, turnedCube );

                voxelizer.implementationType = variant == Variant.ManifoldManaged ? ImplementationType.Managed : ImplementationType.Burst;
                voxelizer.errorThreshold     = errorThreshold;

                return voxelizer;
            }
            default: {
                var voxelizer = this.createVoxelizer<OccupancyDualContouring>( 4, solver, turnedCube );

                voxelizer.implementationType = ImplementationType.Burst;

                return voxelizer;
            }
        }
    }

    private Vector3[] contourAt( Variant variant, QEFSolverType solver, float deviation ) {
        var voxelizer = this.create( variant, solver );

        voxelizer.normalDeviation = deviation;

        return this.voxelize( voxelizer ).mesh.vertices;
    }

    public enum Algorithm {
        Uniform,
        Adaptive,
        Manifold
    }

    // the managed and Burst contours are the same at a deviation other than the default, so that every place either of
    // them makes a QEF is given the setting. One made with the default instead changes nothing at the default, and when
    // an implementation makes QEFs in two places the other one still moves the contour
    [Test]
    public void managedAndBurstAgreeAwayFromTheDefault( [Values] Algorithm algorithm ) {
        var variant = algorithm switch {
            Algorithm.Uniform  => Variant.UniformManaged,
            Algorithm.Adaptive => Variant.AdaptiveManaged,
            _                  => Variant.ManifoldManaged
        };

        var voxelizer = this.create( variant, QEFSolverType.Probabilistic );

        voxelizer.normalDeviation = 0.5f;

        var managed = this.voxelize( voxelizer );

        voxelizer.implementationType = ImplementationType.Burst;

        var burst = this.voxelize( voxelizer );

        // 16^3 cells or leaves
        BurstParity.assertSameContour( managed, burst, 1.0f / 16.0f );
    }

    [Test]
    public void deviationChangesOnlyTheProbabilisticContour( [Values] Variant variant, [Values] QEFSolverType solver ) {
        var low  = this.contourAt( variant, solver, 0.05f );
        var high = this.contourAt( variant, solver, 0.5f );

        Assert.Greater( low.Length, 0, "the mesh is empty" );

        if( solver == QEFSolverType.Probabilistic ) {
            CollectionAssert.AreNotEqual( low, high, "the contour does not change with the deviation" );
        }
        else {
            CollectionAssert.AreEqual( low, high, "the contour changes with the deviation" );
        }
    }

}
