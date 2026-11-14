using System;
using System.Collections.Generic;
using UnityEngine;

using Combination        = DensityFunction.Combination;
using MaterialIndex      = SurfaceExtractor.MaterialIndex;
using ImplementationType = SurfaceExtractor.Implementation.Type;
using VertexNormals      = SurfaceExtractor.VertexNormals;
using QEFSolverType      = QEFSolver.Type;

// the contours of every algorithm and implementation, on surfaces that between them have everything a contour can
// hold, for the tests of the passes that run on finished meshes: MeshSimplificationTest and BurstCreasesTest
public abstract class ContourVariantsTest : TopologyTest {

    // an algorithm with one of its implementations, and with or without the simplification of its own
    public enum Contour {
        UniformManaged,
        UniformBurst,
        UniformGPU,
        AdaptiveManaged,
        AdaptiveBurst,
        AdaptiveSimplified,
        ManifoldManaged,
        ManifoldBurst,
        ManifoldClustered,
        OccupancyBurst,
        OccupancyGPU
    }

    public enum Surface {
        Sphere,
        ThreeBalls,
        CubesAndSphere,
        SphereCutaway,
        TwoSpheres,
        SlabWithBand,
        TurnedShapes,
        NewShapes,
        HollowsAcrossFaces,
        BallAcrossBounds,
        PlateAcrossBounds
    }

    // surfaces that leave the grid, which spans -0.5 to 0.5: a ball through one side, and a plate with a band of a
    // second material that leaves with it, so that the lines between the materials run into the open edges
    protected static readonly Shape[] ballAcrossBounds = { sphere( new Vector3( 0.3f, 0.07f, -0.04f ), 0.35f ) };

    protected static readonly Shape[] plateAcrossBounds = {
        cuboid( Vector3.zero,                     new Vector3( 0.7f,  0.1f, 0.7f ) ),
        cuboid( new Vector3( 0.02f, 0.0f, 0.0f ), new Vector3( 0.15f, 0.1f, 0.7f ), Combination.Union, MaterialIndex.Material1 )
    };

    protected static Shape[] shapesOf( Surface surface ) {
        return surface switch {
            Surface.Sphere             => shapesOf( Model.Sphere ),
            Surface.ThreeBalls         => shapesOf( Model.ThreeBalls ),
            Surface.CubesAndSphere     => shapesOf( Model.CubesAndSphere ),
            Surface.SphereCutaway      => shapesOf( Model.SphereCutaway ),
            Surface.TwoSpheres         => twoSpheres,
            Surface.SlabWithBand       => slabWithBand,
            Surface.TurnedShapes       => turnedShapes,
            Surface.NewShapes          => newShapes,
            Surface.HollowsAcrossFaces => hollowsAcrossFaces,
            Surface.BallAcrossBounds   => ballAcrossBounds,
            Surface.PlateAcrossBounds  => plateAcrossBounds,
            _                          => throw new Exception( "Unknown surface specified" )
        };
    }

    protected static bool isManaged( Contour contour ) {
        return contour == Contour.UniformManaged || contour == Contour.AdaptiveManaged || contour == Contour.ManifoldManaged;
    }

    // the error threshold of the scenes whose octrees simplify
    protected const float errorThreshold = 2e-4f;

    // a voxelizer of 2^cellsExponent cells or leaves across
    protected Voxelizer voxelizerOf( Contour contour, IEnumerable<Shape> shapes, int cellsExponent ) {
        var implementation = contour switch {
            Contour.UniformManaged or Contour.AdaptiveManaged or Contour.ManifoldManaged => ImplementationType.Managed,
            Contour.UniformGPU or Contour.OccupancyGPU                                   => ImplementationType.GPU,
            _                                                                            => ImplementationType.Burst
        };

        Voxelizer voxelizer;

        switch( contour ) {
            case Contour.UniformManaged:
            case Contour.UniformBurst:
            case Contour.UniformGPU: {
                voxelizer = this.createUniformVoxelizer( cellsExponent, implementation, QEFSolverType.SVD, shapes );
                break;
            }
            case Contour.AdaptiveManaged:
            case Contour.AdaptiveBurst:
            case Contour.AdaptiveSimplified: {
                var adaptive = this.createVoxelizer<AdaptiveDualContouring>( cellsExponent + 1, QEFSolverType.SVD, shapes );

                adaptive.simplification = contour == Contour.AdaptiveSimplified;
                adaptive.errorThreshold = errorThreshold;
                adaptive.topologySafe   = true;

                voxelizer = adaptive;
                break;
            }
            case Contour.ManifoldManaged:
            case Contour.ManifoldBurst:
            case Contour.ManifoldClustered: {
                var manifold = this.createVoxelizer<ManifoldDualContouring>( cellsExponent + 1, QEFSolverType.SVD, shapes );

                manifold.errorThreshold  = contour == Contour.ManifoldClustered ? errorThreshold : 0.0f;
                manifold.enforceManifold = true;

                voxelizer = manifold;
                break;
            }
            default: {
                voxelizer = this.createVoxelizer<OccupancyDualContouring>( cellsExponent, QEFSolverType.SVD, shapes );
                break;
            }
        }

        voxelizer.implementationType   = implementation;
        voxelizer.vertexNormals        = VertexNormals.Creased;
        voxelizer.keepDebugInformation = false;

        return voxelizer;
    }

}
