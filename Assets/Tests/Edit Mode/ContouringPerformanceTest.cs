using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using NUnit.Framework;
using Unity.PerformanceTesting;
using Unity.Profiling;
using UnityEngine;
using UnityEngine.TestTools;

using Object             = UnityEngine.Object;
using ImplementationType = SurfaceExtractor.Implementation.Type;
using QEFSolverType      = QEFSolver.Type;

// how long contouring takes, in all and stage by stage, on grids of growing size. The stages are the profiler markers
// that the implementations set around each of them. Results go to the performance test report and, as one line per
// test, to the console. The larger grids take minutes, so the tests only run when asked for
[Explicit( "Takes minutes, run it by hand" ), Category( "Performance" )]
public class ContouringPerformanceTest : TopologyTest {

    public enum Algorithm {
        Uniform,
        Adaptive,
        Manifold
    }

    // the three overlapping balls of the "Simplification" scenes, curved with creases where the balls meet
    private static readonly Shape[] model = shapesOf( Model.ThreeBalls );

    // the error threshold of the SVD scenes that simplify
    private const float errorThreshold = 2e-4f;

    // the stages of each implementation, as their profiler markers are named
    private static string[] stagesOf( Algorithm algorithm, ImplementationType implementation ) {
        var stages = ( algorithm, implementation ) switch {
            ( Algorithm.Uniform,  ImplementationType.Managed ) => new[] { "BuildGrid",   "SampleCorners", "SolveVertices", "Contour", "BuildMesh", "DebugInformation" },
            ( Algorithm.Adaptive, ImplementationType.Managed ) => new[] { "BuildOctree", "SampleCorners", "SolveVertices", "Simplify", "Contour", "BuildMesh", "DebugInformation" },
            ( Algorithm.Manifold, ImplementationType.Managed ) => new[] { "BuildOctree", "SampleCorners", "SolveVertices", "Cluster", "Validate", "MarkCollapsed", "Contour", "BuildMesh", "DebugInformation" },
            ( Algorithm.Uniform,  ImplementationType.Burst )   => new[] { "SampleCorners", "SolveVertices", "Contour", "BuildMesh" },
            ( Algorithm.Adaptive, ImplementationType.Burst )   => new[] { "SampleCorners", "SolveVertices", "Simplify", "Contour", "BuildMesh" },
            _                                                  => throw new Exception( "Unknown implementation specified" )
        };

        return stages.Select( ( stage ) => $"{algorithm}DualContouring.{implementation}.{stage}" ).ToArray( );
    }

    // the largest grid each implementation is measured on, as the exponent of its cells per axis. Above that a single
    // contour takes too long or too much memory to be worth measuring. The managed implementations need about 10 GB for
    // 128^3 cells, so 256^3 would need about 80
    private static int largestGridOf( Algorithm algorithm, ImplementationType implementation ) {
        return ( algorithm, implementation ) switch {
            ( Algorithm.Uniform,  ImplementationType.Managed ) => 7,
            ( Algorithm.Adaptive, ImplementationType.Managed ) => 7,
            ( Algorithm.Manifold, ImplementationType.Managed ) => 7,
            ( Algorithm.Uniform,  ImplementationType.Burst )   => 8,
            ( Algorithm.Adaptive, ImplementationType.Burst )   => 7,
            _                                                  => throw new Exception( "Unknown implementation specified" )
        };
    }

    private static bool exists( Algorithm algorithm, ImplementationType implementation ) {
        return implementation == ImplementationType.Managed || ( algorithm != Algorithm.Manifold && implementation == ImplementationType.Burst );
    }

    // a uniform grid has 2^resolution cells across, an octree 2^(resolution - 1) leaves
    private Voxelizer create( Algorithm algorithm, ImplementationType implementation, int cellsExponent ) {
        switch( algorithm ) {
            case Algorithm.Uniform: {
                return this.createUniformVoxelizer( cellsExponent, implementation, QEFSolverType.SVD, model );
            }
            case Algorithm.Adaptive: {
                var voxelizer = this.createVoxelizer<AdaptiveDualContouring>( cellsExponent + 1, QEFSolverType.SVD, model );

                voxelizer.implementationType = implementation;
                voxelizer.simplification     = true;
                voxelizer.errorThreshold     = errorThreshold;
                voxelizer.topologySafe       = true;

                return voxelizer;
            }
            case Algorithm.Manifold: {
                var voxelizer = this.createVoxelizer<ManifoldDualContouring>( cellsExponent + 1, QEFSolverType.SVD, model );

                voxelizer.implementationType = implementation;
                voxelizer.errorThreshold     = errorThreshold;
                voxelizer.enforceManifold    = true;

                return voxelizer;
            }
            default: {
                throw new Exception( "Unknown algorithm specified" );
            }
        }
    }

    private static double median( List<double> samples ) {
        var sorted = samples.OrderBy( ( sample ) => sample ).ToArray( );

        return sorted.Length % 2 == 1 ? sorted[sorted.Length / 2] : ( sorted[( sorted.Length / 2 ) - 1] + sorted[sorted.Length / 2] ) / 2.0;
    }

    [UnityTest, Performance]
    public IEnumerator voxelize(
        [Values] Algorithm algorithm,
        [Values( ImplementationType.Managed, ImplementationType.Burst )] ImplementationType implementation,
        [Values( 4, 5, 6, 7, 8 )] int cellsExponent
    ) {
        if( !exists( algorithm, implementation ) ) {
            Assert.Ignore( $"{algorithm} dual contouring has no {implementation} implementation yet" );
        }
        if( algorithm != Algorithm.Uniform && cellsExponent > 7 ) {
            Assert.Ignore( "an octree is at most 2^7 leaves across" );
        }
        if( cellsExponent > largestGridOf( algorithm, implementation ) ) {
            Assert.Ignore( $"grids above 2^{largestGridOf( algorithm, implementation )} cells across take too long for this implementation" );
        }

        var voxelizer = this.create( algorithm, implementation, cellsExponent );
        var volumes   = voxelizer.GetComponentsInChildren<Volume>( );

        // what is measured is contouring. The managed implementations build their debug information either way, the
        // Burst ones only when asked to, as they are in a scene with debug drawing off
        voxelizer.keepDebugInformation = false;

        // a recorder adds up the time of its marker over the current frame, and starts again from zero on the next. Every
        // contour gets a frame of its own, and its stages are what the recorders gained while it ran
        var stages    = stagesOf( algorithm, implementation );
        var recorders = stages.Select( ( stage ) => new ProfilerRecorder( stage, 16, ProfilerRecorderOptions.SumAllSamplesInFrame ) ).ToArray( );

        var samples = new Dictionary<string, List<double>>( );

        void record( string name, double milliseconds ) {
            if( !samples.ContainsKey( name ) ) {
                samples.Add( name, new( ) );
            }
            samples[name].Add( milliseconds );

            Measure.Custom( new SampleGroup( name, SampleUnit.Millisecond ), milliseconds );
        }

        // small grids are measured a few times after a warm up, large ones once. Burst code is fast enough to always be
        // measured a few times, and the warm up keeps its compilation out of the measurements
        var warmups      = cellsExponent <= 6 || implementation == ImplementationType.Burst ? 1 : 0;
        var measurements = cellsExponent <= 6 || implementation == ImplementationType.Burst ? 3 : 1;

        var vertices  = 0;
        var triangles = 0;

        try {
            foreach( var recorder in recorders ) {
                recorder.Start( );
            }

            for( var run = 0; run < warmups + measurements; ++run ) {
                GC.Collect( );

                yield return null;

                var before = recorders.Select( ( recorder ) => recorder.CurrentValue ).ToArray( );

                var contouring = Stopwatch.StartNew( );

                var mesh = voxelizer.voxelize( volumes ).visit( ( cpu ) => cpu.mesh, ( gpu ) => voxelizer.convert( gpu ).mesh );

                contouring.Stop( );

                var after = recorders.Select( ( recorder ) => recorder.CurrentValue ).ToArray( );

                vertices  = mesh.vertexCount;
                triangles = mesh.triangles.Length / 3;

                // what Voxelizer.Start does to the mesh afterwards, for the split vertices that most scenes use
                var splitting = Stopwatch.StartNew( );

                Voxelizer.splitVertices( mesh );

                splitting.Stop( );

                var finishing = Stopwatch.StartNew( );

                mesh.RecalculateBounds( );
                mesh.OptimizeReorderVertexBuffer( );

                finishing.Stop( );

                Object.DestroyImmediate( mesh );

                if( run < warmups ) {
                    continue;
                }

                record( "Voxelize",      contouring.Elapsed.TotalMilliseconds );
                record( "SplitVertices", splitting.Elapsed.TotalMilliseconds );
                record( "FinishMesh",    finishing.Elapsed.TotalMilliseconds );

                for( var stage = 0; stage < stages.Length; ++stage ) {
                    record( stages[stage].Substring( stages[stage].LastIndexOf( '.' ) + 1 ), ( after[stage] - before[stage] ) / 1e6 );
                }
            }
        }
        finally {
            foreach( var recorder in recorders ) {
                recorder.Dispose( );
            }
        }

        var cells = 1 << cellsExponent;

        UnityEngine.Debug.Log(
            $"[performance] {algorithm} {implementation} {cells}^3: {vertices} vertices, {triangles} triangles, median of {measurements} in ms: " +
            string.Join( ", ", samples.Select( ( group ) => $"{group.Key} {median( group.Value ):0.0}" ) )
        );
    }

}
