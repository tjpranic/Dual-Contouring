using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

using VoxelType     = SurfaceExtractor.Voxel.Type;
using QEFSolverType = QEFSolver.Type;

// BurstQEF against SVDQEF and SimpleQEF. The SVD solver is the same float operations in the same order, the simple solver
// keeps the sums of AtA, AtB and BtB instead of the planes, the same math with different rounding. Either way the results
// differ from the managed ones in the last bits only: Mono keeps the intermediate results of an expression at a higher
// precision than Burst. Mass points are sums and a division, each of them a single operation, and agree to the bit
public class BurstQEFTest {

    private const int   groups              = 300;   // of three QEFs each, which are also combined into one
    private const int   minimizerIterations = 6;
    private const float cellSize            = 1.0f / 16.0f;

    // relative to the size of the value. The vertex of the SVD solver goes through the eigenvectors of AtA and the
    // inverse of its eigenvalues, which can be as small as the truncation at 0.1, and that magnifies the last bits
    private const float vertexTolerance = 1e-5f;
    private const float errorTolerance  = 1e-5f;

    // the hermite samples of a QEF, the range of them in the sample arrays, and the cell they are in
    private struct Case {

        public int     start;
        public int     count;
        public Vector3 center;

    }

    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    private struct QEFJob : IJob {

        public QEFSolverType type;

        [ReadOnly] public NativeArray<float3> intersections;
        [ReadOnly] public NativeArray<float3> normals;
        [ReadOnly] public NativeArray<int>    starts;
        [ReadOnly] public NativeArray<int>    counts;
        [ReadOnly] public NativeArray<float3> centers;

        // for every case, then for every group combined
        public NativeArray<float3> vertices;
        public NativeArray<float>  errors;
        public NativeArray<float3> massPoints;
        public NativeArray<float>  massPointErrors;
        public NativeArray<int>    intersectionCounts;
        public NativeArray<bool>   managed;

        private BurstQEF build( int index ) {
            var qef = BurstQEF.create( this.type, minimizerIterations );

            for( var sample = this.starts[index]; sample < this.starts[index] + this.counts[index]; ++sample ) {
                qef.add( this.intersections[sample], this.normals[sample] );
            }

            return qef;
        }

        public void Execute( ) {
            this.managed[0] = BurstTestSupport.runningManaged( );

            var cases = this.starts.Length;

            for( var group = 0; group < cases / 3; ++group ) {
                // the first two QEFs of the group are solved, which gives them a feature dimension, the third is not
                var qefs = new NativeArray<BurstQEF>( 3, Allocator.Temp );

                for( var member = 0; member < 3; ++member ) {
                    var index = ( group * 3 ) + member;
                    var qef   = this.build( index );

                    var solved = qef;
                    solved.solve( this.centers[index], out var vertex, out var error );
                    solved.solveMassPoint( out var massPoint, out var massPointError );

                    this.vertices[index]           = vertex;
                    this.errors[index]             = error;
                    this.massPoints[index]         = massPoint;
                    this.massPointErrors[index]    = massPointError;
                    this.intersectionCounts[index] = qef.intersectionCount;

                    qefs[member] = member < 2 ? solved : qef;
                }

                var combined = BurstQEF.create( this.type, minimizerIterations );
                for( var member = 0; member < 3; ++member ) {
                    combined.combine( qefs[member] );
                }

                var combinedIndex = cases + group;

                combined.solve( this.centers[group * 3], out var combinedVertex, out var combinedError );
                combined.solveMassPoint( out var combinedMassPoint, out var combinedMassPointError );

                this.vertices[combinedIndex]           = combinedVertex;
                this.errors[combinedIndex]             = combinedError;
                this.massPoints[combinedIndex]         = combinedMassPoint;
                this.massPointErrors[combinedIndex]    = combinedMassPointError;
                this.intersectionCounts[combinedIndex] = combined.intersectionCount;

                qefs.Dispose( );
            }
        }

    }

    // hermite samples in a cell of a grid of 16^3: on a plane, an edge or a corner with a little noise in the normals,
    // or with normals in every direction
    private static ( List<Vector3> intersections, List<Vector3> normals, Case[] cases ) generate( System.Random random ) {
        var intersections = new List<Vector3>( );
        var normals       = new List<Vector3>( );
        var cases         = new Case[groups * 3];

        for( var index = 0; index < cases.Length; ++index ) {
            var center = BurstTestSupport.range( random, -Vector3.one * 0.4f, Vector3.one * 0.4f );
            var count  = 1 + random.Next( 12 );
            var kind   = random.Next( 4 );

            var features = new[] { BurstTestSupport.direction( random ), BurstTestSupport.direction( random ), BurstTestSupport.direction( random ) };

            cases[index] = new Case { start = intersections.Count, count = count, center = center };

            for( var sample = 0; sample < count; ++sample ) {
                intersections.Add( center + BurstTestSupport.range( random, -Vector3.one * cellSize / 2, Vector3.one * cellSize / 2 ) );

                var normal = kind == 0 ? BurstTestSupport.direction( random ) : features[sample % kind] + ( BurstTestSupport.direction( random ) * 1e-3f );

                normals.Add( normal.normalized );
            }
        }

        return ( intersections, normals, cases );
    }

    private static QEFSolver createManaged( QEFSolverType type ) {
        return type == QEFSolverType.Simple ? new SimpleQEF( minimizerIterations ) : new SVDQEF( minimizerIterations );
    }

    [Test]
    public void qefsSolveAsTheManagedOnesDo( [Values] QEFSolverType type ) {
        var ( intersections, normals, cases ) = generate( new System.Random( 3 ) );

        var outputs = cases.Length + groups;

        using var nativeIntersections = new NativeArray<float3>( intersections.Select( ( v ) => ( float3 )v ).ToArray( ), Allocator.TempJob );
        using var nativeNormals       = new NativeArray<float3>( normals.Select( ( v ) => ( float3 )v ).ToArray( ), Allocator.TempJob );
        using var starts              = new NativeArray<int>( cases.Select( ( c ) => c.start ).ToArray( ), Allocator.TempJob );
        using var counts              = new NativeArray<int>( cases.Select( ( c ) => c.count ).ToArray( ), Allocator.TempJob );
        using var centers             = new NativeArray<float3>( cases.Select( ( c ) => ( float3 )c.center ).ToArray( ), Allocator.TempJob );
        using var vertices            = new NativeArray<float3>( outputs, Allocator.TempJob );
        using var errors              = new NativeArray<float>( outputs, Allocator.TempJob );
        using var massPoints          = new NativeArray<float3>( outputs, Allocator.TempJob );
        using var massPointErrors     = new NativeArray<float>( outputs, Allocator.TempJob );
        using var intersectionCounts  = new NativeArray<int>( outputs, Allocator.TempJob );
        using var managed             = new NativeArray<bool>( 1, Allocator.TempJob );

        new QEFJob {
            type               = type,
            intersections      = nativeIntersections,
            normals            = nativeNormals,
            starts             = starts,
            counts             = counts,
            centers            = centers,
            vertices           = vertices,
            errors             = errors,
            massPoints         = massPoints,
            massPointErrors    = massPointErrors,
            intersectionCounts = intersectionCounts,
            managed            = managed
        }.Run( );

        BurstTestSupport.assertRanUnderBurst( managed );

        // the same, with the managed solvers
        var expectedVertices        = new Vector3[outputs];
        var expectedErrors          = new float[outputs];
        var expectedMassPoints      = new Vector3[outputs];
        var expectedMassPointErrors = new float[outputs];
        var expectedCounts          = new int[outputs];

        QEFSolver build( int index ) {
            var qef = createManaged( type );
            for( var sample = cases[index].start; sample < cases[index].start + cases[index].count; ++sample ) {
                qef.add( intersections[sample], normals[sample] );
            }
            return qef;
        }

        SurfaceExtractor.Voxel cellOf( int index ) {
            return new AdaptiveDualContouring.Voxel( VoxelType.Leaf, 0, cases[index].center, Vector3.one * cellSize );
        }

        for( var group = 0; group < groups; ++group ) {
            var members = new QEFSolver[3];

            for( var member = 0; member < 3; ++member ) {
                var index = ( group * 3 ) + member;
                var qef   = build( index );

                ( expectedVertices[index], expectedErrors[index] )         = qef.solve( cellOf( index ) );
                ( expectedMassPoints[index], expectedMassPointErrors[index] ) = qef.solveMassPoint( );
                expectedCounts[index] = qef.intersectionCount;

                members[member] = member < 2 ? qef : build( index );
            }

            var combined = createManaged( type );
            foreach( var member in members ) {
                combined.combine( member );
            }

            var combinedIndex = cases.Length + group;

            ( expectedVertices[combinedIndex], expectedErrors[combinedIndex] )         = combined.solve( cellOf( group * 3 ) );
            ( expectedMassPoints[combinedIndex], expectedMassPointErrors[combinedIndex] ) = combined.solveMassPoint( );
            expectedCounts[combinedIndex] = combined.intersectionCount;
        }

        CollectionAssert.AreEqual( expectedCounts, intersectionCounts.ToArray( ), "intersection counts" );

        var actualVertices   = vertices.ToArray( ).Select( ( v ) => ( Vector3 )v ).ToArray( );
        var actualMassPoints = massPoints.ToArray( ).Select( ( v ) => ( Vector3 )v ).ToArray( );

        BurstTestSupport.assertSame  ( "mass points",       expectedMassPoints,      actualMassPoints );
        BurstTestSupport.assertClose ( "vertices",          expectedVertices,        actualVertices,             vertexTolerance );
        BurstTestSupport.assertClose ( "errors",            expectedErrors,          errors.ToArray( ),          errorTolerance );
        BurstTestSupport.assertClose ( "mass point errors", expectedMassPointErrors, massPointErrors.ToArray( ), errorTolerance );
    }

}
