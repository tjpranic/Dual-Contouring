using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

// the search for the nearest point of a mesh that a remesh puts its vertices back on (SurfaceGrid): the nearest point
// of a triangle against answers worked out by hand and against a dense sampling of it, and the grid against a search
// over every triangle and every segment, with the sub-mesh, the facing and the lines that it keeps to
public class BurstSurfaceGridTest {

    private struct Query {

        public double3 point;
        public double  reach;

        // a search of the surface: the sub-mesh and the direction it faces. Or a search of the lines
        public bool    onLines;
        public int     subMesh;
        public double3 facing;
        public int     line;
        public int     otherLine;

    }

    private struct Answer {

        public bool    found;
        public double3 point;
        public int     item;

    }

    // every query by the grid, and by a search over every triangle or segment under the same rules
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    private struct QueryJob : IJob {

        public SurfaceGrid grid;

        [ReadOnly] public NativeArray<Query> queries;

        public NativeArray<Answer> answers;
        public NativeArray<Answer> searched;
        public NativeArray<bool>   managed;

        public void Execute( ) {
            this.managed[0] = BurstTestSupport.runningManaged( );

            for( var index = 0; index < this.queries.Length; ++index ) {
                var query  = this.queries[index];
                var answer = new Answer( );

                answer.found = query.onLines
                    ? this.grid.nearestOnLines( query.point, query.line, query.otherLine, query.reach, out answer.point, out answer.item )
                    : this.grid.nearestOnSurface( query.point, query.subMesh, query.facing, query.reach, out answer.point, out answer.item );

                this.answers[index]  = answer;
                this.searched[index] = query.onLines ? this.searchLines( query ) : this.searchSurface( query );
            }
        }

        private Answer searchSurface( in Query query ) {
            var answer = new Answer { item = -1, point = query.point };
            var best   = query.reach * query.reach;

            for( var triangle = 0; triangle < this.grid.triangleCount; ++triangle ) {
                if( !this.grid.counts( triangle, query.subMesh, query.facing ) ) {
                    continue;
                }

                var on       = SurfaceGrid.nearestOnTriangle( query.point, this.grid.corners[triangle * 3], this.grid.corners[( triangle * 3 ) + 1], this.grid.corners[( triangle * 3 ) + 2] );
                var distance = math.distancesq( query.point, on );

                if( distance < best ) {
                    best         = distance;
                    answer.point = on;
                    answer.item  = triangle;
                }
            }

            answer.found = answer.item != -1;

            return answer;
        }

        private Answer searchLines( in Query query ) {
            var answer = new Answer { item = -1, point = query.point };
            var best   = query.reach * query.reach;

            for( var segment = 0; segment < this.grid.segmentCount; ++segment ) {
                if( this.grid.lines[segment] != query.line && this.grid.lines[segment] != query.otherLine ) {
                    continue;
                }

                var on       = SurfaceGrid.nearestOnSegment( query.point, this.grid.ends[segment * 2], this.grid.ends[( segment * 2 ) + 1] );
                var distance = math.distancesq( query.point, on );

                if( distance < best ) {
                    best         = distance;
                    answer.point = on;
                    answer.item  = segment;
                }
            }

            answer.found = answer.item != -1;

            return answer;
        }

    }

    private static ( Answer[] grid, Answer[] searched ) ask( SurfaceGrid grid, Query[] queries ) {
        using var nativeQueries = new NativeArray<Query>( queries, Allocator.TempJob );
        using var answers       = new NativeArray<Answer>( queries.Length, Allocator.TempJob );
        using var searched      = new NativeArray<Answer>( queries.Length, Allocator.TempJob );
        using var managed       = new NativeArray<bool>( 1, Allocator.TempJob );

        new QueryJob { grid = grid, queries = nativeQueries, answers = answers, searched = searched, managed = managed }.Run( );

        BurstTestSupport.assertRanUnderBurst( managed );

        return ( answers.ToArray( ), searched.ToArray( ) );
    }

    // ---- the nearest point of a triangle ----

    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    private struct TriangleJob : IJob {

        public double3 a, b, c;

        [ReadOnly] public NativeArray<double3> points;

        public NativeArray<double3> nearest;
        public NativeArray<double>  sampled; // the least distance to a dense grid of points of the triangle
        public NativeArray<bool>    managed;

        public void Execute( ) {
            this.managed[0] = BurstTestSupport.runningManaged( );

            const int steps = 400;

            for( var index = 0; index < this.points.Length; ++index ) {
                this.nearest[index] = SurfaceGrid.nearestOnTriangle( this.points[index], this.a, this.b, this.c );

                var least = double.MaxValue;

                for( var i = 0; i <= steps; ++i )
                for( var j = 0; i + j <= steps; ++j ) {
                    var sample = this.a + ( ( this.b - this.a ) * ( ( double )i / steps ) ) + ( ( this.c - this.a ) * ( ( double )j / steps ) );

                    least = math.min( least, math.distance( this.points[index], sample ) );
                }

                this.sampled[index] = least;
            }
        }

    }

    [Test]
    public void theNearestPointOfATriangleIsOfTheRegionThePointIsOver( ) {
        // over the face, beyond each corner, beyond each edge, and in the plane
        var cases = new[] {
            ( new double3( 0.2, 0.2, 5.0 ),  new double3( 0.2, 0.2, 0.0 ) ),
            ( new double3( -1.0, -1.0, 3.0 ), new double3( 0.0, 0.0, 0.0 ) ),
            ( new double3( 2.0, -1.0, 1.0 ),  new double3( 1.0, 0.0, 0.0 ) ),
            ( new double3( -1.0, 3.0, -2.0 ), new double3( 0.0, 1.0, 0.0 ) ),
            ( new double3( 0.5, -3.0, 0.0 ),  new double3( 0.5, 0.0, 0.0 ) ),
            ( new double3( -2.0, 0.5, 1.0 ),  new double3( 0.0, 0.5, 0.0 ) ),
            ( new double3( 1.0, 1.0, -1.0 ),  new double3( 0.5, 0.5, 0.0 ) ),
            ( new double3( 0.25, 0.25, 0.0 ), new double3( 0.25, 0.25, 0.0 ) )
        };

        var points = cases.Select( ( item ) => item.Item1 ).ToArray( );

        // a second triangle, turned and stretched, against the dense sampling only
        var turned = new[] { new double3( 0.3, -1.2, 2.0 ), new double3( 2.5, 0.4, 1.0 ), new double3( -0.6, 1.9, -0.4 ) };

        foreach( var ( a, b, c, check ) in new[] { ( double3.zero, new double3( 1, 0, 0 ), new double3( 0, 1, 0 ), true ), ( turned[0], turned[1], turned[2], false ) } ) {
            using var nativePoints = new NativeArray<double3>( points, Allocator.TempJob );
            using var nearest      = new NativeArray<double3>( points.Length, Allocator.TempJob );
            using var sampled      = new NativeArray<double>( points.Length, Allocator.TempJob );
            using var managed      = new NativeArray<bool>( 1, Allocator.TempJob );

            new TriangleJob { a = a, b = b, c = c, points = nativePoints, nearest = nearest, sampled = sampled, managed = managed }.Run( );

            BurstTestSupport.assertRanUnderBurst( managed );

            for( var index = 0; index < points.Length; ++index ) {
                var distance = math.distance( points[index], nearest[index] );

                if( check ) {
                    Assert.LessOrEqual( math.distance( cases[index].Item2, nearest[index] ), 1e-12, $"the nearest point to {points[index]}" );
                }

                // no point of the triangle is nearer, and some point near it is about as near
                Assert.LessOrEqual( distance, sampled[index] + 1e-12, $"the nearest point to {points[index]} is no further than any sample" );
                Assert.Less( sampled[index] - distance, 0.01, $"a sample is about as near to {points[index]}" );
            }
        }
    }

    // ---- the grid ----

    // the box with a line between two materials, an octagonal prism and a cone, with their features at 40 degrees
    private static IEnumerable<( string name, HandBuiltMesh mesh )> meshes( ) {
        yield return ( "box with a line", new HandBuiltMesh( ).box( Vector3Int.zero, Vector3Int.one * 4, ( centre, axis, side ) => centre.x < 2.0f ? 1 : 0 ) );
        yield return ( "prism", new HandBuiltMesh( ).prism( 8, 2.0f, 3.0f ) );
        yield return ( "cone", new HandBuiltMesh( ).cone( 32, 1.0f, 1.0f ) );
    }

    // points near every triangle, at its middle, beyond its edges and beyond its corners, searched for with the
    // triangle's own sub-mesh and facing, and with its facing turned round
    private static Query[] surfaceQueries( SurfaceGrid grid, double reach ) {
        var queries = new List<Query>( );
        var offsets = new[] { new double3( 0.13, -0.07, 0.21 ), new double3( -0.31, 0.17, -0.05 ), new double3( 0.02, 0.4, -0.33 ) };

        for( var triangle = 0; triangle < grid.triangleCount; ++triangle ) {
            if( grid.subMeshes[triangle] == -1 ) {
                continue;
            }

            var a      = grid.corners[triangle * 3];
            var b      = grid.corners[( triangle * 3 ) + 1];
            var c      = grid.corners[( triangle * 3 ) + 2];
            var normal = grid.normals[triangle];
            var spots  = new[] { ( a + b + c ) / 3.0, ( a + b ) / 2.0, ( b + c ) / 2.0, a, c };

            for( var spot = 0; spot < spots.Length; ++spot ) {
                var point = spots[spot] + ( normal * ( 0.25 * ( spot - 1 ) ) ) + offsets[spot % offsets.Length];

                queries.Add( new Query { point = point, reach = reach, subMesh = grid.subMeshes[triangle], facing = normal } );
                queries.Add( new Query { point = point, reach = reach, subMesh = grid.subMeshes[triangle], facing = -normal } );
            }
        }
        return queries.ToArray( );
    }

    [Test]
    public void theGridFindsWhatASearchOfEveryTriangleFinds( [Values( 0.3, 1.0, 3.0 )] double cellSize ) {
        foreach( var ( name, built ) in meshes( ) ) {
            using var run  = new FeatureMeshRun( built, 40.0 );
            using var grid = SurfaceGrid.of( run.mesh, cellSize, Allocator.TempJob );

            var queries = surfaceQueries( grid, 1.5 );
            var ( answers, searched ) = ask( grid, queries );

            var found = 0;

            for( var index = 0; index < queries.Length; ++index ) {
                var what = $"{name} in cells of {cellSize}: from {queries[index].point} for sub-mesh {queries[index].subMesh} facing {queries[index].facing}";

                Assert.AreEqual( searched[index].found, answers[index].found, $"{what}: whether there is one" );

                if( !searched[index].found ) {
                    continue;
                }

                ++found;

                Assert.AreEqual( searched[index].item, answers[index].item, $"{what}: the triangle" );
                Assert.LessOrEqual( math.distance( searched[index].point, answers[index].point ), 1e-12, $"{what}: the point" );

                // and the triangle is one the search keeps to
                Assert.AreEqual( queries[index].subMesh, grid.subMeshes[answers[index].item], $"{what}: of the sub-mesh" );
                Assert.Greater( math.dot( grid.normals[answers[index].item], queries[index].facing ), 0.0, $"{what}: facing within 90 degrees" );
            }

            Debug.Log( $"{name} in cells of {cellSize}: {found} of {queries.Length} found within the reach" );

            Assert.Greater( found, queries.Length / 4, $"{name}: most searches find a point" );
        }
    }

    [Test]
    public void theGridFindsWhatASearchOfEverySegmentFinds( [Values( 0.3, 1.0, 3.0 )] double cellSize ) {
        foreach( var ( name, built ) in meshes( ) ) {
            using var run  = new FeatureMeshRun( built, 40.0 );
            using var grid = SurfaceGrid.of( run.mesh, cellSize, Allocator.TempJob );

            Assert.AreEqual( run.featureEdgeCount, grid.segmentCount, $"{name}: a segment for every feature edge" );

            var queries = new List<Query>( );

            for( var segment = 0; segment < grid.segmentCount; ++segment ) {
                var middle = ( grid.ends[segment * 2] + grid.ends[( segment * 2 ) + 1] ) / 2.0;

                // its own line, and its own with the line of the next segment
                queries.Add( new Query { onLines = true, point = middle + new double3( 0.3, -0.2, 0.1 ), reach = 2.0, line = grid.lines[segment], otherLine = grid.lines[segment] } );
                queries.Add( new Query { onLines = true, point = grid.ends[segment * 2] + new double3( -0.15, 0.25, 0.4 ), reach = 2.0, line = grid.lines[segment], otherLine = grid.lines[( segment + 1 ) % grid.segmentCount] } );
            }

            var ( answers, searched ) = ask( grid, queries.ToArray( ) );

            for( var index = 0; index < queries.Count; ++index ) {
                var what = $"{name} in cells of {cellSize}: from {queries[index].point} on lines {queries[index].line} and {queries[index].otherLine}";

                Assert.IsTrue( searched[index].found, $"{what}: a segment is within the reach" );
                Assert.AreEqual( searched[index].found, answers[index].found, $"{what}: whether there is one" );
                Assert.AreEqual( searched[index].item, answers[index].item, $"{what}: the segment" );
                Assert.LessOrEqual( math.distance( searched[index].point, answers[index].point ), 1e-12, $"{what}: the point" );

                var line = grid.lines[answers[index].item];

                Assert.IsTrue( line == queries[index].line || line == queries[index].otherLine, $"{what}: on one of the lines" );
            }
        }
    }

    [Test]
    public void aNearerTriangleOfAnotherMaterialOrFacingAwayIsPassedOver( ) {
        // two flat squares a tenth apart, the lower facing down and of sub-mesh 0, the upper facing up and of sub-mesh 1
        var built = new HandBuiltMesh( );

        // first a triangle that uses a vertex twice, which the lists leave out
        built.triangles.Add( new BurstMesh.Triangle( built.vertex( new Vector3( 0, 0, 0 ) ), built.vertex( new Vector3( 0, 0, 0 ) ), built.vertex( new Vector3( 0, 1, 0 ) ), 1 ) );

        built.quad( new Vector3( 0, 0, 0 ), new Vector3( 0, 1, 0 ), new Vector3( 1, 1, 0 ), new Vector3( 1, 0, 0 ), 0 );
        built.quad( new Vector3( 0, 0, 0.1f ), new Vector3( 1, 0, 0.1f ), new Vector3( 1, 1, 0.1f ), new Vector3( 0, 1, 0.1f ), 1 );

        using var run  = new FeatureMeshRun( built, 40.0 );
        using var grid = SurfaceGrid.of( run.mesh, 0.5, Allocator.TempJob );

        // the grid's triangles are the mesh's, in its order, the one left out among them
        Assert.AreEqual( built.triangles.Count, grid.triangleCount, "triangles" );
        Assert.AreEqual( -1, grid.subMeshes[0], "the triangle left out has no sub-mesh" );

        for( var triangle = 1; triangle < built.triangles.Count; ++triangle ) {
            var corners = new[] { built.triangles[triangle].a, built.triangles[triangle].b, built.triangles[triangle].c };

            Assert.AreEqual( built.triangles[triangle].subMeshIndex, grid.subMeshes[triangle], $"sub-mesh of triangle {triangle}" );

            for( var corner = 0; corner < 3; ++corner ) {
                Assert.AreEqual( ( double3 )( float3 )built.positions[corners[corner]], grid.corners[( triangle * 3 ) + corner], $"corner {corner} of triangle {triangle}" );
            }
        }

        var point   = new double3( 0.5, 0.5, 0.02 );
        var queries = new[] {
            new Query { point = point, reach = 1.0, subMesh = 0, facing = new double3( 0, 0, -1 ) },
            new Query { point = point, reach = 1.0, subMesh = 1, facing = new double3( 0, 0, 1 ) },
            new Query { point = point, reach = 1.0, subMesh = 0, facing = new double3( 0, 0, 1 ) },
            new Query { point = point, reach = 1.0, subMesh = 1, facing = new double3( 0, 0, -1 ) },
            new Query { point = point, reach = 0.05, subMesh = 1, facing = new double3( 0, 0, 1 ) }
        };

        var ( answers, _ ) = ask( grid, queries );

        Assert.IsTrue( answers[0].found, "the lower square, nearer" );
        Assert.AreEqual( 0.0, answers[0].point.z, 1e-6, "on the lower square" );

        Assert.IsTrue( answers[1].found, "the upper square, though the lower is nearer" );
        Assert.AreEqual( 0.1, answers[1].point.z, 1e-6, "on the upper square" );

        Assert.IsFalse( answers[2].found, "nothing of sub-mesh 0 faces up" );
        Assert.IsFalse( answers[3].found, "nothing of sub-mesh 1 faces down" );
        Assert.IsFalse( answers[4].found, "the upper square is beyond the reach" );

        Assert.IsTrue( grid.nearestOnSurface( point, 0, 1.0, out var lower, out var lowerTriangle ), "an unoriented search includes the downward-facing square" );
        Assert.AreEqual( 0.0, lower.z, 1e-6 );
        Assert.Less( grid.normals[lowerTriangle].z, 0.0 );
        Assert.IsTrue( grid.nearestOnSurface( point, 1, 1.0, out var upper, out _ ), "an unoriented search still respects material" );
        Assert.AreEqual( 0.1, upper.z, 1e-6 );
        Assert.IsFalse( grid.nearestOnSurface( point, 1, 0.05, out _, out _ ), "an unoriented search still respects reach" );
        Assert.IsTrue( grid.nearestOnSurface( point, SurfaceGrid.anySubMesh, 1.0, out var any, out _ ) );
        Assert.AreEqual( lower, any, "without a material restriction, the nearer square wins regardless of facing" );
    }

}
