using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

using Faces    = BurstFieldAligned.Faces;
using Graph    = BurstFieldAligned.Graph;
using Sharp    = BurstMeshSimplifier.Sharp;
using Symmetry = BurstFields.Symmetry;

// Field Aligned's mesh (BurstFieldAligned; Documents/Plans/Remeshing.md, step 5): the parts of the extraction on graphs
// built by hand, each against what it is to do, and the whole on meshes whose remesh is known in closed form: a flat
// square and a flat triangle give their lattices, a box along the axes its rows of quads, a band of a second material
// its lines. Every job runs compiled by Burst
public class BurstFieldAlignedTest {

    private static readonly double featureCosine = math.cos( math.radians( 40.0 ) );

    // ---- the parts ----

    [Test]
    public void aStepIsToANeighbourOfTheLattice( ) {
        foreach( var steps in new[] { new int2( 1, 0 ), new int2( 0, 1 ), new int2( -1, 0 ), new int2( 0, -1 ) } ) {
            Assert.IsTrue( BurstFieldAligned.isStep( steps, Symmetry.Quads ),     $"{steps} for squares" );
            Assert.IsTrue( BurstFieldAligned.isStep( steps, Symmetry.Triangles ), $"{steps} for triangles" );
        }

        // the third side of a triangle of the lattice, which for squares is a diagonal
        foreach( var steps in new[] { new int2( 1, -1 ), new int2( -1, 1 ) } ) {
            Assert.IsFalse( BurstFieldAligned.isStep( steps, Symmetry.Quads ),    $"{steps} for squares" );
            Assert.IsTrue( BurstFieldAligned.isStep( steps, Symmetry.Triangles ), $"{steps} for triangles" );
        }

        // a diagonal, the height of two triangles away, and further, and none
        foreach( var steps in new[] { new int2( 1, 1 ), new int2( -1, -1 ), new int2( 2, 0 ), new int2( 2, -1 ), int2.zero } ) {
            Assert.IsFalse( BurstFieldAligned.isStep( steps, Symmetry.Quads ),     $"{steps} for squares" );
            Assert.IsFalse( BurstFieldAligned.isStep( steps, Symmetry.Triangles ), $"{steps} for triangles" );
        }
    }

    // the fan a polygon is made of, from each corner in turn, and how flat its triangles lie together
    private static double flatnessOf( double3[] corners, int from ) {
        var least = double.PositiveInfinity;
        var count = corners.Length;

        for( var step = 1; step + 2 < count; ++step ) {
            var a = math.normalize( math.cross( corners[( from + step ) % count] - corners[from], corners[( from + step + 1 ) % count] - corners[from] ) );
            var b = math.normalize( math.cross( corners[( from + step + 1 ) % count] - corners[from], corners[( from + step + 2 ) % count] - corners[from] ) );

            least = math.min( least, math.dot( a, b ) );
        }

        return least;
    }

    private static int fanOf( double3[] corners ) {
        using var positions = new NativeArray<double3>( corners, Allocator.TempJob );
        using var indices   = new NativeArray<int>( Enumerable.Range( 0, corners.Length ).ToArray( ), Allocator.TempJob );

        return BurstFieldAligned.fanOf( positions, indices, 0, corners.Length );
    }

    [Test]
    public void aQuadIsSplitAlongTheDiagonalThatLeavesItFlatter( ) {
        // a square with one corner raised, and then another: the diagonal through the raised corner folds it less
        foreach( var raised in new[] { 0, 1, 2, 3 } ) {
            var corners = new[] { new double3( 0, 0, 0 ), new double3( 1, 0, 0 ), new double3( 1, 1, 0 ), new double3( 0, 1, 0 ) };

            corners[raised].z = 0.4;

            var from = fanOf( corners );

            Assert.Greater( flatnessOf( corners, from ), flatnessOf( corners, ( from + 1 ) % 4 ) + 1e-9, $"corner {raised} raised: the fan from {from} is the flatter" );
        }
    }

    [Test]
    public void aPentagonIsAFanFromTheCornerThatLeavesItFlattest( ) {
        // a dart: from every corner but the one it bends in at, a triangle of the fan folds over
        var dart = new[] { new double3( 0, 0, 0 ), new double3( 2, 0, 0 ), new double3( 2, 2, 0 ), new double3( 1, 0.5, 0 ), new double3( 0, 2, 0 ) };

        Assert.AreEqual( 3, fanOf( dart ), "the fan from the corner it bends in at" );

        // a regular pentagon lies flat from every corner, and the first is taken
        var regular = Enumerable.Range( 0, 5 ).Select( ( k ) => new double3( math.cos( 2.0 * math.PI * k / 5.0 ), math.sin( 2.0 * math.PI * k / 5.0 ), 0.0 ) ).ToArray( );

        Assert.AreEqual( 0, fanOf( regular ), "a regular pentagon" );

        // a fan with a triangle without area is the worst
        var straight = new[] { new double3( 0, 0, 0 ), new double3( 1, 0, 0 ), new double3( 2, 0, 0 ), new double3( 2, 1, 0 ), new double3( 0, 1, 0 ) };

        Assert.AreNotEqual( 0, fanOf( straight ),  "not from a corner whose fan has a triangle along the straight side" );
        Assert.AreNotEqual( 2, fanOf( straight ),  "not from the other end of the straight side" );
    }

    [Test]
    public void theFlagsOfAFanAreTheEdgesOfItsPolygon( ) {
        Assert.AreEqual( BurstRemeshing.allEdges, BurstFieldAligned.flagsOf( 1, 3 ), "a triangle" );

        Assert.AreEqual( 0b011, BurstFieldAligned.flagsOf( 1, 4 ), "a quad's first triangle" );
        Assert.AreEqual( 0b110, BurstFieldAligned.flagsOf( 2, 4 ), "a quad's second triangle" );

        Assert.AreEqual( 0b011, BurstFieldAligned.flagsOf( 1, 5 ), "a pentagon's first triangle" );
        Assert.AreEqual( 0b010, BurstFieldAligned.flagsOf( 2, 5 ), "a pentagon's middle triangle" );
        Assert.AreEqual( 0b110, BurstFieldAligned.flagsOf( 3, 5 ), "a pentagon's last triangle" );
    }

    // the polygons the flags of triangles make: the triangles joined across edges that both leave out, and each face's
    // edges, chained round it in the triangles' turn
    private static List<int[]> polygonsOf( IList<BurstMesh.Triangle> triangles, IList<byte> flags ) {
        var parents = Enumerable.Range( 0, triangles.Count ).ToArray( );

        int find( int item ) {
            while( parents[item] != item ) {
                item = parents[item] = parents[parents[item]];
            }
            return item;
        }

        var leftOut = new Dictionary<( int, int ), int>( );

        for( var triangle = 0; triangle < triangles.Count; ++triangle ) {
            var corners = new[] { triangles[triangle].a, triangles[triangle].b, triangles[triangle].c };

            for( var k = 0; k < 3; ++k ) {
                if( ( flags[triangle] & ( 1 << k ) ) != 0 ) {
                    continue;
                }

                var a   = corners[k];
                var b   = corners[( k + 1 ) % 3];
                var key = a < b ? ( a, b ) : ( b, a );

                if( leftOut.TryGetValue( key, out var other ) ) {
                    parents[find( triangle )] = find( other );
                }
                else {
                    leftOut.Add( key, triangle );
                }
            }
        }

        var edges = new Dictionary<int, Dictionary<int, int>>( );

        for( var triangle = 0; triangle < triangles.Count; ++triangle ) {
            var corners = new[] { triangles[triangle].a, triangles[triangle].b, triangles[triangle].c };
            var face    = find( triangle );

            if( !edges.TryGetValue( face, out var next ) ) {
                edges.Add( face, next = new Dictionary<int, int>( ) );
            }

            for( var k = 0; k < 3; ++k ) {
                if( ( flags[triangle] & ( 1 << k ) ) != 0 ) {
                    next.Add( corners[k], corners[( k + 1 ) % 3] );
                }
            }
        }

        var polygons = new List<int[]>( );

        foreach( var face in edges.Keys.OrderBy( ( face ) => face ) ) {
            var next    = edges[face];
            var start   = next.Keys.Min( );
            var polygon = new List<int> { start };

            for( var corner = next[start]; corner != start; corner = next[corner] ) {
                polygon.Add( corner );
            }

            Assert.AreEqual( next.Count, polygon.Count, $"the edges of face {face} go round it once" );

            polygons.Add( polygon.ToArray( ) );
        }

        return polygons;
    }

    // a polygon from its least corner, in its turn
    private static string keyOf( IEnumerable<int> polygon ) {
        var corners = polygon.ToArray( );
        var least   = Array.IndexOf( corners, corners.Min( ) );

        return string.Join( " ", Enumerable.Range( 0, corners.Length ).Select( ( k ) => corners[( least + k ) % corners.Length] ) );
    }

    [Test]
    public void theTrianglesOfPolygonsGiveBackThePolygons( ) {
        // a triangle, a quad and a pentagon side by side, flat, as a polygon list
        var positions = new[] {
            new double3( 0, 0, 0 ), new double3( 1, 0, 0 ), new double3( 0, 1, 0 ),
            new double3( 2, 0, 0 ), new double3( 3, 0, 0 ), new double3( 3, 1, 0 ), new double3( 2, 1, 0 ),
            new double3( 5, 0, 0 ), new double3( 6, 0, 0 ), new double3( 6.5, 1, 0 ), new double3( 5.5, 1.8, 0 ), new double3( 4.5, 1, 0 )
        };
        var polygons = new[] { new[] { 0, 1, 2 }, new[] { 3, 4, 5, 6 }, new[] { 7, 8, 9, 10, 11 } };

        using var graphPositions = new NativeArray<double3>( positions, Allocator.TempJob );
        using var starts         = new NativeArray<int>( new[] { 0, 3, 7, 12 }, Allocator.TempJob );
        using var corners        = new NativeArray<int>( polygons.SelectMany( ( polygon ) => polygon ).ToArray( ), Allocator.TempJob );
        using var faces          = new NativeArray<int>( new[] { 0, 1, 2 }, Allocator.TempJob );
        using var subMeshes      = new NativeArray<int>( new[] { 0, 1, 0 }, Allocator.TempJob );
        using var written        = new NativeList<double3>( Allocator.TempJob );
        using var own            = new NativeList<double3>( Allocator.TempJob );
        using var triangles      = new NativeList<BurstMesh.Triangle>( Allocator.TempJob );
        using var flags          = new NativeList<byte>( Allocator.TempJob );

        new BurstFieldAligned.WriteJob {
            graphPositions = graphPositions, polygonStarts = starts, polygonCorners = corners, polygonFaces = faces, faceSubMeshes = subMeshes, positions = written, ownNormals = own, triangles = triangles, flags = flags
        }.Run( );

        Assert.AreEqual( 1 + 2 + 3, triangles.Length, "triangles: one, two and three" );
        Assert.AreEqual( positions.Length, written.Length, "every corner a vertex" );

        CollectionAssert.AreEqual( new[] { 0, 1, 1, 0, 0, 0 }, triangles.AsArray( ).Select( ( triangle ) => triangle.subMeshIndex ).ToArray( ), "the sub-mesh of each polygon's face" );

        var given = polygonsOf( triangles.AsArray( ).ToArray( ), flags.AsArray( ).ToArray( ) ).Select( keyOf ).OrderBy( ( key ) => key ).ToArray( );

        CollectionAssert.AreEqual( polygons.Select( keyOf ).OrderBy( ( key ) => key ).ToArray( ), given, "the polygons, from the triangles and their flags" );

        // and every triangle faces the way its polygon does
        foreach( var triangle in triangles.AsArray( ) ) {
            Assert.Greater( math.cross( written[triangle.b] - written[triangle.a], written[triangle.c] - written[triangle.a] ).z, 0.0, "facing up" );
        }
    }

    // ---- the graph read off the fields ----

    // the inputs of ClusterJob, built by hand: vertices with their lattice points, and links between them that say
    // what the two lattice points are
    private sealed class Links : IDisposable {

        public NativeArray<float3>           positions;
        public NativeArray<float3>           normals;
        public NativeArray<float3>           lattice;
        public NativeArray<byte>             held;
        public NativeArray<int>              linkStarts;
        public NativeArray<BurstFields.Link> links;
        public NativeArray<byte>             kinds;
        public NativeArray<float>            distances;

        // a mesh of no edges, unless one is given
        public NativeArray<int>   firstEdges;
        public NativeArray<int>   nextEdges;
        public NativeArray<Sharp> edgeSharp;
        public NativeArray<int>   edgeLines;

        public Links( int count, IEnumerable<( int a, int b, byte kind, float distance )> given, Func<int, float3> latticeOf = null, int[] heldVertices = null, ( int a, int b, bool sharp )? crease = null ) {
            var lists = Enumerable.Range( 0, count ).Select( ( _ ) => new List<( int other, byte kind, float distance )>( ) ).ToArray( );

            foreach( var ( a, b, kind, distance ) in given ) {
                lists[a].Add( ( b, kind, distance ) );
                lists[b].Add( ( a, BurstFieldAligned.Apart, 0.0f ) );
            }

            var starts = new List<int> { 0 };
            var all    = new List<( int other, byte kind, float distance )>( );

            foreach( var list in lists ) {
                all.AddRange( list.OrderBy( ( link ) => link.other ) );
                starts.Add( all.Count );
            }

            this.positions  = new NativeArray<float3>( Enumerable.Range( 0, count ).Select( ( vertex ) => new float3( vertex, 0, 0 ) ).ToArray( ), Allocator.TempJob );
            this.normals    = new NativeArray<float3>( Enumerable.Repeat( new float3( 0, 0, 1 ), count ).ToArray( ), Allocator.TempJob );
            this.lattice    = new NativeArray<float3>( Enumerable.Range( 0, count ).Select( ( vertex ) => latticeOf?.Invoke( vertex ) ?? new float3( vertex, 0, 0 ) ).ToArray( ), Allocator.TempJob );
            this.held       = new NativeArray<byte>( Enumerable.Range( 0, count ).Select( ( vertex ) => heldVertices != null && heldVertices.Contains( vertex ) ? ( byte )1 : ( byte )0 ).ToArray( ), Allocator.TempJob );
            this.linkStarts = new NativeArray<int>( starts.ToArray( ), Allocator.TempJob );
            this.links      = new NativeArray<BurstFields.Link>( all.Select( ( link ) => new BurstFields.Link { vertex = link.other, weight = 1.0f } ).ToArray( ), Allocator.TempJob );
            this.kinds      = new NativeArray<byte>( all.Select( ( link ) => link.kind ).ToArray( ), Allocator.TempJob );
            this.distances  = new NativeArray<float>( all.Select( ( link ) => link.distance ).ToArray( ), Allocator.TempJob );

            var first = Enumerable.Repeat( -1, count ).ToArray( );
            var next  = new List<int>( );
            var sharp = new List<Sharp>( );
            var lines = new List<int>( );

            if( crease.HasValue ) {
                var ( a, b, isSharp ) = crease.Value;

                // the edge's two ends, 2 e and 2 e + 1, in the lists of its vertices
                first[a] = 0;
                first[b] = 1;
                next.AddRange( new[] { -1, -1 } );
                sharp.Add( isSharp ? Sharp.Materials : Sharp.None );
                lines.Add( 0 );
            }

            this.firstEdges = new NativeArray<int>( first, Allocator.TempJob );
            this.nextEdges  = new NativeArray<int>( next.ToArray( ), Allocator.TempJob );
            this.edgeSharp  = new NativeArray<Sharp>( sharp.ToArray( ), Allocator.TempJob );
            this.edgeLines  = new NativeArray<int>( lines.ToArray( ), Allocator.TempJob );
        }

        public void Dispose( ) {
            this.positions.Dispose( );
            this.normals.Dispose( );
            this.lattice.Dispose( );
            this.held.Dispose( );
            this.linkStarts.Dispose( );
            this.links.Dispose( );
            this.kinds.Dispose( );
            this.distances.Dispose( );
            this.firstEdges.Dispose( );
            this.nextEdges.Dispose( );
            this.edgeSharp.Dispose( );
            this.edgeLines.Dispose( );
        }

        // the graph read off them, with the vertex of the graph each went into, and the counts
        public int[] cluster( Graph graph, float rho, out int[] counts ) {
            using var clusters = new NativeArray<int>( this.positions.Length, Allocator.TempJob );
            using var numbers  = new NativeArray<int>( 6, Allocator.TempJob );

            new BurstFieldAligned.ClusterJob {
                positions   = this.positions,
                normals     = this.normals,
                lattice     = this.lattice,
                held        = this.held,
                linkStarts  = this.linkStarts,
                links       = this.links,
                kinds       = this.kinds,
                distances   = this.distances,
                firstEdges  = this.firstEdges,
                nextEdges   = this.nextEdges,
                edgeSharp   = this.edgeSharp,
                edgeLines   = this.edgeLines,
                graph       = graph,
                clusters    = clusters,
                counts      = numbers,
                vertexCount = this.positions.Length,
                rho         = rho
            }.Run( );

            counts = numbers.ToArray( );

            return clusters.ToArray( );
        }

    }

    private static List<int> neighboursOf( Graph graph, int vertex ) {
        var neighbours = new List<int>( );

        for( var entry = graph.heads[vertex]; entry != -1; entry = graph.entries[entry].y ) {
            if( graph.entries[entry].x >= 0 ) {
                neighbours.Add( graph.entries[entry].x );
            }
        }

        neighbours.Sort( );

        return neighbours;
    }

    private const byte Apart = BurstFieldAligned.Apart;
    private const byte Same  = BurstFieldAligned.Same;
    private const byte Step  = BurstFieldAligned.Step;

    // vertices whose lattice points are one merge, the nearest first; but two already joined by an edge whose lattice
    // points are a step apart stay apart (section 3.8)
    [Test]
    public void theNearestMergeFirstAndTwoJoinedByAStepStayApart( ) {
        // 0 and 1 are nearer than 1 and 2, and 0 is a step from 2: once 0 and 1 are one, 1 and 2 are joined by a step
        using var links = new Links( 3, new[] { ( 0, 1, Same, 0.1f ), ( 1, 2, Same, 0.2f ), ( 0, 2, Step, 0.0f ) } );
        using var graph = new Graph( Allocator.TempJob );

        var clusters = links.cluster( graph, 1.0f, out var counts );

        Assert.AreEqual( 2, graph.vertexCount, "vertices" );
        Assert.AreEqual( clusters[0], clusters[1], "0 and 1 are one" );
        Assert.AreNotEqual( clusters[1], clusters[2], "1 and 2 are not" );
        Assert.AreEqual( new[] { 1, 2, 1, 1, 2, 0 }, counts, "a step, two of one point, a merge, a conflict, two vertices, none spurious" );

        CollectionAssert.AreEqual( new[] { clusters[2] }, neighboursOf( graph, clusters[0] ), "joined by the step" );

        // the other way round, 1 and 2 are the nearer: they are one, and 0 stays apart
        using var other      = new Links( 3, new[] { ( 0, 1, Same, 0.3f ), ( 1, 2, Same, 0.2f ), ( 0, 2, Step, 0.0f ) } );
        using var otherGraph = new Graph( Allocator.TempJob );

        var otherClusters = other.cluster( otherGraph, 1.0f, out _ );

        Assert.AreEqual( otherClusters[1], otherClusters[2], "1 and 2 are one" );
        Assert.AreNotEqual( otherClusters[0], otherClusters[1], "0 and 1 are not" );
    }

    // a vertex of the graph is where the lattice points of its vertices are, each weighted by how near it is to its own
    // vertex: exp( -9 |p - v|^2 / rho^2 )
    [Test]
    public void aVertexIsAtTheMeanOfItsLatticePointsWeightedByHowNearEachIsToItsVertex( ) {
        // 0 at x = 0 with its point a tenth of rho off, 1 at x = 1 with its point a third of rho off, both points apart
        // from each other; 2 a step away so that the group has an edge
        float3 latticeOf( int vertex ) {
            return vertex == 0 ? new float3( 0.0f, 0.2f, 0.0f ) : vertex == 1 ? new float3( 1.0f, 0.6667f, 0.0f ) : new float3( 2.0f, 0.0f, 0.0f );
        }

        using var links = new Links( 3, new[] { ( 0, 1, Same, 0.1f ), ( 1, 2, Step, 0.0f ) }, latticeOf );
        using var graph = new Graph( Allocator.TempJob );

        var clusters = links.cluster( graph, 2.0f, out _ );

        var w0       = math.exp( -9.0 * 0.04 / 4.0 );
        var w1       = math.exp( -9.0 * ( 0.6667 * 0.6667 ) / 4.0 );
        var expected = ( ( new double3( 0.0, 0.2, 0.0 ) * w0 ) + ( new double3( 1.0, 0.6667, 0.0 ) * w1 ) ) / ( w0 + w1 );

        Assert.AreEqual( 0.0, math.distance( expected, graph.positions[clusters[0]] ), 1e-6, $"where it is: {graph.positions[clusters[0]]} against {expected}" );
        Assert.AreEqual( 2, graph.members[clusters[0]], "of two" );
    }

    // a group of vertices fewer than a tenth of the mean is spurious (section 3.8), and so is one with no edge
    [Test]
    public void aGroupOfFewerThanATenthOfTheMeanIsLeftOut( ) {
        // two chains of 30 vertices of one point, each a step from a vertex alone; and a vertex alone that is a step from
        // nothing. The mean is 61 / 3, and a tenth of it 2.03
        var given = new List<( int, int, byte, float )>( );

        for( var vertex = 0; vertex < 29; ++vertex ) {
            given.Add( ( vertex, vertex + 1, Same, vertex ) );
            given.Add( ( 30 + vertex, 31 + vertex, Same, vertex ) );
        }

        given.Add( ( 0, 30, Step, 0.0f ) );
        given.Add( ( 29, 60, Step, 0.0f ) );

        using var links = new Links( 62, given );
        using var graph = new Graph( Allocator.TempJob );

        var clusters = links.cluster( graph, 1.0f, out var counts );

        Assert.AreEqual( 2, graph.vertexCount, "the two chains" );
        Assert.AreEqual( -1, clusters[60], "the vertex alone with a step to a chain" );
        Assert.AreEqual( -1, clusters[61], "the vertex with no step" );
        Assert.AreEqual( 1, counts[5], "spurious" );
        Assert.AreEqual( 30, graph.members[clusters[0]], "a chain" );
    }

    // smoothing leaves a vertex where it is where one of its vertices is held, or on a crease, and not where one is on a
    // line that is hard
    [Test]
    public void aVertexOfAHeldVertexOrOfACreaseIsFixed( [Values( "held", "crease", "hard", "none" )] string which ) {
        using var links = new Links(
            3, new[] { ( 0, 1, Same, 0.1f ), ( 1, 2, Step, 0.0f ) },
            heldVertices: which == "held" ? new[] { 1 } : null,
            crease: which == "crease" ? ( 1, 2, false ) : which == "hard" ? ( 1, 2, true ) : null
        );
        using var graph = new Graph( Allocator.TempJob );

        var clusters = links.cluster( graph, 1.0f, out _ );

        Assert.AreEqual( which == "held" || which == "crease", graph.fixedVertices[clusters[0]], "the group of 0 and 1" );
    }

    // a graph built by hand, flat in z = 0, every vertex facing up unless it is given otherwise
    private static Graph graphOf( double2[] points, ( int, int )[] edges, bool down = false ) {
        var graph = new Graph( Allocator.TempJob );

        foreach( var point in points ) {
            graph.add( new double3( point, 0.0 ), new double3( 0.0, 0.0, down ? -1.0 : 1.0 ), false, 1 );
        }

        foreach( var ( a, b ) in edges ) {
            graph.join( a, b );
        }

        return graph;
    }

    private static int[] snap( Graph graph, double rho, bool quads ) {
        using var counts = new NativeArray<int>( 4, Allocator.TempJob );

        new BurstFieldAligned.SnapJob { graph = graph, rho = rho, quads = quads, counts = counts }.Run( );

        return counts.ToArray( );
    }

    // a vertex nearer than 0.3 lengths to an edge between two of its neighbours goes to its middle and splits it
    [Test]
    public void aVertexNearAnEdgeGoesToItsMiddleAndSplitsIt( ) {
        using var graph = graphOf( new[] { new double2( 1.1, 0.25 ), new double2( 0, 0 ), new double2( 2, 0 ) }, new[] { ( 0, 1 ), ( 1, 2 ) } );

        var counts = snap( graph, 1.0, true );

        Assert.AreEqual( new[] { 1, 0, 0, 0 }, counts, "snapped onto an edge" );
        Assert.AreEqual( 0.0, math.distance( new double3( 1, 0, 0 ), graph.positions[0] ), 1e-12, "at the middle of the edge" );

        CollectionAssert.AreEqual( new[] { 1, 2 }, neighboursOf( graph, 0 ), "the edge split" );
        CollectionAssert.AreEqual( new[] { 0 },    neighboursOf( graph, 1 ), "an end" );
        CollectionAssert.AreEqual( new[] { 0 },    neighboursOf( graph, 2 ), "the other end" );

        // further than 0.3 lengths from it, it stays
        using var far = graphOf( new[] { new double2( 1.1, 0.35 ), new double2( 0, 0 ), new double2( 2, 0 ) }, new[] { ( 0, 1 ), ( 1, 2 ) } );

        Assert.AreEqual( new[] { 0, 0, 0, 0 }, snap( far, 1.0, true ), "further than 0.3" );
    }

    // a vertex nearer than 0.3 lengths to an edge and to one of its ends merges with that end
    [Test]
    public void aVertexNearAnEdgeAndItsEndMergesWithTheEnd( ) {
        // 0 near 1 on the edge from 1 to 2, and joined to 3 besides
        using var graph = graphOf( new[] { new double2( 0.2, 0.1 ), new double2( 0, 0 ), new double2( 2, 0 ), new double2( 0, 2 ) }, new[] { ( 0, 1 ), ( 1, 2 ), ( 0, 3 ) } );

        var counts = snap( graph, 1.0, true );

        Assert.AreEqual( 1, counts[1], "merged" );
        Assert.AreEqual( -1, graph.into[0], "0 is left" );
        Assert.AreEqual( 0, graph.into[1], "1 merged into 0" );
        Assert.AreEqual( 0, graph.resolved( 1 ), "1 ends in 0" );
        Assert.AreEqual( 0.0, math.distance( new double3( 0.1, 0.05, 0 ), graph.positions[0] ), 1e-12, "between the two" );

        CollectionAssert.AreEqual( new[] { 2, 3 }, neighboursOf( graph, 0 ), "the neighbours of both" );
        CollectionAssert.IsEmpty( neighboursOf( graph, 1 ), "the one merged" );
    }

    // for quads, an edge with two neighbours in common is the diagonal of two triangles that are a quad, and goes; for
    // triangles it stays
    [Test]
    public void theDiagonalOfAQuadGoesForQuadsAndStaysForTriangles( [Values] bool quads ) {
        using var graph = graphOf( new[] { new double2( 0, 0 ), new double2( 1, 0 ), new double2( 1, 1 ), new double2( 0, 1 ) }, new[] { ( 0, 1 ), ( 1, 2 ), ( 2, 3 ), ( 3, 0 ), ( 0, 2 ) } );

        var counts = snap( graph, 1.0, quads );

        Assert.AreEqual( quads ? 1 : 0, counts[2], "diagonals taken out" );
        Assert.AreEqual( !quads, graph.has( 0, 2 ), "the diagonal" );
        Assert.IsTrue( graph.has( 0, 1 ) && graph.has( 1, 2 ) && graph.has( 2, 3 ) && graph.has( 3, 0 ), "the sides" );
    }

    // ---- faces ----

    // the faces of a graph, as polygons of its vertices
    private sealed class Walked : IDisposable {

        public NativeList<int>    rimStarts      = new( Allocator.TempJob );
        public NativeList<int>    rim            = new( Allocator.TempJob );
        public NativeList<double> rimAngles      = new( Allocator.TempJob );
        public NativeList<int>    rimFaces       = new( Allocator.TempJob );
        public NativeList<int>    polygonStarts  = new( Allocator.TempJob );
        public NativeList<int>    polygonCorners = new( Allocator.TempJob );
        public NativeList<int>    polygonFaces   = new( Allocator.TempJob );
        public int[]              counts;

        public Walked( Graph graph, Faces faces ) {
            using var numbers = new NativeArray<int>( 8, Allocator.TempJob );

            new BurstFieldAligned.FacesJob {
                graph = graph, faces = faces, rimStarts = this.rimStarts, rim = this.rim, rimAngles = this.rimAngles, rimFaces = this.rimFaces,
                polygonStarts = this.polygonStarts, polygonCorners = this.polygonCorners, polygonFaces = this.polygonFaces, counts = numbers
            }.Run( );

            this.counts = numbers.ToArray( );
        }

        public List<int[]> polygons {
            get {
                return Enumerable.Range( 0, this.polygonFaces.Length )
                    .Select( ( polygon ) => Enumerable.Range( this.polygonStarts[polygon], this.polygonStarts[polygon + 1] - this.polygonStarts[polygon] ).Select( ( corner ) => this.polygonCorners[corner] ).ToArray( ) )
                    .ToList( );
            }
        }

        public void Dispose( ) {
            this.rimStarts.Dispose( );
            this.rim.Dispose( );
            this.rimAngles.Dispose( );
            this.rimFaces.Dispose( );
            this.polygonStarts.Dispose( );
            this.polygonCorners.Dispose( );
            this.polygonFaces.Dispose( );
        }

    }

    // a grid of so many squares a side, its vertices row by row
    private static ( double2[] points, ( int, int )[] edges ) gridOf( int width, int height ) {
        var points = new List<double2>( );
        var edges  = new List<( int, int )>( );

        for( var j = 0; j <= height; ++j )
        for( var i = 0; i <= width; ++i ) {
            points.Add( new double2( i, j ) );

            var vertex = ( j * ( width + 1 ) ) + i;

            if( i < width ) {
                edges.Add( ( vertex, vertex + 1 ) );
            }
            if( j < height ) {
                edges.Add( ( vertex, vertex + width + 1 ) );
            }
        }

        return ( points.ToArray( ), edges.ToArray( ) );
    }

    private static double signedArea( Graph graph, int[] polygon ) {
        var area = 0.0;

        for( var k = 0; k < polygon.Length; ++k ) {
            var a = graph.positions[polygon[k]];
            var b = graph.positions[polygon[( k + 1 ) % polygon.Length]];

            area += ( a.x * b.y ) - ( b.x * a.y );
        }

        return area / 2.0;
    }

    // the faces of a grid are its squares, each turning the way its vertices' normals say, which is the way the input
    // faces; the rim round it, which turns the other way, is no face
    [Test]
    public void theFacesOfAGridAreItsSquaresTurningTheWayItsNormalsFace( [Values] bool down ) {
        var ( points, edges ) = gridOf( 3, 3 );

        using var graph  = graphOf( points, edges, down );
        using var walked = new Walked( graph, Faces.QuadDominant );

        var polygons = walked.polygons;

        Assert.AreEqual( 9, polygons.Count, "faces" );
        Assert.AreEqual( 9, walked.counts[1], "walked quads" );
        Assert.AreEqual( 0, walked.counts[0] + walked.counts[2] + walked.counts[3], "walked triangles, pentagons and larger" );
        Assert.AreEqual( 0, walked.counts[4], "holes filled" );

        foreach( var polygon in polygons ) {
            Assert.AreEqual( 4, polygon.Length, "a square" );
            Assert.AreEqual( down ? -1.0 : 1.0, signedArea( graph, polygon ), 1e-12, $"{keyOf( polygon )}: one square, turning the way the normals face" );
        }

        // every edge inside the grid is in two faces, every edge of the rim in one
        var sides = polygons.SelectMany( ( polygon ) => polygon.Select( ( corner, k ) => ( corner, polygon[( k + 1 ) % 4] ) ) ).ToList( );

        Assert.AreEqual( sides.Count, sides.Distinct( ).Count( ), "each edge from a vertex to a neighbour in one face" );
        Assert.AreEqual( 2 * 12 + 12, sides.Count, "the edges inside twice and the rim's once" );
    }

    // triangles are walked first: a square with its diagonal is two triangles, for quads as for triangles
    [Test]
    public void aSquareWithItsDiagonalIsTwoTriangles( [Values( Faces.QuadDominant, Faces.Triangles )] Faces faces ) {
        using var graph  = graphOf( new[] { new double2( 0, 0 ), new double2( 1, 0 ), new double2( 1, 1 ), new double2( 0, 1 ) }, new[] { ( 0, 1 ), ( 1, 2 ), ( 2, 3 ), ( 3, 0 ), ( 0, 2 ) } );
        using var walked = new Walked( graph, faces );

        Assert.AreEqual( 2, walked.counts[0], "walked triangles" );
        Assert.AreEqual( 0, walked.counts[1], "walked quads" );

        CollectionAssert.AreEquivalent( new[] { "0 1 2", "0 2 3" }, walked.polygons.Select( keyOf ), "the triangles" );
    }

    // a walk that meets a vertex of one neighbour on its way fails, and one that comes back to its first vertex is done
    // (the code). A face that every walk round meets such a vertex on is a hole: one of up to six sides is filled, and
    // a larger one is left
    [Test]
    public void aHoleOfUpToSixSidesIsFilledAndALargerOneLeft( [Values( 1, 2, 3 )] int length ) {
        // a grid three squares high, whose middle row has a face of so many squares in it, of 2 length + 2 sides, with a
        // vertex of one neighbour inside it at two of its opposite corners: every walk round it meets one or the other
        var width = length + 2;
        var ( points, edges ) = gridOf( width, 3 );

        int at( int i, int j ) {
            return ( j * ( width + 1 ) ) + i;
        }

        var inside   = Enumerable.Range( 2, length - 1 ).Select( ( i ) => ( at( i, 1 ), at( i, 2 ) ) ).ToList( );
        var kept     = edges.Where( ( edge ) => !inside.Contains( edge ) ).ToList( );
        var first    = points.Length;
        var second   = points.Length + 1;
        var vertices = points.Append( new double2( 1.3, 1.5 ) ).Append( new double2( length + 0.7, 1.5 ) ).ToArray( );

        kept.Add( ( at( 1, 1 ), first ) );
        kept.Add( ( at( length + 1, 2 ), second ) );

        using var graph  = graphOf( vertices, kept.ToArray( ) );
        using var walked = new Walked( graph, Faces.QuadDominant );

        var sides   = ( 2 * length ) + 2;
        var squares = ( 3 * width ) - length;

        Assert.AreEqual( sides <= 6 ? 1 : 0, walked.counts[4], "holes filled" );
        Assert.AreEqual( squares + ( sides == 4 ? 1 : sides == 6 ? 2 : 0 ), walked.polygons.Count, "the squares round it, and the hole as a quad or cut into two" );
        Assert.IsFalse( walked.polygons.Any( ( polygon ) => polygon.Contains( first ) || polygon.Contains( second ) ), "no face has a vertex of one neighbour" );
    }

    // a face of more than five sides is cut into quads, the run of four corners with the angles nearest a square's
    // first, until it has five at most; for triangles every face is cut into triangles
    [Test]
    public void aLargerFaceIsCutIntoQuadsOrTriangles( [Values( 5, 6, 7, 8 )] int sides, [Values( Faces.QuadDominant, Faces.Triangles )] Faces faces ) {
        var ring  = Enumerable.Range( 0, sides ).Select( ( k ) => new double2( math.cos( 2.0 * math.PI * k / sides ), math.sin( 2.0 * math.PI * k / sides ) ) ).ToArray( );
        var edges = Enumerable.Range( 0, sides ).Select( ( k ) => ( k, ( k + 1 ) % sides ) ).ToArray( );

        using var graph  = graphOf( ring, edges );
        using var walked = new Walked( graph, faces );

        var polygons = walked.polygons;
        var lengths  = polygons.Select( ( polygon ) => polygon.Length ).OrderBy( ( length ) => length ).ToArray( );

        if( faces == Faces.Triangles ) {
            Assert.AreEqual( Enumerable.Repeat( 3, sides - 2 ).ToArray( ), lengths, "triangles" );
        }
        else {
            var expected = sides switch {
                5 => new[] { 5 },
                6 => new[] { 4, 4 },
                7 => new[] { 4, 5 },
                _ => new[] { 4, 4, 4 }
            };

            Assert.AreEqual( expected, lengths, "quads, and a pentagon kept" );
        }

        // the pieces cover the face once: their areas add up to its
        Assert.AreEqual( signedArea( graph, Enumerable.Range( 0, sides ).ToArray( ) ), polygons.Sum( ( polygon ) => signedArea( graph, polygon ) ), 1e-9, "areas" );
        Assert.IsTrue( walked.polygonFaces.AsArray( ).ToArray( ).All( ( face ) => face == 0 ), "all of the one walked face" );
    }

    // of the runs of four corners, the one whose quad's angles are nearest a square's is cut off first: here a hexagon
    // pulled out at its top and its bottom, so that the run from corner 5 round to 2 is the nearest a square
    [Test]
    public void theRunCutOffFirstHasTheAnglesNearestASquares( ) {
        var ring = new[] { new double2( 1, -1.1 ), new double2( 1, 1.1 ), new double2( 0, 1.4 ), new double2( -1, 1 ), new double2( -1, -1 ), new double2( 0, -1.4 ) };

        double scoreOf( int first ) {
            var score = 0.0;

            for( var corner = 0; corner < 4; ++corner ) {
                var before = new double3( ring[( first + ( ( corner + 3 ) % 4 ) ) % 6], 0 );
                var at     = new double3( ring[( first + corner ) % 6], 0 );
                var after  = new double3( ring[( first + ( ( corner + 1 ) % 4 ) ) % 6], 0 );

                score += math.abs( math.degrees( BurstFieldAligned.angleBetween( before - at, after - at ) ) - 90.0 );
            }

            return score;
        }

        var best = Enumerable.Range( 0, 6 ).OrderBy( scoreOf ).First( );

        Assert.Less( scoreOf( best ) + 1.0, Enumerable.Range( 0, 6 ).Where( ( first ) => first != best ).Min( scoreOf ), "one run is the best by a degree" );
        Assert.AreNotEqual( 0, best, "the best run is not the first" );

        using var graph  = graphOf( ring, Enumerable.Range( 0, 6 ).Select( ( k ) => ( k, ( k + 1 ) % 6 ) ).ToArray( ) );
        using var walked = new Walked( graph, Faces.QuadDominant );

        var expected = Enumerable.Range( 0, 4 ).Select( ( corner ) => ( best + corner ) % 6 ).ToArray( );

        Assert.AreEqual( keyOf( expected ), keyOf( walked.polygons[0] ), "the quad cut off first" );
    }

    // for quads only, every polygon is split into as many quads as it has sides, round a vertex at its middle, with the
    // middle of an edge shared by the two polygons on it
    [Test]
    public void quadsOnlySplitsEveryPolygonRoundItsMiddle( ) {
        // a square and a triangle that share the edge from 1 to 2
        using var graph  = graphOf( new[] { new double2( 0, 0 ), new double2( 1, 0 ), new double2( 1, 1 ), new double2( 0, 1 ), new double2( 2, 0.5 ) }, new[] { ( 0, 1 ), ( 1, 2 ), ( 2, 3 ), ( 3, 0 ), ( 1, 4 ), ( 4, 2 ) } );
        using var walked = new Walked( graph, Faces.Quads );

        var polygons = walked.polygons;

        Assert.AreEqual( 4 + 3, polygons.Count, "quads" );
        Assert.IsTrue( polygons.All( ( polygon ) => polygon.Length == 4 ), "every one a quad" );

        // the two middles, and the middles of the six edges
        Assert.AreEqual( 5 + 2 + 6, graph.vertexCount, "vertices" );
        Assert.AreEqual( 1.0 + 0.5, polygons.Sum( ( polygon ) => signedArea( graph, polygon ) ), 1e-9, "the area of both" );

        var middle = Enumerable.Range( 5, graph.vertexCount - 5 ).Single( ( vertex ) => math.distance( graph.positions[vertex], new double3( 1, 0.5, 0 ) ) < 1e-12 );

        Assert.AreEqual( 4, polygons.Count( ( polygon ) => polygon.Contains( middle ) ), "the middle of the edge they share is in two quads of each" );
    }

    // ---- the whole ----

    // a mesh remeshed straight by BurstFieldAligned, with the faces its flags make
    private sealed class Remeshed {

        public readonly double3[]            positions;
        public readonly double3[]            normals;
        public readonly BurstMesh.Triangle[] triangles;
        public readonly byte[]               flags;
        public readonly List<int[]>          polygons;

        public BurstFieldAligned.Report report;

        public Remeshed( HandBuiltMesh built, BurstFieldAligned.Settings settings, double cornerAngle = 0.0 ) {
            using var positions     = built.nativePositions( );
            using var normals       = built.nativeNormals( );
            using var triangles     = built.nativeTriangles( );
            using var cornerNormals = cornerAngle > 0.0 ? BurstRemeshing.cornerNormalsOf( positions, normals, triangles, cornerAngle, Allocator.TempJob ) : new NativeArray<float3>( 0, Allocator.TempJob );
            using var outPositions  = new NativeList<float3>( Allocator.TempJob );
            using var outNormals    = new NativeList<float3>( Allocator.TempJob );
            using var outTriangles  = new NativeList<BurstMesh.Triangle>( Allocator.TempJob );
            using var outFlags      = new NativeList<byte>( Allocator.TempJob );

            BurstFieldAligned.remesh( positions, normals, triangles, cornerNormals, settings, outPositions, outNormals, outTriangles, outFlags, out this.report );

            this.positions = outPositions.AsArray( ).Select( ( position ) => ( double3 )position ).ToArray( );
            this.normals   = outNormals.AsArray( ).Select( ( normal ) => ( double3 )normal ).ToArray( );
            this.triangles = outTriangles.AsArray( ).ToArray( );
            this.flags     = outFlags.AsArray( ).ToArray( );
            this.polygons  = polygonsOf( this.triangles, this.flags );
        }

        // how many edges of the faces each vertex has
        public int[] edgesPerVertex {
            get {
                var edges = new HashSet<( int, int )>( );

                foreach( var polygon in this.polygons ) {
                    for( var k = 0; k < polygon.Length; ++k ) {
                        var a = polygon[k];
                        var b = polygon[( k + 1 ) % polygon.Length];

                        edges.Add( a < b ? ( a, b ) : ( b, a ) );
                    }
                }

                var counts = new int[this.positions.Length];

                foreach( var ( a, b ) in edges ) {
                    ++counts[a];
                    ++counts[b];
                }

                return counts;
            }
        }

        // how many of the faces' edges are in one face only
        public int openEdges {
            get {
                return this.polygons
                    .SelectMany( ( polygon ) => polygon.Select( ( corner, k ) => ( corner, polygon[( k + 1 ) % polygon.Length] ) ) )
                    .Select( ( edge ) => edge.Item1 < edge.Item2 ? edge : ( edge.Item2, edge.Item1 ) )
                    .GroupBy( ( edge ) => edge )
                    .Count( ( group ) => group.Count( ) == 1 );
            }
        }

    }

    private static BurstFieldAligned.Settings settingsOf( Faces faces, double length ) {
        return new BurstFieldAligned.Settings { faces = faces, length = length, featureCosine = featureCosine };
    }

    // a point that is a whole number of lengths along each axis
    private static double offLattice( double3 point, double length ) {
        var steps = point / length;

        return math.cmax( math.abs( steps - math.round( steps ) ) ) * length;
    }

    // a flat square gives a square lattice: at a length that fits it, every vertex at a lattice point, every face a
    // quad, every vertex inside of four edges
    [Test]
    public void aFlatSquareGivesASquareLattice( [Values( Faces.QuadDominant, Faces.Quads )] Faces faces ) {
        var length   = faces == Faces.Quads ? 1.0 : 2.0;
        var remeshed = new Remeshed( new HandBuiltMesh( ).patch( 8, 8 ), settingsOf( faces, length ) );
        var side     = ( int )( 8 / length ) + 1;

        Assert.AreEqual( side * side, remeshed.positions.Length, "vertices" );
        Assert.AreEqual( ( side - 1 ) * ( side - 1 ), remeshed.polygons.Count, "faces" );
        Assert.IsTrue( remeshed.polygons.All( ( polygon ) => polygon.Length == 4 ), "every face a quad" );

        var edges = remeshed.edgesPerVertex;

        for( var vertex = 0; vertex < remeshed.positions.Length; ++vertex ) {
            var position = remeshed.positions[vertex];
            var onRim    = position.x < 1e-3 || position.y < 1e-3 || position.x > 8 - 1e-3 || position.y > 8 - 1e-3;
            var corner   = ( position.x < 1e-3 || position.x > 8 - 1e-3 ) && ( position.y < 1e-3 || position.y > 8 - 1e-3 );

            Assert.AreEqual( 0.0, offLattice( position, length ), 1e-4, $"vertex {vertex} at {position}: a lattice point" );
            Assert.AreEqual( corner ? 2 : onRim ? 3 : 4, edges[vertex], $"vertex {vertex} at {position}: its edges" );
        }

        // two rounds of smoothing (the code), each of which moves every vertex off the rim, which is held: to where
        // it is, as the middle of its neighbours is
        Assert.AreEqual( BurstFieldAligned.smoothingRounds * ( side - 2 ) * ( side - 2 ), remeshed.report.moves, "moves" );
        Assert.AreEqual( 2, BurstFieldAligned.smoothingRounds, "rounds of smoothing" );
    }

    // a flat triangle of equal sides, of so many small ones a side, its rim open
    private static HandBuiltMesh flatTriangle( int size ) {
        var built = new HandBuiltMesh( );

        Vector3 at( int i, int j ) {
            return new Vector3( i + ( 0.5f * j ), ( float )( j * Math.Sqrt( 3.0 ) / 2.0 ), 0.0f );
        }

        for( var j = 0; j < size; ++j )
        for( var i = 0; i + j < size; ++i ) {
            built.triangle( at( i, j ), at( i + 1, j ), at( i, j + 1 ) );

            if( i + j + 1 < size ) {
                built.triangle( at( i + 1, j ), at( i + 1, j + 1 ), at( i, j + 1 ) );
            }
        }

        return built;
    }

    // a flat triangle gives a triangular lattice: at a third of its side, the ten points of the lattice, nine triangles,
    // and the vertex inside of six edges
    [Test]
    public void aFlatTriangleGivesATriangularLattice( ) {
        var remeshed = new Remeshed( flatTriangle( 9 ), settingsOf( Faces.Triangles, 3.0 ) );

        Assert.AreEqual( 10, remeshed.positions.Length, "vertices" );
        Assert.AreEqual( 9,  remeshed.polygons.Count,   "faces" );
        Assert.IsTrue( remeshed.polygons.All( ( polygon ) => polygon.Length == 3 ), "every face a triangle" );

        var edges = remeshed.edgesPerVertex;
        var axis  = new double3( 0.5, math.sqrt( 3.0 ) / 2.0, 0.0 );

        for( var vertex = 0; vertex < remeshed.positions.Length; ++vertex ) {
            var position = remeshed.positions[vertex];
            var along    = position.y / axis.y / 3.0;
            var first    = ( position.x - ( along * 3.0 * axis.x ) ) / 3.0;

            Assert.AreEqual( 0.0, math.abs( along - math.round( along ) ) + math.abs( first - math.round( first ) ), 1e-4, $"vertex {vertex} at {position}: a lattice point" );

            var inside = math.round( along ) == 1.0 && math.round( first ) == 1.0;

            Assert.AreEqual( inside ? 6 : edges[vertex], edges[vertex], $"vertex {vertex} at {position}: its edges" );
        }

        Assert.AreEqual( 1, edges.Count( ( count ) => count == 6 ), "the vertex inside" );
    }

    // a box along the axes gives rows of quads along its edges: at a length of one, a vertex at every whole point of its
    // surface, a quad on every unit square, every vertex of four edges but the corners of three, and closed
    [Test]
    public void aBoxAlongTheAxesGivesItsRowsOfQuads( [Values( Faces.QuadDominant, Faces.Quads )] Faces faces ) {
        var remeshed = new Remeshed( new HandBuiltMesh( ).box( Vector3Int.zero, Vector3Int.one * 4 ), settingsOf( faces, 1.0 ) );

        Assert.AreEqual( 98, remeshed.positions.Length, "vertices" );
        Assert.AreEqual( 96, remeshed.polygons.Count,   "faces" );
        Assert.IsTrue( remeshed.polygons.All( ( polygon ) => polygon.Length == 4 ), "every face a quad" );
        Assert.AreEqual( 0, remeshed.openEdges, "open edges" );

        var edges = remeshed.edgesPerVertex;

        for( var vertex = 0; vertex < remeshed.positions.Length; ++vertex ) {
            var position = remeshed.positions[vertex];
            var corner   = math.all( math.abs( position - ( math.round( position / 4.0 ) * 4.0 ) ) < 0.1 );

            // measured: 0.016 at the most, at the corners, which the fields only near
            Assert.AreEqual( 0.0, offLattice( position, 1.0 ), 0.05, $"vertex {vertex} at {position}: a whole point" );
            Assert.AreEqual( corner ? 3 : 4, edges[vertex], $"vertex {vertex} at {position}: its edges" );
        }
    }

    // a box as triangles is closed, with every vertex on its surface
    [Test]
    public void aBoxAsTrianglesIsClosedOnItsSurface( ) {
        var remeshed = new Remeshed( new HandBuiltMesh( ).box( Vector3Int.zero, Vector3Int.one * 4 ), settingsOf( Faces.Triangles, 1.0 ) );

        Assert.IsTrue( remeshed.polygons.All( ( polygon ) => polygon.Length == 3 ), "every face a triangle" );
        Assert.AreEqual( 0, remeshed.openEdges, "open edges" );

        foreach( var position in remeshed.positions ) {
            var fromSurface = math.cmin( math.min( math.abs( position ), math.abs( position - 4.0 ) ) );

            Assert.AreEqual( 0.0, fromSurface, 1e-3, $"{position}: on the surface" );
        }
    }

    // a band of a second material keeps its lines, which the fields are held to, and each face the material of its
    // area: the areas of both are the input's
    [Test]
    public void aBandOfASecondMaterialKeepsItsLinesAndItsArea( ) {
        var remeshed = new Remeshed( new HandBuiltMesh( ).patch( 8, 8, ( i, j ) => i >= 3 && i < 5 ? 1 : 0 ), settingsOf( Faces.QuadDominant, 1.0 ) );

        var areas = new double[2];

        foreach( var triangle in remeshed.triangles ) {
            var a = remeshed.positions[triangle.a];
            var b = remeshed.positions[triangle.b];
            var c = remeshed.positions[triangle.c];

            areas[triangle.subMeshIndex] += 0.5 * math.length( math.cross( b - a, c - a ) );

            var middle = ( a + b + c ) / 3.0;

            Assert.AreEqual( middle.x > 3.0 && middle.x < 5.0 ? 1 : 0, triangle.subMeshIndex, $"the triangle round {middle}" );
        }

        // the lattice points come within about a hundredth of a length of the lattice along the held lines, which the
        // fields' six iterations a level leave them short of: measured, 0.012 at the most
        foreach( var position in remeshed.positions ) {
            Assert.AreEqual( 0.0, offLattice( position, 1.0 ), 0.02, $"{position}: a whole point" );
        }

        Assert.AreEqual( 48.0, areas[0], 0.05, "the first material's area" );
        Assert.AreEqual( 16.0, areas[1], 0.05, "the band's area" );
        Assert.AreEqual( 0, remeshed.report.unmapped, "faces nothing mapped into" );
    }

    // a face takes the material most of its area has, not most of its triangles: a square of two whose four sides
    // hold its lattice to one quad, five eighths of it a second material in squares of a quarter and three eighths the
    // first in squares of an eighth. The quarters are short enough that the mesh the fields are on keeps them, at
    // twice its mean edge (BurstFields.longestEdge), so the first material has the more triangles there too
    [Test]
    public void aFaceTakesTheMaterialOfMostOfItsArea( ) {
        var built = new HandBuiltMesh( );

        // the second material on x from 0 to 1.25: squares of a quarter, and a last column of each made three triangles
        // to meet the eighths
        for( var j = 0; j < 8; ++j ) {
            var y = j * 0.25f;

            for( var i = 0; i < 4; ++i ) {
                var x = i * 0.25f;

                built.quad( new Vector3( x, y, 0 ), new Vector3( x + 0.25f, y, 0 ), new Vector3( x + 0.25f, y + 0.25f, 0 ), new Vector3( x, y + 0.25f, 0 ), 1 );
            }

            built.triangle( new Vector3( 1.0f, y, 0 ),         new Vector3( 1.25f, y, 0 ),          new Vector3( 1.25f, y + 0.125f, 0 ), 1 );
            built.triangle( new Vector3( 1.0f, y, 0 ),         new Vector3( 1.25f, y + 0.125f, 0 ), new Vector3( 1.0f, y + 0.25f, 0 ),   1 );
            built.triangle( new Vector3( 1.0f, y + 0.25f, 0 ), new Vector3( 1.25f, y + 0.125f, 0 ), new Vector3( 1.25f, y + 0.25f, 0 ),  1 );
        }

        // the first on x from 1.25 to 2, in squares of an eighth
        for( var i = 0; i < 6; ++i )
        for( var j = 0; j < 16; ++j ) {
            Vector3 at( int di, int dj ) {
                return new Vector3( 1.25f + ( ( i + di ) * 0.125f ), ( j + dj ) * 0.125f, 0 );
            }

            built.quad( at( 0, 0 ), at( 1, 0 ), at( 1, 1 ), at( 0, 1 ) );
        }

        Assert.Greater( built.triangles.Count( ( triangle ) => triangle.subMeshIndex == 0 ), 2 * built.triangles.Count( ( triangle ) => triangle.subMeshIndex == 1 ), "the first material's triangles more than twice as many" );

        var settings = settingsOf( Faces.QuadDominant, 2.0 );

        settings.freeMaterials = true;

        var remeshed = new Remeshed( built, settings );

        Assert.AreEqual( 1, remeshed.polygons.Count, "one face" );
        Assert.AreEqual( 4, remeshed.polygons[0].Length, "a quad" );
        Assert.IsTrue( remeshed.triangles.All( ( triangle ) => triangle.subMeshIndex == 1 ), "the second material, which has five eighths of its area" );
    }

    // the materials of the two triangle faces of a square, from one small triangle of the mesh the fields are on, of a
    // sub-mesh, whose corners went into the vertices given, over an input of the two sub-meshes given
    private static ( int first, int second, int unmapped ) materialsOfASquare( double3[] corners, int subMesh, int[] clusters, int firstInput, int secondInput ) {
        using var graph  = graphOf( new[] { new double2( 0, 0 ), new double2( 1, 0 ), new double2( 1, 1 ), new double2( 0, 1 ) }, new[] { ( 0, 1 ), ( 1, 2 ), ( 2, 3 ), ( 3, 0 ), ( 0, 2 ) } );
        using var walked = new Walked( graph, Faces.Triangles );

        Assert.AreEqual( 2, walked.counts[7], "faces" );

        var first  = walked.polygonFaces[walked.polygons.FindIndex( ( polygon ) => keyOf( polygon ) == "0 1 2" )];
        var second = walked.polygonFaces[walked.polygons.FindIndex( ( polygon ) => keyOf( polygon ) == "0 2 3" )];

        using var inputPositions = new NativeArray<float3>( new[] { new float3( 0, 0, 0 ), new float3( 1, 0, 0 ), new float3( 1, 1, 0 ), new float3( 0, 1, 0 ) }, Allocator.TempJob );
        using var inputTriangles = new NativeArray<BurstMesh.Triangle>( new[] { new BurstMesh.Triangle( 0, 1, 2, firstInput ), new BurstMesh.Triangle( 0, 2, 3, secondInput ) }, Allocator.TempJob );
        using var grid           = BurstFieldAligned.gridOf( inputPositions, inputTriangles, Allocator.TempJob );

        using var positions = new NativeArray<double3>( corners, Allocator.TempJob );
        using var triangles = new NativeArray<int3>( new[] { new int3( 0, 1, 2 ) }, Allocator.TempJob );
        using var subMeshes = new NativeArray<int>( new[] { subMesh }, Allocator.TempJob );
        using var into      = new NativeArray<int>( clusters, Allocator.TempJob );
        using var faces     = new NativeArray<int>( 2, Allocator.TempJob );
        using var unmapped  = new NativeArray<int>( 1, Allocator.TempJob );

        new BurstFieldAligned.MaterialsJob {
            graph          = graph,
            rimStarts      = walked.rimStarts.AsArray( ),
            rimAngles      = walked.rimAngles.AsArray( ),
            rimFaces       = walked.rimFaces.AsArray( ),
            polygonStarts  = walked.polygonStarts.AsArray( ),
            polygonCorners = walked.polygonCorners.AsArray( ),
            polygonFaces   = walked.polygonFaces.AsArray( ),
            positions      = positions,
            triangles      = triangles,
            subMeshes      = subMeshes,
            clusters       = into,
            grid           = grid,
            faceSubMeshes  = faces,
            unmapped       = unmapped,
            reach          = 1.0,
            faceCount      = 2
        }.Run( );

        return ( faces[first], faces[second], unmapped[0] );
    }

    // a face nothing of the mesh the fields are on maps into takes the material of the input at its middle: one small
    // triangle inside the first face, near corner 1, and the input's second material under the second face
    [Test]
    public void aFaceNothingMapsIntoTakesTheMaterialAtItsMiddle( ) {
        var ( first, second, unmapped ) = materialsOfASquare( new[] { new double3( 0.9, 0.05, 0 ), new double3( 0.95, 0.1, 0 ), new double3( 0.85, 0.1, 0 ) }, 0, new[] { 1, 1, 1 }, 0, 1 );

        Assert.AreEqual( 1, unmapped, "faces nothing mapped into" );
        Assert.AreEqual( 0, first,    "the first face, from the triangle in it" );
        Assert.AreEqual( 1, second,   "the second face, from the input at its middle" );
    }

    // a triangle maps through the vertex nearest its middle of those its corners went into: here one corner went into
    // corner 3, from which all of the square lies in the second face's corner, and two into corner 1, near it
    [Test]
    public void aTriangleMapsThroughTheNearestOfItsCornersVertices( ) {
        var ( first, second, unmapped ) = materialsOfASquare( new[] { new double3( 0.8, 0.15, 0 ), new double3( 0.85, 0.2, 0 ), new double3( 0.75, 0.2, 0 ) }, 1, new[] { 3, 1, 1 }, 0, 0 );

        Assert.AreEqual( 1, unmapped, "faces nothing mapped into" );
        Assert.AreEqual( 1, first,    "the first face, which the triangle is in" );
        Assert.AreEqual( 0, second,   "the second face, from the input at its middle" );
    }

    // the normals out are the input's as it is shown creased, where each vertex lies on it; or without the input's, the
    // vertex's own triangles'. On a box creased at 40 degrees every vertex of an edge has the normal of one of its
    // faces, and its own triangles lean between the two
    [Test]
    public void theNormalsOutAreTheInputsCreasedOrTheVertexsOwn( [Values] bool input ) {
        var remeshed = new Remeshed( new HandBuiltMesh( ).box( Vector3Int.zero, Vector3Int.one * 4 ), settingsOf( Faces.QuadDominant, 1.0 ), input ? 40.0 : 0.0 );
        var onEdges  = 0;

        for( var vertex = 0; vertex < remeshed.positions.Length; ++vertex ) {
            var position = remeshed.positions[vertex];
            var faces    = math.csum( math.select( int3.zero, new int3( 1 ), ( math.abs( position ) < 0.1 ) | ( math.abs( position - 4.0 ) < 0.1 ) ) );
            var normal   = remeshed.normals[vertex];

            Assert.AreEqual( 1.0, math.length( normal ), 1e-5, $"vertex {vertex}: a unit normal" );

            if( faces == 2 ) {
                ++onEdges;

                Assert.AreEqual( input, math.cmax( math.abs( normal ) ) > 0.999, $"vertex {vertex} at {position}: a face's normal, {normal}" );
            }
            else if( faces == 1 ) {
                Assert.Greater( math.cmax( math.abs( normal ) ), 0.999, $"vertex {vertex} at {position}: inside a face, its normal {normal}" );
            }
        }

        Assert.AreEqual( 12 * 3, onEdges, "vertices on the box's edges, not at its corners" );
    }

    // a mesh the fields would be on with more vertices than the limit gives nothing, and says so
    [Test]
    public void aMeshTooLargeForTheFieldsGivesNothing( ) {
        var limited = settingsOf( Faces.QuadDominant, 1.0 );

        limited.mostVertices = 100;

        var remeshed = new Remeshed( new HandBuiltMesh( ).patch( 8, 8 ), limited );

        Assert.IsTrue( remeshed.report.fields.tooLarge, "too large" );
        Assert.AreEqual( 0, remeshed.positions.Length, "vertices" );
        Assert.AreEqual( 0, remeshed.triangles.Length, "triangles" );

        var unlimited = new Remeshed( new HandBuiltMesh( ).patch( 8, 8 ), settingsOf( Faces.QuadDominant, 1.0 ) );

        Assert.IsFalse( unlimited.report.fields.tooLarge, "without a limit" );
        Assert.Greater( unlimited.report.fields.vertices, 100, "the vertices the fields are on" );
    }

    // the same mesh gives the same remesh every time (finding 12)
    [Test]
    public void theSameMeshGivesTheSameRemeshEveryTime( [Values] Faces faces ) {
        var built  = new HandBuiltMesh( ).box( Vector3Int.zero, new Vector3Int( 5, 3, 4 ) );
        var first  = new Remeshed( built, settingsOf( faces, 0.7 ) );
        var second = new Remeshed( built, settingsOf( faces, 0.7 ) );

        Assert.Greater( first.triangles.Length, 0, "triangles" );

        CollectionAssert.AreEqual( first.positions, second.positions, "positions" );
        CollectionAssert.AreEqual( first.normals,   second.normals,   "normals" );
        CollectionAssert.AreEqual( first.triangles, second.triangles, "triangles" );
        CollectionAssert.AreEqual( first.flags,     second.flags,     "flags" );
    }

}
