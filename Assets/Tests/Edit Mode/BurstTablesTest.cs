using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;

using Tables        = OctreeContouringTables<AdaptiveDualContouring.Voxel>;
using Axis          = OctreeContouringTables<AdaptiveDualContouring.Voxel>.Axis;
using Position      = OctreeContouringTables<AdaptiveDualContouring.Voxel>.Position;
using Node          = Octree<AdaptiveDualContouring.Voxel>;
using MaterialIndex = SurfaceExtractor.MaterialIndex;
using VoxelType     = SurfaceExtractor.Voxel.Type;

// the constant tables of the Burst code against the managed tables and layouts they were taken from, read from inside a
// job so that they are known to work in Burst compiled code as well
public class BurstTablesTest {

    // every table, entry by entry, in the order the managed lookups below go through them
    [BurstCompile( CompileSynchronously = true )]
    private struct ReadTablesJob : IJob {

        public NativeList<int>   entries;
        public NativeArray<bool> managed;

        public void Execute( ) {
            this.managed[0] = BurstTestSupport.runningManaged( );

            for( var corner = 0; corner < 8; ++corner ) {
                for( var axis = 0; axis < 3; ++axis ) {
                    this.entries.Add( BurstTables.lookupCornerOffset( corner, axis ) );
                }
            }
            for( var edge = 0; edge < 12; ++edge ) {
                for( var end = 0; end < 2; ++end ) {
                    this.entries.Add( BurstTables.lookupEdgeCorner( edge, end ) );
                }
            }
            for( var solidCorners = 0; solidCorners < 256; ++solidCorners ) {
                this.entries.Add( BurstTables.lookupVertexCount( solidCorners ) );
                for( var edge = 0; edge < 12; ++edge ) {
                    this.entries.Add( BurstTables.lookupEdgeVertex( solidCorners, edge ) );
                }
            }
            for( var axis = 0; axis < 3; ++axis ) {
                for( var group = 0; group < 2; ++group ) {
                    for( var place = 0; place < 4; ++place ) {
                        this.entries.Add( BurstTables.lookupEdgeNodeWithinNode( axis, group, place ) );
                    }
                }
                for( var pair = 0; pair < 4; ++pair ) {
                    for( var side = 0; side < 2; ++side ) {
                        this.entries.Add( BurstTables.lookupFacePairWithinNode( axis, pair, side ) );
                    }
                }
                for( var pair = 0; pair < 4; ++pair ) {
                    for( var side = 0; side < 2; ++side ) {
                        this.entries.Add( BurstTables.lookupFacePairWithinFacePair( axis, pair, side ) );
                    }
                }
                for( var edge = 0; edge < 4; ++edge ) {
                    for( var place = 0; place < 4; ++place ) {
                        this.entries.Add( BurstTables.lookupEdgeNodeWithinFacePairNode( axis, edge, place ) );
                        this.entries.Add( BurstTables.lookupEdgeNodeWithinFacePairChild( axis, edge, place ) );
                    }
                    this.entries.Add( BurstTables.lookupEdgeNodeWithinFacePairAxis( axis, edge ) );
                }
                for( var group = 0; group < 2; ++group ) {
                    for( var place = 0; place < 4; ++place ) {
                        this.entries.Add( BurstTables.lookupEdgeNodeWithinEdgeNodes( axis, group, place ) );
                    }
                }
                for( var place = 0; place < 4; ++place ) {
                    this.entries.Add( BurstTables.lookupEdgeIndexWithinEdgeNodes( axis, place ) );
                }
                for( var side = 0; side < 2; ++side ) {
                    for( var edge = 0; edge < 4; ++edge ) {
                        this.entries.Add( BurstTables.lookupEdgeIndexWithinFacePair( axis, side, edge ) );
                    }
                }
            }
            for( var face = 0; face < 6; ++face ) {
                this.entries.Add( BurstTables.lookupFaceAxis( face ) );
                this.entries.Add( BurstTables.lookupFaceSide( face ) );
            }
            for( var edge = 0; edge < 12; ++edge ) {
                for( var side = 0; side < 2; ++side ) {
                    this.entries.Add( BurstTables.lookupEdgeFace( edge, side ) );
                }
            }
            for( var solidCorners = 0; solidCorners < 256; ++solidCorners ) {
                this.entries.Add( BurstTables.lookupJoinedFace( solidCorners ) );
                this.entries.Add( BurstTables.complement( solidCorners ) );
                for( var across = 0; across < 256; ++across ) {
                    this.entries.Add( BurstTables.sharesJoinedFace( solidCorners, across ) ? 1 : 0 );
                }
                for( var vertex = 0; vertex < 4; ++vertex ) {
                    for( var other = 0; other < 4; ++other ) {
                        this.entries.Add( BurstTables.lookupSeparation( solidCorners, vertex, other, out var normal, out var twiceOffset ) ? 1 : 0 );
                        this.entries.Add( normal.x );
                        this.entries.Add( normal.y );
                        this.entries.Add( normal.z );
                        this.entries.Add( twiceOffset );
                    }
                }
                for( var edge = 0; edge < 12; ++edge ) {
                    for( var side = 0; side < 2; ++side ) {
                        this.entries.Add( BurstTables.lookupEdgeAcrossFace( solidCorners, edge, side ) );
                    }
                }
            }
        }

    }

    // the same entries from the managed code: the corner and edge layouts of a voxel, DualMarchingCubes, and the
    // octree contouring tables applied to an octree whose nodes can be told apart
    private static List<int> managedEntries( ) {
        var entries = new List<int>( );

        // a unit cell with its minimum at the origin puts every corner at its offset
        var cell = new AdaptiveDualContouring.Voxel( VoxelType.Leaf, 0, Vector3.one * 0.5f, Vector3.one );

        foreach( var corner in cell.corners ) {
            entries.Add( ( int )corner.position.x );
            entries.Add( ( int )corner.position.y );
            entries.Add( ( int )corner.position.z );
        }
        foreach( var edge in cell.edges ) {
            entries.Add( Array.IndexOf( cell.corners, edge.corners[0] ) );
            entries.Add( Array.IndexOf( cell.corners, edge.corners[1] ) );
        }

        for( var solidCorners = 0; solidCorners < 256; ++solidCorners ) {
            var vertices = DualMarchingCubes.lookupVertexEdges( solidCorners );

            entries.Add( vertices.Length );
            for( var edge = 0; edge < 12; ++edge ) {
                entries.Add( Array.FindIndex( vertices, ( edges ) => edges.Contains( edge ) ) );
            }
        }

        var octree = Node.build(
            new AdaptiveDualContouring.Voxel( VoxelType.Internal, 0, Vector3.zero, Vector3.one ),
            ( parent ) => parent.depth == 2 ? null : Enumerable.Range( 0, 8 ).Select(
                ( _ ) => new AdaptiveDualContouring.Voxel( parent.depth < 1 ? VoxelType.Internal : VoxelType.Leaf, parent.depth + 1, Vector3.zero, Vector3.one )
            ).ToArray( )
        );

        int childOf( Node parent, Node node ) {
            var index = Array.IndexOf( parent.children, node );
            Assert.GreaterOrEqual( index, 0, "a node that is not a child of the node it should be" );
            return index;
        }

        // which of the given nodes the given one is a child of
        int nodeOf( Node[] nodes, Node node ) {
            var index = Array.FindIndex( nodes, ( candidate ) => candidate.children.Contains( node ) );
            Assert.GreaterOrEqual( index, 0, "a node that is not a child of any of the given nodes" );
            return index;
        }

        var two  = new[] { octree.children[0], octree.children[1] };
        var four = new[] { octree.children[0], octree.children[1], octree.children[2], octree.children[3] };

        foreach( var axis in new[] { Axis.X, Axis.Y, Axis.Z } ) {
            foreach( var group in Tables.lookupEdgeNodesWithinNode( octree, axis, Position.Root ) ) {
                entries.AddRange( group.Select( ( node ) => childOf( octree, node ) ) );
            }
            foreach( var pair in Tables.lookupFacePairsWithinNode( octree, axis, Position.Root ) ) {
                entries.AddRange( pair.Select( ( node ) => childOf( octree, node ) ) );
            }
            foreach( var pair in Tables.lookupFacePairsWithinFacePairs( two, axis, Position.Root ) ) {
                Assert.AreEqual( 0, nodeOf( two, pair[0] ), "the first of a face pair comes from the first node" );
                Assert.AreEqual( 1, nodeOf( two, pair[1] ), "the second of a face pair comes from the second node" );

                entries.Add( childOf( two[0], pair[0] ) );
                entries.Add( childOf( two[1], pair[1] ) );
            }
            foreach( var ( group, newAxis ) in Tables.lookupEdgeNodesWithinFacePairs( two, axis, Position.Root ) ) {
                foreach( var node in group ) {
                    var index = nodeOf( two, node );

                    entries.Add( index );
                    entries.Add( childOf( two[index], node ) );
                }
                entries.Add( ( int )newAxis );
            }
            foreach( var group in Tables.lookupEdgeNodesWithinEdgeNodes( four, axis, Position.Root ) ) {
                for( var place = 0; place < 4; ++place ) {
                    Assert.AreEqual( place, nodeOf( four, group[place] ), "the n-th node around an edge comes from the n-th node" );

                    entries.Add( childOf( four[place], group[place] ) );
                }
            }
            for( var place = 0; place < 4; ++place ) {
                entries.Add( Tables.lookupEdgeIndexWithinEdgeNodes( place, axis, Position.Root ) );
            }
            for( var side = 0; side < 2; ++side ) {
                entries.AddRange( Tables.lookupEdgeIndicesWithinFacePair( side, axis, Position.Root ) );
            }
        }

        for( var face = 0; face < 6; ++face ) {
            entries.Add( DualMarchingCubes.lookupFaceAxis( face ) );
            entries.Add( DualMarchingCubes.lookupFaceSide( face ) );
        }
        for( var edge = 0; edge < 12; ++edge ) {
            for( var side = 0; side < 2; ++side ) {
                entries.Add( DualMarchingCubes.lookupEdgeFace( edge, side ) );
            }
        }
        for( var solidCorners = 0; solidCorners < 256; ++solidCorners ) {
            entries.Add( DualMarchingCubes.lookupJoinedFace( solidCorners ) );
            entries.Add( DualMarchingCubes.complement( solidCorners ) );
            for( var across = 0; across < 256; ++across ) {
                entries.Add( DualMarchingCubes.sharesJoinedFace( solidCorners, across ) ? 1 : 0 );
            }
            for( var vertex = 0; vertex < 4; ++vertex ) {
                for( var other = 0; other < 4; ++other ) {
                    entries.Add( DualMarchingCubes.lookupSeparation( solidCorners, vertex, other, out var normal, out var twiceOffset ) ? 1 : 0 );
                    entries.Add( normal.x );
                    entries.Add( normal.y );
                    entries.Add( normal.z );
                    entries.Add( twiceOffset );
                }
            }
            for( var edge = 0; edge < 12; ++edge ) {
                for( var side = 0; side < 2; ++side ) {
                    entries.Add( DualMarchingCubes.lookupEdgeAcrossFace( solidCorners, edge, side ) );
                }
            }
        }

        return entries;
    }

    [Test]
    public void tablesAreTheManagedOnes( ) {
        using var entries = new NativeList<int>( Allocator.TempJob );
        using var managed = new NativeArray<bool>( 1, Allocator.TempJob );

        new ReadTablesJob { entries = entries, managed = managed }.Run( );

        BurstTestSupport.assertRanUnderBurst( managed );

        CollectionAssert.AreEqual( managedEntries( ), entries.AsArray( ).ToArray( ) );
    }

    [Test]
    public void solidCornersAreTheBitsOfTheCornersThatAreNotEmpty( ) {
        var random = new System.Random( 4 );

        for( var test = 0; test < 1000; ++test ) {
            var corners = Enumerable.Range( 0, 8 ).Select( ( _ ) => BurstTestSupport.materials[random.Next( BurstTestSupport.materials.Length )] ).ToArray( );

            var expected = Enumerable.Range( 0, 8 ).Sum( ( corner ) => corners[corner] != MaterialIndex.Void ? 1 << corner : 0 );

            Assert.AreEqual( expected, BurstTables.solidCornersOf( corners[0], corners[1], corners[2], corners[3], corners[4], corners[5], corners[6], corners[7] ) );
        }
    }

    // the faces of a cell, which the manifold criterion counts intersections around: each is the four edges that lie
    // in one of the six planes that bound the cell
    [Test]
    public void faceEdgesAreTheEdgesInEachPlaneOfACell( ) {
        var planes = new HashSet<( int axis, int side )>( );

        for( var face = 0; face < 6; ++face ) {
            var edges   = Enumerable.Range( 0, 4 ).Select( ( index ) => BurstTables.lookupFaceEdge( face, index ) ).ToArray( );
            var corners = edges.SelectMany( ( edge ) => new[] { BurstTables.lookupEdgeCorner( edge, 0 ), BurstTables.lookupEdgeCorner( edge, 1 ) } ).Distinct( ).ToArray( );

            Assert.AreEqual( 4, edges.Distinct( ).Count( ), $"edges of face {face}" );
            Assert.AreEqual( 4, corners.Length, $"corners of face {face}" );

            // the axis along which all four corners are on the same side
            var shared = Enumerable.Range( 0, 3 ).Where(
                ( axis ) => corners.Select( ( corner ) => BurstTables.lookupCornerOffset( corner, axis ) ).Distinct( ).Count( ) == 1
            ).ToArray( );

            Assert.AreEqual( 1, shared.Length, $"planes the corners of face {face} lie in" );

            planes.Add( ( shared[0], BurstTables.lookupCornerOffset( corners[0], shared[0] ) ) );

            Assert.AreEqual( shared[0], BurstTables.lookupFaceAxis( face ), $"axis of face {face}" );
            Assert.AreEqual( BurstTables.lookupCornerOffset( corners[0], shared[0] ), BurstTables.lookupFaceSide( face ), $"side of face {face}" );

            foreach( var edge in edges ) {
                Assert.IsTrue( BurstTables.lookupEdgeFace( edge, 0 ) == face || BurstTables.lookupEdgeFace( edge, 1 ) == face, $"edge {edge} lies in face {face}" );
            }
        }

        Assert.AreEqual( 6, planes.Count, "faces" );
    }

    [Test]
    public void edgesShareACornerWhereTheyMeet( ) {
        for( var edge = 0; edge < 12; ++edge ) {
            for( var other = 0; other < 12; ++other ) {
                var ends      = new[] { BurstTables.lookupEdgeCorner( edge,  0 ), BurstTables.lookupEdgeCorner( edge,  1 ) };
                var otherEnds = new[] { BurstTables.lookupEdgeCorner( other, 0 ), BurstTables.lookupEdgeCorner( other, 1 ) };

                Assert.AreEqual( ends.Intersect( otherEnds ).Any( ), BurstTables.edgesShareCorner( edge, other ), $"edges {edge} and {other}" );
            }
        }
    }

}
