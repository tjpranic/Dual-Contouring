using System.Linq;
using NUnit.Framework;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;

using Node                 = Octree<AdaptiveDualContouring.Voxel>;
using MaterialIndex        = SurfaceExtractor.MaterialIndex;
using VoxelType            = SurfaceExtractor.Voxel.Type;
using CornerMaterials      = BurstOctreeTopology.CornerMaterials;
using ChildCornerMaterials = BurstOctreeTopology.ChildCornerMaterials;

// BurstOctreeTopology against OctreeTopology: the quasi-manifold test on every cell of empty space and two materials and
// on random cells of more, and the collapse test on nodes whose 3 x 3 x 3 points are cut by planes or set at random
public class BurstOctreeTopologyTest {

    [BurstCompile( CompileSynchronously = true )]
    private struct QuasiManifoldJob : IJob {

        [ReadOnly] public NativeArray<CornerMaterials> cells;

        public NativeArray<bool> results;
        public NativeArray<bool> managed;

        public void Execute( ) {
            this.managed[0] = BurstTestSupport.runningManaged( );

            for( var cell = 0; cell < this.cells.Length; ++cell ) {
                this.results[cell] = BurstOctreeTopology.isQuasiManifold( this.cells[cell] );
            }
        }

    }

    [BurstCompile( CompileSynchronously = true )]
    private struct CollapseJob : IJob {

        [ReadOnly] public NativeArray<ChildCornerMaterials> nodes;

        public NativeArray<bool> results;
        public NativeArray<bool> managed;

        public void Execute( ) {
            this.managed[0] = BurstTestSupport.runningManaged( );

            for( var node = 0; node < this.nodes.Length; ++node ) {
                this.results[node] = BurstOctreeTopology.collapsePreservesTopology( this.nodes[node] );
            }
        }

    }

    private static CornerMaterials toCell( MaterialIndex[] corners ) {
        var cell = new CornerMaterials( );
        for( var corner = 0; corner < 8; ++corner ) {
            cell[corner] = corners[corner];
        }
        return cell;
    }

    [Test]
    public void quasiManifoldCellsAreTheManagedOnes( ) {
        var values = new[] { MaterialIndex.Void, MaterialIndex.Material0, MaterialIndex.Material1 };

        // every cell of empty space and two materials, 3^8 of them
        var cells = Enumerable.Range( 0, 6561 ).Select(
            ( code ) => Enumerable.Range( 0, 8 ).Select( ( corner ) => values[( code / ( int )Mathf.Pow( 3, corner ) ) % 3] ).ToArray( )
        ).ToList( );

        // and random cells of the materials of the other tests, which give some corners two materials at once
        var random = new System.Random( 5 );
        for( var cell = 0; cell < 5000; ++cell ) {
            cells.Add( Enumerable.Range( 0, 8 ).Select( ( _ ) => BurstTestSupport.materials[random.Next( BurstTestSupport.materials.Length )] ).ToArray( ) );
        }

        using var nativeCells = new NativeArray<CornerMaterials>( cells.Select( toCell ).ToArray( ), Allocator.TempJob );
        using var results     = new NativeArray<bool>( cells.Count, Allocator.TempJob );
        using var managed     = new NativeArray<bool>( 1, Allocator.TempJob );

        new QuasiManifoldJob { cells = nativeCells, results = results, managed = managed }.Run( );

        BurstTestSupport.assertRanUnderBurst( managed );

        var expected = cells.Select( OctreeTopology.isQuasiManifold ).ToArray( );

        CollectionAssert.AreEqual( expected, results.ToArray( ) );

        Assert.Greater( expected.Count( ( result ) => result ),  0, "no cell was quasi-manifold" );
        Assert.Greater( expected.Count( ( result ) => !result ), 0, "every cell was quasi-manifold" );
    }

    [Test]
    public void collapsesThatPreserveTopologyAreTheManagedOnes( ) {
        var random = new System.Random( 6 );

        var nodes    = new ChildCornerMaterials[4000];
        var expected = new bool[nodes.Length];

        for( var index = 0; index < nodes.Length; ++index ) {
            // the material at each point of the 3 x 3 x 3 grid of a node, in steps of half the node from its minimum.
            // Half of the nodes are cut by one or two planes into regions of different materials, which tend to pass
            // the test, the other half are set at random and tend not to
            var grid = new MaterialIndex[3, 3, 3];

            var planes = Enumerable.Range( 0, 1 + random.Next( 2 ) ).Select(
                ( _ ) => ( normal: BurstTestSupport.direction( random ), offset: BurstTestSupport.range( random, -1.0f, 1.0f ), material: BurstTestSupport.materials[1 + random.Next( BurstTestSupport.materials.Length - 1 )] )
            ).ToArray( );

            for( var x = 0; x < 3; ++x ) {
                for( var y = 0; y < 3; ++y ) {
                    for( var z = 0; z < 3; ++z ) {
                        if( index % 2 == 0 ) {
                            var point = new Vector3( x - 1, y - 1, z - 1 );

                            grid[x, y, z] = planes.Where( ( plane ) => Vector3.Dot( plane.normal, point ) < plane.offset ).Select( ( plane ) => plane.material ).LastOrDefault( );
                        }
                        else {
                            grid[x, y, z] = BurstTestSupport.materials[random.Next( BurstTestSupport.materials.Length )];
                        }
                    }
                }
            }

            // corner o of the child at corner c of the node is halfway between corners c and o of the node
            MaterialIndex between( int corner, int other ) {
                return grid[
                    BurstTables.lookupCornerOffset( corner, 0 ) + BurstTables.lookupCornerOffset( other, 0 ),
                    BurstTables.lookupCornerOffset( corner, 1 ) + BurstTables.lookupCornerOffset( other, 1 ),
                    BurstTables.lookupCornerOffset( corner, 2 ) + BurstTables.lookupCornerOffset( other, 2 )
                ];
            }

            var node = Node.build(
                new AdaptiveDualContouring.Voxel( VoxelType.Internal, 0, Vector3.zero, Vector3.one ),
                ( parent ) => parent.depth == 1 ? null : Enumerable.Range( 0, 8 ).Select(
                    ( _ ) => new AdaptiveDualContouring.Voxel( VoxelType.Leaf, 1, Vector3.zero, Vector3.one )
                ).ToArray( )
            );

            for( var corner = 0; corner < 8; ++corner ) {
                node.data.corners[corner].materialIndex = between( corner, corner );

                for( var other = 0; other < 8; ++other ) {
                    node.children[corner].data.corners[other].materialIndex = between( corner, other );

                    nodes[index][corner, other] = between( corner, other );
                }
            }

            expected[index] = OctreeTopology.collapsePreservesTopology( node );
        }

        using var nativeNodes = new NativeArray<ChildCornerMaterials>( nodes, Allocator.TempJob );
        using var results     = new NativeArray<bool>( nodes.Length, Allocator.TempJob );
        using var managed     = new NativeArray<bool>( 1, Allocator.TempJob );

        new CollapseJob { nodes = nativeNodes, results = results, managed = managed }.Run( );

        BurstTestSupport.assertRanUnderBurst( managed );

        CollectionAssert.AreEqual( expected, results.ToArray( ) );

        Assert.Greater( expected.Count( ( result ) => result ),  100, "few nodes could collapse" );
        Assert.Greater( expected.Count( ( result ) => !result ), 100, "few nodes could not collapse" );
    }

}
