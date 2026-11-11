using System.Linq;
using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

using MaterialIndex = SurfaceExtractor.MaterialIndex;
using VoxelType     = SurfaceExtractor.Voxel.Type;
using CornerData    = BurstDebugInformation.CornerData;
using EdgeData      = BurstDebugInformation.EdgeData;
using VoxelData     = BurstDebugInformation.VoxelData;

// the records Burst code fills in become the corners, edges and voxels that the debug view and the tests look at
public class BurstDebugInformationTest {

    // a cell with its minimum at the origin, of size 1 / 4, with one corner solid, shown twice as two cells holding
    // a vertex each would be
    private static ( BurstDebugInformation.Corner[] corners, BurstDebugInformation.Edge[] edges, BurstDebugInformation.Voxel[] voxels ) convertCell( ) {
        const float size = 0.25f;

        var corners = new CornerData[8];
        var edges   = new EdgeData[12];
        var voxels  = new VoxelData[2];

        for( var corner = 0; corner < 8; ++corner ) {
            corners[corner] = new CornerData {
                position      = new float3( BurstTables.lookupCornerOffset( corner, 0 ), BurstTables.lookupCornerOffset( corner, 1 ), BurstTables.lookupCornerOffset( corner, 2 ) ) * size,
                density       = corner == 0 ? -0.1f : 0.1f,
                materialIndex = corner == 0 ? MaterialIndex.Material1 : MaterialIndex.Void
            };
        }

        for( var edge = 0; edge < 12; ++edge ) {
            edges[edge] = new EdgeData {
                corner0      = BurstTables.lookupEdgeCorner( edge, 0 ),
                corner1      = BurstTables.lookupEdgeCorner( edge, 1 ),
                intersection = new float3( edge, 0.0f, 0.0f ),
                normal       = new float3( 0.0f, edge, 0.0f )
            };
        }

        for( var voxel = 0; voxel < 2; ++voxel ) {
            var data = new VoxelData {
                type            = VoxelType.Leaf,
                center          = new float3( size / 2 ),
                size            = new float3( size ),
                vertex          = new float3( voxel, 0.01f, 0.02f ),
                normal          = new float3( 0.0f, 0.0f, 1.0f ),
                hasFeaturePoint = true
            };

            for( var corner = 0; corner < 8; ++corner ) {
                data.setCorner( corner, corner );
            }
            for( var edge = 0; edge < 12; ++edge ) {
                data.setEdge( edge, edge );
            }

            voxels[voxel] = data;
        }

        using var nativeCorners = new NativeArray<CornerData>( corners, Allocator.Temp );
        using var nativeEdges   = new NativeArray<EdgeData>( edges, Allocator.Temp );
        using var nativeVoxels  = new NativeArray<VoxelData>( voxels, Allocator.Temp );

        return BurstDebugInformation.convert( nativeCorners, nativeEdges, nativeVoxels );
    }

    [Test]
    public void voxelsAreLaidOutLikeTheManagedOnes( ) {
        var ( corners, edges, voxels ) = convertCell( );

        // the managed voxel of the same cell
        var expected = new AdaptiveDualContouring.Voxel( VoxelType.Leaf, 0, Vector3.one * 0.125f, Vector3.one * 0.25f );

        foreach( var voxel in voxels ) {
            Assert.AreEqual( VoxelType.Leaf,   voxel.type );
            Assert.AreEqual( expected.center,  voxel.center );
            Assert.AreEqual( expected.size,    voxel.size );
            Assert.AreEqual( expected.extents, voxel.extents );
            Assert.AreEqual( expected.minimum, voxel.minimum );
            Assert.AreEqual( expected.maximum, voxel.maximum );
            Assert.IsTrue( voxel.hasFeaturePoint );

            CollectionAssert.AreEqual( expected.corners.Select( ( corner ) => corner.position ), voxel.corners.Select( ( corner ) => corner.position ), "corner layout" );

            for( var edge = 0; edge < 12; ++edge ) {
                Assert.AreEqual( expected.edges[edge].corners[0].position, voxel.edges[edge].corners[0].position, $"start of edge {edge}" );
                Assert.AreEqual( expected.edges[edge].corners[1].position, voxel.edges[edge].corners[1].position, $"end of edge {edge}" );
            }
        }

        Assert.AreEqual( new Vector3( 1.0f, 0.01f, 0.02f ), voxels[1].vertex );
        Assert.AreEqual( Vector3.forward,                   voxels[1].normal );
    }

    // a corner or an edge that two voxels share is one object, as it is in the lists of corners and edges
    [Test]
    public void sharedCornersAndEdgesAreTheSameObjects( ) {
        var ( corners, edges, voxels ) = convertCell( );

        for( var corner = 0; corner < 8; ++corner ) {
            Assert.AreSame( corners[corner], voxels[0].corners[corner] );
            Assert.AreSame( corners[corner], voxels[1].corners[corner] );
        }
        for( var edge = 0; edge < 12; ++edge ) {
            Assert.AreSame( edges[edge], voxels[0].edges[edge] );
            Assert.AreSame( edges[edge], voxels[1].edges[edge] );

            Assert.AreSame( corners[BurstTables.lookupEdgeCorner( edge, 0 )], edges[edge].corners[0] );
            Assert.AreSame( corners[BurstTables.lookupEdgeCorner( edge, 1 )], edges[edge].corners[1] );
        }
    }

    // two vertices of the same cell are two voxels, but the same corner is the same corner wherever it comes from
    [Test]
    public void voxelsAreTheSameOnlyAsThemselves( ) {
        var ( corners, _, voxels ) = convertCell( );
        var ( others, _, _ )       = convertCell( );

        Assert.IsTrue  ( voxels[0].Equals( voxels[0] ) );
        Assert.IsFalse ( voxels[0].Equals( voxels[1] ) );
        Assert.IsTrue  ( corners[3].Equals( others[3] ) );
        Assert.IsFalse ( corners[3].Equals( others[4] ) );
    }

    [Test]
    public void edgesFromTheSolidCornerCrossTheContour( ) {
        var ( _, edges, _ ) = convertCell( );

        var crossing = edges.Where( ( edge ) => edge.intersectsContour( ) ).Select( ( edge ) => System.Array.IndexOf( edges, edge ) ).ToArray( );

        // the three edges at corner 0
        CollectionAssert.AreEquivalent( new[] { 0, 4, 8 }, crossing );

        Assert.AreEqual( new Vector3( 4.0f, 0.0f, 0.0f ), edges[4].intersection );
        Assert.AreEqual( new Vector3( 0.0f, 4.0f, 0.0f ), edges[4].normal );
        Assert.AreEqual( -0.1f,                           edges[4].corners[0].density );
        Assert.AreEqual( MaterialIndex.Material1,         edges[4].corners[0].materialIndex );
    }

}
