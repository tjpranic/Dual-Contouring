using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

using MaterialIndex = SurfaceExtractor.MaterialIndex;
using VoxelType     = SurfaceExtractor.Voxel.Type;

// the corners, edges and voxels of a contour that Burst code built, for the debug view and the tests. Burst code fills in
// records that refer to each other by index. They are copied into objects only when someone asks for them: that is when
// debug drawing is on or a test looks at them, and costs nothing otherwise
public static class BurstDebugInformation {

    public struct CornerData {

        public float3        position;
        public float         density;
        public MaterialIndex materialIndex;

    }

    // an edge runs from its first corner to its second, from the minimum to the maximum of the axis it runs along
    public struct EdgeData {

        public int    corner0;
        public int    corner1;
        public float3 intersection;
        public float3 normal;

    }

    // the corners of a voxel in the order of the corner layout, its edges in the order of the edge layout
    public unsafe struct VoxelData {

        public VoxelType type;
        public float3    center;
        public float3    size;
        public float3    vertex;
        public float3    normal;
        public bool      hasFeaturePoint;

        private fixed int corners[SurfaceExtractor.Voxel.CornerCount];
        private fixed int edges[SurfaceExtractor.Voxel.EdgeCount];

        public int corner( int index ) {
            return this.corners[index];
        }

        public void setCorner( int index, int corner ) {
            this.corners[index] = corner;
        }

        public int edge( int index ) {
            return this.edges[index];
        }

        public void setEdge( int index, int edge ) {
            this.edges[index] = edge;
        }

    }

    public class Corner : SurfaceExtractor.Corner {

        public Vector3       position      { get; }
        public float         density       { get; set; }
        public MaterialIndex materialIndex { get; set; }

        public Corner( CornerData data ) {
            this.position      = data.position;
            this.density       = data.density;
            this.materialIndex = data.materialIndex;
        }

        public bool Equals( SurfaceExtractor.Corner other ) {
            return this.position.Equals( other.position );
        }

        public override int GetHashCode( ) {
            return SurfaceExtractor.hashOf( this.position );
        }

    }

    public class Edge : SurfaceExtractor.Edge {

        public SurfaceExtractor.Corner[] corners      { get; }
        public Vector3                   intersection { get; set; }
        public Vector3                   normal       { get; set; }

        public Edge( EdgeData data, Corner[] corners ) {
            this.corners      = new SurfaceExtractor.Corner[] { corners[data.corner0], corners[data.corner1] };
            this.intersection = data.intersection;
            this.normal       = data.normal;
        }

        public bool intersectsContour( ) {
            return BurstSurfaceExtractor.intersectsContour( this.corners[0].materialIndex, this.corners[1].materialIndex );
        }

        public bool Equals( SurfaceExtractor.Edge other ) {
            return this.corners[0].Equals( other.corners[0] ) && this.corners[1].Equals( other.corners[1] );
        }

        // the corners in order, as Equals compares them. The XOR of their hashes would cancel out the coordinates the
        // two corners of an edge along an axis have in common
        public override int GetHashCode( ) {
            return System.HashCode.Combine( this.corners[0].GetHashCode( ), this.corners[1].GetHashCode( ) );
        }

    }

    // a cell, or one of the vertices of a cell with several: two voxels are the same only if they are the same object
    public class Voxel : SurfaceExtractor.Voxel {

        public VoxelType                 type            { get; set; }
        public Vector3                   center          { get; }
        public Vector3                   size            { get; }
        public Vector3                   extents         { get; }
        public Vector3                   minimum         { get; }
        public Vector3                   maximum         { get; }
        public Vector3                   vertex          { get; }
        public Vector3                   normal          { get; }
        public SurfaceExtractor.Corner[] corners         { get; }
        public SurfaceExtractor.Edge[]   edges           { get; }
        public bool                      hasFeaturePoint { get; }

        public Voxel( VoxelData data, Corner[] corners, Edge[] edges ) {
            this.type    = data.type;
            this.center  = data.center;
            this.size    = data.size;
            this.extents = this.size / 2;
            this.minimum = this.center - this.extents;
            this.maximum = this.center + this.extents;
            this.vertex  = data.vertex;
            this.normal  = data.normal;

            this.hasFeaturePoint = data.hasFeaturePoint;

            this.corners = new SurfaceExtractor.Corner[SurfaceExtractor.Voxel.CornerCount];
            for( var corner = 0; corner < this.corners.Length; ++corner ) {
                this.corners[corner] = corners[data.corner( corner )];
            }

            this.edges = new SurfaceExtractor.Edge[SurfaceExtractor.Voxel.EdgeCount];
            for( var edge = 0; edge < this.edges.Length; ++edge ) {
                this.edges[edge] = edges[data.edge( edge )];
            }
        }

        public bool Equals( SurfaceExtractor.Voxel other ) {
            return ReferenceEquals( this, other );
        }

        public override int GetHashCode( ) {
            return this.center.GetHashCode( ) ^ this.size.GetHashCode( );
        }

    }

    // the objects of the records, where the corners of the edges and voxels, and the edges of the voxels, are the same
    // objects as those in the lists of corners and edges
    public static ( Corner[] corners, Edge[] edges, Voxel[] voxels ) convert(
        NativeArray<CornerData> cornerData,
        NativeArray<EdgeData>   edgeData,
        NativeArray<VoxelData>  voxelData
    ) {
        var corners = new Corner[cornerData.Length];
        for( var corner = 0; corner < corners.Length; ++corner ) {
            corners[corner] = new Corner( cornerData[corner] );
        }

        var edges = new Edge[edgeData.Length];
        for( var edge = 0; edge < edges.Length; ++edge ) {
            edges[edge] = new Edge( edgeData[edge], corners );
        }

        var voxels = new Voxel[voxelData.Length];
        for( var voxel = 0; voxel < voxels.Length; ++voxel ) {
            voxels[voxel] = new Voxel( voxelData[voxel], corners, edges );
        }

        return ( corners, edges, voxels );
    }

}
