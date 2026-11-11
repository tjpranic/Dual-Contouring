using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;

// the mesh of a contour that Burst code built, laid out as SurfaceExtractor.createMesh lays out the meshes of the
// managed implementations: sub-mesh n holds the triangles on the surface of material bit n in the order they came in,
// a material that is not on the surface leaves its sub-mesh empty, and past 65,535 vertices the indices are 32 bit
public static class BurstMesh {

    // a triangle of the contour and the sub-mesh it goes in
    public struct Triangle {

        public int a;
        public int b;
        public int c;
        public int subMeshIndex;

        public Triangle( int a, int b, int c, int subMeshIndex ) {
            this.a            = a;
            this.b            = b;
            this.c            = c;
            this.subMeshIndex = subMeshIndex;
        }

    }

    public static IndexFormat indexFormatOf( int vertexCount ) {
        return vertexCount > ushort.MaxValue ? IndexFormat.UInt32 : IndexFormat.UInt16;
    }

    // writes the contour into the mesh data. Positions go in vertex stream 0 and normals in stream 1
    [BurstCompile( CompileSynchronously = true, FloatMode = FloatMode.Strict, FloatPrecision = FloatPrecision.Standard )]
    public struct WriteMeshJob : IJob {

        [ReadOnly] public NativeArray<float3>   vertices;
        [ReadOnly] public NativeArray<float3>   normals;
        [ReadOnly] public NativeArray<Triangle> triangles;

        public Mesh.MeshData mesh;

        public void Execute( ) {
            var attributes = new NativeArray<VertexAttributeDescriptor>( 2, Allocator.Temp );

            attributes[0] = new VertexAttributeDescriptor( VertexAttribute.Position, VertexAttributeFormat.Float32, 3, 0 );
            attributes[1] = new VertexAttributeDescriptor( VertexAttribute.Normal,   VertexAttributeFormat.Float32, 3, 1 );

            this.mesh.SetVertexBufferParams( this.vertices.Length, attributes );

            attributes.Dispose( );

            this.mesh.GetVertexData<float3>( 0 ).CopyFrom( this.vertices );
            this.mesh.GetVertexData<float3>( 1 ).CopyFrom( this.normals );

            var indexFormat = indexFormatOf( this.vertices.Length );

            this.mesh.SetIndexBufferParams( this.triangles.Length * 3, indexFormat );

            // the sub-meshes go up to the highest material on the surface
            var subMeshCount = 0;
            for( var triangle = 0; triangle < this.triangles.Length; ++triangle ) {
                subMeshCount = math.max( subMeshCount, this.triangles[triangle].subMeshIndex + 1 );
            }

            // where the indices of each sub-mesh start, one after the other in the order of the sub-meshes
            var starts = new NativeArray<int>( subMeshCount + 1, Allocator.Temp );
            for( var triangle = 0; triangle < this.triangles.Length; ++triangle ) {
                starts[this.triangles[triangle].subMeshIndex + 1] += 3;
            }
            for( var subMeshIndex = 0; subMeshIndex < subMeshCount; ++subMeshIndex ) {
                starts[subMeshIndex + 1] += starts[subMeshIndex];
            }

            var next = new NativeArray<int>( subMeshCount, Allocator.Temp );
            NativeArray<int>.Copy( starts, next, subMeshCount );

            if( indexFormat == IndexFormat.UInt32 ) {
                var indices = this.mesh.GetIndexData<uint>( );

                for( var triangle = 0; triangle < this.triangles.Length; ++triangle ) {
                    var t     = this.triangles[triangle];
                    var index = next[t.subMeshIndex];

                    indices[index + 0] = ( uint )t.a;
                    indices[index + 1] = ( uint )t.b;
                    indices[index + 2] = ( uint )t.c;

                    next[t.subMeshIndex] = index + 3;
                }
            }
            else {
                var indices = this.mesh.GetIndexData<ushort>( );

                for( var triangle = 0; triangle < this.triangles.Length; ++triangle ) {
                    var t     = this.triangles[triangle];
                    var index = next[t.subMeshIndex];

                    indices[index + 0] = ( ushort )t.a;
                    indices[index + 1] = ( ushort )t.b;
                    indices[index + 2] = ( ushort )t.c;

                    next[t.subMeshIndex] = index + 3;
                }
            }

            // the bounds are worked out once the mesh is complete, on the main thread
            this.mesh.subMeshCount = subMeshCount;
            for( var subMeshIndex = 0; subMeshIndex < subMeshCount; ++subMeshIndex ) {
                this.mesh.SetSubMesh(
                    subMeshIndex,
                    new SubMeshDescriptor( starts[subMeshIndex], starts[subMeshIndex + 1] - starts[subMeshIndex], MeshTopology.Triangles ),
                    MeshUpdateFlags.DontRecalculateBounds | MeshUpdateFlags.DontValidateIndices
                );
            }

            starts.Dispose( );
            next.Dispose( );
        }

    }

    // builds the mesh, on the calling thread
    public static Mesh createMesh( NativeArray<float3> vertices, NativeArray<float3> normals, NativeArray<Triangle> triangles ) {
        var meshes = Mesh.AllocateWritableMeshData( 1 );

        new WriteMeshJob {
            vertices  = vertices,
            normals   = normals,
            triangles = triangles,
            mesh      = meshes[0]
        }.Run( );

        var mesh = new Mesh( );

        Mesh.ApplyAndDisposeWritableMeshData( meshes, mesh );

        mesh.RecalculateBounds( );

        return mesh;
    }

}
