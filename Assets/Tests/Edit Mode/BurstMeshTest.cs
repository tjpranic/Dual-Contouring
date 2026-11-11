using System.Linq;
using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;

using Object   = UnityEngine.Object;
using Triangle = BurstMesh.Triangle;

// BurstMesh builds the same mesh from the same triangles as SurfaceExtractor.createMesh: the same vertices, normals,
// index format, sub-meshes and bounds
public class BurstMeshTest {

    private static void assertSameMesh( int vertexCount, int triangleCount, int subMeshes, int seed ) {
        var random = new System.Random( seed );

        var vertices  = Enumerable.Range( 0, vertexCount ).Select( ( _ ) => BurstTestSupport.range( random, -Vector3.one * 0.5f, Vector3.one * 0.5f ) ).ToArray( );
        var normals   = Enumerable.Range( 0, vertexCount ).Select( ( _ ) => BurstTestSupport.direction( random ) ).ToArray( );
        var triangles = Enumerable.Range( 0, triangleCount ).Select(
            ( _ ) => new Triangle( random.Next( vertexCount ), random.Next( vertexCount ), random.Next( vertexCount ), random.Next( subMeshes ) )
        ).ToArray( );

        using var nativeVertices  = new NativeArray<float3>( vertices.Select( ( v ) => ( float3 )v ).ToArray( ), Allocator.TempJob );
        using var nativeNormals   = new NativeArray<float3>( normals.Select( ( v ) => ( float3 )v ).ToArray( ), Allocator.TempJob );
        using var nativeTriangles = new NativeArray<Triangle>( triangles, Allocator.TempJob );

        var expected = SurfaceExtractor.createMesh( vertices, normals, triangles.Select( ( t ) => ( t.subMeshIndex, new[] { t.a, t.b, t.c } ) ) );
        var actual   = BurstMesh.createMesh( nativeVertices, nativeNormals, nativeTriangles );

        try {
            Assert.AreEqual( expected.indexFormat,  actual.indexFormat,  "index format" );
            Assert.AreEqual( expected.subMeshCount, actual.subMeshCount, "sub-meshes" );

            CollectionAssert.AreEqual( expected.vertices, actual.vertices, "vertices" );
            CollectionAssert.AreEqual( expected.normals,  actual.normals,  "normals" );

            for( var subMesh = 0; subMesh < expected.subMeshCount; ++subMesh ) {
                CollectionAssert.AreEqual( expected.GetTriangles( subMesh ), actual.GetTriangles( subMesh ), $"triangles of sub-mesh {subMesh}" );

                Assert.AreEqual( expected.GetSubMesh( subMesh ).bounds, actual.GetSubMesh( subMesh ).bounds, $"bounds of sub-mesh {subMesh}" );
            }

            Assert.AreEqual( expected.bounds, actual.bounds, "bounds" );
        }
        finally {
            Object.DestroyImmediate( expected );
            Object.DestroyImmediate( actual );
        }
    }

    [Test]
    public void meshOfSeveralMaterialsIsTheManagedOne( ) {
        assertSameMesh( 500, 1000, 4, 7 );
    }

    // with the triangles of only some materials, a material in between has an empty sub-mesh
    [Test]
    public void materialsThatAreNotThereHaveEmptySubMeshes( ) {
        var random = new System.Random( 8 );

        var vertices  = Enumerable.Range( 0, 30 ).Select( ( _ ) => BurstTestSupport.range( random, -Vector3.one, Vector3.one ) ).ToArray( );
        var triangles = new[] { new Triangle( 0, 1, 2, 0 ), new Triangle( 3, 4, 5, 2 ), new Triangle( 6, 7, 8, 0 ) };

        using var nativeVertices  = new NativeArray<float3>( vertices.Select( ( v ) => ( float3 )v ).ToArray( ), Allocator.TempJob );
        using var nativeNormals   = new NativeArray<float3>( vertices.Length, Allocator.TempJob );
        using var nativeTriangles = new NativeArray<Triangle>( triangles, Allocator.TempJob );

        var mesh = BurstMesh.createMesh( nativeVertices, nativeNormals, nativeTriangles );

        try {
            Assert.AreEqual( 3, mesh.subMeshCount );

            CollectionAssert.AreEqual( new[] { 0, 1, 2, 6, 7, 8 }, mesh.GetTriangles( 0 ) );
            CollectionAssert.AreEqual( new int[0],                 mesh.GetTriangles( 1 ) );
            CollectionAssert.AreEqual( new[] { 3, 4, 5 },          mesh.GetTriangles( 2 ) );
        }
        finally {
            Object.DestroyImmediate( mesh );
        }
    }

    [Test]
    public void meshPast65535VerticesIsTheManagedOne( ) {
        assertSameMesh( 80000, 30000, 2, 9 );
    }

    [Test]
    public void meshPast65535VerticesHas32BitIndices( ) {
        using var vertices  = new NativeArray<float3>( 70000, Allocator.TempJob );
        using var normals   = new NativeArray<float3>( 70000, Allocator.TempJob );
        using var triangles = new NativeArray<Triangle>( new[] { new Triangle( 0, 65536, 69999, 0 ) }, Allocator.TempJob );

        var mesh = BurstMesh.createMesh( vertices, normals, triangles );

        try {
            Assert.AreEqual( IndexFormat.UInt32, mesh.indexFormat );

            CollectionAssert.AreEqual( new[] { 0, 65536, 69999 }, mesh.GetTriangles( 0 ) );
        }
        finally {
            Object.DestroyImmediate( mesh );
        }
    }

    [Test]
    public void meshWithoutTrianglesIsTheManagedOne( ) {
        assertSameMesh( 10, 0, 1, 10 );
    }

}
