using System.Linq;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.Rendering;

using Object = UnityEngine.Object;

// the meshes that every implementation builds its contour into, and the split vertices of flat shading
public class MeshTest {

    // a strip of quads along x, two triangles each, that share their vertices with the quads next to them
    private static ( Vector3[] vertices, int[] indices ) strip( int quads ) {
        var vertices = new Vector3[( quads + 1 ) * 2];

        for( var column = 0; column <= quads; ++column ) {
            vertices[( column * 2 ) + 0] = new Vector3( column, 0.0f, 0.0f );
            vertices[( column * 2 ) + 1] = new Vector3( column, 1.0f, 0.0f );
        }

        var indices = new int[quads * 6];

        for( var quad = 0; quad < quads; ++quad ) {
            var corner = quad * 2;

            indices[( quad * 6 ) + 0] = corner;
            indices[( quad * 6 ) + 1] = corner + 1;
            indices[( quad * 6 ) + 2] = corner + 2;
            indices[( quad * 6 ) + 3] = corner + 2;
            indices[( quad * 6 ) + 4] = corner + 1;
            indices[( quad * 6 ) + 5] = corner + 3;
        }

        return ( vertices, indices );
    }

    private static Mesh createMesh( Vector3[] vertices, params ( int subMeshIndex, int[] indices )[] polygons ) {
        return SurfaceExtractor.createMesh( vertices, Enumerable.Repeat( Vector3.back, vertices.Length ).ToArray( ), polygons );
    }

    [Test]
    public void polygonsGoInTheSubMeshOfTheirMaterial( ) {
        var ( vertices, indices ) = strip( 2 );

        var first  = indices.Take( 6 ).ToArray( );
        var second = indices.Skip( 6 ).ToArray( );

        var mesh = createMesh( vertices, ( 2, first ), ( 0, second ) );

        try {
            Assert.AreEqual( 3, mesh.subMeshCount, "sub-meshes, up to the highest material" );

            CollectionAssert.AreEqual( second,          mesh.GetTriangles( 0 ), "first material" );
            CollectionAssert.AreEqual( new int[0],      mesh.GetTriangles( 1 ), "second material, which no polygon has" );
            CollectionAssert.AreEqual( first,           mesh.GetTriangles( 2 ), "third material" );
        }
        finally {
            Object.DestroyImmediate( mesh );
        }
    }

    [Test]
    public void meshUpTo65535VerticesHas16BitIndices( ) {
        var ( vertices, indices ) = strip( 32766 ); // 65,534 vertices

        var mesh = createMesh( vertices, ( 0, indices ) );

        try {
            Assert.AreEqual( IndexFormat.UInt16, mesh.indexFormat );
            Assert.AreEqual( vertices.Length,    mesh.vertexCount );
        }
        finally {
            Object.DestroyImmediate( mesh );
        }
    }

    [Test]
    public void meshPast65535VerticesHas32BitIndices( ) {
        var ( vertices, indices ) = strip( 40000 ); // 80,002 vertices

        var mesh = createMesh( vertices, ( 0, indices ) );

        try {
            Assert.AreEqual( IndexFormat.UInt32, mesh.indexFormat );
            Assert.AreEqual( vertices.Length,    mesh.vertexCount );

            CollectionAssert.AreEqual( indices, mesh.GetTriangles( 0 ), "indices, which go past 65,535" );
        }
        finally {
            Object.DestroyImmediate( mesh );
        }
    }

    // every corner of every triangle becomes a vertex of its own, in the same place, and the triangles of each sub-mesh
    // stay in that sub-mesh
    private static void assertSplit( Vector3[] vertices, ( int subMeshIndex, int[] indices )[] polygons, Mesh mesh ) {
        Assert.AreEqual( polygons.Sum( ( polygon ) => polygon.indices.Length ), mesh.vertexCount, "vertices, one for every corner of every triangle" );

        var split = mesh.vertices;
        var used  = new bool[split.Length];

        foreach( var ( subMeshIndex, indices ) in polygons ) {
            var triangles = mesh.GetTriangles( subMeshIndex );

            Assert.AreEqual( indices.Length, triangles.Length, $"indices of sub-mesh {subMeshIndex}" );

            for( var index = 0; index < indices.Length; ++index ) {
                Assert.IsFalse( used[triangles[index]], "a vertex used by two corners" );

                used[triangles[index]] = true;

                Assert.AreEqual( vertices[indices[index]], split[triangles[index]], "a corner that moved" );
            }
        }
    }

    [Test]
    public void splitVerticesGivesEveryCornerAVertexOfItsOwn( ) {
        var ( vertices, indices ) = strip( 3 );

        var polygons = new[] { ( 0, indices.Take( 12 ).ToArray( ) ), ( 1, indices.Skip( 12 ).ToArray( ) ) };

        var mesh = createMesh( vertices, polygons );

        try {
            Voxelizer.splitVertices( mesh );

            assertSplit( vertices, polygons, mesh );

            // the strip is flat, so the normal of every vertex is the normal of the strip
            foreach( var normal in mesh.normals ) {
                Assert.Less( Vector3.Distance( Vector3.back, normal ), 1e-6f, "the normal of a vertex is not that of its face" );
            }
        }
        finally {
            Object.DestroyImmediate( mesh );
        }
    }

    // with several materials, one of them nowhere on the surface. Unity empties a mesh of its sub-meshes when its index
    // format changes, so a split that only kept the first sub-mesh would lose the triangles of every other material
    [Test]
    public void splitVerticesPast65535VerticesHave32BitIndices( ) {
        var ( vertices, indices ) = strip( 12000 ); // 24,002 shared vertices, 72,000 once they are split

        var polygons = new[] { ( 0, indices.Take( 48000 ).ToArray( ) ), ( 1, new int[0] ), ( 2, indices.Skip( 48000 ).ToArray( ) ) };

        var mesh = createMesh( vertices, polygons );

        try {
            Assert.AreEqual( IndexFormat.UInt16, mesh.indexFormat, "before splitting" );

            Voxelizer.splitVertices( mesh );

            Assert.AreEqual( IndexFormat.UInt32, mesh.indexFormat, "after splitting" );
            Assert.AreEqual( 3,                  mesh.subMeshCount, "sub-meshes after splitting" );

            assertSplit( vertices, polygons, mesh );
        }
        finally {
            Object.DestroyImmediate( mesh );
        }
    }

}
