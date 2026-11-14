using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using UnityEngine;

// compares meshes by their triangles: the positions and normals of the corners of each, sub-mesh by sub-mesh, in
// sorted order. The GPU appends vertices and quads in whatever order its threads get there, and reordering the vertex
// buffer moves vertices around, so only this stays the same where the surface does
public static class MeshComparison {

    public static List<string> trianglesOf( Mesh mesh ) {
        var vertices  = mesh.vertices;
        var normals   = mesh.normals;
        var triangles = new List<string>( );

        string corner( int index ) {
            var p = vertices[index];
            var n = normals[index];
            return $"{p.x:R},{p.y:R},{p.z:R}/{n.x:R},{n.y:R},{n.z:R}";
        }

        for( var subMesh = 0; subMesh < mesh.subMeshCount; ++subMesh ) {
            var indices   = mesh.GetTriangles( subMesh );
            var inSubMesh = new List<string>( );

            for( var index = 0; index < indices.Length; index += 3 ) {
                inSubMesh.Add( $"{subMesh}: {corner( indices[index] )} {corner( indices[index + 1] )} {corner( indices[index + 2] )}" );
            }

            inSubMesh.Sort( StringComparer.Ordinal );
            triangles.AddRange( inSubMesh );
        }

        return triangles;
    }

    public static void assertSameTriangles( Mesh expected, Mesh actual ) {
        Assert.AreEqual( expected.subMeshCount, actual.subMeshCount, "sub-meshes" );

        var expectedTriangles = trianglesOf( expected );
        var actualTriangles   = trianglesOf( actual );

        Assert.AreEqual( expectedTriangles.Count, actualTriangles.Count, "triangles" );

        for( var triangle = 0; triangle < expectedTriangles.Count; ++triangle ) {
            if( expectedTriangles[triangle] != actualTriangles[triangle] ) {
                Assert.Fail( $"triangle {triangle} differs: expected {expectedTriangles[triangle]}, got {actualTriangles[triangle]}" );
            }
        }
    }

    // a new mesh of the triangles of a mesh, with its vertices at one position made one, which keeps the normal of the
    // first of them: a creased mesh as it was before it was creased. The contours have no two vertices at one position,
    // which step 0 of the mesh simplification plan counted
    public static Mesh welded( Mesh mesh ) {
        var vertices      = mesh.vertices;
        var normals       = mesh.normals;
        var indices       = new Dictionary<Vector3, int>( );
        var places        = new int[vertices.Length];
        var kept          = new List<Vector3>( );
        var normalsOfKept = new List<Vector3>( );

        for( var vertex = 0; vertex < vertices.Length; ++vertex ) {
            if( !indices.TryGetValue( vertices[vertex], out var place ) ) {
                place = kept.Count;

                indices.Add( vertices[vertex], place );
                kept.Add( vertices[vertex] );
                normalsOfKept.Add( normals[vertex] );
            }
            places[vertex] = place;
        }

        var result = new Mesh { indexFormat = mesh.indexFormat };

        result.SetVertices( kept );
        result.SetNormals( normalsOfKept );
        result.subMeshCount = mesh.subMeshCount;

        for( var subMesh = 0; subMesh < mesh.subMeshCount; ++subMesh ) {
            result.SetTriangles( mesh.GetTriangles( subMesh ).Select( ( index ) => places[index] ).ToArray( ), subMesh );
        }

        return result;
    }

}
