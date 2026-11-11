using System;
using System.Linq;
using NUnit.Framework;
using UnityEngine;

using CPUVoxelization = SurfaceExtractor.Implementation.CPU.Voxelization;

// how the contour of a Burst implementation is held to that of the managed one. Both go through the cells or nodes in
// the same order and create the vertices of the mesh in the order polygons first use them, so their meshes are the
// same triangle for triangle and their debug information lists the same voxels in the same order. Vertices and hermite
// data differ in the last bits: Mono keeps the intermediate results of an expression at a higher precision than Burst
public static class BurstParity {

    // in cells of the finest grid, for vertices and intersections. A vertex goes through the inverse of the
    // eigenvalues of its QEF, which magnifies the last bits of the hermite data more than the hermite data itself, and
    // the vertex of a collapsed octree node has the QEFs of all the leaves below it added up. The farthest apart vertices
    // seen were those of collapsed nodes, 0.002 leaves apart on an octree of 32^3 leaves
    public const float vertexTolerance       = 5e-3f;
    public const float intersectionTolerance = 1e-4f;
    public const float normalTolerance       = 1e-3f;

    // how many of the deviations are past the tolerance, and the largest of them
    public static void assertWithin( string what, float[] deviations, float tolerance ) {
        var past = deviations.Count( ( deviation ) => !( deviation <= tolerance ) );

        Assert.AreEqual( 0, past, $"{past} of {deviations.Length} {what} are more than {tolerance} apart, the farthest by {( deviations.Length > 0 ? deviations.Max( ) : 0.0f )}" );
    }

    // cell is the size of a cell of the finest grid
    public static void assertSameContour( CPUVoxelization managed, CPUVoxelization burst, float cell ) {
        // the voxels, in the same order, with a vertex in the same ones and in the same place
        var managedVoxels = managed.voxels.ToArray( );
        var burstVoxels   = burst.voxels.ToArray( );

        Assert.AreEqual( managedVoxels.Length, burstVoxels.Length, "voxels" );

        var misplaced = Enumerable.Range( 0, managedVoxels.Length ).Where(
            ( index ) => managedVoxels[index].center != burstVoxels[index].center || managedVoxels[index].size != burstVoxels[index].size || managedVoxels[index].type != burstVoxels[index].type
        ).ToArray( );

        Assert.IsEmpty( misplaced, "voxels that are not the same, in the same order" );

        var disagreeing = Enumerable.Range( 0, managedVoxels.Length ).Where(
            ( index ) => managedVoxels[index].hasFeaturePoint != burstVoxels[index].hasFeaturePoint
        ).Select(
            ( index ) => managedVoxels[index].center
        ).ToArray( );

        Assert.IsEmpty( disagreeing, "voxels that have a vertex in one implementation only" );

        var withVertex = Enumerable.Range( 0, managedVoxels.Length ).Where( ( index ) => managedVoxels[index].hasFeaturePoint ).ToArray( );

        Assert.IsNotEmpty( withVertex, "no voxel has a vertex" );

        assertWithin( "vertices of voxels, in cells", withVertex.Select( ( index ) => Vector3.Distance( managedVoxels[index].vertex, burstVoxels[index].vertex ) / cell ).ToArray( ), vertexTolerance );
        assertWithin( "normals of voxels",            withVertex.Select( ( index ) => Vector3.Distance( managedVoxels[index].normal, burstVoxels[index].normal ) ).ToArray( ),        normalTolerance );

        // the hermite data of the contour, on every edge of the voxels. Edges across a material boundary are left out:
        // each managed voxel has edges of its own, and the managed implementations show the edge of the first voxel it
        // is in, which has the hermite data of the boundary only if the contour passes through that voxel
        var managedEdges = managed.edges.ToDictionary( ( edge ) => ( edge.corners[0].position, edge.corners[1].position ) );
        var burstEdges   = burst.edges.ToArray( );

        Assert.AreEqual( managedEdges.Count, burstEdges.Length, "edges" );

        var pairs = burstEdges.Select( ( edge ) => ( expected: managedEdges[( edge.corners[0].position, edge.corners[1].position )], actual: edge ) ).ToArray( );

        Assert.IsEmpty(
            pairs.Where( ( pair ) => pair.expected.intersectsContour( ) != pair.actual.intersectsContour( ) ).Select( ( pair ) => pair.actual.corners[0].position ).ToArray( ),
            "edges that cross the contour in one implementation only"
        );

        var crossing = pairs.Where( ( pair ) => pair.expected.intersectsContour( ) ).ToArray( );

        Assert.IsNotEmpty( crossing, "no edge crosses the contour" );

        assertWithin( "intersections, in cells", crossing.Select( ( pair ) => Vector3.Distance( pair.expected.intersection, pair.actual.intersection ) / cell ).ToArray( ), intersectionTolerance );
        assertWithin( "hermite normals",         crossing.Select( ( pair ) => Vector3.Distance( pair.expected.normal,       pair.actual.normal ) ).ToArray( ),              normalTolerance );

        Assert.AreEqual( managed.corners.Count( ), burst.corners.Count( ), "corners" );

        // the mesh, triangle for triangle
        Assert.AreEqual( managed.mesh.subMeshCount, burst.mesh.subMeshCount, "sub-meshes" );
        Assert.AreEqual( managed.mesh.vertexCount,  burst.mesh.vertexCount,  "vertices" );

        for( var subMesh = 0; subMesh < managed.mesh.subMeshCount; ++subMesh ) {
            CollectionAssert.AreEqual( managed.mesh.GetTriangles( subMesh ), burst.mesh.GetTriangles( subMesh ), $"triangles of sub-mesh {subMesh}" );
        }

        assertWithin( "vertices of the mesh, in cells", managed.mesh.vertices.Zip( burst.mesh.vertices, ( expected, actual ) => Vector3.Distance( expected, actual ) / cell ).ToArray( ), vertexTolerance );
        assertWithin( "normals of the mesh",            managed.mesh.normals.Zip( burst.mesh.normals,   ( expected, actual ) => Vector3.Distance( expected, actual ) ).ToArray( ),        normalTolerance );
    }

    // that a mesh is closed and every edge is shared by two triangles that agree on which side faces out: every edge
    // runs once in each direction. MeshTopology takes far longer than a contour on the hundreds of thousands of
    // triangles of the largest grids, so the edges are sorted instead
    public static void assertClosedAndOriented( Mesh mesh ) {
        var triangles = mesh.triangles;

        Assert.Greater( triangles.Length, 0, "the mesh is empty" );

        var directed = new long[triangles.Length];
        for( var triangle = 0; triangle < triangles.Length; triangle += 3 ) {
            for( var corner = 0; corner < 3; ++corner ) {
                long start = triangles[triangle + corner];
                long end   = triangles[triangle + ( ( corner + 1 ) % 3 )];

                directed[triangle + corner] = ( start << 32 ) | end;
            }
        }

        var reversed = directed.Select( ( edge ) => ( ( edge & 0xFFFFFFFFL ) << 32 ) | ( edge >> 32 ) ).ToArray( );

        Array.Sort( directed );
        Array.Sort( reversed );

        var repeated = 0;
        var unpaired = 0;
        for( var edge = 0; edge < directed.Length; ++edge ) {
            if( edge > 0 && directed[edge] == directed[edge - 1] ) {
                ++repeated;
            }
            if( directed[edge] != reversed[edge] ) {
                ++unpaired;
            }
        }

        Assert.AreEqual( 0, repeated, "edges that two triangles run along in the same direction" );
        Assert.AreEqual( 0, unpaired, "edges that no triangle runs along in the other direction" );
    }

}
