using System.Linq;
using UnityEngine;

public partial interface SurfaceExtractor {

    // the mesh of a contour that is still on the GPU, as every GPU implementation leaves it: a counter of vertices and
    // one of quads, vertices with their normals, and quads of two triangles each. Reads the two counters first, then
    // only as many vertices, normals and quads as the contour wrote
    public static Mesh readMesh( Implementation.GPU.Voxelization voxelization ) {
        var countsData = new int[2];

        voxelization.counts.GetData( countsData );

        var verticesData = new Vector3[countsData[0]];
        var normalsData  = new Vector3[countsData[0]];
        var quadsData    = new UniformDualContouring.Quad.Data[countsData[1]];

        if( countsData[0] > 0 ) {
            voxelization.vertices.GetData ( verticesData, 0, 0, countsData[0] );
            voxelization.normals.GetData  ( normalsData,  0, 0, countsData[0] );
        }
        if( countsData[1] > 0 ) {
            voxelization.quads.GetData( quadsData, 0, 0, countsData[1] );
        }

        var quads = quadsData.Select( ( quadData ) => new UniformDualContouring.Quad( quadData ) );

        return createMesh(
            verticesData,
            normalsData,
            quads.Select( ( quad ) => ( quad.subMeshIndex, quad.indices ) )
        );
    }

}
