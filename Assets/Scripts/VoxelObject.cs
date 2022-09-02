using System;
using UnityEngine;
using UnityEngine.Rendering;

public class VoxelObject<Voxelizer, Model> : MonoBehaviour
    where Voxelizer : SurfaceExtractor, new( )
    where Model     : DensityFunction,  new( )
{

    private SurfaceExtractor surfaceExtractor;
    private DensityFunction  densityFunction;

    public enum VertexMode {
        Shared,
        Split
    }

    public int        resolution = 1;
    public VertexMode vertexMode = VertexMode.Shared;
    public Material   material;

    [Flags]
    public enum Debug {
        Off           = 0,
        Cells         = 1,
        Corners       = 2,
        Edges         = 4,
        Intersections = 8,
        Minimizers    = 16
    }

    [Space]

    public Debug    debugFlags                = Debug.Off;
    public float    debugCornerSize           = 0.1f;
    public float    debugEdgeSize             = 0.1f;
    public float    debugMinimizingVertexSize = 0.1f;
    public float    debugSurfaceNormalSize    = 0.1f;
    public Mesh     debugCellMesh;
    public Material debugCellMaterial;
    public Mesh     debugCornerMesh;
    public Material debugCornerMaterial;
    public Material debugEdgeMaterial;
    public Material debugEdgeIntersectionMaterial;
    public Material debugMinimizingVertexMaterial;
    public Material debugSurfaceNormalMaterial;

    void Start( ) {
        this.surfaceExtractor = new Voxelizer( );
        this.densityFunction  = new Model( );

        var ( positions, normals, indices ) = this.surfaceExtractor.voxelize( this.densityFunction, this.resolution );

        var meshRenderer          = this.gameObject.AddComponent<MeshRenderer>( );
            meshRenderer.material = this.material ? this.material : new Material( Shader.Find( "Standard" ) );

        var meshFilter = this.gameObject.AddComponent<MeshFilter>( );

        meshFilter.mesh = new Mesh {
            vertices  = positions,
            normals   = normals,
            triangles = indices
        };

        if( this.vertexMode == VertexMode.Split ) {
            // convert to flat shading by splitting vertices
            var triangles = meshFilter.mesh.triangles;
            var vertices  = new Vector3[triangles.Length];
            for( var triangleIndex = 0; triangleIndex < triangles.Length; ++triangleIndex ) {
                vertices[triangleIndex]  = meshFilter.mesh.vertices[triangles[triangleIndex]];
                triangles[triangleIndex] = triangleIndex;
            }
            meshFilter.mesh.vertices  = vertices;
            meshFilter.mesh.triangles = triangles;

            meshFilter.mesh.RecalculateBounds( );
            meshFilter.mesh.RecalculateNormals( );
        }

        meshFilter.mesh.OptimizeReorderVertexBuffer( );

        if( this.debugFlags != Debug.Off ) {
            var debugObject                  = new GameObject( "Debug" );
                debugObject.transform.parent = this.gameObject.transform;

            var voxels = this.surfaceExtractor.getVoxels( );

            var voxelsObject                  = new GameObject( $"Voxels" );
                voxelsObject.transform.parent = debugObject.transform;

            var voxelCount = 0;
            foreach( var voxel in voxels ) {
                var voxelObject                      = new GameObject( $"Voxel {++voxelCount}" );
                    voxelObject.transform.parent     = voxelsObject.transform;
                    voxelObject.transform.position   = voxel.getCenter( );
                    voxelObject.transform.localScale = voxel.getSize( );

                if( this.debugCellMaterial ) {
                    var cellMeshRenderer          = voxelObject.AddComponent<MeshRenderer>( );
                        cellMeshRenderer.material = this.debugCellMaterial;

                    cellMeshRenderer.enabled = ( this.debugFlags & Debug.Cells ) == Debug.Cells;
                }
                if( this.debugCellMesh ) {
                    var cellMeshFilter      = voxelObject.AddComponent<MeshFilter>( );
                        cellMeshFilter.mesh = this.debugCellMesh;
                }

                if( voxel.intersectsIsosurface( ) ) {
                    var minimizingVertexObject                      = new GameObject( $"Minimizing Vertex" );
                        minimizingVertexObject.transform.parent     = voxelObject.transform;
                        minimizingVertexObject.transform.position   = voxel.getVertex( );
                        minimizingVertexObject.transform.localScale = new Vector3( this.debugMinimizingVertexSize, this.debugMinimizingVertexSize, this.debugMinimizingVertexSize );

                    if( this.debugMinimizingVertexMaterial ) {
                        var minimizingVertexMeshRenderer          = minimizingVertexObject.AddComponent<MeshRenderer>( );
                            minimizingVertexMeshRenderer.material = this.debugMinimizingVertexMaterial;

                        minimizingVertexMeshRenderer.enabled = ( this.debugFlags & Debug.Minimizers ) == Debug.Minimizers;
                    }
                    if( this.debugCornerMesh ) {
                        var minimizingVertexMeshFilter      = minimizingVertexObject.AddComponent<MeshFilter>( );
                            minimizingVertexMeshFilter.mesh = this.debugCornerMesh;
                    }
                    if( this.debugSurfaceNormalMaterial ) {
                        var minimizingVertexLineRenderer = minimizingVertexObject.AddComponent<LineRenderer>( );

                        minimizingVertexLineRenderer.startWidth                = this.debugSurfaceNormalSize;
                        minimizingVertexLineRenderer.endWidth                  = this.debugSurfaceNormalSize;
                        minimizingVertexLineRenderer.numCapVertices            = 0;
                        minimizingVertexLineRenderer.shadowCastingMode         = ShadowCastingMode.Off;
                        minimizingVertexLineRenderer.receiveShadows            = false;
                        minimizingVertexLineRenderer.material                  = this.debugSurfaceNormalMaterial;
                        minimizingVertexLineRenderer.useWorldSpace             = false;
                        minimizingVertexLineRenderer.startColor                = Color.white;
                        minimizingVertexLineRenderer.endColor                  = Color.white;
                        minimizingVertexLineRenderer.allowOcclusionWhenDynamic = false;

                        minimizingVertexLineRenderer.SetPositions( new Vector3[] { voxel.getVertex( ), voxel.getVertex( ) + voxel.getNormal( ) } );

                        minimizingVertexLineRenderer.enabled = ( this.debugFlags & Debug.Minimizers ) == Debug.Minimizers;
                    }
                }

            }

            var corners = this.surfaceExtractor.getCorners( );

            var cornersObject                  = new GameObject( $"Corners" );
                cornersObject.transform.parent = debugObject.transform;

            var cornerCount = 0;
            foreach( var corner in corners ) {
                var cornerObject                      = new GameObject( $"Corner {++cornerCount}" );
                    cornerObject.transform.parent     = cornersObject.transform;
                    cornerObject.transform.position   = corner.getPosition( );
                    cornerObject.transform.localScale = new Vector3( this.debugCornerSize, this.debugCornerSize, this.debugCornerSize );

                if( this.debugCornerMaterial ) {
                    var cornerMeshRenderer                           = cornerObject.AddComponent<MeshRenderer>( );
                        cornerMeshRenderer.material                  = this.debugCornerMaterial;
                        cornerMeshRenderer.shadowCastingMode         = ShadowCastingMode.Off;
                        cornerMeshRenderer.receiveShadows            = false;
                        cornerMeshRenderer.lightProbeUsage           = LightProbeUsage.Off;
                        cornerMeshRenderer.reflectionProbeUsage      = ReflectionProbeUsage.Off;
                        cornerMeshRenderer.allowOcclusionWhenDynamic = false;

                    cornerMeshRenderer.enabled = ( this.debugFlags & Debug.Corners ) == Debug.Corners;
                }
                if( this.debugCornerMesh ) {
                    var cornerMeshFilter      = cornerObject.AddComponent<MeshFilter>( );
                        cornerMeshFilter.mesh = this.debugCornerMesh;
                }
            }

            var edges = this.surfaceExtractor.getEdges( );

            var edgesObject                  = new GameObject( $"Edges" );
                edgesObject.transform.parent = debugObject.transform;

            var edgeCount = 0;
            foreach( var edge in edges ) {
                var edgeCorners = edge.getCorners( );

                var edgeObject                  = new GameObject( $"Edge {++edgeCount}" );
                    edgeObject.transform.parent = edgesObject.transform;

                if(
                    this.debugEdgeMaterial || (
                        this.debugEdgeIntersectionMaterial && edgeCorners[0].getSign( ) != edgeCorners[1].getSign( )
                    )
                ) {
                    var edgeLineRenderer = edgeObject.AddComponent<LineRenderer>( );

                    edgeLineRenderer.startWidth                = this.debugEdgeSize;
                    edgeLineRenderer.endWidth                  = this.debugEdgeSize;
                    edgeLineRenderer.numCapVertices            = 0;
                    edgeLineRenderer.shadowCastingMode         = ShadowCastingMode.Off;
                    edgeLineRenderer.receiveShadows            = false;
                    edgeLineRenderer.material                  = edgeCorners[0].getSign( ) != edgeCorners[1].getSign( ) ? this.debugEdgeIntersectionMaterial : this.debugEdgeMaterial;
                    edgeLineRenderer.useWorldSpace             = false;
                    edgeLineRenderer.startColor                = Color.white;
                    edgeLineRenderer.endColor                  = Color.white;
                    edgeLineRenderer.allowOcclusionWhenDynamic = false;

                    edgeLineRenderer.SetPositions( new Vector3[] { edgeCorners[0].getPosition( ), edgeCorners[1].getPosition( ) } );

                    edgeLineRenderer.enabled = edgeCorners[0].getSign( ) != edgeCorners[1].getSign( )
                        ? ( this.debugFlags & Debug.Intersections ) == Debug.Intersections
                        : ( this.debugFlags & Debug.Edges         ) == Debug.Edges;
                }
            }

            debugObject.transform.localScale = Vector3.one;

        }

    }

}