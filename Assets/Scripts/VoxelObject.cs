using System;
using UnityEngine;
using UnityEngine.Rendering;

public class VoxelObject<Voxelizer, Model> : MonoBehaviour
    where Voxelizer : SurfaceExtractor, new( )
    where Model     : DensityFunction,  new( )
{

    protected SurfaceExtractor surfaceExtractor;
    protected DensityFunction  densityFunction;

    public int resolution = 1;

    [Flags]
    public enum Debug {
        Off           = 0,
        Cells         = 1,
        Corners       = 2,
        Edges         = 4,
        Intersections = 8,
        Minimizers    = 16
    }

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

    public VoxelObject( ) {
        this.surfaceExtractor = new Voxelizer( );
        this.densityFunction  = new Model( );
    }

    void Start( ) {
        var ( positions, normals, indices ) = this.surfaceExtractor.voxelize( this.densityFunction, this.resolution );

        var meshRenderer          = this.gameObject.AddComponent<MeshRenderer>( );
            meshRenderer.material = new Material( Shader.Find( "Standard" ) );

        var meshFilter = this.gameObject.AddComponent<MeshFilter>( );

        meshFilter.mesh = new Mesh {
            vertices  = positions,
            normals   = normals,
            triangles = indices
        };

        if( this.debugFlags != Debug.Off ) {
            var debugObject                  = new GameObject( "Debug" );
                debugObject.transform.parent = this.gameObject.transform;

            var cells = this.surfaceExtractor.getGridCells( );

            var cellCount = 0;
            foreach( var cell in cells ) {
                var cellObject                      = new GameObject( $"Cell {++cellCount}" );
                    cellObject.transform.parent     = debugObject.transform;
                    cellObject.transform.position   = cell.center;
                    cellObject.transform.localScale = cell.size;

                if( this.debugCellMaterial ) {
                    var cellMeshRenderer          = cellObject.AddComponent<MeshRenderer>( );
                        cellMeshRenderer.material = this.debugCellMaterial;

                    cellMeshRenderer.enabled = ( this.debugFlags & Debug.Cells ) == Debug.Cells;
                }
                if( this.debugCellMesh ) {
                    var cellMeshFilter      = cellObject.AddComponent<MeshFilter>( );
                        cellMeshFilter.mesh = this.debugCellMesh;
                }

                var cornersObject                  = new GameObject( $"Corners" );
                    cornersObject.transform.parent = cellObject.transform;

                var cornerCount = 0;
                foreach( var corner in cell.corners ) {
                    var cornerObject                      = new GameObject( $"Corner {++cornerCount}" );
                        cornerObject.transform.parent     = cornersObject.transform;
                        cornerObject.transform.position   = corner.position;
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

                var edgesObject                  = new GameObject( $"Edges" );
                    edgesObject.transform.parent = cellObject.transform;

                var edgeCount = 0;
                foreach( var edge in cell.edges ) {
                    var edgeObject                  = new GameObject( $"Edge {++edgeCount}" );
                        edgeObject.transform.parent = edgesObject.transform;

                    if(
                        this.debugEdgeMaterial || (
                            this.debugEdgeIntersectionMaterial && edge.corners.Item1.sign != edge.corners.Item2.sign
                        )
                    ) {
                        var edgeLineRenderer = edgeObject.AddComponent<LineRenderer>( );

                        edgeLineRenderer.startWidth                = this.debugEdgeSize;
                        edgeLineRenderer.endWidth                  = this.debugEdgeSize;
                        edgeLineRenderer.numCapVertices            = 0;
                        edgeLineRenderer.shadowCastingMode         = ShadowCastingMode.Off;
                        edgeLineRenderer.receiveShadows            = false;
                        edgeLineRenderer.material                  = edge.corners.Item1.sign != edge.corners.Item2.sign ? this.debugEdgeIntersectionMaterial : this.debugEdgeMaterial;
                        edgeLineRenderer.useWorldSpace             = false;
                        edgeLineRenderer.startColor                = Color.white;
                        edgeLineRenderer.endColor                  = Color.white;
                        edgeLineRenderer.allowOcclusionWhenDynamic = false;

                        edgeLineRenderer.SetPositions( new Vector3[] { edge.corners.Item1.position, edge.corners.Item2.position } );

                        edgeLineRenderer.enabled = edge.corners.Item1.sign != edge.corners.Item2.sign
                            ? ( this.debugFlags & Debug.Intersections ) == Debug.Intersections
                            : ( this.debugFlags & Debug.Edges         ) == Debug.Edges;
                    }
                }

                if( cell.index > -1 ) {
                    var minimizingVertexObject                      = new GameObject( $"Minimizing Vertex" );
                        minimizingVertexObject.transform.parent     = cellObject.transform;
                        minimizingVertexObject.transform.position   = cell.vertex;
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

                        minimizingVertexLineRenderer.SetPositions( new Vector3[] { cell.vertex, cell.vertex + cell.normal } );

                        minimizingVertexLineRenderer.enabled = ( this.debugFlags & Debug.Minimizers ) == Debug.Minimizers;
                    }
                }
            }

            debugObject.transform.localScale = Vector3.one;
        }
    }

}