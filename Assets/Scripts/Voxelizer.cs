using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;

[RequireComponent( typeof( MeshRenderer ) )]
public abstract class Voxelizer : MonoBehaviour, SurfaceExtractor {

    [SerializeField( )]
    private int _resolution = 2;
    public int resolution {
        get { return this._resolution;  }
        set { this._resolution = value; }
    }

    [SerializeField( )]
    private int _minimizerIterations = 6;
    public int minimizerIterations {
        get { return this._minimizerIterations;  }
        set { this._minimizerIterations = value; }
    }

    [SerializeField( )]
    private int _binarySearchIterations = 6;
    public int binarySearchIterations {
        get { return this._binarySearchIterations;  }
        set { this._binarySearchIterations = value; }
    }

    [SerializeField( )]
    private int _surfaceCorrectionIterations = 6;
    public int surfaceCorrectionIterations {
        get { return this._surfaceCorrectionIterations;  }
        set { this._surfaceCorrectionIterations = value; }
    }

    public enum VertexMode {
        Shared,
        Split
    }

    public VertexMode vertexMode = VertexMode.Shared;

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
    public float    debugCornerSize           = 0.0125f;
    public float    debugEdgeSize             = 0.025f;
    public float    debugMinimizingVertexSize = 0.1f;
    public float    debugSurfaceNormalSize    = 0.025f;
    public Mesh     debugCellMesh;
    public Material debugCellMaterial;
    public Mesh     debugCornerMesh;
    public Material debugCornerMaterial;
    public Material debugEdgeMaterial;
    public Material debugEdgeIntersectionMaterial;
    public Material debugMinimizingVertexMaterial;
    public Material debugSurfaceNormalMaterial;

    public virtual void Start( ) {

        var volumes = this.GetComponentsInChildren<Volume>( );

        this.voxelize( volumes );

        var meshFilter = this.gameObject.AddComponent<MeshFilter>( );

        meshFilter.mesh = this.mesh;

        // convert to flat shading by splitting vertices
        if( this.vertexMode == VertexMode.Split ) {

            var vertices  = new Vector3[this.mesh.triangles.Length];
            var triangles = new Dictionary<int, int[]>( );

            var subMeshTriangleIndexStart = 0;
            for( var subMeshIndex = 0; subMeshIndex < this.mesh.subMeshCount; ++subMeshIndex ) {
                var subMeshTriangles = meshFilter.mesh.GetTriangles( subMeshIndex );
                for( var subMeshTriangleIndex = subMeshTriangleIndexStart; subMeshTriangleIndex < subMeshTriangleIndexStart + subMeshTriangles.Length; ++subMeshTriangleIndex ) {
                    var triangleIndex = subMeshTriangleIndex - subMeshTriangleIndexStart;

                    vertices[subMeshTriangleIndex]  = meshFilter.mesh.vertices[subMeshTriangles[triangleIndex]];
                    subMeshTriangles[triangleIndex] = subMeshTriangleIndex;
                }
                triangles.Add( subMeshIndex, subMeshTriangles );
                subMeshTriangleIndexStart += subMeshTriangles.Length;
            }

            meshFilter.mesh.vertices = vertices;
            for( var subMeshIndex = 0; subMeshIndex < this.mesh.subMeshCount; ++subMeshIndex ) {
                meshFilter.mesh.SetTriangles( triangles[subMeshIndex], subMeshIndex );
            }

            meshFilter.mesh.RecalculateBounds( );
            meshFilter.mesh.RecalculateNormals( );
        }

        meshFilter.mesh.OptimizeReorderVertexBuffer( );

        // debug rendering
        if( this.debugFlags != Debug.Off ) {
            var debugObject                  = new GameObject( "Debug" );
                debugObject.transform.parent = this.gameObject.transform;

            var voxelsObject                  = new GameObject( $"Voxels" );
                voxelsObject.transform.parent = debugObject.transform;

            var voxelCount = 0;
            foreach( var voxel in this.voxels ) {
                var voxelObject                      = new GameObject( $"Voxel {++voxelCount}" );
                    voxelObject.transform.parent     = voxelsObject.transform;
                    voxelObject.transform.position   = voxel.center;
                    voxelObject.transform.localScale = voxel.size;

                if( this.debugCellMaterial ) {
                    var cellMeshRenderer          = voxelObject.AddComponent<MeshRenderer>( );
                        cellMeshRenderer.material = this.debugCellMaterial;

                    cellMeshRenderer.enabled = ( this.debugFlags & Debug.Cells ) == Debug.Cells;
                }
                if( this.debugCellMesh ) {
                    var cellMeshFilter      = voxelObject.AddComponent<MeshFilter>( );
                        cellMeshFilter.mesh = this.debugCellMesh;
                }

                if( voxel.hasFeaturePoint( ) ) {
                    var minimizingVertexObject                      = new GameObject( $"Minimizing Vertex" );
                        minimizingVertexObject.transform.parent     = voxelObject.transform;
                        minimizingVertexObject.transform.position   = voxel.vertex;
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

                        minimizingVertexLineRenderer.SetPositions( new Vector3[] { voxel.vertex, voxel.vertex + voxel.normal } );

                        minimizingVertexLineRenderer.enabled = ( this.debugFlags & Debug.Minimizers ) == Debug.Minimizers;
                    }
                }

            }

            var cornersObject                  = new GameObject( $"Corners" );
                cornersObject.transform.parent = debugObject.transform;

            var cornerCount = 0;
            foreach( var corner in this.corners ) {
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
                edgesObject.transform.parent = debugObject.transform;

            var edgeCount = 0;
            foreach( var edge in this.edges ) {
                var edgeObject                  = new GameObject( $"Edge {++edgeCount}" );
                    edgeObject.transform.parent = edgesObject.transform;

                if(
                    this.debugEdgeMaterial || (
                        this.debugEdgeIntersectionMaterial && edge.intersectsContour( )
                    )
                ) {
                    var edgeLineRenderer = edgeObject.AddComponent<LineRenderer>( );

                    edgeLineRenderer.startWidth                = this.debugEdgeSize;
                    edgeLineRenderer.endWidth                  = this.debugEdgeSize;
                    edgeLineRenderer.numCapVertices            = 0;
                    edgeLineRenderer.shadowCastingMode         = ShadowCastingMode.Off;
                    edgeLineRenderer.receiveShadows            = false;
                    edgeLineRenderer.material                  = edge.intersectsContour( ) ? this.debugEdgeIntersectionMaterial : this.debugEdgeMaterial;
                    edgeLineRenderer.useWorldSpace             = false;
                    edgeLineRenderer.startColor                = Color.white;
                    edgeLineRenderer.endColor                  = Color.white;
                    edgeLineRenderer.allowOcclusionWhenDynamic = false;

                    edgeLineRenderer.SetPositions( new Vector3[] { edge.corners[0].position, edge.corners[1].position } );

                    edgeLineRenderer.enabled = edge.intersectsContour( )
                        ? ( this.debugFlags & Debug.Intersections ) == Debug.Intersections
                        : ( this.debugFlags & Debug.Edges         ) == Debug.Edges;
                }
            }

            debugObject.transform.localScale = Vector3.one;

        }

    }

    public abstract SurfaceExtractor.Implementation            implementation            { get; set; }
    public abstract SurfaceExtractor.IntersectionApproximation intersectionApproximation { get; set; }
    public abstract QEFSolver.Type                             qefSolver                 { get; set; }

    public abstract Mesh                                 mesh    { get; }
    public abstract IEnumerable<SurfaceExtractor.Corner> corners { get; }
    public abstract IEnumerable<SurfaceExtractor.Edge>   edges   { get; }
    public abstract IEnumerable<SurfaceExtractor.Voxel>  voxels  { get; }

    public abstract void voxelize( IEnumerable<DensityFunction> densityFunctions );

}

[CustomEditor( typeof( Voxelizer ) )]
public class VoxelizerEditor : Editor {

    public override void OnInspectorGUI( ) {
        base.OnInspectorGUI( );

        var voxelizer = this.target as Voxelizer;

        // only SVD solver is available for GPU implementation
        if( voxelizer.implementation == SurfaceExtractor.Implementation.GPU ) {
            voxelizer.qefSolver = QEFSolver.Type.SVD;
        }
    }

}