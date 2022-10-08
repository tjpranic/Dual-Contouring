using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;

using Implementation            = SurfaceExtractor.Implementation;
using ImplementationType        = SurfaceExtractor.Implementation.Type;
using CPUVoxelization           = SurfaceExtractor.Implementation.CPU.Voxelization;
using GPUVoxelization           = SurfaceExtractor.Implementation.GPU.Voxelization;
using IntersectionApproximation = SurfaceExtractor.IntersectionApproximation;
using VertexNormals             = SurfaceExtractor.VertexNormals;
using QEFSolverType             = QEFSolver.Type;

[RequireComponent( typeof( MeshRenderer ) )]
public abstract class Voxelizer : MonoBehaviour, SurfaceExtractor {

    [SerializeField( ), Range( 2, 8 )]
    private int _resolution = 2;
    public int resolution {
        get { return this._resolution;  }
        set { this._resolution = value; }
    }

    [SerializeField( ), Range( 0, 12 )]
    private int _minimizerIterations = 6;
    public int minimizerIterations {
        get { return this._minimizerIterations;  }
        set { this._minimizerIterations = value; }
    }

    [SerializeField( ), Range( 0, 12 )]
    private int _binarySearchIterations = 6;
    public int binarySearchIterations {
        get { return this._binarySearchIterations;  }
        set { this._binarySearchIterations = value; }
    }

    [SerializeField( ), Range( 0, 12 )]
    private int _surfaceCorrectionIterations = 6;
    public int surfaceCorrectionIterations {
        get { return this._surfaceCorrectionIterations;  }
        set { this._surfaceCorrectionIterations = value; }
    }

    [SerializeField( )]
    private ImplementationType _implementation = ImplementationType.CPU;
    public ImplementationType implementationType {
        get { return this._implementation;  }
        set { this._implementation = value; }
    }

    [SerializeField( )]
    private QEFSolverType _qefSolver = QEFSolverType.Simple;
    public QEFSolverType qefSolverType {
        get { return this._qefSolver;  }
        set { this._qefSolver = value; }
    }

    [SerializeField( )]
    private IntersectionApproximation _intersectionApproximation = IntersectionApproximation.BinarySearch;
    public IntersectionApproximation intersectionApproximation {
        get { return this._intersectionApproximation;  }
        set { this._intersectionApproximation = value; }
    }

    [SerializeField( )]
    private VertexNormals _vertexNormals = VertexNormals.Shared;
    public VertexNormals vertexNormals {
        get { return this._vertexNormals;  }
        set { this._vertexNormals = value; }
    }

    protected abstract Implementation implementation { get; set; }

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

    private bool          directToGPU = false;
    private Material      material;
    private ComputeBuffer argumentsBuffer;

    public virtual void Start( ) {

        var volumes = this.GetComponentsInChildren<Volume>( );

        var voxelization = this.voxelize( volumes );

        if( this.debugFlags == Debug.Off && this.implementationType == ImplementationType.GPU ) {
            // when debug mode is off, draw the GPU generated mesh without moving it to CPU memory
            this.directToGPU = voxelization.tryVisit(
                ( GPUVoxelization voxelization ) => {
                    var meshRenderer = this.GetComponent<MeshRenderer>( );

                    UnityEngine.Debug.Assert( meshRenderer != null );

                    this.material = new Material( Resources.Load<Shader>( "Shaders/RenderContourDiffuse" ) );

                    this.material.SetBuffer( "vertices",     voxelization.vertices );
                    this.material.SetBuffer( "normals",      voxelization.normals );
                    this.material.SetBuffer( "quads",        voxelization.quads );
                    this.material.SetMatrix( "localToWorld", this.transform.localToWorldMatrix );
                    this.material.SetMatrix( "worldToLocal", this.transform.worldToLocalMatrix );

                    for( var materialIndex = 0; materialIndex < meshRenderer.materials.Length; ++materialIndex ) {
                        this.material.SetColor( $"color{materialIndex}", meshRenderer.materials[materialIndex].color );
                    }

                    this.argumentsBuffer = voxelization.arguments;
                }
            );
        }

        if( !this.directToGPU ) {

            var ( mesh, corners, edges, voxels ) = voxelization.visit(
                ( voxelization ) => {
                    var mesh = voxelization.mesh;

                    // convert to flat shading by splitting vertices
                    if( this.vertexNormals == VertexNormals.Split ) {
                        var vertices  = new Vector3[mesh.triangles.Length];
                        var triangles = new Dictionary<int, int[]>( );

                        var subMeshTriangleIndexStart = 0;
                        for( var subMeshIndex = 0; subMeshIndex < mesh.subMeshCount; ++subMeshIndex ) {
                            var subMeshTriangles = mesh.GetTriangles( subMeshIndex );
                            for( var subMeshTriangleIndex = subMeshTriangleIndexStart; subMeshTriangleIndex < subMeshTriangleIndexStart + subMeshTriangles.Length; ++subMeshTriangleIndex ) {
                                var triangleIndex = subMeshTriangleIndex - subMeshTriangleIndexStart;

                                vertices[subMeshTriangleIndex]  = mesh.vertices[subMeshTriangles[triangleIndex]];
                                subMeshTriangles[triangleIndex] = subMeshTriangleIndex;
                            }
                            triangles.Add( subMeshIndex, subMeshTriangles );
                            subMeshTriangleIndexStart += subMeshTriangles.Length;
                        }

                        mesh.vertices = vertices;
                        for( var subMeshIndex = 0; subMeshIndex < mesh.subMeshCount; ++subMeshIndex ) {
                            mesh.SetTriangles( triangles[subMeshIndex], subMeshIndex );
                        }

                        mesh.RecalculateNormals( );
                    }

                    return ( mesh, voxelization.corners, voxelization.edges, voxelization.voxels );
                },
                ( voxelization ) => {
                    var converted = this.convert( voxelization );
                    return ( converted.mesh, converted.corners, converted.edges, converted.voxels );
                }
            );

            mesh.RecalculateBounds( );
            mesh.OptimizeReorderVertexBuffer( );

            var meshFilter = this.gameObject.AddComponent<MeshFilter>( );

            meshFilter.mesh = mesh;

            this.renderDebugInformation( corners, edges, voxels );

        }

    }

    private void renderDebugInformation(
        IEnumerable<SurfaceExtractor.Corner> corners,
        IEnumerable<SurfaceExtractor.Edge>   edges,
        IEnumerable<SurfaceExtractor.Voxel>  voxels
    ) {
        if( this.debugFlags != Debug.Off ) {
            var debugObject                  = new GameObject( "Debug" );
                debugObject.transform.parent = this.gameObject.transform;

            var voxelsObject                  = new GameObject( $"Voxels" );
                voxelsObject.transform.parent = debugObject.transform;

            var voxelCount = 0;
            foreach( var voxel in voxels ) {
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
            foreach( var corner in corners ) {
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
            foreach( var edge in edges ) {
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

    public abstract Either<CPUVoxelization, GPUVoxelization> voxelize( IEnumerable<DensityFunction> densityFunctions );

    public abstract CPUVoxelization convert( GPUVoxelization voxelization );

    public void Update( ) {
        if( this.directToGPU ) {
            Graphics.DrawProceduralIndirect(
                this.material,
                new Bounds( this.transform.position, this.transform.localScale ),
                MeshTopology.Triangles,
                this.argumentsBuffer
            );
        }
    }

}

[CustomEditor( typeof( Voxelizer ) )]
public class VoxelizerEditor : Editor {

    public override void OnInspectorGUI( ) {
        base.OnInspectorGUI( );

        var voxelizer = this.target as Voxelizer;

        // only SVD solver is available for GPU implementation
        if( voxelizer.implementationType == ImplementationType.GPU ) {
            voxelizer.qefSolverType = QEFSolverType.SVD;
        }
    }

}