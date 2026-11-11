using System;
using System.Collections.Generic;
using System.Linq;
using Shapes;
using Unity.Profiling;
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
public abstract class Voxelizer : ImmediateModeShapeDrawer, SurfaceExtractor {

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
    private int _surfaceCorrectionIterations = 0;
    public int surfaceCorrectionIterations {
        get { return this._surfaceCorrectionIterations;  }
        set { this._surfaceCorrectionIterations = value; }
    }

    [SerializeField( )]
    private ImplementationType _implementation = ImplementationType.Managed;
    public ImplementationType implementationType {
        get { return this._implementation;  }
        set { this._implementation = value; }
    }

    [SerializeField( )]
    private QEFSolverType _qefSolver = QEFSolverType.SVD;
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

    public Debug debugFlags = Debug.Off;

    [Tooltip( "Draw over the contour instead of being hidden by it" )]
    public bool debugAlwaysOnTop = false;

    [Tooltip( "Edge, intersection and surface normal thickness in pixels" ), Min( 0.5f )]
    public float debugLineThickness = 1.5f;

    [Tooltip( "Corner radius in pixels" ), Min( 0.5f )]
    public float debugCornerRadius = 2.0f;

    [Tooltip( "Minimizing vertex radius in pixels" ), Min( 0.5f )]
    public float debugMinimizingVertexRadius = 3.0f;

    [Tooltip( "Surface normal length, relative to the size of its voxel" ), Min( 0.0f )]
    public float debugSurfaceNormalLength = 0.5f;

    public Color debugCellColor             = new( 1.0f, 1.0f, 1.0f, 0.125f );
    public Color debugCornerColor           = new( 1.0f, 1.0f, 1.0f, 0.125f );
    public Color debugEdgeColor             = new( 1.0f, 1.0f, 1.0f, 0.125f );
    public Color debugEdgeIntersectionColor = Color.red;
    public Color debugMinimizingVertexColor = Color.blue;
    public Color debugSurfaceNormalColor    = Color.blue;

    // the Burst implementations keep a contour outside of managed memory, and only copy its corners, edges and voxels out
    // of it when someone is going to look at them: the debug view, or a test that sets this
    [HideInInspector]
    public bool keepDebugInformation = false;

    public bool debugInformationWanted {
        get { return this.debugFlags != Debug.Off || this.keepDebugInformation; }
    }

    private bool          directToGPU = false;
    private Material      material;
    private ComputeBuffer argumentsBuffer;

    // snapshot of the voxelization, in voxelizer object space, for drawing debug information each frame
    private struct DebugEdge {
        public Vector3 start;
        public Vector3 end;
        public bool    intersectsContour;
    }

    private struct DebugVoxel {
        public Vector3 center;
        public Vector3 size;
        public bool    hasFeaturePoint;
        public Vector3 vertex;
        public Vector3 normal;
    }

    private Vector3[]    debugCorners;
    private DebugEdge[]  debugEdges;
    private DebugVoxel[] debugVoxels;

    private static readonly ProfilerMarker voxelizeMarker      = new( "Voxelizer.Voxelize" );
    private static readonly ProfilerMarker splitVerticesMarker = new( "Voxelizer.SplitVertices" );
    private static readonly ProfilerMarker finishMeshMarker    = new( "Voxelizer.FinishMesh" );
    private static readonly ProfilerMarker captureDebugMarker  = new( "Voxelizer.CaptureDebugInformation" );

    public virtual void Start( ) {

        var volumes = this.GetComponentsInChildren<Volume>( );

        voxelizeMarker.Begin( );

        var voxelization = this.voxelize( volumes );

        voxelizeMarker.End( );

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

                    // convert to flat shading by splitting vertices. The GPU does that itself, in the contour kernel
                    if( this.vertexNormals == VertexNormals.Split ) {
                        splitVerticesMarker.Begin( );

                        splitVertices( mesh );

                        splitVerticesMarker.End( );
                    }

                    return ( mesh, voxelization.corners, voxelization.edges, voxelization.voxels );
                },
                ( voxelization ) => {
                    var converted = this.convert( voxelization );
                    return ( converted.mesh, converted.corners, converted.edges, converted.voxels );
                }
            );

            finishMeshMarker.Begin( );

            mesh.RecalculateBounds( );
            mesh.OptimizeReorderVertexBuffer( );

            finishMeshMarker.End( );

            var meshFilter = this.gameObject.AddComponent<MeshFilter>( );

            meshFilter.mesh = mesh;

            captureDebugMarker.Begin( );

            this.captureDebugInformation( corners, edges, voxels );

            captureDebugMarker.End( );

        }

    }

    // gives every triangle three vertices of its own, which makes the normals of the mesh the normals of its faces.
    // The vertices are read once: every read of Mesh.vertices returns a new copy of them
    public static void splitVertices( Mesh mesh ) {
        var shared = mesh.vertices;

        var subMeshes = new int[mesh.subMeshCount][];
        for( var subMeshIndex = 0; subMeshIndex < subMeshes.Length; ++subMeshIndex ) {
            subMeshes[subMeshIndex] = mesh.GetTriangles( subMeshIndex );
        }

        var vertices = new Vector3[subMeshes.Sum( ( triangles ) => triangles.Length )];

        var vertexIndex = 0;
        foreach( var triangles in subMeshes ) {
            for( var index = 0; index < triangles.Length; ++index ) {
                vertices[vertexIndex] = shared[triangles[index]];
                triangles[index]      = vertexIndex;

                ++vertexIndex;
            }
        }

        // three vertices for every triangle pass 65,535 vertices three times as soon
        if( vertices.Length > ushort.MaxValue ) {
            mesh.indexFormat = IndexFormat.UInt32;
        }

        mesh.vertices = vertices;
        for( var subMeshIndex = 0; subMeshIndex < subMeshes.Length; ++subMeshIndex ) {
            mesh.SetTriangles( subMeshes[subMeshIndex], subMeshIndex );
        }

        mesh.RecalculateNormals( );
    }

    private void captureDebugInformation(
        IEnumerable<SurfaceExtractor.Corner> corners,
        IEnumerable<SurfaceExtractor.Edge>   edges,
        IEnumerable<SurfaceExtractor.Voxel>  voxels
    ) {
        if( this.debugFlags == Debug.Off ) {
            return;
        }

        this.debugCorners = corners.Select( ( corner ) => corner.position ).ToArray( );

        this.debugEdges = edges.Select(
            ( edge ) => new DebugEdge {
                start             = edge.corners[0].position,
                end               = edge.corners[1].position,
                intersectsContour = edge.intersectsContour( )
            }
        ).ToArray( );

        this.debugVoxels = voxels.Select(
            ( voxel ) => new DebugVoxel {
                center          = voxel.center,
                size            = voxel.size,
                hasFeaturePoint = voxel.hasFeaturePoint,
                vertex          = voxel.vertex,
                normal          = voxel.normal
            }
        ).ToArray( );
    }

    public override void DrawShapes( Camera camera ) {
        if( this.debugFlags == Debug.Off || this.debugVoxels == null ) {
            return;
        }

        using( Draw.Command( camera ) ) {

            Draw.ResetAllDrawStates( );

            Draw.BlendMode      = ShapesBlendMode.Transparent;
            Draw.ZTest          = this.debugAlwaysOnTop ? CompareFunction.Always : CompareFunction.LessEqual;
            Draw.LineGeometry   = LineGeometry.Billboard;
            Draw.ThicknessSpace = ThicknessSpace.Pixels;
            Draw.Thickness      = this.debugLineThickness;
            Draw.RadiusSpace    = ThicknessSpace.Pixels;

            // the voxelization is in voxelizer object space
            var localToWorld = this.transform.localToWorldMatrix;

            // cells are sized in voxelizer object space, so they're drawn in it
            if( ( this.debugFlags & Debug.Cells ) == Debug.Cells ) {
                Draw.Matrix = localToWorld;

                foreach( var voxel in this.debugVoxels ) {
                    Draw.Cuboid( voxel.center, voxel.size, this.debugCellColor );
                }

                Draw.ResetMatrix( );
            }

            // everything else is sized in pixels and drawn in world space, otherwise the voxelizer's scale would scale
            // the sphere radii too

            if( ( this.debugFlags & Debug.Corners ) == Debug.Corners ) {
                foreach( var corner in this.debugCorners ) {
                    Draw.Sphere( localToWorld.MultiplyPoint3x4( corner ), this.debugCornerRadius, this.debugCornerColor );
                }
            }

            var drawEdges         = ( this.debugFlags & Debug.Edges         ) == Debug.Edges;
            var drawIntersections = ( this.debugFlags & Debug.Intersections ) == Debug.Intersections;

            if( drawEdges || drawIntersections ) {
                foreach( var edge in this.debugEdges ) {
                    if( edge.intersectsContour ? drawIntersections : drawEdges ) {
                        Draw.Line(
                            localToWorld.MultiplyPoint3x4( edge.start ),
                            localToWorld.MultiplyPoint3x4( edge.end ),
                            edge.intersectsContour ? this.debugEdgeIntersectionColor : this.debugEdgeColor
                        );
                    }
                }
            }

            // normals and vertices are drawn in separate passes, Shapes only instances consecutive draws of the same shape
            if( ( this.debugFlags & Debug.Minimizers ) == Debug.Minimizers ) {
                foreach( var voxel in this.debugVoxels ) {
                    if( voxel.hasFeaturePoint ) {
                        var normalLength = this.debugSurfaceNormalLength * Mathf.Min( voxel.size.x, Mathf.Min( voxel.size.y, voxel.size.z ) );

                        Draw.Line(
                            localToWorld.MultiplyPoint3x4( voxel.vertex ),
                            localToWorld.MultiplyPoint3x4( voxel.vertex + ( voxel.normal * normalLength ) ),
                            this.debugSurfaceNormalColor
                        );
                    }
                }
                foreach( var voxel in this.debugVoxels ) {
                    if( voxel.hasFeaturePoint ) {
                        Draw.Sphere( localToWorld.MultiplyPoint3x4( voxel.vertex ), this.debugMinimizingVertexRadius, this.debugMinimizingVertexColor );
                    }
                }
            }

        }
    }

    public abstract Either<CPUVoxelization, GPUVoxelization> voxelize( IEnumerable<DensityFunction> densityFunctions );

    public abstract CPUVoxelization convert( GPUVoxelization voxelization );

    // releases what an implementation holds on to outside of managed memory, such as compute buffers
    public virtual void OnDestroy( ) { }

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