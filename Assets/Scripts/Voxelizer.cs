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

    // the size of a cell of the grid in voxelizer object space, where the grid is 1 across
    public virtual float cellSize {
        get { return 1.0f / ( 1 << this.resolution ); }
    }

    // the normals an implementation is asked for. A mesh that is going to be simplified needs vertices that its
    // triangles share, so it is asked for with shared normals and split afterwards
    public VertexNormals contourNormals {
        get { return this.meshSimplification ? VertexNormals.Shared : this.vertexNormals; }
    }

    [Flags]
    public enum Debug {
        Off           = 0,
        Cells         = 1,
        Corners       = 2,
        Edges         = 4,
        Intersections = 8,
        Minimizers    = 16
    }

    // what is done to the mesh of every contour before it is shown (BurstMeshSimplifier)
    [Header( "Mesh Simplification" )]

    [Tooltip( "Simplify the mesh of the contour by collapsing its edges, cheapest first, for as long as it stays within the error" )]
    public bool meshSimplification = false;

    [Tooltip( "How far a collapse may leave a vertex from the surface it stands for, as a root mean square distance in cells" ), Min( 0.0f )]
    public float simplificationError = 0.05f;

    // what is drawn over the contour, which changes nothing about the contour itself (Wireframe)
    [Header( "Wireframe" )]

    [Tooltip( "Draw the edges of the triangles over the contour: over the shaded surface, or over a flat fill that hides the surface and the lines behind it" )]
    public Wireframe.Look wireframe = Wireframe.Look.Off;

    [Tooltip( "Line width in pixels" ), Min( 0.5f )]
    public float wireframeLineWidth = 1.0f;

    [Tooltip( "Colour of the lines over the surface. Its alpha blends them" )]
    public Color wireframeLineColor = Color.black;

    [Tooltip( "Colour under the lines with Lines Only, where the lines of each material take its colour" )]
    public Color wireframeFillColor = Color.black;

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

    // the GPU's vertices and quads that are drawn from, which the wireframe is drawn from too
    private ComputeBuffer verticesBuffer;
    private ComputeBuffer quadsBuffer;

    // draws the wireframe, made the first time it is on and let go by release, and the renderer's materials it reads
    // the colours of the sub-meshes from
    private          Wireframe      wireframeDrawing;
    private readonly List<Material> wireframeMaterials = new( );

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

    // how many shapes the debug view draws each frame, worked out when the debug data is captured. Drawing takes about
    // half a microsecond to a microsecond of the CPU per shape and camera, so this tells how much it slows things down
    public int debugDrawCount { get; private set; }

    private static readonly ProfilerMarker voxelizeMarker      = new( "Voxelizer.Voxelize" );
    private static readonly ProfilerMarker simplifyMarker      = new( "Voxelizer.Simplify" );
    private static readonly ProfilerMarker splitVerticesMarker = new( "Voxelizer.SplitVertices" );
    private static readonly ProfilerMarker finishMeshMarker    = new( "Voxelizer.FinishMesh" );
    private static readonly ProfilerMarker captureDebugMarker  = new( "Voxelizer.CaptureDebugInformation" );

    // how a contour went: how long it took in all, and the size of its mesh. A mesh drawn straight from the GPU is
    // never read back, so its size is not known
    public readonly struct ContourStatistics {

        public readonly double milliseconds;
        public readonly int?   vertexCount;
        public readonly int?   triangleCount;

        public ContourStatistics( double milliseconds, int? vertexCount, int? triangleCount ) {
            this.milliseconds  = milliseconds;
            this.vertexCount   = vertexCount;
            this.triangleCount = triangleCount;
        }

    }

    // the meshes that contour makes are named this and are not saved with the scene, which tells them apart from a
    // mesh someone put in the mesh filter by hand
    private const string    contourMeshName  = "Contour";
    private const HideFlags contourMeshFlags = HideFlags.DontSaveInEditor | HideFlags.DontSaveInBuild;

    // the last mesh this voxelizer made, so it can be destroyed even when the mesh filter has gone first
    private Mesh contourMesh;

    public virtual void Start( ) {
        this.contour( );
    }

    // contours the volumes below the voxelizer and shows the result, as the overload below does
    public ContourStatistics contour( bool drawFromGPU = true, bool reorderVertices = true ) {
        return this.contour( this.GetComponentsInChildren<Volume>( ), drawFromGPU, reorderVertices );
    }

    // contours the density functions and shows the result in place of whatever contour was shown before, so it can be
    // called again and again.
    // - drawFromGPU: with the GPU implementation and the debug view off, draw the mesh straight from the compute
    //   buffers instead of reading it back. That drawing happens in Update, which only runs in play mode. A mesh that
    //   is simplified is read back whatever this says, since it is simplified on the CPU.
    // - reorderVertices: reorder the vertex buffer, which makes the mesh a little faster to draw and takes 5 to 10% of
    //   a Burst contour of 128^3 cells
    public ContourStatistics contour( IEnumerable<DensityFunction> densityFunctions, bool drawFromGPU = true, bool reorderVertices = true ) {
        var stopwatch = System.Diagnostics.Stopwatch.StartNew( );

        voxelizeMarker.Begin( );

        var voxelization = this.voxelize( densityFunctions );

        voxelizeMarker.End( );

        this.stopDrawingFromGPU( );

        if( drawFromGPU && this.debugFlags == Debug.Off && this.implementationType == ImplementationType.GPU && !this.meshSimplification ) {
            // when debug mode is off, draw the GPU generated mesh without moving it to CPU memory
            this.directToGPU = voxelization.tryVisit(
                ( GPUVoxelization voxelization ) => {
                    var meshRenderer = this.GetComponent<MeshRenderer>( );

                    UnityEngine.Debug.Assert( meshRenderer != null );

                    this.material = new Material( Resources.Load<Shader>( "Shaders/RenderContourDiffuse" ) ) { hideFlags = HideFlags.DontSave };

                    this.material.SetBuffer( "vertices",     voxelization.vertices );
                    this.material.SetBuffer( "normals",      voxelization.normals );
                    this.material.SetBuffer( "quads",        voxelization.quads );
                    this.material.SetMatrix( "localToWorld", this.transform.localToWorldMatrix );
                    this.material.SetMatrix( "worldToLocal", this.transform.worldToLocalMatrix );

                    // the shared materials, because reading Renderer.materials makes a copy of each
                    var materials = meshRenderer.sharedMaterials;

                    for( var materialIndex = 0; materialIndex < materials.Length; ++materialIndex ) {
                        this.material.SetColor( $"color{materialIndex}", materials[materialIndex].color );
                    }

                    this.argumentsBuffer = voxelization.arguments;
                    this.verticesBuffer  = voxelization.vertices;
                    this.quadsBuffer     = voxelization.quads;
                }
            );
        }

        if( this.directToGPU ) {
            this.show( null );
            this.captureDebugInformation( Enumerable.Empty<SurfaceExtractor.Corner>( ), Enumerable.Empty<SurfaceExtractor.Edge>( ), Enumerable.Empty<SurfaceExtractor.Voxel>( ) );

            return new( stopwatch.Elapsed.TotalMilliseconds, null, null );
        }

        // the GPU splits vertices for flat shading itself, in the contour kernel, unless it was asked for shared ones
        var splitByGPU = !this.meshSimplification;

        var ( mesh, corners, edges, voxels ) = voxelization.visit(
            ( voxelization ) => {
                splitByGPU = false;

                return ( voxelization.mesh, voxelization.corners, voxelization.edges, voxelization.voxels );
            },
            ( voxelization ) => {
                // with the debug view off nothing looks at the corners, edges and voxels, which take far longer to
                // read back than the mesh
                if( this.debugFlags == Debug.Off ) {
                    return ( this.readMesh( voxelization ), Enumerable.Empty<SurfaceExtractor.Corner>( ), Enumerable.Empty<SurfaceExtractor.Edge>( ), Enumerable.Empty<SurfaceExtractor.Voxel>( ) );
                }

                var converted = this.convert( voxelization );
                return ( converted.mesh, converted.corners, converted.edges, converted.voxels );
            }
        );

        // a mesh without triangles has nothing to simplify, and no buffers to read
        if( this.meshSimplification && mesh.vertexCount > 0 ) {
            simplifyMarker.Begin( );

            var simplified = BurstMeshSimplifier.simplify( mesh, this.simplificationError * this.cellSize, out _ );

            destroy( mesh );

            mesh = simplified;

            simplifyMarker.End( );
        }

        // convert to flat shading by splitting vertices
        if( this.vertexNormals == VertexNormals.Split && !splitByGPU ) {
            splitVerticesMarker.Begin( );

            splitVertices( mesh );

            splitVerticesMarker.End( );
        }

        finishMeshMarker.Begin( );

        mesh.RecalculateBounds( );

        if( reorderVertices ) {
            mesh.OptimizeReorderVertexBuffer( );
        }

        finishMeshMarker.End( );

        mesh.name      = contourMeshName;
        mesh.hideFlags = contourMeshFlags;

        this.show( mesh );

        captureDebugMarker.Begin( );

        this.captureDebugInformation( corners, edges, voxels );

        captureDebugMarker.End( );

        var triangleCount = 0;
        for( var subMeshIndex = 0; subMeshIndex < mesh.subMeshCount; ++subMeshIndex ) {
            triangleCount += ( int )mesh.GetIndexCount( subMeshIndex ) / 3;
        }

        return new( stopwatch.Elapsed.TotalMilliseconds, mesh.vertexCount, triangleCount );
    }

    // takes away what contour shows: the mesh in the mesh filter, whichever voxelizer made it, the mesh this voxelizer
    // made last, the drawing straight from the GPU and the debug data
    public void clear( ) {
        this.stopDrawingFromGPU( );
        this.show( null );

        if( this.contourMesh != null ) {
            destroy( this.contourMesh );
        }
        this.contourMesh = null;

        this.captureDebugInformation( Enumerable.Empty<SurfaceExtractor.Corner>( ), Enumerable.Empty<SurfaceExtractor.Edge>( ), Enumerable.Empty<SurfaceExtractor.Voxel>( ) );
    }

    // puts the mesh in the mesh filter, which is added the first time, in place of the contour shown before, which is
    // destroyed. Voxelizers on the same object share the mesh filter, so this replaces the contour of any of them
    private void show( Mesh mesh ) {
        var meshFilter = this.GetComponent<MeshFilter>( );

        if( meshFilter == null ) {
            if( mesh == null ) {
                return;
            }
            meshFilter = this.gameObject.AddComponent<MeshFilter>( );
        }

        var shown = meshFilter.sharedMesh;

        meshFilter.sharedMesh = mesh;
        this.contourMesh      = mesh;

        if( shown != null && shown != mesh && shown.name == contourMeshName && ( shown.hideFlags & contourMeshFlags ) == contourMeshFlags ) {
            destroy( shown );
        }
    }

    private void stopDrawingFromGPU( ) {
        if( this.material != null ) {
            destroy( this.material );
        }

        this.directToGPU     = false;
        this.material        = null;
        this.argumentsBuffer = null;
        this.verticesBuffer  = null;
        this.quadsBuffer     = null;
    }

    // Destroy waits for the end of the frame, which only comes in play mode
    private static void destroy( UnityEngine.Object item ) {
        if( Application.isPlaying ) {
            Destroy( item );
        }
        else {
            DestroyImmediate( item );
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

        // changing the index format leaves the mesh with a single empty sub-mesh, so the sub-meshes are set up again
        // before their triangles go back in
        mesh.subMeshCount = subMeshes.Length;

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
        // the debug data of an earlier contour would show the wrong thing
        this.debugCorners   = null;
        this.debugEdges     = null;
        this.debugVoxels    = null;
        this.debugDrawCount = 0;

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

        // what DrawShapes goes on to draw
        var drawEdges         = ( this.debugFlags & Debug.Edges         ) == Debug.Edges;
        var drawIntersections = ( this.debugFlags & Debug.Intersections ) == Debug.Intersections;

        if( ( this.debugFlags & Debug.Cells ) == Debug.Cells ) {
            this.debugDrawCount += this.debugVoxels.Length;
        }
        if( ( this.debugFlags & Debug.Corners ) == Debug.Corners ) {
            this.debugDrawCount += this.debugCorners.Length;
        }
        if( drawEdges || drawIntersections ) {
            foreach( var edge in this.debugEdges ) {
                if( edge.intersectsContour ? drawIntersections : drawEdges ) {
                    ++this.debugDrawCount;
                }
            }
        }
        if( ( this.debugFlags & Debug.Minimizers ) == Debug.Minimizers ) {
            foreach( var voxel in this.debugVoxels ) {
                if( voxel.hasFeaturePoint ) {
                    // its normal and the vertex itself
                    this.debugDrawCount += 2;
                }
            }
        }
    }

    // draws the edges of the contour this voxelizer shows over it, for the camera, as the wireframe settings say.
    // DrawShapes calls this for every camera in play mode, and the interactive editor forwards to DrawShapes in edit
    // mode.
    // - A mesh is drawn by the mesh renderer, so there are no lines while it is off, or while the mesh filter holds a
    //   mesh this voxelizer did not make.
    // - A contour drawn straight from the GPU is drawn by Update for every camera, whatever the renderer, with the
    //   transform it had when it was contoured, and its lines are drawn the same way
    public void drawWireframe( Camera camera ) {
        if( this.wireframe == Wireframe.Look.Off || camera.cameraType == CameraType.Preview || camera.cameraType == CameraType.Reflection ) {
            return;
        }

        var meshFilter   = this.GetComponent<MeshFilter>( );
        var meshRenderer = this.GetComponent<MeshRenderer>( );

        var style = new Wireframe.Style {
            look      = this.wireframe,
            lineWidth = this.wireframeLineWidth,
            lineColor = this.wireframeLineColor,
            fillColor = this.wireframeFillColor
        };

        if( this.directToGPU ) {
            if( !this.gameObject.activeInHierarchy ) {
                return;
            }

            this.wireframeDrawing ??= new Wireframe( );

            meshRenderer.GetSharedMaterials( this.wireframeMaterials );

            this.wireframeDrawing.drawQuads(
                camera,
                this.verticesBuffer,
                this.quadsBuffer,
                this.argumentsBuffer,
                this.material.GetMatrix( "localToWorld" ),
                new Bounds( this.transform.position, this.transform.localScale ),
                this.gameObject.layer,
                style,
                this.wireframeMaterials
            );

            return;
        }

        // Unity also leaves the lines out for a camera that does not see the layer. Checking it here saves setting them up
        var shown = this.contourMesh != null && meshFilter != null && meshFilter.sharedMesh == this.contourMesh;
        var drawn = meshRenderer.enabled && this.gameObject.activeInHierarchy && ( camera.cullingMask & ( 1 << this.gameObject.layer ) ) != 0;

        if( !shown || !drawn ) {
            return;
        }

        this.wireframeDrawing ??= new Wireframe( );

        // the list is filled in place, where Renderer.sharedMaterials would make a new array each time
        meshRenderer.GetSharedMaterials( this.wireframeMaterials );

        this.wireframeDrawing.draw( camera, this.contourMesh, this.transform.localToWorldMatrix, this.gameObject.layer, style, this.wireframeMaterials );
    }

    public override void DrawShapes( Camera camera ) {
        this.drawWireframe( camera );

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

    // the mesh of a GPU voxelization, read back on its own. convert also reads back the corners, edges and voxels,
    // which takes far longer
    public abstract Mesh readMesh( GPUVoxelization voxelization );

    // releases what an implementation holds on to outside of managed memory, such as compute buffers. The next contour
    // makes them anew, and what is shown stays. The voxelizer itself lets go of the wireframe's material and the mesh's
    // buffers, which the next drawing takes again, so an implementation that overrides this calls it
    public virtual void release( ) {
        this.wireframeDrawing?.Dispose( );
        this.wireframeDrawing = null;
    }

    public virtual void OnDestroy( ) {
        this.clear( );
        this.release( );
    }

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