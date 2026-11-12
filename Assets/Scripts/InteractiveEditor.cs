using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;
using UnityEngine;

using MaterialIndex      = SurfaceExtractor.MaterialIndex;
using ImplementationType = SurfaceExtractor.Implementation.Type;
using Combination        = DensityFunction.Combination;

// contours the shapes below it in edit mode, with one of its three voxelizers at a time. Each voxelizer keeps its own
// settings, and only the selected one shows in the inspector. Shapes are read off their transforms as Unity shows
// them: a direct child of scale 1 fills the grid, whatever the scale of this object, and groups between a shape and
// this object move and scale the shapes in them. Volume.origin and Volume.extents read transforms differently, as if
// the voxelizer had scale 1, and the example scenes are built for that, so they are left as they are.
// The voxelizers do not contour by themselves, in play mode either: this decides when they do
[ExecuteAlways]
[RequireComponent( typeof( UniformDualContouring ), typeof( AdaptiveDualContouring ), typeof( ManifoldDualContouring ) )]
[RequireComponent( typeof( MeshFilter ) )]
public class InteractiveEditor : MonoBehaviour {

    public enum Algorithm {
        Uniform,
        Adaptive,
        Manifold
    }

    // a shape as the editor contours it, in the object space of the editor
    public sealed class Shape : DensityFunction {

        public DensityFunction.Type type          { get; }
        public Vector3              origin        { get; }
        public Quaternion           rotation      { get; }
        public Vector3              extents       { get; }
        public Combination          combination   { get; set; }
        public MaterialIndex        materialIndex { get; set; }

        private readonly DensityFunction.Frame frame;

        public Shape( DensityFunction.Type type, Vector3 origin, Quaternion rotation, Vector3 extents, Combination combination, MaterialIndex materialIndex ) {
            this.type          = type;
            this.origin        = origin;
            this.rotation      = rotation;
            this.extents       = extents;
            this.combination   = combination;
            this.materialIndex = materialIndex;

            this.frame = new DensityFunction.Frame( origin, rotation );
        }

        public float sample( Vector3 position ) {
            return DensityFunction.sample( this.type, this.frame.local( position ), this.extents );
        }

    }

    [SerializeField( )]
    private Algorithm _algorithm = Algorithm.Uniform;
    public Algorithm algorithm {
        get { return this._algorithm; }
    }

    // the voxelizer of the selected algorithm
    public Voxelizer voxelizer {
        get { return this.voxelizerOf( this._algorithm ); }
    }

    [SerializeField( ), Min( 0.0f ), Tooltip( "How long changes have to settle, in seconds, before a Burst or GPU variant contours by itself" )]
    private float _debounce = 0.1f;
    public float debounce {
        get { return this._debounce;  }
        set { this._debounce = value; }
    }

    // the last contour, for the inspector. Not kept over a script recompile
    public Voxelizer.ContourStatistics? lastContour { get; private set; }

    // the managed implementations take seconds on larger grids, so they only contour when asked to
    public bool contoursByItself {
        get { return this.voxelizer.implementationType != ImplementationType.Managed; }
    }

    // when to contour by itself. Not kept over a script recompile, where OnEnable sets it up again
    [NonSerialized]
    private RemeshScheduler scheduler = new( );

    // what the scheduler said at the last update, for the inspector
    public RemeshScheduler.State state { get; private set; } = RemeshScheduler.State.UpToDate;

    // whether automatic contouring is paused, and how long the contour that paused it took
    public bool   paused             { get { return this.scheduler.paused; } }
    public double lastTimedDuration  { get { return this.scheduler.lastDuration ?? 0.0; } }

    // the time in seconds and whether Unity is the active application, which tests replace with their own
    [NonSerialized]
    public Func<double> clock;
    [NonSerialized]
    public Func<bool> applicationActive;

    private double now {
        get {
            if( this.clock != null ) {
                return this.clock( );
            }
#if UNITY_EDITOR
            return UnityEditor.EditorApplication.timeSinceStartup;
#else
            return Time.realtimeSinceStartupAsDouble;
#endif
        }
    }

    private bool active {
        get {
            if( this.applicationActive != null ) {
                return this.applicationActive( );
            }
#if UNITY_EDITOR
            return UnityEditorInternal.InternalEditorUtility.isApplicationActive;
#else
            return Application.isFocused;
#endif
        }
    }

    public Voxelizer voxelizerOf( Algorithm algorithm ) {
        return algorithm switch {
            Algorithm.Uniform  => this.GetComponent<UniformDualContouring>( ),
            Algorithm.Adaptive => this.GetComponent<AdaptiveDualContouring>( ),
            Algorithm.Manifold => this.GetComponent<ManifoldDualContouring>( ),
            _                  => throw new Exception( "Unknown algorithm specified" )
        };
    }

    // a uniform grid of resolution r has 2^r cells across and an octree 2^(r - 1) leaves, so this is the exponent of the
    // cells across
    private static int cellsExponentOf( Voxelizer voxelizer ) {
        return voxelizer is UniformDualContouring ? voxelizer.resolution : voxelizer.resolution - 1;
    }

    // the range of the resolution slider
    private const int minimumResolution = 2;
    private const int maximumResolution = 8;

    // switches to the algorithm. The settings all voxelizers have carry over, so the same shapes are compared like for
    // like: the same number of cells, as far as the other's resolution goes, and the same implementation, except that
    // GPU becomes Burst for the octrees, which have no GPU implementation. What only one algorithm has, such as the
    // simplification threshold, stays with its voxelizer
    public void select( Algorithm algorithm ) {
        var from = this.voxelizer;
        var to   = this.voxelizerOf( algorithm );

        if( from != to ) {
            var resolution = cellsExponentOf( from ) + ( to is UniformDualContouring ? 0 : 1 );

            to.resolution                  = Mathf.Clamp( resolution, minimumResolution, maximumResolution );
            to.minimizerIterations         = from.minimizerIterations;
            to.binarySearchIterations      = from.binarySearchIterations;
            to.surfaceCorrectionIterations = from.surfaceCorrectionIterations;
            to.qefSolverType               = from.qefSolverType;
            to.intersectionApproximation   = from.intersectionApproximation;
            to.vertexNormals               = from.vertexNormals;
            to.debugFlags                  = from.debugFlags;

            to.implementationType = from.implementationType == ImplementationType.GPU && !( to is UniformDualContouring )
                ? ImplementationType.Burst
                : from.implementationType;

            // the voxelizer that is no longer used gives back its compute buffers. Its mesh stays until the next contour
            from.release( );
        }

        this._algorithm = algorithm;

        this.showSelected( );
    }

    // only the selected voxelizer shows in the inspector
    private void showSelected( ) {
        var changed = false;

        foreach( Algorithm algorithm in Enum.GetValues( typeof( Algorithm ) ) ) {
            var voxelizer = this.voxelizerOf( algorithm );

            if( voxelizer == null ) {
                continue;
            }

            var flags = algorithm == this._algorithm
                ? voxelizer.hideFlags & ~HideFlags.HideInInspector
                : voxelizer.hideFlags |  HideFlags.HideInInspector;

            if( voxelizer.hideFlags != flags ) {
                voxelizer.hideFlags = flags;
                changed             = true;
            }
        }

#if UNITY_EDITOR
        if( changed ) {
            UnityEditor.ActiveEditorTracker.sharedTracker.ForceRebuild( );
        }
#endif
    }

    // every shape below this object, in hierarchy order, including those on inactive objects and disabled ones, which
    // the inspector lists. A shape on this object itself is not among them
    public List<Volume> volumes( ) {
        var volumes = new List<Volume>( );

        foreach( var volume in this.GetComponentsInChildren<Volume>( true ) ) {
            if( volume.transform != this.transform ) {
                volumes.Add( volume );
            }
        }

        return volumes;
    }

    // whether the shape is contoured, and if so, how: on an active object, enabled, and below this object, not on it
    public bool tryGetShape( Volume volume, out Shape shape ) {
        if( volume == null || !volume.enabled || !volume.gameObject.activeInHierarchy || volume.transform == this.transform || !volume.transform.IsChildOf( this.transform ) ) {
            shape = null;
            return false;
        }

        var placement = this.placementOf( volume.transform );

        shape = new Shape( volume.type, placement.position, placement.rotation, placement.size.absolute( ) / 2, volume.combination, volume.materialIndex );
        return true;
    }

    // the shapes to contour, in hierarchy order, which is the order their operations apply in. Shapes on inactive
    // objects, disabled shapes and a shape on this object itself are left out
    public List<DensityFunction> shapes( ) {
        var shapes = new List<DensityFunction>( );

        foreach( var volume in this.GetComponentsInChildren<Volume>( ) ) {
            if( this.tryGetShape( volume, out var shape ) ) {
                shapes.Add( shape );
            }
        }

        return shapes;
    }

    // the debug view gets in the way above this many shapes a frame, about 16 ms of the CPU for each camera
    public const int debugDrawLimit = 30000;

    // what is likely a mistake, about the shapes or the debug view, one line each, naming the shape
    public List<string> warnings( ) {
        var warnings  = new List<string>( );
        var materials = this.GetComponent<MeshRenderer>( ).sharedMaterials.Length;
        var first     = true;

        var drawn = this.voxelizer.debugDrawCount;

        if( drawn > debugDrawLimit ) {
            warnings.Add( $"The debug view draws {drawn:N0} shapes every frame for every camera, which slows the editor down. Turn off Cells, Corners or Edges, or lower the resolution." );
        }

        foreach( var volume in this.GetComponents<Volume>( ) ) {
            warnings.Add( $"{volume.name} has a shape on the editor's own object, which is left out. Shapes go on objects below it." );
        }

        foreach( var volume in this.volumes( ) ) {
            if( !this.tryGetShape( volume, out var shape ) ) {
                continue;
            }

            var name = volume.name;

            if( first && shape.combination != Combination.Union ) {
                warnings.Add( $"{name} is the first shape and a{( shape.combination == Combination.Intersection ? "n" : "" )} {shape.combination}. Operations start from empty space, so only a Union can add anything." );
            }
            first = false;

            if( shape.materialIndex == MaterialIndex.Void ) {
                warnings.Add( $"{name} has no material, so it changes nothing. Give it one, such as Material0." );
            }
            else if( shape.combination == Combination.Union && highestMaterialBitOf( shape.materialIndex ) >= materials ) {
                warnings.Add( $"{name} adds material {highestMaterialBitOf( shape.materialIndex )}, but the renderer has {materials} material{( materials == 1 ? "" : "s" )}, so its surface is not drawn." );
            }

            if( shape.extents.x == 0.0f || shape.extents.y == 0.0f || shape.extents.z == 0.0f ) {
                warnings.Add( $"{name} has no size along an axis." );
            }
            else if( outsideTheGrid( shape ) ) {
                warnings.Add( $"{name} lies entirely outside the grid, so it changes nothing. The grid is the box drawn around the editor." );
            }

            if( this.placementOf( volume.transform ).sheared ) {
                warnings.Add( $"{name} is turned inside a group that is scaled unevenly, which would slant its sides. It is contoured with square sides instead, as its outline shows. Scale the group evenly, or scale the shape itself." );
            }
        }

        return warnings;
    }

    private static int highestMaterialBitOf( MaterialIndex materialIndex ) {
        var bit = -1;

        for( var value = ( int )materialIndex; value != 0; value >>= 1 ) {
            ++bit;
        }

        return bit;
    }

    // the grid spans -0.5 to 0.5 along each axis of this object. A turned shape reaches as far along an axis as the box
    // around it does
    private static bool outsideTheGrid( Shape shape ) {
        var reach = reachOf( shape );

        for( var axis = 0; axis < 3; ++axis ) {
            if( Mathf.Abs( shape.origin[axis] ) - reach[axis] >= 0.5f ) {
                return true;
            }
        }
        return false;
    }

    // how far the shape reaches from its origin along each axis of this object: half the size of the box around it
    private static Vector3 reachOf( Shape shape ) {
        var x = shape.rotation * new Vector3( shape.extents.x, 0.0f, 0.0f );
        var y = shape.rotation * new Vector3( 0.0f, shape.extents.y, 0.0f );
        var z = shape.rotation * new Vector3( 0.0f, 0.0f, shape.extents.z );

        // an ellipsoid reaches as far as the point of its surface furthest out. Every other shape is taken to reach as far
        // as the corners of its box, which a cuboid does, and the others do not quite: that can only leave out a warning
        return shape.type == DensityFunction.Type.Ellipsoid
            ? new Vector3(
                Mathf.Sqrt( ( x.x * x.x ) + ( y.x * y.x ) + ( z.x * z.x ) ),
                Mathf.Sqrt( ( x.y * x.y ) + ( y.y * y.y ) + ( z.y * z.y ) ),
                Mathf.Sqrt( ( x.z * x.z ) + ( y.z * y.z ) + ( z.z * z.z ) )
            )
            : x.absolute( ) + y.absolute( ) + z.absolute( );
    }

    // how far from square the axes of a shape can be before it counts as sheared: a cosine of the angle between two of
    // them, about 0.006 degrees off a right angle. A quarter turn worked out in floats is much closer than that
    private const float shearTolerance = 1e-4f;

    // where a transform below this object is, how it is turned and how large, in the object space of this object. It
    // goes up through the groups in between the way Unity does, without going through world space, so that a direct
    // child gives exactly its local position, rotation and scale. A group scaled the same on every axis, or a shape that
    // is not turned within its group, keeps the shape exactly a box or an ellipsoid along its own axes. Otherwise the
    // group's scale stretches the shape along axes of its own and can slant its sides: then sheared is true, and the
    // shape is given square axes as close to the slanted ones as there are
    private ( Vector3 position, Quaternion rotation, Vector3 size, bool sheared ) placementOf( Transform shape ) {
        var position = shape.localPosition;
        var rotation = shape.localRotation;
        var size     = shape.localScale;
        var sheared  = false;

        for( var parent = shape.parent; parent != null && parent != this.transform; parent = parent.parent ) {
            var scale = parent.localScale;

            position = parent.localPosition + ( parent.localRotation * Vector3.Scale( scale, position ) );

            // Equals compares exactly, == only to within about a tenth of a degree
            if( rotation.Equals( Quaternion.identity ) ) {
                size = Vector3.Scale( scale, size );
            }
            else if( scale.x == scale.y && scale.y == scale.z ) {
                size *= scale.x;
            }
            else {
                ( rotation, size, sheared ) = scaleTurned( rotation, size, scale, sheared );
            }

            rotation = parent.localRotation * rotation;
        }

        return ( position, rotation, size, sheared );
    }

    // a shape turned by its rotation and scaled by an uneven scale along the axes it is turned in. Each of the shape's
    // axes ends up scaled by the group, and when they are still square to each other the shape is still a box or an
    // ellipsoid, of their lengths. A turn by quarter turns, or a scale that is even along the axes the turn mixes, keeps
    // them square
    private static ( Quaternion rotation, Vector3 size, bool sheared ) scaleTurned( Quaternion rotation, Vector3 size, Vector3 scale, bool sheared ) {
        var x = Vector3.Scale( scale, rotation * new Vector3( size.x, 0.0f, 0.0f ) );
        var y = Vector3.Scale( scale, rotation * new Vector3( 0.0f, size.y, 0.0f ) );
        var z = Vector3.Scale( scale, rotation * new Vector3( 0.0f, 0.0f, size.z ) );

        var scaled = new Vector3( x.magnitude, y.magnitude, z.magnitude );

        // a shape without size along an axis has no direction there to keep, and is warned about by itself
        if( scaled.x == 0.0f || scaled.y == 0.0f || scaled.z == 0.0f ) {
            return ( rotation, scaled, sheared );
        }

        var xAxis = x / scaled.x;
        var yAxis = y / scaled.y;
        var zAxis = z / scaled.z;

        sheared |= Mathf.Abs( Vector3.Dot( xAxis, yAxis ) ) > shearTolerance || Mathf.Abs( Vector3.Dot( yAxis, zAxis ) ) > shearTolerance || Mathf.Abs( Vector3.Dot( zAxis, xAxis ) ) > shearTolerance;

        // square axes from the x axis and the y axis, made square to it. A mirrored set of axes turns the same way:
        // boxes and ellipsoids are the same shape mirrored along their own axes
        var squareY = ( yAxis - ( Vector3.Dot( yAxis, xAxis ) * xAxis ) ).normalized;
        var squareZ = Vector3.Cross( xAxis, squareY );

        return ( Quaternion.LookRotation( squareZ, squareY ), scaled, sheared );
    }

    // what a contour of the shapes would be made from: the selected algorithm, the settings of its voxelizer that change
    // the contour, and each shape as it is contoured, in order. Two signatures are equal exactly when the contours
    // would be. The debug flags are in it because the Burst implementations only copy debug data out when they are on.
    // Names, the debug view's colours and sizes, the renderer and this object's own transform are not: none of them
    // changes the mesh
    public string signatureOf( List<DensityFunction> shapes ) {
        var voxelizer = this.voxelizer;
        var text      = new StringBuilder( );

        void append( object value ) {
            text.Append( value is float number ? number.ToString( "R", CultureInfo.InvariantCulture ) : value.ToString( ) ).Append( ',' );
        }

        append( this._algorithm );
        append( voxelizer.resolution );
        append( voxelizer.minimizerIterations );
        append( voxelizer.binarySearchIterations );
        append( voxelizer.surfaceCorrectionIterations );
        append( voxelizer.implementationType );
        append( voxelizer.qefSolverType );
        append( voxelizer.intersectionApproximation );
        append( voxelizer.vertexNormals );
        append( ( int )voxelizer.debugFlags );

        switch( voxelizer ) {
            case AdaptiveDualContouring adaptive: {
                append( adaptive.simplification );
                append( adaptive.errorThreshold );
                append( adaptive.topologySafe );
                break;
            }
            case ManifoldDualContouring manifold: {
                append( manifold.errorThreshold );
                append( manifold.enforceManifold );
                break;
            }
        }

        foreach( var shape in shapes ) {
            // the rotation as every implementation undoes it, in the rows of its frame. Two quaternions of opposite sign
            // give the same rows, and turn the shape the same way
            var frame = new DensityFunction.Frame( shape );

            text.Append( '|' );
            append( shape.type );
            append( shape.origin.x );
            append( shape.origin.y );
            append( shape.origin.z );
            append( frame.xAxis.x );
            append( frame.xAxis.y );
            append( frame.xAxis.z );
            append( frame.yAxis.x );
            append( frame.yAxis.y );
            append( frame.yAxis.z );
            append( frame.zAxis.x );
            append( frame.zAxis.y );
            append( frame.zAxis.z );
            append( shape.extents.x );
            append( shape.extents.y );
            append( shape.extents.z );
            append( shape.combination );
            append( ( int )shape.materialIndex );
        }

        return text.ToString( );
    }

    public string signature( ) {
        return this.signatureOf( this.shapes( ) );
    }

    // contours the shapes with the selected voxelizer, whatever its implementation, as "Contour" does. The mesh is read
    // back from the GPU, because drawing straight from it only happens in play mode, and its vertices are not
    // reordered, which takes time and only makes drawing it a little faster
    public Voxelizer.ContourStatistics contour( ) {
        var shapes = this.shapes( );

        return this.contour( shapes, this.signatureOf( shapes ) );
    }

    private Voxelizer.ContourStatistics contour( List<DensityFunction> shapes, string signature ) {
        var statistics = this.voxelizer.contour( shapes, drawFromGPU: false, reorderVertices: false );

        this.padSubMeshes( );

        this.scheduler.contoured( this.now, signature, statistics.milliseconds / 1000.0, this.active );

        this.lastContour = statistics;
        this.state       = RemeshScheduler.State.UpToDate;

#if UNITY_EDITOR
        UnityEditor.SceneView.RepaintAll( );
#endif

        return statistics;
    }

    // Unity draws every material past the last sub-mesh over the last sub-mesh. Empty sub-meshes up to the number of
    // materials keep each material to its own, whichever materials the shapes use at the moment
    private void padSubMeshes( ) {
        var mesh      = this.GetComponent<MeshFilter>( ).sharedMesh;
        var materials = this.GetComponent<MeshRenderer>( ).sharedMaterials.Length;

        if( mesh != null && mesh.subMeshCount < materials ) {
            mesh.subMeshCount = materials;
        }
    }

    // one step of contouring by itself: works out what would be contoured now and contours it when the scheduler says
    // so. The editor calls it on every editor update, and tests with a clock of their own
    public void tick( ) {
        var shapes    = this.shapes( );
        var signature = this.signatureOf( shapes );

        this.scheduler.debounce = this._debounce;

        this.state = this.scheduler.update( this.now, signature, this.contoursByItself, this.active );

        if( this.state == RemeshScheduler.State.Due ) {
            this.contour( shapes, signature );
        }
        else {
            // a material added to the renderer needs no contour, only a sub-mesh of its own
            this.padSubMeshes( );
        }
    }

    public void Awake( ) {
        // a voxelizer contours in Start, which is only called for one that is enabled
        if( Application.isPlaying ) {
            foreach( Algorithm algorithm in Enum.GetValues( typeof( Algorithm ) ) ) {
                this.voxelizerOf( algorithm ).enabled = false;
            }
        }
    }

    // when the scene opens, after a script recompile and when play mode starts. A mesh that is shown then survived a
    // script recompile and is of what would be contoured now. When nothing is shown, a Burst or GPU variant contours
    // at the first editor update, while Unity is the active application
    public void OnEnable( ) {
        this.showSelected( );

        this.scheduler = new RemeshScheduler( );

        var signature = this.signature( );

        if( this.GetComponent<MeshFilter>( ).sharedMesh != null ) {
            this.scheduler.shown( signature );
        }
        else {
            this.scheduler.nothingShown( signature );
        }

#if UNITY_EDITOR
        UnityEditor.EditorApplication.update -= this.onEditorUpdate;
        UnityEditor.EditorApplication.update += this.onEditorUpdate;
#endif

        UnityEngine.Rendering.RenderPipelineManager.beginCameraRendering -= this.drawDebugInformation;
        UnityEngine.Rendering.RenderPipelineManager.beginCameraRendering += this.drawDebugInformation;
        Camera.onPreRender -= this.drawDebugInformation;
        Camera.onPreRender += this.drawDebugInformation;
    }

    // also just before a script recompile, which would lose track of the compute buffers
    public void OnDisable( ) {
#if UNITY_EDITOR
        UnityEditor.EditorApplication.update -= this.onEditorUpdate;
#endif

        UnityEngine.Rendering.RenderPipelineManager.beginCameraRendering -= this.drawDebugInformation;
        Camera.onPreRender -= this.drawDebugInformation;

        foreach( Algorithm algorithm in Enum.GetValues( typeof( Algorithm ) ) ) {
            this.voxelizerOf( algorithm )?.release( );
        }
    }

#if UNITY_EDITOR
    // every change, from the inspector, a handle, the hierarchy or an undo, shows up as a change of the signature, so
    // polling it covers them all
    private void onEditorUpdate( ) {
        if( this == null ) {
            UnityEditor.EditorApplication.update -= this.onEditorUpdate;
            return;
        }

        this.tick( );
    }
#endif

    public void OnDestroy( ) {
        foreach( Algorithm algorithm in Enum.GetValues( typeof( Algorithm ) ) ) {
            var voxelizer = this.voxelizerOf( algorithm );

            if( voxelizer != null ) {
                voxelizer.clear( );
                voxelizer.release( );
            }
        }
    }

    // the debug view of the selected voxelizer. A voxelizer draws it itself only once its OnEnable has run, which edit
    // mode does not call and play mode does for a voxelizer this has disabled. Preview and reflection cameras are left
    // out, as the voxelizer leaves them out
    private void drawDebugInformation( Camera camera ) {
        if( camera.cameraType == CameraType.Preview || camera.cameraType == CameraType.Reflection ) {
            return;
        }

        var voxelizer = this.voxelizer;

        if( voxelizer != null && voxelizer.debugFlags != Voxelizer.Debug.Off ) {
            voxelizer.DrawShapes( camera );
        }
    }

    private void drawDebugInformation( UnityEngine.Rendering.ScriptableRenderContext context, Camera camera ) {
        this.drawDebugInformation( camera );
    }

#if UNITY_EDITOR
    // adding the editor adds the voxelizers it requires first, which puts their long inspectors above its own. Once it
    // is added, it moves itself up above them. Moving goes by the components the inspector shows, which leaves out the
    // hidden voxelizers
    public void Reset( ) {
        UnityEditor.EditorApplication.delayCall += ( ) => {
            if( this != null ) {
                this.moveAboveVoxelizers( );
            }
        };
    }

    public void moveAboveVoxelizers( ) {
        for( var step = 0; step < 8; ++step ) {
            var shown     = this.GetComponents<Component>( ).Where( ( component ) => ( component.hideFlags & HideFlags.HideInInspector ) == 0 ).ToList( );
            var voxelizer = shown.FindIndex( ( component ) => component is Voxelizer );

            if( voxelizer < 0 || shown.IndexOf( this ) < voxelizer ) {
                return;
            }

            UnityEditorInternal.ComponentUtility.MoveComponentUp( this );
        }
    }

    // the size a shape is added at: half the grid across
    public const float addedShapeScale = 0.5f;

    // adds a shape at the centre of the grid, after the others: a Union of material 0. Undoable
    public Volume addShape( DensityFunction.Type type ) {
        var name       = UnityEditor.GameObjectUtility.GetUniqueNameForSibling( this.transform, type.ToString( ) );
        var gameObject = new GameObject( name );

        gameObject.transform.SetParent( this.transform, false );
        gameObject.transform.localScale = Vector3.one * addedShapeScale;
        gameObject.layer                = this.gameObject.layer;

        var volume = ( Volume )gameObject.AddComponent( Volume.componentOf( type ) );

        volume.combination   = Combination.Union;
        volume.materialIndex = MaterialIndex.Material0;

        UnityEditor.Undo.RegisterCreatedObjectUndo( gameObject, $"Add {type}" );

        return volume;
    }

    // removes the shape with its object, or only the shape when the object holds others below it. Undoable
    public void removeShape( Volume volume ) {
        if( volume.transform.childCount > 0 ) {
            UnityEditor.Undo.DestroyObjectImmediate( volume );
        }
        else {
            UnityEditor.Undo.DestroyObjectImmediate( volume.gameObject );
        }
    }

    // moves a shape from one place in the order of volumes( ) to another, as dragging it in the inspector's list does:
    // in front of the shape that will follow it, or after the last one, under the same parent as that shape. A shape
    // that changes groups keeps where it is in the world, so only the order changes. Undoable
    public void moveShape( int from, int to ) {
        var volumes = this.volumes( );

        if( from == to || from < 0 || from >= volumes.Count || to < 0 || to >= volumes.Count ) {
            return;
        }

        var moving = volumes[from].transform;

        volumes.RemoveAt( from );

        Transform parent;
        int       index;

        if( to < volumes.Count ) {
            var next = volumes[to].transform;

            parent = next.parent;
            index  = next.GetSiblingIndex( );
        }
        else {
            var previous = volumes[volumes.Count - 1].transform;

            parent = previous.parent;
            index  = previous.GetSiblingIndex( ) + 1;
        }

        // an object cannot go below itself
        if( parent == moving || parent.IsChildOf( moving ) ) {
            return;
        }

        var group = UnityEditor.Undo.GetCurrentGroup( );

        if( moving.parent != parent ) {
            UnityEditor.Undo.SetTransformParent( moving, parent, true, "Reorder Shape" );
        }
        else if( moving.GetSiblingIndex( ) < index ) {
            // taking it out moves those after it up by one
            --index;
        }

        UnityEditor.Undo.SetSiblingIndex( moving, index, "Reorder Shape" );
        UnityEditor.Undo.CollapseUndoOperations( group );
    }

    // the grid, which the shapes are contoured in
    public void OnDrawGizmos( ) {
        Gizmos.matrix = this.transform.localToWorldMatrix;
        Gizmos.color  = new Color( 1.0f, 1.0f, 1.0f, 0.25f );

        Gizmos.DrawWireCube( Vector3.zero, Vector3.one );
    }
#endif

#if UNITY_EDITOR
    // an undo can change the selected algorithm, which the inspector has to follow. The inspector cannot be rebuilt
    // from inside OnValidate, so that waits for the editor's next update
    public void OnValidate( ) {
        UnityEditor.EditorApplication.delayCall += ( ) => {
            if( this != null ) {
                this.showSelected( );
            }
        };
    }
#endif

}

#if UNITY_EDITOR
[UnityEditor.CustomEditor( typeof( InteractiveEditor ) )]
public class InteractiveEditorEditor : UnityEditor.Editor {

    private UnityEditorInternal.ReorderableList shapeList;
    private List<Volume>                        listed = new( );
    private Volume                              toRemove;

    public override void OnInspectorGUI( ) {
        var editor = this.target as InteractiveEditor;

        var algorithm = ( InteractiveEditor.Algorithm )UnityEditor.EditorGUILayout.EnumPopup( "Algorithm", editor.algorithm );

        if( algorithm != editor.algorithm ) {
            UnityEditor.Undo.RecordObjects(
                new UnityEngine.Object[] {
                    editor,
                    editor.voxelizerOf( InteractiveEditor.Algorithm.Uniform ),
                    editor.voxelizerOf( InteractiveEditor.Algorithm.Adaptive ),
                    editor.voxelizerOf( InteractiveEditor.Algorithm.Manifold )
                },
                "Select Algorithm"
            );

            editor.select( algorithm );
        }

        this.serializedObject.Update( );
        UnityEditor.EditorGUILayout.PropertyField( this.serializedObject.FindProperty( "_debounce" ) );
        this.serializedObject.ApplyModifiedProperties( );

        UnityEditor.EditorGUILayout.Space( );

        if( GUILayout.Button( "Contour" ) ) {
            editor.contour( );
        }

        UnityEditor.EditorGUILayout.LabelField( statusOf( editor ), UnityEditor.EditorStyles.wordWrappedLabel );

        if( editor.lastContour is Voxelizer.ContourStatistics statistics ) {
            UnityEditor.EditorGUILayout.LabelField( $"Last contour: {statistics.milliseconds:F1} ms, {statistics.vertexCount:N0} vertices, {statistics.triangleCount:N0} triangles" );
        }

        foreach( var warning in editor.warnings( ) ) {
            UnityEditor.EditorGUILayout.HelpBox( warning, UnityEditor.MessageType.Warning );
        }

        UnityEditor.EditorGUILayout.Space( );

        this.drawShapeList( editor );

        UnityEditor.EditorGUILayout.BeginHorizontal( );

        if( GUILayout.Button( "Add Cuboid" ) ) {
            editor.addShape( DensityFunction.Type.Cuboid );
        }
        if( GUILayout.Button( "Add Ellipsoid" ) ) {
            editor.addShape( DensityFunction.Type.Ellipsoid );
        }

        UnityEditor.EditorGUILayout.EndHorizontal( );

        UnityEditor.EditorGUILayout.LabelField(
            "Outlines in the scene view: Union green, Intersection yellow, Subtraction red. The box is the grid.",
            UnityEditor.EditorStyles.wordWrappedMiniLabel
        );
    }

    // the shapes in the order their operations apply. Each row has the shape's enabled box, its name, which selects it,
    // its operation and material, and a button that removes it. Dragging a row moves the shape
    private void drawShapeList( InteractiveEditor editor ) {
        this.listed.Clear( );
        this.listed.AddRange( editor.volumes( ) );

        if( this.shapeList == null ) {
            this.shapeList = new UnityEditorInternal.ReorderableList( this.listed, typeof( Volume ), true, true, false, false ) {
                drawHeaderCallback = ( rect ) => UnityEditor.EditorGUI.LabelField( rect, "Shapes, in the order their operations apply" ),
                // no add and remove buttons, so no room for them either
                footerHeight       = 0.0f
            };
        }

        this.shapeList.list                        = this.listed;
        this.shapeList.drawElementCallback         = ( rect, index, active, focused ) => this.drawShape( rect, this.listed[index], editor );
        this.shapeList.onReorderCallbackWithDetails = ( list, from, to ) => editor.moveShape( from, to );

        this.shapeList.DoLayoutList( );

        if( this.toRemove != null ) {
            editor.removeShape( this.toRemove );
            this.toRemove = null;
        }
    }

    private void drawShape( Rect rect, Volume volume, InteractiveEditor editor ) {
        if( volume == null ) {
            return;
        }

        var serialized = new UnityEditor.SerializedObject( volume );

        serialized.Update( );

        const float spacing = 4.0f;
        const float toggle  = 16.0f;
        const float remove  = 22.0f;

        var height = UnityEditor.EditorGUIUtility.singleLineHeight;
        var y      = rect.y + ( ( rect.height - height ) / 2 );

        // shapes in groups are indented by how deep they are
        var depth = 0;
        for( var parent = volume.transform.parent; parent != null && parent != editor.transform; parent = parent.parent ) {
            ++depth;
        }

        var x     = rect.x + ( depth * 12.0f );
        var width = rect.xMax - x - toggle - remove - ( 3 * spacing );

        UnityEditor.EditorGUI.PropertyField( new Rect( x, y, toggle, height ), serialized.FindProperty( "m_Enabled" ), GUIContent.none );
        x += toggle + spacing;

        var nameWidth = width * 0.36f;
        var style     = new GUIStyle( UnityEditor.EditorStyles.label );

        if( !editor.tryGetShape( volume, out _ ) ) {
            style.normal.textColor = Color.gray;
        }

        if( GUI.Button( new Rect( x, y, nameWidth, height ), new GUIContent( $"{volume.name} ({volume.type})", "Select it" ), style ) ) {
            UnityEditor.Selection.activeGameObject = volume.gameObject;
        }
        x += nameWidth + spacing;

        // "Intersection" is the longest operation, longer than any material
        var fields         = width - nameWidth - spacing;
        var operationWidth = fields * 0.55f;
        var materialWidth  = fields - operationWidth;

        UnityEditor.EditorGUI.PropertyField( new Rect( x, y, operationWidth, height ), serialized.FindProperty( "_combination" ), GUIContent.none );
        x += operationWidth + spacing;

        UnityEditor.EditorGUI.PropertyField( new Rect( x, y, materialWidth, height ), serialized.FindProperty( "_materialIndex" ), GUIContent.none );
        x += materialWidth + spacing;

        serialized.ApplyModifiedProperties( );

        if( GUI.Button( new Rect( x, y, remove, height ), new GUIContent( "-", "Remove it" ) ) ) {
            this.toRemove = volume;
        }
    }

    private static string statusOf( InteractiveEditor editor ) {
        return editor.state switch {
            RemeshScheduler.State.UpToDate   => "Up to date",
            RemeshScheduler.State.Due        => "Contouring",
            RemeshScheduler.State.Waiting    => "Waiting for changes to settle",
            RemeshScheduler.State.Manual     => "Out of date. Managed implementations only contour when Contour is pressed",
            RemeshScheduler.State.Paused     => $"Out of date. Contouring by itself is paused, because the last contour took {editor.lastTimedDuration:F1} s. It resumes after a contour under {RemeshScheduler.pauseLimit:F0} s",
            RemeshScheduler.State.Background => "Out of date. Contours once Unity is the active application again",
            _                                => editor.state.ToString( )
        };
    }

    // the status changes with the editor's updates, not with events in the inspector
    public override bool RequiresConstantRepaint( ) {
        return true;
    }

}

// the outline of each shape of an interactive editor, where, as turned and as large as it is contoured, so subtracted
// and intersected shapes can be seen and clicked. Shapes of other voxelizers, in the example scenes, are left alone
public static class InteractiveEditorGizmos {

    public static Color colourOf( Combination combination ) {
        return combination switch {
            Combination.Union        => new Color( 0.4f, 1.0f, 0.4f ),
            Combination.Intersection => new Color( 1.0f, 0.85f, 0.25f ),
            Combination.Subtraction  => new Color( 1.0f, 0.35f, 0.35f ),
            _                        => Color.white
        };
    }

    [UnityEditor.DrawGizmo( UnityEditor.GizmoType.NonSelected | UnityEditor.GizmoType.Selected | UnityEditor.GizmoType.Pickable )]
    private static void drawCuboid( Cuboid cuboid, UnityEditor.GizmoType gizmoType ) {
        drawShape( cuboid, gizmoType );
    }

    [UnityEditor.DrawGizmo( UnityEditor.GizmoType.NonSelected | UnityEditor.GizmoType.Selected | UnityEditor.GizmoType.Pickable )]
    private static void drawEllipsoid( Ellipsoid ellipsoid, UnityEditor.GizmoType gizmoType ) {
        drawShape( ellipsoid, gizmoType );
    }

    private static void drawShape( Volume volume, UnityEditor.GizmoType gizmoType ) {
        var editor = volume.GetComponentInParent<InteractiveEditor>( true );

        if( editor == null || !editor.tryGetShape( volume, out var shape ) ) {
            return;
        }

        var colour = colourOf( shape.combination );

        colour.a = ( gizmoType & UnityEditor.GizmoType.Selected ) != 0 ? 1.0f : 0.5f;

        Gizmos.color  = colour;
        Gizmos.matrix = editor.transform.localToWorldMatrix * Matrix4x4.TRS( shape.origin, Quaternion.Normalize( shape.rotation ), shape.extents * 2 );

        if( shape.type == DensityFunction.Type.Cuboid ) {
            Gizmos.DrawWireCube( Vector3.zero, Vector3.one );
        }
        else {
            Gizmos.DrawWireSphere( Vector3.zero, 0.5f );
        }
    }

}
#endif
