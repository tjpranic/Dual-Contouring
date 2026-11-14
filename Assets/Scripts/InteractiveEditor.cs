using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;
using UnityEngine;

using MaterialIndex      = SurfaceExtractor.MaterialIndex;
using ImplementationType = SurfaceExtractor.Implementation.Type;
using Combination        = DensityFunction.Combination;
using VertexNormals      = SurfaceExtractor.VertexNormals;

// contours the shapes below it in edit mode, with one of its four voxelizers at a time. Each voxelizer keeps its own
// settings, and only the selected one shows in the inspector. Shapes are read off their transforms as Unity shows
// them: a direct child of scale 1 fills the grid, whatever the scale of this object, and groups between a shape and
// this object move and scale the shapes in them. Volume.origin and Volume.extents read transforms differently, as if
// the voxelizer had scale 1, and the example scenes are built for that, so they are left as they are.
// The voxelizers do not contour by themselves, in play mode either: this decides when they do
[ExecuteAlways]
[RequireComponent( typeof( UniformDualContouring ), typeof( AdaptiveDualContouring ), typeof( ManifoldDualContouring ) )]
[RequireComponent( typeof( OccupancyDualContouring ), typeof( MeshFilter ) )]
public class InteractiveEditor : MonoBehaviour {

    public enum Algorithm {
        Uniform,
        Adaptive,
        Manifold,
        Occupancy
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

    [SerializeField( ), Min( 0.5f ), Tooltip( "How thick the outlines of the shapes are in the scene view, in pixels" )]
    private float _outlineThickness = 1.0f;
    public float outlineThickness {
        get { return this._outlineThickness;  }
        set { this._outlineThickness = value; }
    }

    [SerializeField( ), Tooltip( "Draw the outlines of the shapes over everything, instead of hidden where the contour or anything else is in front of them" )]
    private bool _outlinesOnTop = false;
    public bool outlinesOnTop {
        get { return this._outlinesOnTop;  }
        set { this._outlinesOnTop = value; }
    }

    // the size of a cell of the selected voxelizer's grid, in the object space of this object, where the grid is 1 across
    public float cellSize {
        get { return 1.0f / ( 1 << cellsExponentOf( this.voxelizer ) ); }
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

    // when to simplify the contour that is shown again by itself, which takes no contour (Voxelizer.simplify): under
    // another error or line weight, and at another detail. Each has a scheduler of its own because each takes its own
    // time: at 128^3 cells a contour that is simplified takes 290 ms, simplifying it again 260, and another detail 4.
    // Both are Burst code whatever the implementation, so they also run by themselves for a managed one
    [NonSerialized]
    private RemeshScheduler simplifierScheduler = new( );
    [NonSerialized]
    private RemeshScheduler detailScheduler = new( );

    // what the schedulers said at the last update, for the inspector, and whether that is about simplifying the contour
    // that is shown and not about contouring
    public RemeshScheduler.State state { get; private set; } = RemeshScheduler.State.UpToDate;

    public bool stateIsOfSimplification { get; private set; }

    // whether automatic contouring is paused, and how long the contour or the simplification that paused it took
    public bool   paused             { get { return this.scheduler.paused; } }
    public double lastTimedDuration  { get { return ( this.stateIsOfSimplification ? this.simplifierScheduler.lastDuration : this.scheduler.lastDuration ) ?? 0.0; } }

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
            Algorithm.Uniform   => this.GetComponent<UniformDualContouring>( ),
            Algorithm.Adaptive  => this.GetComponent<AdaptiveDualContouring>( ),
            Algorithm.Manifold  => this.GetComponent<ManifoldDualContouring>( ),
            Algorithm.Occupancy => this.GetComponent<OccupancyDualContouring>( ),
            _                   => throw new Exception( "Unknown algorithm specified" )
        };
    }

    // every voxelizer of this object, the selected one and the others, in the order of their algorithms. One that is
    // missing, on an object made before its algorithm was added, is left out
    public IEnumerable<Voxelizer> voxelizers( ) {
        foreach( Algorithm algorithm in Enum.GetValues( typeof( Algorithm ) ) ) {
            var voxelizer = this.voxelizerOf( algorithm );

            if( voxelizer != null ) {
                yield return voxelizer;
            }
        }
    }

    // uniform and occupancy-based dual contouring contour a uniform grid, the other two an octree
    private static bool hasUniformGrid( Voxelizer voxelizer ) {
        return voxelizer is UniformDualContouring || voxelizer is OccupancyDualContouring;
    }

    // a uniform grid of resolution r has 2^r cells across and an octree 2^(r - 1) leaves, so this is the exponent of the
    // cells across
    private static int cellsExponentOf( Voxelizer voxelizer ) {
        return hasUniformGrid( voxelizer ) ? voxelizer.resolution : voxelizer.resolution - 1;
    }

    // the implementation a voxelizer takes over from another: the same where it has it. The octrees have no GPU
    // implementation and occupancy-based dual contouring no managed one, and both take Burst instead
    private static ImplementationType implementationFor( Voxelizer voxelizer, ImplementationType implementationType ) {
        var missing = voxelizer switch {
            UniformDualContouring   => false,
            OccupancyDualContouring => implementationType == ImplementationType.Managed,
            _                       => implementationType == ImplementationType.GPU
        };

        return missing ? ImplementationType.Burst : implementationType;
    }

    // the range of the resolution slider
    private const int minimumResolution = 2;
    private const int maximumResolution = 8;

    // switches to the algorithm. The settings all voxelizers have carry over, so the same shapes are compared like for
    // like: the same number of cells, as far as the other's resolution goes, and the same implementation, or Burst
    // where the other does not have it. What only one algorithm has, such as the simplification threshold, stays with
    // its voxelizer. The settings occupancy-based dual contouring does not use pass through it all the same, so they
    // are still there on the way back
    public void select( Algorithm algorithm ) {
        var from = this.voxelizer;
        var to   = this.voxelizerOf( algorithm );

        if( from != to ) {
            var resolution = cellsExponentOf( from ) + ( hasUniformGrid( to ) ? 0 : 1 );

            to.resolution                  = Mathf.Clamp( resolution, minimumResolution, maximumResolution );
            to.minimizerIterations         = from.minimizerIterations;
            to.binarySearchIterations      = from.binarySearchIterations;
            to.surfaceCorrectionIterations = from.surfaceCorrectionIterations;
            to.qefSolverType               = from.qefSolverType;
            to.intersectionApproximation   = from.intersectionApproximation;
            to.vertexNormals               = from.vertexNormals;
            to.creaseAngle                 = from.creaseAngle;
            to.smoothNormals               = from.smoothNormals;
            to.debugFlags                  = from.debugFlags;

            to.meshSimplification   = from.meshSimplification;
            to.simplificationError  = from.simplificationError;
            to.lineQuadricWeight    = from.lineQuadricWeight;
            to.simplificationDetail = from.simplificationDetail;

            to.wireframe          = from.wireframe;
            to.wireframeLineWidth = from.wireframeLineWidth;
            to.wireframeLineColor = from.wireframeLineColor;
            to.wireframeFillColor = from.wireframeFillColor;

            to.implementationType = implementationFor( to, from.implementationType );

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

            if( roundOrSquareAcross( shape.type ) && !Mathf.Approximately( shape.extents.x, shape.extents.z ) && shape.extents.x != 0.0f && shape.extents.z != 0.0f ) {
                var across = shape.type == DensityFunction.Type.Pyramid ? "square" : "round";

                warnings.Add( $"{name} is not as wide along x as along z. A {nameOf( shape.type ).ToLowerInvariant( )} is {across} across, so it is contoured as wide as the narrower of the two, as its outline shows. Scale it the same along x and z." );
            }

            if( this.placementOf( volume.transform ).sheared ) {
                warnings.Add( $"{name} is turned inside a group that is scaled unevenly, which would slant its sides. It is contoured with square sides instead, as its outline shows. Scale the group evenly, or scale the shape itself." );
            }
        }

        return warnings;
    }

    // a type as the inspector names it, a space before each word: "Hexagonal Prism"
    public static string nameOf( DensityFunction.Type type ) {
        var name = new StringBuilder( );

        foreach( var letter in type.ToString( ) ) {
            if( char.IsUpper( letter ) && name.Length > 0 ) {
                name.Append( ' ' );
            }
            name.Append( letter );
        }

        return name.ToString( );
    }

    // the shapes that take the smaller of their x and z extents for both
    private static bool roundOrSquareAcross( DensityFunction.Type type ) {
        return type == DensityFunction.Type.Cylinder || type == DensityFunction.Type.Cone || type == DensityFunction.Type.Capsule ||
               type == DensityFunction.Type.Torus    || type == DensityFunction.Type.Pyramid;
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
    // changes the mesh.
    //
    // Whether the mesh is simplified is in it: a contour that is not simplified keeps nothing to simplify later. How
    // it is simplified is not, since that takes no contour (simplifierSignature, detailSignature)
    public string signatureOf( List<DensityFunction> shapes ) {
        var voxelizer = this.voxelizer;
        var text      = new StringBuilder( );

        void append( object value ) {
            text.Append( value is float number ? number.ToString( "R", CultureInfo.InvariantCulture ) : value.ToString( ) ).Append( ',' );
        }

        append( this._algorithm );
        append( voxelizer.resolution );
        append( voxelizer.minimizerIterations );
        append( voxelizer.implementationType );
        append( voxelizer.qefSolverType );
        append( voxelizer.vertexNormals );

        // how creased normals break, which says nothing about split ones
        if( voxelizer.vertexNormals == VertexNormals.Creased ) {
            append( voxelizer.creaseAngle );
            append( voxelizer.smoothNormals );
        }

        append( ( int )voxelizer.debugFlags );
        append( voxelizer.meshSimplification );

        // the settings that place points by distance. Occupancy-based dual contouring has searches of its own instead,
        // and its contour is the same whatever these are
        if( !( voxelizer is OccupancyDualContouring ) ) {
            append( voxelizer.binarySearchIterations );
            append( voxelizer.surfaceCorrectionIterations );
            append( voxelizer.intersectionApproximation );
        }

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
            case OccupancyDualContouring occupancy: {
                append( occupancy.edgeSearchIterations );
                append( occupancy.faceSearchSamples );
                append( occupancy.faceSearchIterations );
                append( occupancy.keepVerticesInCells );
                append( occupancy.splitQuads );
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

    // what the collapses that simplify a contour depend on, beside the contour: the error and the line weight. And
    // what the mesh shown depends on beside those: the detail. With simplification off neither is anything
    public string simplifierSignature( ) {
        var voxelizer = this.voxelizer;

        return voxelizer.meshSimplification ? FormattableString.Invariant( $"{voxelizer.simplificationError:R},{voxelizer.lineQuadricWeight:R}" ) : "";
    }

    public string detailSignature( ) {
        var voxelizer = this.voxelizer;

        return voxelizer.meshSimplification ? FormattableString.Invariant( $"{voxelizer.simplificationDetail:R}" ) : "";
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

        // the contour is simplified as the settings are now. Simplifying it again takes as long as simplifying it took
        // of the contour, which for a managed implementation is a small part of it
        this.simplifierScheduler.contoured( this.now, this.simplifierSignature( ), ( statistics.simplificationMilliseconds ?? 0.0 ) / 1000.0, this.active );
        this.detailScheduler.shown( this.detailSignature( ) );

        this.lastContour             = statistics;
        this.state                   = RemeshScheduler.State.UpToDate;
        this.stateIsOfSimplification = false;

#if UNITY_EDITOR
        UnityEditor.SceneView.RepaintAll( );
#endif

        return statistics;
    }

    // shows the contour again as the simplification settings are now, without contouring, and notes how long it took
    // with the scheduler that asked for it. Gives whether there was a simplified contour to go back to: after a script
    // recompile there is none, though its mesh is still shown
    private bool simplify( RemeshScheduler due ) {
        if( !( this.voxelizer.simplify( reorderVertices: false ) is Voxelizer.ContourStatistics statistics ) ) {
            return false;
        }

        due.contoured( this.now, due == this.detailScheduler ? this.detailSignature( ) : this.simplifierSignature( ), statistics.milliseconds / 1000.0, this.active );

        // the time stays that of the contour, the sizes are those of what is shown now
        this.lastContour = new Voxelizer.ContourStatistics(
            this.lastContour?.milliseconds ?? statistics.milliseconds, statistics.vertexCount, statistics.triangleCount, statistics.unsimplifiedVertexCount, statistics.unsimplifiedTriangleCount,
            statistics.simplificationMilliseconds
        );

#if UNITY_EDITOR
        UnityEditor.SceneView.RepaintAll( );
#endif

        return true;
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

        this.scheduler.debounce           = this._debounce;
        this.simplifierScheduler.debounce = this._debounce;
        this.detailScheduler.debounce     = this._debounce;

        this.state                   = this.scheduler.update( this.now, signature, this.contoursByItself, this.active );
        this.stateIsOfSimplification = false;

        // a contour that is owed brings the simplification settings with it. With the contour up to date, what can be
        // behind is how it is simplified: the collapses first, which the mesh at any detail is made from
        if( this.state == RemeshScheduler.State.UpToDate ) {
            var behind = this.simplifierScheduler;

            this.state = behind.update( this.now, this.simplifierSignature( ), true, this.active );

            if( this.state == RemeshScheduler.State.UpToDate ) {
                behind     = this.detailScheduler;
                this.state = behind.update( this.now, this.detailSignature( ), true, this.active );
            }

            this.stateIsOfSimplification = this.state != RemeshScheduler.State.UpToDate;

            if( this.state == RemeshScheduler.State.Due ) {
                if( this.simplify( behind ) || this.nothingShown ) {
                    // the mesh shown is of all the settings as they are now, the detail with the others. A contour
                    // of nothing has nothing to simplify, and is up to date as it is
                    this.simplifierScheduler.shown( this.simplifierSignature( ) );
                    this.detailScheduler.shown( this.detailSignature( ) );

                    this.state                   = RemeshScheduler.State.UpToDate;
                    this.stateIsOfSimplification = false;
                }
                else {
                    // what the mesh shown was simplified from is gone, and it takes a contour to have it again
                    this.scheduler.invalidate( );

                    this.state                   = this.scheduler.update( this.now, signature, this.contoursByItself, this.active );
                    this.stateIsOfSimplification = false;
                }
            }
        }

        if( this.state == RemeshScheduler.State.Due ) {
            this.contour( shapes, signature );
        }
        else {
            // a material added to the renderer needs no contour, only a sub-mesh of its own
            this.padSubMeshes( );
        }
    }

    private bool nothingShown {
        get {
            var mesh = this.GetComponent<MeshFilter>( ).sharedMesh;

            return mesh == null || mesh.vertexCount == 0;
        }
    }

    public void Awake( ) {
        // a voxelizer contours in Start, which is only called for one that is enabled
        if( Application.isPlaying ) {
            foreach( var voxelizer in this.voxelizers( ) ) {
                voxelizer.enabled = false;
            }
        }
    }

    // when the scene opens, after a script recompile and when play mode starts. A mesh that is shown then survived a
    // script recompile and is of what would be contoured now. When nothing is shown, a Burst or GPU variant contours
    // at the first editor update, while Unity is the active application
    public void OnEnable( ) {
        this.showSelected( );

        this.scheduler           = new RemeshScheduler( );
        this.simplifierScheduler = new RemeshScheduler( );
        this.detailScheduler     = new RemeshScheduler( );

        var signature = this.signature( );

        if( this.GetComponent<MeshFilter>( ).sharedMesh != null ) {
            this.scheduler.shown( signature );
        }
        else {
            this.scheduler.nothingShown( signature );
        }

        this.simplifierScheduler.shown( this.simplifierSignature( ) );
        this.detailScheduler.shown( this.detailSignature( ) );

#if UNITY_EDITOR
        UnityEditor.EditorApplication.update -= this.onEditorUpdate;
        UnityEditor.EditorApplication.update += this.onEditorUpdate;
#endif

        UnityEngine.Rendering.RenderPipelineManager.beginCameraRendering -= this.drawDebugViewAndWireframe;
        UnityEngine.Rendering.RenderPipelineManager.beginCameraRendering += this.drawDebugViewAndWireframe;
        Camera.onPreRender -= this.drawDebugViewAndWireframe;
        Camera.onPreRender += this.drawDebugViewAndWireframe;
    }

    // also just before a script recompile, which would lose track of the compute buffers
    public void OnDisable( ) {
#if UNITY_EDITOR
        UnityEditor.EditorApplication.update -= this.onEditorUpdate;
#endif

        UnityEngine.Rendering.RenderPipelineManager.beginCameraRendering -= this.drawDebugViewAndWireframe;
        Camera.onPreRender -= this.drawDebugViewAndWireframe;

        foreach( var voxelizer in this.voxelizers( ) ) {
            voxelizer.release( );
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
        foreach( var voxelizer in this.voxelizers( ) ) {
            voxelizer.clear( );
            voxelizer.release( );
        }
    }

    // the debug view and the wireframe of the selected voxelizer. A voxelizer draws them itself only once its OnEnable
    // has run, which edit mode does not call and play mode does for a voxelizer this has disabled. Preview and
    // reflection cameras are left out, as the voxelizer leaves them out
    private void drawDebugViewAndWireframe( Camera camera ) {
        if( camera.cameraType == CameraType.Preview || camera.cameraType == CameraType.Reflection ) {
            return;
        }

        var voxelizer = this.voxelizer;

        if( voxelizer != null && ( voxelizer.debugFlags != Voxelizer.Debug.Off || voxelizer.wireframe != Wireframe.Look.Off ) ) {
            voxelizer.DrawShapes( camera );
        }
    }

    private void drawDebugViewAndWireframe( UnityEngine.Rendering.ScriptableRenderContext context, Camera camera ) {
        this.drawDebugViewAndWireframe( camera );
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

    // the scale a shape of the type is added at. A cube of half the grid would give a capsule that is a ball and a torus
    // without a hole, so those two are added at shapes of their own
    public static Vector3 addedScaleOf( DensityFunction.Type type ) {
        return type switch {
            DensityFunction.Type.Capsule => new Vector3( 0.25f, 0.5f, 0.25f ),
            DensityFunction.Type.Torus   => new Vector3( 0.5f,  0.2f, 0.5f  ),
            _                            => Vector3.one * addedShapeScale
        };
    }

    // adds a shape at the centre of the grid, after the others: a Union of material 0. Undoable
    public Volume addShape( DensityFunction.Type type ) {
        var name       = UnityEditor.GameObjectUtility.GetUniqueNameForSibling( this.transform, type.ToString( ) );
        var gameObject = new GameObject( name );

        gameObject.transform.SetParent( this.transform, false );
        gameObject.transform.localScale = addedScaleOf( type );
        gameObject.layer                = this.gameObject.layer;

        var volume = ( Volume )gameObject.AddComponent( Volume.componentOf( type ) );

        volume.combination   = Combination.Union;
        volume.materialIndex = MaterialIndex.Material0;

        UnityEditor.Undo.RegisterCreatedObjectUndo( gameObject, $"Add {nameOf( type )}" );

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

    private const int buttonsPerRow = 3;

    private UnityEditorInternal.ReorderableList shapeList;
    private List<Volume>                        listed = new( );
    private Volume                              toRemove;

    public override void OnInspectorGUI( ) {
        var editor = this.target as InteractiveEditor;

        var algorithm = ( InteractiveEditor.Algorithm )UnityEditor.EditorGUILayout.EnumPopup( "Algorithm", editor.algorithm );

        if( algorithm != editor.algorithm ) {
            UnityEditor.Undo.RecordObjects( editor.voxelizers( ).Prepend<UnityEngine.Object>( editor ).ToArray( ), "Select Algorithm" );

            editor.select( algorithm );
        }

        this.serializedObject.Update( );
        UnityEditor.EditorGUILayout.PropertyField( this.serializedObject.FindProperty( "_debounce" ) );
        UnityEditor.EditorGUILayout.PropertyField( this.serializedObject.FindProperty( "_outlineThickness" ) );
        UnityEditor.EditorGUILayout.PropertyField( this.serializedObject.FindProperty( "_outlinesOnTop" ) );
        this.serializedObject.ApplyModifiedProperties( );

        UnityEditor.EditorGUILayout.Space( );

        if( GUILayout.Button( "Contour" ) ) {
            editor.contour( );
        }

        UnityEditor.EditorGUILayout.LabelField( statusOf( editor ), UnityEditor.EditorStyles.wordWrappedLabel );

        if( editor.lastContour is Voxelizer.ContourStatistics statistics ) {
            UnityEditor.EditorGUILayout.LabelField( $"Last contour: {statistics.milliseconds:F1} ms, {statistics.vertexCount:N0} vertices, {statistics.triangleCount:N0} triangles" );

            if( statistics.unsimplifiedVertexCount.HasValue ) {
                UnityEditor.EditorGUILayout.LabelField( $"Simplified from {statistics.unsimplifiedVertexCount:N0} vertices, {statistics.unsimplifiedTriangleCount:N0} triangles" );
            }
        }

        foreach( var warning in editor.warnings( ) ) {
            UnityEditor.EditorGUILayout.HelpBox( warning, UnityEditor.MessageType.Warning );
        }

        UnityEditor.EditorGUILayout.Space( );

        this.drawShapeList( editor );

        // a button for every type, three to a row and all as wide: a grid of buttons of which none is selected, and which
        // gives the index of the one pressed
        UnityEditor.EditorGUILayout.LabelField( "Add" );

        var types   = ( DensityFunction.Type[] )Enum.GetValues( typeof( DensityFunction.Type ) );
        var pressed = GUILayout.SelectionGrid( -1, types.Select( InteractiveEditor.nameOf ).ToArray( ), buttonsPerRow );

        if( pressed >= 0 ) {
            editor.addShape( types[pressed] );
        }

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

        if( GUI.Button( new Rect( x, y, nameWidth, height ), new GUIContent( $"{volume.name} ({InteractiveEditor.nameOf( volume.type )})", "Select it" ), style ) ) {
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
            RemeshScheduler.State.Due        => editor.stateIsOfSimplification ? "Simplifying" : "Contouring",
            RemeshScheduler.State.Waiting    => "Waiting for changes to settle",
            RemeshScheduler.State.Manual     => "Out of date. Managed implementations only contour when Contour is pressed",
            RemeshScheduler.State.Paused     => editor.stateIsOfSimplification
                ? $"Out of date. Simplifying by itself is paused, because the last time took {editor.lastTimedDuration:F1} s. Press Contour; it resumes after a contour under {RemeshScheduler.pauseLimit:F0} s"
                : $"Out of date. Contouring by itself is paused, because the last contour took {editor.lastTimedDuration:F1} s. It resumes after a contour under {RemeshScheduler.pauseLimit:F0} s",
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

    // the grey of the scene view's ground, which outlines of shapes that are not selected are dimmed toward
    private static readonly Color dimmedToward = new( 0.35f, 0.35f, 0.35f );

    // the colour of an outline, by the shape's operation, and dimmed unless the shape is selected. Outlines are opaque:
    // Shapes blends every line by itself, so half opaque lines showed brighter wherever two of them overlapped. Dimmed
    // half way to the ground's grey, they look much as they did half opaque over it
    public static Color colourOf( Combination combination, bool selected ) {
        var colour = combination switch {
            Combination.Union        => new Color( 0.4f, 1.0f, 0.4f ),
            Combination.Intersection => new Color( 1.0f, 0.85f, 0.25f ),
            Combination.Subtraction  => new Color( 1.0f, 0.35f, 0.35f ),
            _                        => Color.white
        };

        return selected ? colour : Color.Lerp( colour, dimmedToward, 0.5f );
    }

    // how many lines a whole circle of an outline is drawn with
    public const int circleSegments = 48;

    // the lines of a shape's outline in its own frame, made from its extents as the shape is: the edges of the shapes
    // with flat faces, and around curved ones, circles and lines along their sides. Every end of a line lies on the shape
    public static List<( Vector3 start, Vector3 end )> outlineOf( DensityFunction.Type type, Vector3 extents ) {
        var lines  = new List<( Vector3 start, Vector3 end )>( );
        var across = Mathf.Min( extents.x, extents.z );
        var y      = extents.y;

        // the four ways out from the axis of a round shape
        var outward = new[] { Vector3.right, Vector3.forward, Vector3.left, Vector3.back };

        void line( Vector3 start, Vector3 end ) {
            if( start != end ) {
                lines.Add( ( start, end ) );
            }
        }

        // an arc of the ellipse about the centre with the two radii, from one angle to the other in radians
        void arc( Vector3 centre, Vector3 radius, Vector3 otherRadius, float from, float to ) {
            var steps = Mathf.Max( 1, Mathf.CeilToInt( circleSegments * ( to - from ) / ( 2.0f * Mathf.PI ) ) );

            Vector3 at( int step ) {
                var angle = from + ( ( to - from ) * step / steps );

                return centre + ( radius * Mathf.Cos( angle ) ) + ( otherRadius * Mathf.Sin( angle ) );
            }

            for( var step = 0; step < steps; ++step ) {
                line( at( step ), at( step + 1 ) );
            }
        }

        void circle( Vector3 centre, Vector3 radius, Vector3 otherRadius ) {
            arc( centre, radius, otherRadius, 0.0f, 2.0f * Mathf.PI );
        }

        // a circle about the axis
        void ring( float height, float radius ) {
            if( radius > 0.0f ) {
                circle( new Vector3( 0.0f, height, 0.0f ), new Vector3( radius, 0.0f, 0.0f ), new Vector3( 0.0f, 0.0f, radius ) );
            }
        }

        void edges( Vector3[] corners, int[] ends ) {
            for( var index = 0; index < ends.Length; index += 2 ) {
                line( corners[ends[index]], corners[ends[index + 1]] );
            }
        }

        switch( type ) {
            case DensityFunction.Type.Cuboid: {
                var corners = Enumerable.Range( 0, 8 ).Select(
                    ( corner ) => new Vector3( ( corner & 1 ) == 0 ? -extents.x : extents.x, ( corner & 2 ) == 0 ? -y : y, ( corner & 4 ) == 0 ? -extents.z : extents.z )
                ).ToArray( );

                // corners whose indices differ in one bit share an edge
                edges( corners, new[] { 0, 1, 2, 3, 4, 5, 6, 7, 0, 2, 1, 3, 4, 6, 5, 7, 0, 4, 1, 5, 2, 6, 3, 7 } );
                break;
            }
            case DensityFunction.Type.Ellipsoid: {
                var alongX = new Vector3( extents.x, 0.0f, 0.0f );
                var alongY = new Vector3( 0.0f, y, 0.0f );
                var alongZ = new Vector3( 0.0f, 0.0f, extents.z );

                circle( Vector3.zero, alongX, alongY );
                circle( Vector3.zero, alongX, alongZ );
                circle( Vector3.zero, alongY, alongZ );
                break;
            }
            case DensityFunction.Type.Cylinder: {
                ring( -y, across );
                ring( y, across );

                foreach( var direction in outward ) {
                    line( ( direction * across ) - new Vector3( 0.0f, y, 0.0f ), ( direction * across ) + new Vector3( 0.0f, y, 0.0f ) );
                }
                break;
            }
            case DensityFunction.Type.Cone: {
                ring( -y, across );

                foreach( var direction in outward ) {
                    line( ( direction * across ) - new Vector3( 0.0f, y, 0.0f ), new Vector3( 0.0f, y, 0.0f ) );
                }
                break;
            }
            case DensityFunction.Type.Capsule: {
                var radius = Mathf.Min( across, y );
                var half   = y - radius;

                ring( -half, radius );
                ring(  half, radius );

                foreach( var direction in outward ) {
                    line( ( direction * radius ) - new Vector3( 0.0f, half, 0.0f ), ( direction * radius ) + new Vector3( 0.0f, half, 0.0f ) );
                }

                // the half balls at the ends, across x and across z
                foreach( var direction in new[] { Vector3.right, Vector3.forward } ) {
                    arc( new Vector3( 0.0f, half, 0.0f ),  direction * radius, Vector3.up * radius, 0.0f,     Mathf.PI );
                    arc( new Vector3( 0.0f, -half, 0.0f ), direction * radius, Vector3.up * radius, Mathf.PI, 2.0f * Mathf.PI );
                }
                break;
            }
            case DensityFunction.Type.Torus: {
                var tube   = Mathf.Min( y, across );
                var middle = across - tube;

                // outside, inside, on top and underneath, and four cuts through the tube
                ring( 0.0f, across );
                ring( 0.0f, middle - tube );
                ring( tube, middle );
                ring( -tube, middle );

                foreach( var direction in outward ) {
                    circle( direction * middle, direction * tube, Vector3.up * tube );
                }
                break;
            }
            case DensityFunction.Type.Pyramid: {
                var corners = new[] {
                    new Vector3( 0.0f, y, 0.0f ),
                    new Vector3( -across, -y, -across ), new Vector3( across, -y, -across ), new Vector3( across, -y, across ), new Vector3( -across, -y, across )
                };

                edges( corners, new[] { 1, 2, 2, 3, 3, 4, 4, 1, 0, 1, 0, 2, 0, 3, 0, 4 } );
                break;
            }
            case DensityFunction.Type.Octahedron: {
                var corners = new[] {
                    new Vector3( extents.x, 0.0f, 0.0f ), new Vector3( -extents.x, 0.0f, 0.0f ),
                    new Vector3( 0.0f, y, 0.0f ),         new Vector3( 0.0f, -y, 0.0f ),
                    new Vector3( 0.0f, 0.0f, extents.z ), new Vector3( 0.0f, 0.0f, -extents.z )
                };

                // every two corners that are not opposite each other share an edge
                for( var first = 0; first < 6; ++first ) {
                    for( var second = first + 1; second < 6; ++second ) {
                        if( first / 2 != second / 2 ) {
                            line( corners[first], corners[second] );
                        }
                    }
                }
                break;
            }
            case DensityFunction.Type.HexagonalPrism: {
                var corner  = HexagonalPrism.inradiusOf( extents ) / HexagonalPrism.cos30;
                var hexagon = Enumerable.Range( 0, 6 ).Select(
                    ( index ) => new Vector3( corner * Mathf.Cos( index * Mathf.PI / 3.0f ), 0.0f, corner * Mathf.Sin( index * Mathf.PI / 3.0f ) )
                ).ToArray( );

                for( var index = 0; index < 6; ++index ) {
                    var next = hexagon[( index + 1 ) % 6];

                    line( hexagon[index] - new Vector3( 0.0f, y, 0.0f ), next - new Vector3( 0.0f, y, 0.0f ) );
                    line( hexagon[index] + new Vector3( 0.0f, y, 0.0f ), next + new Vector3( 0.0f, y, 0.0f ) );
                    line( hexagon[index] - new Vector3( 0.0f, y, 0.0f ), hexagon[index] + new Vector3( 0.0f, y, 0.0f ) );
                }
                break;
            }
            default: {
                throw new Exception( "Unknown density function type specified" );
            }
        }

        return lines;
    }

    private const UnityEditor.GizmoType outlined = UnityEditor.GizmoType.NonSelected | UnityEditor.GizmoType.Selected | UnityEditor.GizmoType.Pickable;

    [UnityEditor.DrawGizmo( outlined )] private static void drawCuboid         ( Cuboid         volume, UnityEditor.GizmoType gizmoType ) { drawShape( volume, gizmoType ); }
    [UnityEditor.DrawGizmo( outlined )] private static void drawEllipsoid      ( Ellipsoid      volume, UnityEditor.GizmoType gizmoType ) { drawShape( volume, gizmoType ); }
    [UnityEditor.DrawGizmo( outlined )] private static void drawCylinder       ( Cylinder       volume, UnityEditor.GizmoType gizmoType ) { drawShape( volume, gizmoType ); }
    [UnityEditor.DrawGizmo( outlined )] private static void drawCone           ( Cone           volume, UnityEditor.GizmoType gizmoType ) { drawShape( volume, gizmoType ); }
    [UnityEditor.DrawGizmo( outlined )] private static void drawCapsule        ( Capsule        volume, UnityEditor.GizmoType gizmoType ) { drawShape( volume, gizmoType ); }
    [UnityEditor.DrawGizmo( outlined )] private static void drawTorus          ( Torus          volume, UnityEditor.GizmoType gizmoType ) { drawShape( volume, gizmoType ); }
    [UnityEditor.DrawGizmo( outlined )] private static void drawPyramid        ( Pyramid        volume, UnityEditor.GizmoType gizmoType ) { drawShape( volume, gizmoType ); }
    [UnityEditor.DrawGizmo( outlined )] private static void drawOctahedron     ( Octahedron     volume, UnityEditor.GizmoType gizmoType ) { drawShape( volume, gizmoType ); }
    [UnityEditor.DrawGizmo( outlined )] private static void drawHexagonalPrism ( HexagonalPrism volume, UnityEditor.GizmoType gizmoType ) { drawShape( volume, gizmoType ); }

    // how much nearer the camera an outline is drawn than it is, in cells of the selected voxelizer's grid. The contour is
    // off the shapes by up to a fraction of a cell, and where an outline lies on it, it would cut through the outline
    public const float outlineNearerByCells = 0.5f;

    // the point moved the distance toward a camera along its line of sight, which leaves it where it was on the screen
    public static Vector3 nearerTheCamera( Vector3 point, Vector3 cameraPosition, Vector3 cameraForward, bool orthographic, float distance ) {
        var toward = orthographic ? -cameraForward : ( cameraPosition - point ).normalized;

        return point + ( toward * distance );
    }

    private static void drawShape( Volume volume, UnityEditor.GizmoType gizmoType ) {
        var editor = volume.GetComponentInParent<InteractiveEditor>( true );

        if( editor == null || !editor.tryGetShape( volume, out var shape ) ) {
            return;
        }

        var colour = colourOf( shape.combination, ( gizmoType & UnityEditor.GizmoType.Selected ) != 0 );

        // the outline is made in the shape's frame at its size, so its matrix only places and turns it
        var frame = editor.transform.localToWorldMatrix * Matrix4x4.TRS( shape.origin, Quaternion.Normalize( shape.rotation ), Vector3.one );

        // in the world, and unless they are drawn over everything, moved nearer the camera that draws them
        var camera = Camera.current;
        var scale  = editor.transform.lossyScale;
        var nearer = camera == null || editor.outlinesOnTop ? 0.0f : editor.cellSize * outlineNearerByCells * Mathf.Max( Mathf.Abs( scale.x ), Mathf.Max( Mathf.Abs( scale.y ), Mathf.Abs( scale.z ) ) );

        Vector3 toWorld( Vector3 local ) {
            var world = frame.MultiplyPoint3x4( local );

            return nearer > 0.0f ? nearerTheCamera( world, camera.transform.position, camera.transform.forward, camera.orthographic, nearer ) : world;
        }

        var lines = outlineOf( shape.type, shape.extents ).Select( ( line ) => ( start: toWorld( line.start ), end: toWorld( line.end ) ) ).ToArray( );

        using( Shapes.Draw.Scope ) {
            Shapes.Draw.ResetAllDrawStates( );

            Shapes.Draw.BlendMode      = Shapes.ShapesBlendMode.Transparent;
            Shapes.Draw.LineGeometry   = Shapes.LineGeometry.Billboard;
            Shapes.Draw.ThicknessSpace = Shapes.ThicknessSpace.Pixels;

            // over the contour and everything else, so that a shape shows wherever it is, inside the mesh too. Else hidden
            // where something is in front, as everything in the scene is
            Shapes.Draw.ZTest = editor.outlinesOnTop ? UnityEngine.Rendering.CompareFunction.Always : UnityEngine.Rendering.CompareFunction.LessEqual;

            foreach( var ( start, end ) in lines ) {
                Shapes.Draw.Line( start, end, editor.outlineThickness, colour );
            }
        }

        // Unity's picking sees what Gizmos draws and not what Shapes draws. So the same lines go to Gizmos as well, without
        // colour, only to be clicked, moved nearer the camera as the outline is, so that a click on an outline that lies
        // on the contour picks the outline. Unity picks by depth, so where the contour is in front of an outline, a click
        // picks the contour, also when outlines are drawn over everything
        Gizmos.matrix = Matrix4x4.identity;
        Gizmos.color  = Color.clear;

        Gizmos.DrawLineList( lines.SelectMany( ( line ) => new[] { line.start, line.end } ).ToArray( ) );
    }

}
#endif
