using System;
using System.Collections.Generic;
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
        public Vector3              extents       { get; }
        public Combination          combination   { get; set; }
        public MaterialIndex        materialIndex { get; set; }

        public Shape( DensityFunction.Type type, Vector3 origin, Vector3 extents, Combination combination, MaterialIndex materialIndex ) {
            this.type          = type;
            this.origin        = origin;
            this.extents       = extents;
            this.combination   = combination;
            this.materialIndex = materialIndex;
        }

        public float sample( Vector3 position ) {
            return this.type switch {
                DensityFunction.Type.Cuboid    => Cuboid.sample    ( position, this.origin, this.extents ),
                DensityFunction.Type.Ellipsoid => Ellipsoid.sample ( position, this.origin, this.extents ),
                _                              => throw new Exception( "Unknown density function type specified" )
            };
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

    // the last contour, for the inspector. Not kept over a script recompile
    public Voxelizer.ContourStatistics? lastContour { get; private set; }

    // the managed implementations take seconds on larger grids, so they only contour when asked to
    public bool contoursByItself {
        get { return this.voxelizer.implementationType != ImplementationType.Managed; }
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

    // the shapes to contour, in hierarchy order, which is the order their operations apply in. Shapes on inactive
    // objects, disabled shapes and a shape on this object itself are left out
    public List<DensityFunction> shapes( ) {
        var shapes = new List<DensityFunction>( );

        foreach( var volume in this.GetComponentsInChildren<Volume>( ) ) {
            if( !volume.enabled || volume.transform == this.transform ) {
                continue;
            }

            var ( position, size ) = this.placementOf( volume.transform );

            shapes.Add( new Shape( volume.type, position, size.absolute( ) / 2, volume.combination, volume.materialIndex ) );
        }

        return shapes;
    }

    // where a transform below this object is and how large, in the object space of this object. It goes up through the
    // groups in between the way Unity does, without going through world space, so that a direct child gives exactly
    // its local position and scale. Shapes are axis aligned: their own rotation is not read, and a rotated group only
    // turns where the shapes in it are
    private ( Vector3 position, Vector3 size ) placementOf( Transform shape ) {
        var position = shape.localPosition;
        var size     = shape.localScale;

        for( var parent = shape.parent; parent != null && parent != this.transform; parent = parent.parent ) {
            var scaled = Vector3.Scale( parent.localScale, position );

            position = parent.localPosition + ( parent.localRotation == Quaternion.identity ? scaled : parent.localRotation * scaled );
            size     = Vector3.Scale( parent.localScale, size );
        }

        return ( position, size );
    }

    // contours the shapes with the selected voxelizer, whatever its implementation. The mesh is read back from the GPU,
    // because drawing straight from it only happens in play mode, and its vertices are not reordered, which takes time
    // and only makes drawing it a little faster
    public Voxelizer.ContourStatistics contour( ) {
        var statistics = this.voxelizer.contour( this.shapes( ), drawFromGPU: false, reorderVertices: false );

        // Unity draws every material past the last sub-mesh over the last sub-mesh. Empty sub-meshes up to the number
        // of materials keep each material to its own, whichever materials the shapes use at the moment
        var mesh      = this.GetComponent<MeshFilter>( ).sharedMesh;
        var materials = this.GetComponent<MeshRenderer>( ).sharedMaterials.Length;

        if( mesh != null && mesh.subMeshCount < materials ) {
            mesh.subMeshCount = materials;
        }

        this.lastContour = statistics;

#if UNITY_EDITOR
        UnityEditor.SceneView.RepaintAll( );
#endif

        return statistics;
    }

    public void Awake( ) {
        // a voxelizer contours in Start, which is only called for one that is enabled
        if( Application.isPlaying ) {
            foreach( Algorithm algorithm in Enum.GetValues( typeof( Algorithm ) ) ) {
                this.voxelizerOf( algorithm ).enabled = false;
            }
        }
    }

    // when the scene opens, after a script recompile and when play mode starts. The mesh survives a recompile, so this
    // only contours when nothing is shown
    public void OnEnable( ) {
        this.showSelected( );

        if( this.contoursByItself && this.GetComponent<MeshFilter>( ).sharedMesh == null ) {
            this.contour( );
        }
    }

    // also just before a script recompile, which would lose track of the compute buffers
    public void OnDisable( ) {
        foreach( Algorithm algorithm in Enum.GetValues( typeof( Algorithm ) ) ) {
            this.voxelizerOf( algorithm )?.release( );
        }
    }

    public void OnDestroy( ) {
        foreach( Algorithm algorithm in Enum.GetValues( typeof( Algorithm ) ) ) {
            var voxelizer = this.voxelizerOf( algorithm );

            if( voxelizer != null ) {
                voxelizer.clear( );
                voxelizer.release( );
            }
        }
    }

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

        UnityEditor.EditorGUILayout.Space( );

        if( GUILayout.Button( "Contour" ) ) {
            editor.contour( );
        }

        if( editor.lastContour is Voxelizer.ContourStatistics statistics ) {
            UnityEditor.EditorGUILayout.LabelField( $"Last contour: {statistics.milliseconds:F1} ms, {statistics.vertexCount:N0} vertices, {statistics.triangleCount:N0} triangles" );
        }
    }

}
#endif
