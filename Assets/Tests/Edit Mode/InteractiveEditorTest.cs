using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using UnityEngine;

using Object             = UnityEngine.Object;
using Algorithm          = InteractiveEditor.Algorithm;
using Shape              = InteractiveEditor.Shape;
using Combination        = DensityFunction.Combination;
using MaterialIndex      = SurfaceExtractor.MaterialIndex;
using ImplementationType = SurfaceExtractor.Implementation.Type;
using VertexNormals      = SurfaceExtractor.VertexNormals;
using QEFSolverType      = QEFSolver.Type;
using Approximation      = SurfaceExtractor.IntersectionApproximation;

// the interactive editor in edit mode: choosing a voxelizer, reading shapes off their transforms, contouring, and what
// it does when it is enabled, disabled and destroyed. Everything is made hidden and is not saved to the open scene
public class InteractiveEditorTest {

    public readonly struct Variant {

        public readonly Algorithm          algorithm;
        public readonly ImplementationType implementation;

        public Variant( Algorithm algorithm, ImplementationType implementation ) {
            this.algorithm      = algorithm;
            this.implementation = implementation;
        }

        public override string ToString( ) {
            return $"{this.algorithm} {this.implementation}";
        }

    }

    private static readonly Variant[] variants = {
        new( Algorithm.Uniform,  ImplementationType.Managed ),
        new( Algorithm.Uniform,  ImplementationType.Burst ),
        new( Algorithm.Uniform,  ImplementationType.GPU ),
        new( Algorithm.Adaptive, ImplementationType.Managed ),
        new( Algorithm.Adaptive, ImplementationType.Burst ),
        new( Algorithm.Manifold, ImplementationType.Managed ),
        new( Algorithm.Manifold, ImplementationType.Burst )
    };

    // the variants that contour by themselves
    private static readonly Variant[] automaticVariants = variants.Where( ( variant ) => variant.implementation != ImplementationType.Managed ).ToArray( );

    private readonly List<Object> created = new( );

    // the editors' clock, in seconds, and whether Unity counts as the active application, both the test's own
    private double time        = 0.0;
    private bool   unityActive = true;

    [TearDown]
    public void tearDown( ) {
        // destroying the editor clears its contour and releases the compute buffers
        foreach( var item in this.created ) {
            Object.DestroyImmediate( item );
        }
        this.created.Clear( );
    }

    private InteractiveEditor createEditor( float scale = 1.0f ) {
        var gameObject = new GameObject( "Interactive Editor" ) { hideFlags = HideFlags.HideAndDontSave };

        this.created.Add( gameObject );

        gameObject.transform.localScale = Vector3.one * scale;

        var editor = gameObject.AddComponent<InteractiveEditor>( );

        editor.clock             = ( ) => this.time;
        editor.applicationActive = ( ) => this.unityActive;

        return editor;
    }

    // editor updates at the given times
    private void tickAt( InteractiveEditor editor, params double[] times ) {
        foreach( var time in times ) {
            this.time = time;
            editor.tick( );
        }
    }

    private static Transform createGroup( Transform parent, Vector3 position, float scale ) {
        var group = new GameObject( "Group" ) { hideFlags = HideFlags.HideAndDontSave };

        group.transform.SetParent( parent, false );
        group.transform.localPosition = position;
        group.transform.localScale    = Vector3.one * scale;

        return group.transform;
    }

    private static Volume addShape<T>( Transform parent, Vector3 position, Vector3 scale, Combination combination = Combination.Union, MaterialIndex materialIndex = MaterialIndex.Material0 ) where T : Volume {
        return addShape( typeof( T ), parent, position, scale, combination, materialIndex );
    }

    private static Volume addShape( System.Type component, Transform parent, Vector3 position, Vector3 scale, Combination combination = Combination.Union, MaterialIndex materialIndex = MaterialIndex.Material0 ) {
        var shape = new GameObject( component.Name ) { hideFlags = HideFlags.HideAndDontSave };

        shape.transform.SetParent( parent, false );
        shape.transform.localPosition = position;
        shape.transform.localScale    = scale;

        var volume = ( Volume )shape.AddComponent( component );

        volume.combination   = combination;
        volume.materialIndex = materialIndex;

        return volume;
    }

    // a rounded cube with a bite out of one corner and a ball of a second material sitting in the bite, the model of
    // the scene
    private static void addStarterModel( Transform parent ) {
        addShape<Cuboid>    ( parent, Vector3.zero,                         Vector3.one * 0.6f );
        addShape<Ellipsoid> ( parent, Vector3.zero,                         Vector3.one * 0.8f, Combination.Intersection );
        addShape<Ellipsoid> ( parent, new Vector3( 0.25f, 0.25f, -0.25f ),  Vector3.one * 0.6f, Combination.Subtraction );
        addShape<Ellipsoid> ( parent, new Vector3( 0.12f, 0.12f, -0.12f ),  Vector3.one * 0.24f, Combination.Union, MaterialIndex.Material1 );
    }

    private static Mesh shown( InteractiveEditor editor ) {
        return editor.GetComponent<MeshFilter>( ).sharedMesh;
    }

    private static InteractiveEditor configure( InteractiveEditor editor, Variant variant ) {
        editor.select( variant.algorithm );

        editor.voxelizer.implementationType = variant.implementation;
        editor.voxelizer.resolution         = variant.algorithm == Algorithm.Uniform ? 4 : 5;

        return editor;
    }

    [Test]
    public void addingTheEditorAddsItsVoxelizersAndMeshFilter( ) {
        var editor = this.createEditor( );

        Assert.IsNotNull( editor.GetComponent<UniformDualContouring>( ) );
        Assert.IsNotNull( editor.GetComponent<AdaptiveDualContouring>( ) );
        Assert.IsNotNull( editor.GetComponent<ManifoldDualContouring>( ) );
        Assert.IsNotNull( editor.GetComponent<MeshFilter>( ) );
        Assert.AreEqual( Algorithm.Uniform, editor.algorithm );
    }

    [Test]
    public void onlyTheSelectedVoxelizerShowsInTheInspector( [Values] Algorithm algorithm ) {
        var editor = this.createEditor( );

        editor.select( algorithm );

        foreach( Algorithm other in Enum.GetValues( typeof( Algorithm ) ) ) {
            var hidden = ( editor.voxelizerOf( other ).hideFlags & HideFlags.HideInInspector ) == HideFlags.HideInInspector;

            Assert.AreEqual( other != algorithm, hidden, other.ToString( ) );
        }

        Assert.AreSame( editor.voxelizerOf( algorithm ), editor.voxelizer );
    }

    [Test]
    public void selectingAnAlgorithmCarriesTheCommonSettingsOver( [Values( Algorithm.Adaptive, Algorithm.Manifold )] Algorithm algorithm ) {
        var editor  = this.createEditor( );
        var uniform = editor.voxelizer;

        uniform.resolution                  = 5;
        uniform.minimizerIterations         = 3;
        uniform.binarySearchIterations      = 9;
        uniform.surfaceCorrectionIterations = 2;
        uniform.implementationType          = ImplementationType.Burst;
        uniform.qefSolverType               = QEFSolverType.Simple;
        uniform.intersectionApproximation   = Approximation.LinearInterpolation;
        uniform.vertexNormals               = VertexNormals.Split;
        uniform.debugFlags                  = Voxelizer.Debug.Cells | Voxelizer.Debug.Edges;

        editor.select( algorithm );

        var octree = editor.voxelizer;

        // 2^5 cells across is 2^5 leaves across, at resolution 6
        Assert.AreEqual( 6,                                              octree.resolution );
        Assert.AreEqual( 3,                                              octree.minimizerIterations );
        Assert.AreEqual( 9,                                              octree.binarySearchIterations );
        Assert.AreEqual( 2,                                              octree.surfaceCorrectionIterations );
        Assert.AreEqual( ImplementationType.Burst,                       octree.implementationType );
        Assert.AreEqual( QEFSolverType.Simple,                           octree.qefSolverType );
        Assert.AreEqual( Approximation.LinearInterpolation,              octree.intersectionApproximation );
        Assert.AreEqual( VertexNormals.Split,                            octree.vertexNormals );
        Assert.AreEqual( Voxelizer.Debug.Cells | Voxelizer.Debug.Edges,  octree.debugFlags );

        editor.select( Algorithm.Uniform );

        Assert.AreEqual( 5, uniform.resolution, "back to the same number of cells" );
    }

    [Test]
    public void selectingAnOctreeTurnsGPUIntoBurst( [Values( Algorithm.Adaptive, Algorithm.Manifold )] Algorithm algorithm ) {
        var editor = this.createEditor( );

        editor.voxelizer.implementationType = ImplementationType.GPU;

        editor.select( algorithm );

        Assert.AreEqual( ImplementationType.Burst, editor.voxelizer.implementationType );
    }

    [Test]
    public void selectingKeepsWhatOnlyOneAlgorithmHas( ) {
        var editor   = this.createEditor( );
        var adaptive = editor.voxelizerOf( Algorithm.Adaptive ) as AdaptiveDualContouring;
        var manifold = editor.voxelizerOf( Algorithm.Manifold ) as ManifoldDualContouring;

        adaptive.simplification  = true;
        adaptive.errorThreshold  = 0.125f;
        adaptive.topologySafe    = false;
        manifold.errorThreshold  = 0.25f;
        manifold.enforceManifold = false;

        editor.select( Algorithm.Adaptive );
        editor.select( Algorithm.Manifold );
        editor.select( Algorithm.Uniform );

        Assert.IsTrue  ( adaptive.simplification );
        Assert.AreEqual( 0.125f, adaptive.errorThreshold );
        Assert.IsFalse ( adaptive.topologySafe );
        Assert.AreEqual( 0.25f, manifold.errorThreshold );
        Assert.IsFalse ( manifold.enforceManifold );
    }

    // an octree of resolution 8 has 128^3 leaves, a uniform grid of resolution 2 has 4^3 cells: the slider's ends
    [Test]
    public void resolutionStaysOnTheSlider( ) {
        var editor = this.createEditor( );

        editor.voxelizer.resolution = 8;
        editor.select( Algorithm.Adaptive );

        Assert.AreEqual( 8, editor.voxelizer.resolution );

        editor.voxelizer.resolution = 2;
        editor.select( Algorithm.Uniform );

        Assert.AreEqual( 2, editor.voxelizer.resolution );
    }

    [Test]
    public void aShapeIsWhereItsTransformPutsIt( ) {
        var editor = this.createEditor( 5.0f );

        addShape<Cuboid>( editor.transform, new Vector3( 0.25f, -0.125f, 0.0625f ), new Vector3( 0.5f, 0.25f, 0.125f ), Combination.Subtraction, MaterialIndex.Material1 );

        var shape = editor.shapes( ).Single( ) as Shape;

        Assert.AreEqual( DensityFunction.Type.Cuboid,                 shape.type );
        Assert.AreEqual( new Vector3( 0.25f, -0.125f, 0.0625f ),      shape.origin );
        Assert.AreEqual( new Vector3( 0.25f, 0.125f, 0.0625f ),       shape.extents );
        Assert.AreEqual( Combination.Subtraction,                     shape.combination );
        Assert.AreEqual( MaterialIndex.Material1,                     shape.materialIndex );
    }

    [Test]
    public void groupsMoveAndScaleTheShapesInThem( ) {
        var editor = this.createEditor( 5.0f );
        var outer  = createGroup( editor.transform, new Vector3( 0.25f, 0.0f, 0.0f ), 0.5f );
        var inner  = createGroup( outer,            new Vector3( 0.0f, 0.5f, 0.0f ),  2.0f );

        addShape<Ellipsoid>( inner, new Vector3( 0.25f, 0.0f, 0.0f ), Vector3.one * 0.5f );

        var shape = editor.shapes( ).Single( );

        // inner: (0, 0.5, 0) + 2 (0.25, 0, 0) = (0.5, 0.5, 0), size 1. Outer: (0.25, 0, 0) + 0.5 (0.5, 0.5, 0)
        Assert.AreEqual( new Vector3( 0.5f, 0.25f, 0.0f ), shape.origin );
        Assert.AreEqual( Vector3.one * 0.25f,              shape.extents );
    }

    [Test]
    public void aRotatedGroupTurnsWhereItsShapesAre( ) {
        var editor = this.createEditor( );
        var group  = createGroup( editor.transform, Vector3.zero, 1.0f );

        group.localRotation = Quaternion.Euler( 0.0f, 90.0f, 0.0f );

        addShape<Ellipsoid>( group, new Vector3( 0.25f, 0.0f, 0.0f ), Vector3.one * 0.5f );

        var origin = editor.shapes( ).Single( ).origin;

        Assert.That( Vector3.Distance( new Vector3( 0.0f, 0.0f, -0.25f ), origin ), Is.LessThan( 1e-6f ), origin.ToString( "F6" ) );
    }

    [Test]
    public void aNegativeScaleGivesAPositiveSize( ) {
        var editor = this.createEditor( );

        addShape<Cuboid>( editor.transform, Vector3.zero, new Vector3( -0.5f, 0.5f, -0.25f ) );

        Assert.AreEqual( new Vector3( 0.25f, 0.25f, 0.125f ), editor.shapes( ).Single( ).extents );
    }

    // the corners of the unit cube around a shape's origin
    private static readonly Vector3[] unitCorners = Enumerable.Range( 0, 8 ).Select(
        ( corner ) => new Vector3( ( corner & 1 ) == 0 ? -0.5f : 0.5f, ( corner & 2 ) == 0 ? -0.5f : 0.5f, ( corner & 4 ) == 0 ? -0.5f : 0.5f )
    ).ToArray( );

    // the shape is contoured where Unity shows it: the corners of its box, through its transform and the groups above
    // it, are those of the box the editor contours. A box mirrored along its own axes is the same box, so the corners
    // are compared as a set
    private static void assertWhereUnityShowsIt( InteractiveEditor editor, Volume volume ) {
        Assert.IsTrue( editor.tryGetShape( volume, out var shape ), "the shape is not contoured" );

        var contoured = unitCorners.Select( ( corner ) => shape.origin + ( shape.rotation * Vector3.Scale( corner, shape.extents * 2 ) ) ).ToArray( );

        foreach( var corner in unitCorners ) {
            var shown   = editor.transform.InverseTransformPoint( volume.transform.TransformPoint( corner ) );
            var nearest = contoured.Min( ( other ) => Vector3.Distance( other, shown ) );

            Assert.That( nearest, Is.LessThan( 1e-5f ), $"the corner Unity shows at {shown.ToString( "F5" )}" );
        }
    }

    // for a shape right below the editor that is its local rotation, at any scale of the editor
    [Test]
    public void aTurnedShapeIsTurnedByItsRotation( ) {
        var editor = this.createEditor( 5.0f );
        var volume = addShape<Cuboid>( editor.transform, new Vector3( 0.125f, -0.0625f, 0.0f ), new Vector3( 0.5f, 0.25f, 0.125f ) );

        volume.transform.localRotation = Quaternion.Euler( 30.0f, -60.0f, 75.0f );

        Assert.IsTrue( editor.tryGetShape( volume, out var shape ) );
        Assert.AreEqual( volume.transform.localRotation, shape.rotation );

        assertWhereUnityShowsIt( editor, volume );
    }

    [Test]
    public void theTurnsOfGroupsAddUp( ) {
        var editor = this.createEditor( 5.0f );
        var outer  = createGroup( editor.transform, new Vector3( 0.125f, 0.0f, 0.0f ), 2.0f );
        var inner  = createGroup( outer,            new Vector3( 0.0f, 0.0625f, 0.0f ), 0.25f );

        outer.localRotation = Quaternion.Euler( 20.0f, 0.0f, 0.0f );
        inner.localRotation = Quaternion.Euler( 0.0f, 90.0f, 0.0f );

        var volume = addShape<Ellipsoid>( inner, new Vector3( 0.25f, 0.0f, 0.125f ), new Vector3( 1.0f, 0.5f, 0.25f ) );

        volume.transform.localRotation = Quaternion.Euler( 0.0f, 0.0f, 30.0f );

        assertWhereUnityShowsIt( editor, volume );
    }

    // a group's uneven scale lies along the axes of a shape that is not turned within it, or turned by quarter turns,
    // and along those of a shape turned about the axis the scale is uneven along. Each stays a box. A mirrored group
    // too
    [Test]
    public void aGroupScaledUnevenlyStretchesAShapeAlongItsAxes(
        [Values( "unturned", "quarter turn", "turned about the uneven axis", "mirrored" )] string turn
    ) {
        var editor = this.createEditor( 5.0f );
        var group  = createGroup( editor.transform, new Vector3( 0.0f, 0.125f, 0.0f ), 1.0f );

        group.localRotation = Quaternion.Euler( 0.0f, 30.0f, 0.0f );
        group.localScale    = turn == "mirrored" ? new Vector3( -1.0f, 1.0f, 1.0f ) : new Vector3( 1.0f, 1.0f, 2.0f );

        var volume = addShape<Cuboid>( group, new Vector3( 0.125f, 0.0f, 0.0f ), new Vector3( 0.5f, 0.25f, 0.125f ) );

        volume.transform.localRotation = turn switch {
            "unturned"                     => Quaternion.identity,
            "quarter turn"                 => Quaternion.Euler( 0.0f, 90.0f, 0.0f ),
            "turned about the uneven axis" => Quaternion.Euler( 0.0f, 0.0f, 40.0f ),
            _                              => Quaternion.Euler( 30.0f, -60.0f, 75.0f )
        };

        assertWhereUnityShowsIt( editor, volume );

        CollectionAssert.IsEmpty( editor.warnings( ).Where( ( warning ) => warning.Contains( "slant" ) ), "the shape counts as sheared" );
    }

    [Test]
    public void shapesAreInHierarchyOrder( ) {
        var editor = this.createEditor( );

        var first  = addShape<Cuboid>    ( editor.transform, Vector3.zero, Vector3.one * 0.5f, Combination.Union );
        var second = addShape<Ellipsoid> ( editor.transform, Vector3.zero, Vector3.one * 0.5f, Combination.Subtraction );
        var third  = addShape<Cuboid>    ( editor.transform, Vector3.zero, Vector3.one * 0.5f, Combination.Intersection );

        CollectionAssert.AreEqual(
            new[] { Combination.Union, Combination.Subtraction, Combination.Intersection },
            editor.shapes( ).Select( ( shape ) => shape.combination )
        );

        third.transform.SetSiblingIndex( 0 );

        CollectionAssert.AreEqual(
            new[] { Combination.Intersection, Combination.Union, Combination.Subtraction },
            editor.shapes( ).Select( ( shape ) => shape.combination )
        );
    }

    [Test]
    public void inactiveDisabledAndOwnShapesAreLeftOut( ) {
        var editor = this.createEditor( );

        addShape<Cuboid>( editor.transform, Vector3.zero, Vector3.one * 0.5f, Combination.Union );

        addShape<Cuboid>( editor.transform, Vector3.zero, Vector3.one * 0.5f, Combination.Subtraction ).gameObject.SetActive( false );
        addShape<Cuboid>( editor.transform, Vector3.zero, Vector3.one * 0.5f, Combination.Subtraction ).enabled = false;

        var inactiveGroup = createGroup( editor.transform, Vector3.zero, 1.0f );
        addShape<Cuboid>( inactiveGroup, Vector3.zero, Vector3.one * 0.5f, Combination.Subtraction );
        inactiveGroup.gameObject.SetActive( false );

        editor.gameObject.AddComponent<Ellipsoid>( ).combination = Combination.Subtraction;

        CollectionAssert.AreEqual( new[] { Combination.Union }, editor.shapes( ).Select( ( shape ) => shape.combination ) );
    }

    // with its object at scale 1, the editor's reading of the transforms is the volumes' reading
    [Test]
    public void atScaleOneTheContourIsThatOfTheVolumes( [ValueSource( nameof( variants ) )] Variant variant ) {
        var editor = configure( this.createEditor( ), variant );

        addStarterModel( editor.transform );

        editor.contour( );

        var mesh = Object.Instantiate( shown( editor ) );

        try {
            editor.voxelizer.contour( drawFromGPU: false );

            MeshComparison.assertSameTriangles( shown( editor ), mesh );
        }
        finally {
            Object.DestroyImmediate( mesh );
        }
    }

    // and with turned shapes, whose rotation the volumes read off their transforms as well
    [Test]
    public void atScaleOneTheContourOfTurnedShapesIsThatOfTheVolumes( [ValueSource( nameof( variants ) )] Variant variant ) {
        var editor = configure( this.createEditor( ), variant );

        addStarterModel( editor.transform );

        var volumes = editor.volumes( );

        volumes[0].transform.localRotation = Quaternion.Euler( 20.0f, 30.0f, 40.0f );
        volumes[2].transform.localRotation = Quaternion.Euler( 0.0f, 45.0f, 0.0f );
        volumes[2].transform.localScale    = new Vector3( 0.6f, 0.4f, 0.3f );

        editor.contour( );

        var mesh = Object.Instantiate( shown( editor ) );

        try {
            editor.voxelizer.contour( drawFromGPU: false );

            MeshComparison.assertSameTriangles( shown( editor ), mesh );
        }
        finally {
            Object.DestroyImmediate( mesh );
        }
    }

    // the shapes of the article other than the box and the ellipsoid, each turned, placed much as in TopologyTest.newShapes
    private static void addNewShapes( Transform parent ) {
        void add( System.Type component, Vector3 position, Vector3 rotation, Vector3 scale, Combination combination = Combination.Union, MaterialIndex materialIndex = MaterialIndex.Material0 ) {
            addShape( component, parent, position, scale, combination, materialIndex ).transform.localRotation = Quaternion.Euler( rotation );
        }

        add( typeof( Cylinder ),       new Vector3( -0.2f,  0.15f,  0.0f  ), new Vector3( 20.0f,  0.0f,  30.0f  ), new Vector3( 0.24f, 0.36f, 0.24f ) );
        add( typeof( Cone ),           new Vector3(  0.2f,  0.15f,  0.05f ), new Vector3( -30.0f, 40.0f, 10.0f  ), new Vector3( 0.28f, 0.36f, 0.28f ) );
        add( typeof( Torus ),          new Vector3(  0.0f,  0.1f,   0.0f  ), new Vector3( 70.0f,  10.0f, 0.0f   ), new Vector3( 0.64f, 0.16f, 0.64f ), Combination.Union, MaterialIndex.Material1 );
        add( typeof( Capsule ),        new Vector3(  0.0f, -0.22f, -0.15f ), new Vector3( 0.0f,   45.0f, 80.0f  ), new Vector3( 0.2f,  0.44f, 0.2f  ) );
        add( typeof( Octahedron ),     new Vector3(  0.05f, -0.22f, -0.15f ), new Vector3( 10.0f,  20.0f, 30.0f  ), new Vector3( 0.2f,  0.2f,  0.2f  ), Combination.Subtraction );
        add( typeof( Pyramid ),        new Vector3( -0.22f, -0.2f,   0.2f  ), new Vector3( 0.0f,   30.0f, 15.0f  ), new Vector3( 0.26f, 0.24f, 0.26f ) );
        add( typeof( HexagonalPrism ), new Vector3(  0.22f, -0.2f,   0.22f ), new Vector3( 40.0f,  0.0f,  -20.0f ), new Vector3( 0.26f, 0.2f,  0.24f ) );
    }

    // and with every other type of shape
    [Test]
    public void atScaleOneTheContourOfTheNewShapesIsThatOfTheVolumes( [ValueSource( nameof( variants ) )] Variant variant ) {
        var editor = configure( this.createEditor( ), variant );

        addNewShapes( editor.transform );

        editor.contour( );

        var mesh = Object.Instantiate( shown( editor ) );

        try {
            Assert.Greater( mesh.vertexCount, 0, "the mesh is empty" );

            editor.voxelizer.contour( drawFromGPU: false );

            MeshComparison.assertSameTriangles( shown( editor ), mesh );
        }
        finally {
            Object.DestroyImmediate( mesh );
        }
    }

    // even boxes and uneven ones, wider than high and higher than wide
    private static readonly Vector3[] outlineBoxes = {
        new( 0.25f, 0.25f, 0.25f ),
        new( 0.3f,  0.1f,  0.3f  ),
        new( 0.15f, 0.4f,  0.35f ),
        new( 0.4f,  0.2f,  0.12f )
    };

    // an outline is drawn where the shape is: every line of it begins and ends on the shape, and stays in its box
    [Test]
    public void everyLineOfAnOutlineEndsOnTheShape( [Values] DensityFunction.Type type ) {
        foreach( var extents in outlineBoxes ) {
            var lines = InteractiveEditorGizmos.outlineOf( type, extents );

            Assert.Greater( lines.Count, 0, $"lines in the box of {extents}" );

            foreach( var point in lines.SelectMany( ( line ) => new[] { line.start, line.end } ) ) {
                Assert.That( Mathf.Abs( DensityFunction.sample( type, point, extents ) ), Is.LessThanOrEqualTo( 2e-6f ), $"density at {point:F4}, in the box of {extents}" );
                Assert.That( point.absolute( ), Is.EqualTo( Vector3.Min( point.absolute( ), extents * 1.000001f ) ), $"{point:F4} lies outside the box of {extents}" );
            }
        }
    }

    // an outline drawn nearer the camera, so that the contour does not cut through it, shows where it would on the screen,
    // in front of where it is. Seen by a camera of each kind, at points ahead of it and off to the side
    [Test]
    public void anOutlineDrawnNearerTheCameraStaysWhereItIsOnTheScreen( [Values] bool orthographic ) {
        var gameObject = new GameObject( "Camera" ) { hideFlags = HideFlags.HideAndDontSave };
        this.created.Add( gameObject );

        var camera = gameObject.AddComponent<Camera>( );

        camera.orthographic       = orthographic;
        camera.transform.position = new Vector3( 1.0f, 2.0f, -3.0f );
        camera.transform.rotation = Quaternion.Euler( 20.0f, -15.0f, 0.0f );

        var points = new[] { new Vector3( 0.0f, 0.5f, 1.0f ), new Vector3( 1.5f, -0.5f, 2.0f ), new Vector3( -2.0f, 1.0f, 4.0f ) };

        foreach( var point in points ) {
            var moved  = InteractiveEditorGizmos.nearerTheCamera( point, camera.transform.position, camera.transform.forward, orthographic, 0.05f );
            var before = camera.WorldToScreenPoint( point );
            var after  = camera.WorldToScreenPoint( moved );

            Assert.AreEqual( 0.05f, Vector3.Distance( point, moved ), 1e-5f, $"how far {point} moved" );
            Assert.AreEqual( before.x, after.x, 1e-2f, $"where {point} shows across" );
            Assert.AreEqual( before.y, after.y, 1e-2f, $"where {point} shows up" );
            Assert.Less( after.z, before.z, $"{point} is nearer the camera" );
        }
    }

    // outlines are moved nearer the camera by half a cell of the selected voxelizer's grid, unless they are drawn over
    // everything, which they are not unless set to be
    [Test]
    public void theCellSizeIsThatOfTheSelectedGrid( [Values] Algorithm algorithm ) {
        var editor = this.createEditor( );

        Assert.IsFalse( editor.outlinesOnTop, "outlines on top" );

        editor.select( algorithm );
        editor.voxelizer.resolution = algorithm == Algorithm.Uniform ? 5 : 6;

        // 32 cells across, or 32 leaves
        Assert.AreEqual( 1.0f / 32.0f, editor.cellSize );
    }

    // outlines are opaque, so that where two lines of one overlap they show no brighter, and those of shapes that are not
    // selected are dimmer
    [Test]
    public void outlinesAreOpaqueAndDimmerUnlessSelected( [Values] Combination combination ) {
        var selected = InteractiveEditorGizmos.colourOf( combination, true );
        var dimmed   = InteractiveEditorGizmos.colourOf( combination, false );

        Assert.AreEqual( 1.0f, selected.a, "selected" );
        Assert.AreEqual( 1.0f, dimmed.a,   "not selected" );
        Assert.Less( dimmed.grayscale, selected.grayscale, "not selected is dimmer" );
    }

    // the outline of a shape with flat faces is its edges: as many lines as it has, each ending in two of its corners
    [Test]
    public void theOutlineOfAShapeWithFlatFacesIsItsEdges( [Values( DensityFunction.Type.Cuboid, DensityFunction.Type.Pyramid, DensityFunction.Type.Octahedron, DensityFunction.Type.HexagonalPrism )] DensityFunction.Type type ) {
        var extents = new Vector3( 0.3f, 0.2f, 0.25f );
        var edges   = InteractiveEditorGizmos.outlineOf( type, extents );

        var expected = type switch {
            DensityFunction.Type.Cuboid     => 12,
            DensityFunction.Type.Pyramid    => 8,
            DensityFunction.Type.Octahedron => 12,
            _                               => 18
        };

        Assert.AreEqual( expected, edges.Count );

        // the middle of every edge is on the shape too, as a straight edge is
        foreach( var ( start, end ) in edges ) {
            Assert.That( Mathf.Abs( DensityFunction.sample( type, ( start + end ) / 2.0f, extents ) ), Is.LessThanOrEqualTo( 2e-6f ), $"the middle of the edge from {start:F4} to {end:F4}" );
        }
    }

    // a cylinder, cone, capsule, torus or pyramid is as wide along x as along z, and is contoured as wide as the narrower
    [Test]
    public void aRoundOrSquareShapeOfTwoWidthsIsWarnedAbout( [ValueSource( typeof( ShapeDistanceTest ), nameof( ShapeDistanceTest.roundOrSquare ) )] DensityFunction.Type type, [Values] bool widerAlongX ) {
        var editor = this.createEditor( );

        this.giveMaterials( editor, 1 );

        var volume = addShape( Volume.componentOf( type ), editor.transform, Vector3.zero, widerAlongX ? new Vector3( 0.6f, 0.4f, 0.3f ) : new Vector3( 0.3f, 0.4f, 0.6f ) );

        assertOneWarningAbout( editor, volume.name, "narrower" );

        volume.transform.localScale = new Vector3( 0.3f, 0.4f, 0.3f );

        CollectionAssert.IsEmpty( editor.warnings( ), "as wide one way as the other" );
    }

    // the others take both widths as they are, and the hexagonal prism is the largest regular one in any box
    [Test]
    public void otherShapesOfTwoWidthsAreFine( [Values( DensityFunction.Type.Cuboid, DensityFunction.Type.Ellipsoid, DensityFunction.Type.Octahedron, DensityFunction.Type.HexagonalPrism )] DensityFunction.Type type ) {
        var editor = this.createEditor( );

        this.giveMaterials( editor, 1 );

        addShape( Volume.componentOf( type ), editor.transform, Vector3.zero, new Vector3( 0.6f, 0.4f, 0.3f ) );

        CollectionAssert.IsEmpty( editor.warnings( ) );
    }

    // a cube turned a quarter of a right angle about the vertical reaches out to its corners, half its diagonal, along
    // the two other axes. Its vertices go onto those corners, to within a binary search of a cell. It sits off the grid
    // planes, so that its top and bottom faces do not lie on them
    [Test]
    public void aTurnedCubeIsContouredTurned( [ValueSource( nameof( variants ) )] Variant variant ) {
        var editor = configure( this.createEditor( 5.0f ), variant );

        addShape<Cuboid>( editor.transform, new Vector3( 0.013f, -0.021f, 0.017f ), Vector3.one * 0.5f ).transform.localRotation = Quaternion.Euler( 0.0f, 45.0f, 0.0f );

        editor.contour( );

        var reach = shown( editor ).bounds.extents;
        var half  = 0.25f * Mathf.Sqrt( 2.0f );

        Assert.That( reach.x, Is.EqualTo( half  ).Within( 0.002f ), "x" );
        Assert.That( reach.y, Is.EqualTo( 0.25f ).Within( 0.002f ), "y" );
        Assert.That( reach.z, Is.EqualTo( half  ).Within( 0.002f ), "z" );
    }

    [Test]
    public void theScaleOfTheEditorDoesNotChangeTheContour( ) {
        var small = configure( this.createEditor( 1.0f ), new( Algorithm.Uniform, ImplementationType.Burst ) );
        var large = configure( this.createEditor( 5.0f ), new( Algorithm.Uniform, ImplementationType.Burst ) );

        addStarterModel( small.transform );
        addStarterModel( large.transform );

        small.contour( );
        large.contour( );

        MeshComparison.assertSameTriangles( shown( small ), shown( large ) );
    }

    [Test]
    public void everyVariantContoursTheStarterModel( [ValueSource( nameof( variants ) )] Variant variant ) {
        var editor = configure( this.createEditor( 5.0f ), variant );

        addStarterModel( editor.transform );

        var statistics = editor.contour( );
        var mesh       = shown( editor );

        Assert.Greater( mesh.vertexCount, 0 );
        Assert.AreEqual( 2, mesh.subMeshCount, "both materials show" );
        Assert.Greater( mesh.GetIndexCount( 0 ), 0 );
        Assert.Greater( mesh.GetIndexCount( 1 ), 0 );

        Assert.AreEqual( mesh.vertexCount, statistics.vertexCount );
        Assert.AreEqual( statistics.milliseconds, editor.lastContour?.milliseconds );
    }

    // Unity draws every material past the last sub-mesh over the last sub-mesh, so without a sub-mesh of its own the
    // last of the renderer's materials would colour the surface of the highest material on it
    [Test]
    public void everyMaterialOfTheRendererHasASubMesh( [ValueSource( nameof( variants ) )] Variant variant ) {
        var editor = configure( this.createEditor( ), variant );

        addStarterModel( editor.transform );

        var material = new Material( Shader.Find( "Universal Render Pipeline/Lit" ) );
        this.created.Add( material );

        editor.GetComponent<MeshRenderer>( ).sharedMaterials = new[] { material, material, material, material };

        editor.contour( );

        var mesh = shown( editor );

        Assert.AreEqual( 4, mesh.subMeshCount );
        Assert.Greater( mesh.GetIndexCount( 0 ), 0 );
        Assert.Greater( mesh.GetIndexCount( 1 ), 0 );
        Assert.AreEqual( 0, mesh.GetIndexCount( 2 ) );
        Assert.AreEqual( 0, mesh.GetIndexCount( 3 ) );

        // the triangles stay those of the contour
        var statistics = editor.lastContour.Value;

        Assert.AreEqual( statistics.triangleCount, ( int )( mesh.GetIndexCount( 0 ) + mesh.GetIndexCount( 1 ) ) / 3 );
    }

    [Test]
    public void managedDoesNotContourWhenEnabled( [Values] Algorithm algorithm ) {
        var editor = configure( this.createEditor( ), new( algorithm, ImplementationType.Managed ) );

        addStarterModel( editor.transform );

        editor.enabled = false;
        editor.enabled = true;

        this.tickAt( editor, 1.0, 2.0 );

        Assert.IsTrue( shown( editor ) == null );
        Assert.IsNull( editor.lastContour );
        Assert.AreEqual( RemeshScheduler.State.Manual, editor.state );
    }

    [Test]
    public void burstAndGPUContourWhenEnabledWithNothingShown( [ValueSource( nameof( automaticVariants ) )] Variant variant ) {
        var editor = configure( this.createEditor( ), variant );

        addStarterModel( editor.transform );

        editor.enabled = false;
        editor.enabled = true;

        // without waiting for changes to settle
        this.tickAt( editor, 1.0 );

        Assert.IsTrue( shown( editor ) != null );
        Assert.Greater( shown( editor ).vertexCount, 0 );
    }

    // so a script recompile, which disables and enables everything, costs no contour and keeps a managed mesh
    [Test]
    public void disablingAndEnablingKeepsTheMeshThatIsShown( [Values( ImplementationType.Managed, ImplementationType.Burst )] ImplementationType implementation ) {
        var editor = configure( this.createEditor( ), new( Algorithm.Uniform, implementation ) );

        addStarterModel( editor.transform );

        editor.contour( );

        var mesh = shown( editor );

        editor.enabled = false;

        Assert.AreSame( mesh, shown( editor ), "after disabling" );

        editor.enabled = true;

        Assert.AreSame( mesh, shown( editor ), "after enabling" );
    }

    // the GPU implementation makes its compute buffers anew after the editor was disabled
    [Test]
    public void gpuContoursAfterTheEditorWasDisabled( ) {
        var editor = configure( this.createEditor( ), new( Algorithm.Uniform, ImplementationType.GPU ) );

        addStarterModel( editor.transform );

        editor.contour( );

        var first = Object.Instantiate( shown( editor ) );

        try {
            editor.enabled = false;
            editor.enabled = true;

            editor.contour( );

            MeshComparison.assertSameTriangles( first, shown( editor ) );
        }
        finally {
            Object.DestroyImmediate( first );
        }
    }

    [Test]
    public void removingTheEditorRemovesTheMesh( ) {
        var editor = configure( this.createEditor( ), new( Algorithm.Uniform, ImplementationType.Burst ) );

        addStarterModel( editor.transform );

        editor.contour( );

        var mesh       = shown( editor );
        var meshFilter = editor.GetComponent<MeshFilter>( );

        Object.DestroyImmediate( editor );

        Assert.IsTrue( mesh == null, "the mesh is still there" );
        Assert.IsTrue( meshFilter.sharedMesh == null );
    }

    // switching algorithm shows the old mesh until the next contour, which replaces it
    [Test]
    public void contouringAfterSwitchingReplacesTheMesh( ) {
        var editor = configure( this.createEditor( ), new( Algorithm.Uniform, ImplementationType.Burst ) );

        addStarterModel( editor.transform );

        editor.contour( );

        var uniformMesh = shown( editor );

        editor.select( Algorithm.Manifold );
        editor.contour( );

        Assert.IsTrue( uniformMesh == null, "the mesh of the uniform contour is still there" );
        Assert.AreEqual( 1, editor.GetComponents<MeshFilter>( ).Length );
    }

    // an editor at scale 5 with a cube, and a group holding a ball, for the signature tests
    public sealed class Parts {
        public InteractiveEditor editor;
        public Volume            shape;
        public Transform         group;
        public Volume            grouped;
    }

    private Parts createParts( ) {
        var editor = this.createEditor( 5.0f );
        var group  = createGroup( editor.transform, new Vector3( 0.125f, 0.0f, 0.0f ), 1.0f );

        return new Parts {
            editor  = editor,
            shape   = addShape<Cuboid>( editor.transform, Vector3.zero, Vector3.one * 0.5f ),
            group   = group,
            grouped = addShape<Ellipsoid>( group, Vector3.zero, Vector3.one * 0.25f )
        };
    }

    private static T selected<T>( Parts parts, Algorithm algorithm ) where T : Voxelizer {
        parts.editor.select( algorithm );
        return parts.editor.voxelizer as T;
    }

    // each a setup, then a change, of everything a contour depends on
    private static readonly Dictionary<string, ( Action<Parts> setup, Action<Parts> change )> contourInputs = new( ) {
        ["shape position"]          = ( null, ( p ) => p.shape.transform.localPosition += new Vector3( 0.0625f, 0.0f, 0.0f ) ),
        ["shape rotation"]          = ( null, ( p ) => p.shape.transform.localRotation  = Quaternion.Euler( 0.0f, 45.0f, 0.0f ) ),
        ["shape scale"]             = ( null, ( p ) => p.shape.transform.localScale    *= 0.5f ),
        ["shape operation"]         = ( null, ( p ) => p.shape.combination   = Combination.Subtraction ),
        ["shape material"]          = ( null, ( p ) => p.shape.materialIndex = MaterialIndex.Material1 ),
        ["shape type"]              = ( null, ( p ) => { var shape = p.shape.gameObject; Object.DestroyImmediate( p.shape ); shape.AddComponent<Ellipsoid>( ).materialIndex = MaterialIndex.Material0; } ),
        ["shape order"]             = ( null, ( p ) => p.shape.transform.SetAsFirstSibling( ) ),
        ["shape added"]             = ( null, ( p ) => addShape<Cuboid>( p.editor.transform, Vector3.zero, Vector3.one * 0.125f ) ),
        ["shape removed"]           = ( null, ( p ) => Object.DestroyImmediate( p.shape.gameObject ) ),
        ["shape deactivated"]       = ( null, ( p ) => p.shape.gameObject.SetActive( false ) ),
        ["shape disabled"]          = ( null, ( p ) => p.shape.enabled = false ),
        ["group position"]          = ( null, ( p ) => p.group.localPosition += new Vector3( 0.0f, 0.0625f, 0.0f ) ),
        ["group scale"]             = ( null, ( p ) => p.group.localScale    *= 2.0f ),
        ["group rotation"]          = ( ( p ) => p.grouped.transform.localPosition = new Vector3( 0.125f, 0.0f, 0.0f ), ( p ) => p.group.localRotation = Quaternion.Euler( 0.0f, 90.0f, 0.0f ) ),
        ["group rotation, shape at its centre"] = ( null, ( p ) => p.group.localRotation = Quaternion.Euler( 0.0f, 30.0f, 0.0f ) ),
        ["algorithm"]               = ( null, ( p ) => p.editor.select( Algorithm.Manifold ) ),
        ["resolution"]              = ( null, ( p ) => p.editor.voxelizer.resolution                  += 1 ),
        ["minimizer iterations"]    = ( null, ( p ) => p.editor.voxelizer.minimizerIterations         += 1 ),
        ["binary search iterations"]= ( null, ( p ) => p.editor.voxelizer.binarySearchIterations      += 1 ),
        ["surface correction"]      = ( null, ( p ) => p.editor.voxelizer.surfaceCorrectionIterations += 1 ),
        ["implementation"]          = ( null, ( p ) => p.editor.voxelizer.implementationType          = ImplementationType.Burst ),
        ["solver"]                  = ( null, ( p ) => p.editor.voxelizer.qefSolverType               = QEFSolverType.Simple ),
        ["intersection"]            = ( null, ( p ) => p.editor.voxelizer.intersectionApproximation   = Approximation.LinearInterpolation ),
        ["vertex normals"]          = ( null, ( p ) => p.editor.voxelizer.vertexNormals               = VertexNormals.Split ),
        ["debug flags"]             = ( null, ( p ) => p.editor.voxelizer.debugFlags                  = Voxelizer.Debug.Corners ),
        ["adaptive simplification"] = ( ( p ) => selected<AdaptiveDualContouring>( p, Algorithm.Adaptive ), ( p ) => ( p.editor.voxelizer as AdaptiveDualContouring ).simplification = true ),
        ["adaptive threshold"]      = ( ( p ) => selected<AdaptiveDualContouring>( p, Algorithm.Adaptive ), ( p ) => ( p.editor.voxelizer as AdaptiveDualContouring ).errorThreshold = 0.5f ),
        ["adaptive topology safe"]  = ( ( p ) => selected<AdaptiveDualContouring>( p, Algorithm.Adaptive ), ( p ) => ( p.editor.voxelizer as AdaptiveDualContouring ).topologySafe   = false ),
        ["manifold threshold"]      = ( ( p ) => selected<ManifoldDualContouring>( p, Algorithm.Manifold ), ( p ) => ( p.editor.voxelizer as ManifoldDualContouring ).errorThreshold  = 0.5f ),
        ["manifold criterion"]      = ( ( p ) => selected<ManifoldDualContouring>( p, Algorithm.Manifold ), ( p ) => ( p.editor.voxelizer as ManifoldDualContouring ).enforceManifold = false )
    };

    // and of what a contour does not depend on
    private static readonly Dictionary<string, ( Action<Parts> setup, Action<Parts> change )> otherChanges = new( ) {
        ["shape name"]                   = ( null, ( p ) => p.shape.name = "Renamed" ),
        ["editor position"]              = ( null, ( p ) => p.editor.transform.localPosition = new Vector3( 1.0f, 2.0f, 3.0f ) ),
        ["editor rotation"]              = ( null, ( p ) => p.editor.transform.localRotation = Quaternion.Euler( 10.0f, 20.0f, 30.0f ) ),
        ["editor scale"]                 = ( null, ( p ) => p.editor.transform.localScale    = Vector3.one * 2.0f ),
        ["the same turn, negated"]       = ( ( p ) => p.shape.transform.localRotation = Quaternion.Euler( 0.0f, 45.0f, 0.0f ), ( p ) => { var q = p.shape.transform.localRotation; p.shape.transform.localRotation = new Quaternion( -q.x, -q.y, -q.z, -q.w ); } ),
        ["debounce"]                     = ( null, ( p ) => p.editor.debounce = 1.0f ),
        ["outline thickness"]            = ( null, ( p ) => p.editor.outlineThickness = 6.0f ),
        ["outlines on top"]              = ( null, ( p ) => p.editor.outlinesOnTop    = true ),
        ["debug colour"]                 = ( null, ( p ) => p.editor.voxelizer.debugEdgeColor     = Color.magenta ),
        ["debug line thickness"]         = ( null, ( p ) => p.editor.voxelizer.debugLineThickness = 4.0f ),
        ["renderer materials"]           = ( null, ( p ) => p.editor.GetComponent<MeshRenderer>( ).sharedMaterials = new Material[3] ),
        ["unselected voxelizer setting"] = ( null, ( p ) => ( p.editor.voxelizerOf( Algorithm.Adaptive ) as AdaptiveDualContouring ).errorThreshold = 0.5f ),
        ["unselected voxelizer resolution"] = ( null, ( p ) => p.editor.voxelizerOf( Algorithm.Manifold ).resolution = 7 )
    };

    public static IEnumerable<string> contourInputNames => contourInputs.Keys;
    public static IEnumerable<string> otherChangeNames  => otherChanges.Keys;

    [Test]
    public void theSignatureChangesWith( [ValueSource( nameof( contourInputNames ) )] string input ) {
        var parts = this.createParts( );
        var ( setup, change ) = contourInputs[input];

        setup?.Invoke( parts );

        var before = parts.editor.signature( );

        change( parts );

        Assert.AreNotEqual( before, parts.editor.signature( ) );
    }

    [Test]
    public void theSignatureStaysTheSameWith( [ValueSource( nameof( otherChangeNames ) )] string change ) {
        var parts = this.createParts( );
        var ( setup, apply ) = otherChanges[change];

        setup?.Invoke( parts );

        var before = parts.editor.signature( );

        apply( parts );

        Assert.AreEqual( before, parts.editor.signature( ) );
    }

    // a shape moved and moved back, as by an undo, is up to date again
    [Test]
    public void aChangeUndoneIsUpToDate( ) {
        var parts  = this.createParts( );
        var before = parts.editor.signature( );

        parts.shape.transform.localPosition += new Vector3( 0.0625f, 0.0f, 0.0f );

        Assert.AreNotEqual( before, parts.editor.signature( ) );

        parts.shape.transform.localPosition -= new Vector3( 0.0625f, 0.0f, 0.0f );

        Assert.AreEqual( before, parts.editor.signature( ) );
    }

    [Test]
    public void aChangeContoursByItself( [ValueSource( nameof( automaticVariants ) )] Variant variant ) {
        var editor = configure( this.createEditor( 5.0f ), variant );

        addStarterModel( editor.transform );

        this.time = 0.0;
        editor.contour( );

        var mesh = shown( editor );

        this.tickAt( editor, 1.0 );

        Assert.AreSame( mesh, shown( editor ), "contoured with nothing changed" );
        Assert.AreEqual( RemeshScheduler.State.UpToDate, editor.state );

        editor.transform.GetChild( 0 ).localScale *= 0.75f;

        // contoured at once when the last contour was fast, otherwise once the change has settled
        this.tickAt( editor, 2.0, 2.0 + editor.debounce );

        Assert.IsTrue( mesh == null, "the earlier mesh is still there" );
        Assert.AreEqual( RemeshScheduler.State.UpToDate, editor.state );
    }

    [Test]
    public void selectingAnAlgorithmContoursByItself( ) {
        var editor = configure( this.createEditor( 5.0f ), new( Algorithm.Uniform, ImplementationType.Burst ) );

        addStarterModel( editor.transform );

        editor.contour( );

        var uniformMesh = shown( editor );

        editor.select( Algorithm.Manifold );

        this.tickAt( editor, 1.0, 1.0 + editor.debounce );

        Assert.IsTrue( uniformMesh == null, "the uniform mesh is still there" );
        Assert.AreEqual( RemeshScheduler.State.UpToDate, editor.state );
    }

    [Test]
    public void managedOnlyContoursWhenAsked( [Values] Algorithm algorithm ) {
        var editor = configure( this.createEditor( 5.0f ), new( algorithm, ImplementationType.Managed ) );

        addStarterModel( editor.transform );

        editor.contour( );

        var mesh = shown( editor );

        editor.transform.GetChild( 0 ).localScale *= 0.75f;

        this.tickAt( editor, 1.0, 2.0, 100.0 );

        Assert.AreSame( mesh, shown( editor ) );
        Assert.AreEqual( RemeshScheduler.State.Manual, editor.state );

        editor.contour( );

        this.tickAt( editor, 101.0 );

        Assert.AreEqual( RemeshScheduler.State.UpToDate, editor.state );
    }

    [Test]
    public void nothingByItselfWhileUnityIsInTheBackground( ) {
        var editor = configure( this.createEditor( 5.0f ), new( Algorithm.Uniform, ImplementationType.Burst ) );

        addStarterModel( editor.transform );

        editor.contour( );

        var mesh = shown( editor );

        editor.transform.GetChild( 0 ).localScale *= 0.75f;

        this.unityActive = false;
        this.tickAt( editor, 1.0, 2.0, 100.0 );

        Assert.AreSame( mesh, shown( editor ) );
        Assert.AreEqual( RemeshScheduler.State.Background, editor.state );

        this.unityActive = true;
        this.tickAt( editor, 101.0 );

        Assert.IsTrue( mesh == null, "not contoured once Unity was active again" );
        Assert.AreEqual( RemeshScheduler.State.UpToDate, editor.state );
    }

    // so a script recompile, after which the mesh is still shown, costs no contour
    [Test]
    public void enablingWithAMeshShownIsUpToDate( ) {
        var editor = configure( this.createEditor( 5.0f ), new( Algorithm.Uniform, ImplementationType.Burst ) );

        addStarterModel( editor.transform );

        editor.contour( );

        var mesh = shown( editor );

        editor.enabled = false;
        editor.enabled = true;

        this.tickAt( editor, 50.0, 60.0 );

        Assert.AreSame( mesh, shown( editor ) );
        Assert.AreEqual( RemeshScheduler.State.UpToDate, editor.state );
    }

    // a material added to the renderer only needs a sub-mesh of its own, not a contour
    [Test]
    public void aMaterialAddedToTheRendererGetsASubMeshWithoutAContour( ) {
        var editor = configure( this.createEditor( 5.0f ), new( Algorithm.Uniform, ImplementationType.Managed ) );

        addStarterModel( editor.transform );

        editor.contour( );

        var mesh = shown( editor );

        Assert.AreEqual( 2, mesh.subMeshCount );

        var material = new Material( Shader.Find( "Universal Render Pipeline/Lit" ) );
        this.created.Add( material );

        editor.GetComponent<MeshRenderer>( ).sharedMaterials = new[] { material, material, material };

        this.tickAt( editor, 1.0 );

        Assert.AreSame( mesh, shown( editor ) );
        Assert.AreEqual( 3, mesh.subMeshCount );
        Assert.AreEqual( RemeshScheduler.State.UpToDate, editor.state );
    }

    // a renderer with a material for each of the first n material bits
    private Material[] giveMaterials( InteractiveEditor editor, int count ) {
        var material = new Material( Shader.Find( "Universal Render Pipeline/Lit" ) );
        this.created.Add( material );

        var materials = Enumerable.Repeat( material, count ).ToArray( );

        editor.GetComponent<MeshRenderer>( ).sharedMaterials = materials;

        return materials;
    }

    private static void assertOneWarningAbout( InteractiveEditor editor, string name, string about ) {
        var warnings = editor.warnings( );

        Assert.AreEqual( 1, warnings.Count, string.Join( "\n", warnings ) );
        StringAssert.Contains( name,  warnings[0] );
        StringAssert.Contains( about, warnings[0] );
    }

    [Test]
    public void theStarterModelHasNoWarnings( ) {
        var editor = this.createEditor( 5.0f );

        this.giveMaterials( editor, 2 );
        addStarterModel( editor.transform );

        CollectionAssert.IsEmpty( editor.warnings( ) );
    }

    [Test]
    public void aFirstShapeThatIsNoUnionIsWarnedAbout( [Values( Combination.Intersection, Combination.Subtraction )] Combination combination ) {
        var editor = this.createEditor( );

        this.giveMaterials( editor, 1 );
        addShape<Cuboid>( editor.transform, Vector3.zero, Vector3.one * 0.5f, combination ).name = "Opening";
        addShape<Cuboid>( editor.transform, Vector3.zero, Vector3.one * 0.5f );

        assertOneWarningAbout( editor, "Opening", "first shape" );
    }

    // shapes that are not contoured cannot be the first
    [Test]
    public void aDisabledFirstShapeIsNotWarnedAbout( ) {
        var editor = this.createEditor( );

        this.giveMaterials( editor, 1 );
        addShape<Cuboid>( editor.transform, Vector3.zero, Vector3.one * 0.5f, Combination.Subtraction ).enabled = false;
        addShape<Cuboid>( editor.transform, Vector3.zero, Vector3.one * 0.5f );

        CollectionAssert.IsEmpty( editor.warnings( ) );
    }

    [Test]
    public void aShapeWithoutAMaterialIsWarnedAbout( ) {
        var editor = this.createEditor( );

        this.giveMaterials( editor, 1 );
        addShape<Cuboid>( editor.transform, Vector3.zero, Vector3.one * 0.5f );
        addShape<Ellipsoid>( editor.transform, Vector3.zero, Vector3.one * 0.25f, Combination.Subtraction, MaterialIndex.Void ).name = "Hole";

        assertOneWarningAbout( editor, "Hole", "no material" );
    }

    [Test]
    public void aMaterialTheRendererLacksIsWarnedAbout( ) {
        var editor = this.createEditor( );

        this.giveMaterials( editor, 2 );
        addShape<Cuboid>( editor.transform, Vector3.zero, Vector3.one * 0.5f );
        addShape<Ellipsoid>( editor.transform, Vector3.zero, Vector3.one * 0.25f, Combination.Union, MaterialIndex.Material2 ).name = "Blue";

        assertOneWarningAbout( editor, "Blue", "material 2" );
    }

    // only a union adds a surface of its material
    [Test]
    public void aMaterialTheRendererLacksIsFineForASubtraction( ) {
        var editor = this.createEditor( );

        this.giveMaterials( editor, 1 );
        addShape<Cuboid>( editor.transform, Vector3.zero, Vector3.one * 0.5f );
        addShape<Ellipsoid>( editor.transform, Vector3.zero, Vector3.one * 0.25f, Combination.Subtraction, MaterialIndex.Material0 | MaterialIndex.Material3 );

        CollectionAssert.IsEmpty( editor.warnings( ) );
    }

    [Test]
    public void aShapeOutsideTheGridIsWarnedAbout( ) {
        var editor = this.createEditor( 5.0f );

        this.giveMaterials( editor, 1 );
        addShape<Cuboid>( editor.transform, Vector3.zero, Vector3.one * 0.5f );
        addShape<Ellipsoid>( editor.transform, new Vector3( 0.0f, -1.0f, 0.0f ), Vector3.one * 0.5f ).name = "Far";

        assertOneWarningAbout( editor, "Far", "outside the grid" );
    }

    [Test]
    public void aShapeReachingIntoTheGridIsFine( ) {
        var editor = this.createEditor( 5.0f );

        this.giveMaterials( editor, 1 );
        addShape<Ellipsoid>( editor.transform, new Vector3( 0.0f, -0.625f, 0.0f ), Vector3.one * 0.5f );

        CollectionAssert.IsEmpty( editor.warnings( ) );
    }

    [Test]
    public void aShapeWithoutSizeIsWarnedAbout( ) {
        var editor = this.createEditor( );

        this.giveMaterials( editor, 1 );
        addShape<Ellipsoid>( editor.transform, Vector3.zero, new Vector3( 0.5f, 0.0f, 0.5f ) ).name = "Flat";

        assertOneWarningAbout( editor, "Flat", "no size" );
    }

    [Test]
    public void aTurnedShapeIsFine( ) {
        var editor = this.createEditor( );

        this.giveMaterials( editor, 1 );
        addShape<Cuboid>( editor.transform, Vector3.zero, Vector3.one * 0.5f ).transform.localRotation = Quaternion.Euler( 0.0f, 30.0f, 0.0f );

        CollectionAssert.IsEmpty( editor.warnings( ) );
    }

    [Test]
    public void aShapeInATurnedGroupIsFine( ) {
        var editor = this.createEditor( );
        var group  = createGroup( editor.transform, Vector3.zero, 0.5f );

        group.localRotation = Quaternion.Euler( 30.0f, 0.0f, 0.0f );

        this.giveMaterials( editor, 1 );
        addShape<Ellipsoid>( group, Vector3.zero, Vector3.one * 0.5f ).transform.localRotation = Quaternion.Euler( 0.0f, 0.0f, 45.0f );

        CollectionAssert.IsEmpty( editor.warnings( ) );
    }

    // a box turned inside a group that is scaled unevenly along other axes than it is turned about would have slanted
    // sides. So would one in a turned group inside a group scaled unevenly
    [Test]
    public void aShapeAGroupWouldShearIsWarnedAbout( [Values( "turned shape", "turned group" )] string turned ) {
        var editor = this.createEditor( );
        var outer  = createGroup( editor.transform, Vector3.zero, 1.0f );
        var inner  = createGroup( outer,            Vector3.zero, 1.0f );

        outer.localScale = new Vector3( 1.0f, 1.0f, 2.0f );

        this.giveMaterials( editor, 1 );

        var volume = addShape<Cuboid>( inner, Vector3.zero, Vector3.one * 0.25f );

        if( turned == "turned shape" ) {
            volume.transform.localRotation = Quaternion.Euler( 0.0f, 30.0f, 0.0f );
        }
        else {
            inner.localRotation = Quaternion.Euler( 30.0f, 0.0f, 0.0f );
        }

        assertOneWarningAbout( editor, "Cuboid", "slant" );

        // its outline is the box it is contoured as instead, with square sides
        Assert.IsTrue( editor.tryGetShape( volume, out var shape ) );
        Assert.That( Quaternion.Dot( shape.rotation, shape.rotation ), Is.EqualTo( 1.0f ).Within( 1e-5f ), "the rotation is not a rotation" );
    }

    // a long box lying across below the grid is outside it, the same box turned upright reaches into it. It is long along
    // its own y, so that it is as wide along x as along z, as round shapes are
    [Test]
    public void aTurnedShapeIsOutsideTheGridAsFarAsItReaches( [Values] DensityFunction.Type type ) {
        var editor = this.createEditor( );

        this.giveMaterials( editor, 1 );

        var volume = addShape( Volume.componentOf( type ), editor.transform, new Vector3( 0.0f, -0.6f, 0.0f ), new Vector3( 0.1f, 1.2f, 0.1f ) );

        volume.name = "Bar";

        volume.transform.localRotation = Quaternion.Euler( 0.0f, 0.0f, 90.0f );

        assertOneWarningAbout( editor, "Bar", "outside the grid" );

        volume.transform.localRotation = Quaternion.Euler( 0.0f, 90.0f, 0.0f );

        CollectionAssert.IsEmpty( editor.warnings( ), "turned upright" );
    }

    // the rotation of the editor itself turns the whole mesh, which is fine
    [Test]
    public void aRotatedEditorIsFine( ) {
        var editor = this.createEditor( );

        editor.transform.localRotation = Quaternion.Euler( 0.0f, 45.0f, 0.0f );

        this.giveMaterials( editor, 1 );
        addShape<Ellipsoid>( editor.transform, Vector3.zero, Vector3.one * 0.5f );

        CollectionAssert.IsEmpty( editor.warnings( ) );
    }

    [Test]
    public void aShapeOnTheEditorItselfIsWarnedAbout( ) {
        var editor = this.createEditor( );

        this.giveMaterials( editor, 1 );
        addShape<Ellipsoid>( editor.transform, Vector3.zero, Vector3.one * 0.5f );
        editor.gameObject.AddComponent<Cuboid>( );

        assertOneWarningAbout( editor, "Interactive Editor", "own object" );
    }

    // the inspector lists shapes that are not contoured too, so they can be turned back on
    [Test]
    public void volumesListsEveryShapeBelowTheEditor( ) {
        var editor = this.createEditor( );
        var group  = createGroup( editor.transform, Vector3.zero, 1.0f );

        var enabled  = addShape<Cuboid>    ( editor.transform, Vector3.zero, Vector3.one * 0.5f );
        var disabled = addShape<Ellipsoid> ( editor.transform, Vector3.zero, Vector3.one * 0.5f );
        var inactive = addShape<Cuboid>    ( group,            Vector3.zero, Vector3.one * 0.5f );

        disabled.enabled = false;
        group.gameObject.SetActive( false );
        editor.gameObject.AddComponent<Ellipsoid>( );

        // the group came first
        CollectionAssert.AreEqual( new Volume[] { inactive, enabled, disabled }, editor.volumes( ) );
        Assert.AreEqual( 1, editor.shapes( ).Count );
    }

    // drawing takes about half a microsecond to a microsecond per shape, every frame, for every camera
    [Test]
    public void aDebugViewThatDrawsTooMuchIsWarnedAbout( ) {
        var editor = configure( this.createEditor( ), new( Algorithm.Uniform, ImplementationType.Burst ) );

        this.giveMaterials( editor, 2 );
        addStarterModel( editor.transform );

        // 32^3 cells
        editor.voxelizer.resolution = 5;
        editor.voxelizer.debugFlags = Voxelizer.Debug.Cells;
        editor.contour( );

        Assert.AreEqual( 32 * 32 * 32, editor.voxelizer.debugDrawCount );
        assertOneWarningAbout( editor, ( 32 * 32 * 32 ).ToString( "N0" ), "debug view" );

        // 16^3 cells
        editor.voxelizer.resolution = 4;
        editor.contour( );

        Assert.AreEqual( 16 * 16 * 16, editor.voxelizer.debugDrawCount );
        CollectionAssert.IsEmpty( editor.warnings( ) );
    }

    [Test]
    public void debugDrawCountFollowsTheFlags( ) {
        var editor = configure( this.createEditor( ), new( Algorithm.Uniform, ImplementationType.Burst ) );

        addStarterModel( editor.transform );

        editor.voxelizer.resolution = 3;

        editor.voxelizer.debugFlags = Voxelizer.Debug.Off;
        editor.contour( );
        Assert.AreEqual( 0, editor.voxelizer.debugDrawCount, "off" );

        editor.voxelizer.debugFlags = Voxelizer.Debug.Corners;
        editor.contour( );
        Assert.AreEqual( 9 * 9 * 9, editor.voxelizer.debugDrawCount, "corners of 8^3 cells" );

        editor.voxelizer.debugFlags = Voxelizer.Debug.Edges | Voxelizer.Debug.Intersections;
        editor.contour( );
        Assert.AreEqual( 3 * 8 * 9 * 9, editor.voxelizer.debugDrawCount, "every edge, crossing or not" );

        editor.voxelizer.debugFlags = Voxelizer.Debug.Minimizers;
        editor.contour( );
        Assert.AreEqual( 2 * editor.lastContour.Value.vertexCount, editor.voxelizer.debugDrawCount, "a vertex and a normal for each vertex of the shared mesh" );
    }

    // the editor's own inspector comes before the selected voxelizer's long one. Unity does not reorder the components
    // of an object hidden from the hierarchy, so this one is only kept from being saved
    [Test]
    public void theEditorMovesAboveItsVoxelizers( ) {
        var gameObject = new GameObject( "Interactive Editor" ) { hideFlags = HideFlags.DontSave };

        this.created.Add( gameObject );

        var editor = gameObject.AddComponent<InteractiveEditor>( );

        // moving a component is recorded for undo, and the test runner reverts what a run recorded when it ends, which
        // would bring the object back after it was destroyed. So the test takes its own undo steps back first
        UnityEditor.Undo.IncrementCurrentGroup( );
        var undoGroup = UnityEditor.Undo.GetCurrentGroup( );

        try {
            editor.moveAboveVoxelizers( );

            var shown = editor.GetComponents<Component>( ).Where( ( component ) => ( component.hideFlags & HideFlags.HideInInspector ) == 0 ).ToList( );

            Assert.Less( shown.IndexOf( editor ), shown.IndexOf( editor.voxelizer ) );
            Assert.Greater( shown.IndexOf( editor ), shown.IndexOf( editor.GetComponent<MeshRenderer>( ) ), "it stays below the renderer" );
        }
        finally {
            UnityEditor.Undo.RevertAllDownToGroup( undoGroup );
        }
    }

}
