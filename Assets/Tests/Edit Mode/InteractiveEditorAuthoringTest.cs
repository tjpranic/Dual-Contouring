using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

using Object        = UnityEngine.Object;
using Combination   = DensityFunction.Combination;
using MaterialIndex = SurfaceExtractor.MaterialIndex;

// what the interactive editor's inspector does to the shapes: add, remove and reorder them, each undoable.
// Undo marks the scene it changes as modified, and taking an undo step back can bring objects back. So these tests
// never work in a saved scene: they use an untitled scene, as an edit mode test run has, or a temporary one they close
// again. And before cleaning up, each test takes back every undo step it made, so nothing is left for an undo, the
// test runner's or anyone's, to bring back later
public class InteractiveEditorAuthoringTest {

    private Scene previousScene;
    private Scene temporaryScene;
    private int   undoGroup;

    private InteractiveEditor editor;

    [SetUp]
    public void setUp( ) {
        this.previousScene = SceneManager.GetActiveScene( );

        var untitled = Enumerable.Range( 0, SceneManager.sceneCount ).Select( SceneManager.GetSceneAt ).FirstOrDefault(
            ( scene ) => scene.isLoaded && string.IsNullOrEmpty( scene.path )
        );

        if( untitled.IsValid( ) ) {
            SceneManager.SetActiveScene( untitled );
        }
        else {
            this.temporaryScene = EditorSceneManager.NewScene( NewSceneSetup.EmptyScene, NewSceneMode.Additive );

            SceneManager.SetActiveScene( this.temporaryScene );
        }

        Assert.IsTrue( string.IsNullOrEmpty( SceneManager.GetActiveScene( ).path ), "the tests would change a saved scene" );

        Undo.IncrementCurrentGroup( );
        this.undoGroup = Undo.GetCurrentGroup( );

        var gameObject = new GameObject( "Interactive Editor" );

        gameObject.transform.localScale = Vector3.one * 5.0f;

        this.editor = gameObject.AddComponent<InteractiveEditor>( );
    }

    [TearDown]
    public void tearDown( ) {
        Undo.RevertAllDownToGroup( this.undoGroup );

        if( this.editor != null ) {
            Object.DestroyImmediate( this.editor.gameObject );
        }

        if( this.previousScene.IsValid( ) && this.previousScene.isLoaded ) {
            SceneManager.SetActiveScene( this.previousScene );
        }

        if( this.temporaryScene.IsValid( ) ) {
            EditorSceneManager.CloseScene( this.temporaryScene, true );
        }

        this.temporaryScene = default;
    }

    private Volume addShape<T>( Transform parent, string name, Vector3 position, float scale, Combination combination = Combination.Union ) where T : Volume {
        var shape = new GameObject( name );

        shape.transform.SetParent( parent, false );
        shape.transform.localPosition = position;
        shape.transform.localScale    = Vector3.one * scale;

        var volume = shape.AddComponent<T>( );

        volume.combination   = combination;
        volume.materialIndex = MaterialIndex.Material0;

        return volume;
    }

    private Volume addShape( string name, Combination combination = Combination.Union ) {
        return this.addShape<Cuboid>( this.editor.transform, name, Vector3.zero, 0.5f, combination );
    }

    private string[] order( ) {
        return this.editor.volumes( ).Select( ( volume ) => volume.name ).ToArray( );
    }

    // starts an undo group of the test's own, which revert takes back without touching anything older
    private static int beginUndo( ) {
        Undo.IncrementCurrentGroup( );
        return Undo.GetCurrentGroup( );
    }

    private static void revert( int group ) {
        Undo.RevertAllDownToGroup( group );
    }

    // half the grid across, except the capsule and the torus, which at that size would be a ball and a torus without a
    // hole
    [Test]
    public void aShapeIsAddedAtASizeThatShowsWhatItIs( [Values] DensityFunction.Type type ) {
        var volume = this.editor.addShape( type );

        Assert.IsTrue( this.editor.tryGetShape( volume, out var shape ), "it is contoured" );

        switch( type ) {
            case DensityFunction.Type.Capsule: {
                // longer than it is wide, so it has a straight part
                Assert.Greater( shape.extents.y, shape.extents.x );
                break;
            }
            case DensityFunction.Type.Torus: {
                // the middle is outside it
                Assert.Greater( DensityFunction.sample( type, Vector3.zero, shape.extents ), 0.0f );
                break;
            }
            default: {
                Assert.AreEqual( Vector3.one * InteractiveEditor.addedShapeScale, volume.transform.localScale );
                break;
            }
        }

        var warnings = this.editor.warnings( ).Where( ( warning ) => warning.Contains( volume.name ) && !warning.Contains( "material" ) ).ToArray( );

        CollectionAssert.IsEmpty( warnings, "warnings about anything other than materials" );
    }

    [Test]
    public void addShapeAddsAUnionOfMaterialZeroAtTheCentre( [Values] DensityFunction.Type type ) {
        this.addShape( "First" );

        var volume = this.editor.addShape( type );

        Assert.AreEqual    ( type,                                    volume.type );
        Assert.IsInstanceOf( Volume.componentOf( type ),              volume );
        Assert.AreEqual    ( type.ToString( ),                        volume.name );
        Assert.AreSame     ( this.editor.transform,                   volume.transform.parent );
        Assert.AreEqual    ( Vector3.zero,                            volume.transform.localPosition );
        Assert.AreEqual    ( InteractiveEditor.addedScaleOf( type ),  volume.transform.localScale );
        Assert.AreEqual( Combination.Union,        volume.combination );
        Assert.AreEqual( MaterialIndex.Material0,  volume.materialIndex );
        Assert.IsTrue  ( this.editor.tryGetShape( volume, out _ ), "it is contoured" );

        CollectionAssert.AreEqual( new[] { "First", type.ToString( ) }, this.order( ), "it goes last" );
    }

    [Test]
    public void addedShapesGetNamesOfTheirOwn( ) {
        this.editor.addShape( DensityFunction.Type.Cuboid );
        this.editor.addShape( DensityFunction.Type.Cuboid );

        CollectionAssert.AreEqual( new[] { "Cuboid", "Cuboid (1)" }, this.order( ) );
    }

    [Test]
    public void addingAShapeCanBeUndone( ) {
        this.addShape( "First" );

        var group = beginUndo( );

        this.editor.addShape( DensityFunction.Type.Ellipsoid );

        revert( group );

        CollectionAssert.AreEqual( new[] { "First" }, this.order( ) );
    }

    [Test]
    public void removeShapeRemovesItsObject( ) {
        this.addShape( "First" );

        var second = this.addShape( "Second", Combination.Subtraction );
        var shape  = second.gameObject;

        this.editor.removeShape( second );

        Assert.IsTrue( shape == null, "its object is still there" );
        CollectionAssert.AreEqual( new[] { "First" }, this.order( ) );
    }

    [Test]
    public void removingAShapeCanBeUndone( ) {
        this.addShape( "First" );
        this.addShape( "Second", Combination.Subtraction );

        var group = beginUndo( );

        this.editor.removeShape( this.editor.volumes( )[1] );

        revert( group );

        CollectionAssert.AreEqual( new[] { "First", "Second" }, this.order( ) );
        Assert.AreEqual( Combination.Subtraction, this.editor.volumes( )[1].combination );
    }

    // its object holds other shapes, which stay
    [Test]
    public void removingAShapeThatHoldsOthersRemovesOnlyTheShape( ) {
        var outer = this.addShape( "Outer" );

        this.addShape<Ellipsoid>( outer.transform, "Inner", Vector3.zero, 0.5f );

        this.editor.removeShape( outer );

        Assert.IsTrue( outer == null, "the shape is still there" );
        CollectionAssert.AreEqual( new[] { "Inner" }, this.order( ) );
        Assert.IsNotNull( this.editor.transform.Find( "Outer" ), "its object went with it" );
    }

    [Test]
    public void moveShapeReordersTheShapes( ) {
        this.addShape( "A" );
        this.addShape( "B" );
        this.addShape( "C" );

        this.editor.moveShape( 0, 2 );
        CollectionAssert.AreEqual( new[] { "B", "C", "A" }, this.order( ), "first to last" );

        this.editor.moveShape( 2, 0 );
        CollectionAssert.AreEqual( new[] { "A", "B", "C" }, this.order( ), "last to first" );

        this.editor.moveShape( 1, 0 );
        CollectionAssert.AreEqual( new[] { "B", "A", "C" }, this.order( ), "one up" );

        this.editor.moveShape( 0, 1 );
        CollectionAssert.AreEqual( new[] { "A", "B", "C" }, this.order( ), "one down" );
    }

    [Test]
    public void movingAShapeCanBeUndone( ) {
        this.addShape( "A" );
        this.addShape( "B" );
        this.addShape( "C" );

        var group = beginUndo( );

        this.editor.moveShape( 0, 2 );

        revert( group );

        CollectionAssert.AreEqual( new[] { "A", "B", "C" }, this.order( ) );
    }

    // only the order changes: the shape is contoured where it was
    [Test]
    public void movingAShapeIntoAGroupKeepsWhereItIs( ) {
        this.addShape( "A" );

        var group = new GameObject( "Group" ).transform;

        group.SetParent( this.editor.transform, false );
        group.localPosition = new Vector3( 0.25f, 0.0f, 0.0f );
        group.localScale    = Vector3.one * 0.5f;

        this.addShape<Ellipsoid>( group, "B", Vector3.zero, 0.5f );

        var c = this.addShape<Cuboid>( this.editor.transform, "C", new Vector3( -0.125f, 0.0625f, 0.0f ), 0.25f );

        this.editor.tryGetShape( c, out var before );

        this.editor.moveShape( 2, 1 );

        CollectionAssert.AreEqual( new[] { "A", "C", "B" }, this.order( ) );
        Assert.AreSame( group, c.transform.parent, "it joined the group of the shape it went in front of" );

        this.editor.tryGetShape( c, out var after );

        Assert.That( Vector3.Distance( before.origin,  after.origin ),  Is.LessThan( 1e-6f ), "origin" );
        Assert.That( Vector3.Distance( before.extents, after.extents ), Is.LessThan( 1e-6f ), "extents" );
    }

    [Test]
    public void aShapeCannotMoveBelowItself( ) {
        var outer = this.addShape( "Outer" );

        this.addShape<Ellipsoid>( outer.transform, "Inner", Vector3.zero, 0.5f );

        this.editor.moveShape( 0, 1 );

        CollectionAssert.AreEqual( new[] { "Outer", "Inner" }, this.order( ) );
        Assert.AreSame( this.editor.transform, outer.transform.parent );
    }

    [Test]
    public void movingOutOfRangeDoesNothing( ) {
        this.addShape( "A" );
        this.addShape( "B" );

        this.editor.moveShape( -1, 0 );
        this.editor.moveShape( 0, 2 );
        this.editor.moveShape( 1, 1 );

        CollectionAssert.AreEqual( new[] { "A", "B" }, this.order( ) );
    }

}
