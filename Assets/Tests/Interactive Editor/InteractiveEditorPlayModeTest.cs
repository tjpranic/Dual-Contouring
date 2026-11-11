using System.Collections;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

using Object             = UnityEngine.Object;
using ImplementationType = SurfaceExtractor.Implementation.Type;
using MaterialIndex      = SurfaceExtractor.MaterialIndex;

// in play mode the voxelizers of the interactive editor would contour in Start, each by its own reading of the
// shapes. The editor keeps them from it
public class InteractiveEditorPlayModeTest {

    [UnityTest]
    public IEnumerator onlyTheEditorContoursInPlayMode( ) {
        var gameObject = new GameObject( "Interactive Editor" );

        try {
            gameObject.transform.localScale = Vector3.one * 5.0f;

            var editor = gameObject.AddComponent<InteractiveEditor>( );

            var sphere = new GameObject( "Ellipsoid" );

            sphere.transform.SetParent( gameObject.transform, false );
            sphere.transform.localScale = Vector3.one * 0.6f;
            sphere.AddComponent<Ellipsoid>( ).materialIndex = MaterialIndex.Material0;

            editor.voxelizer.implementationType = ImplementationType.Burst;
            editor.voxelizer.resolution         = 4;

            editor.contour( );

            var mesh = gameObject.GetComponent<MeshFilter>( ).sharedMesh;

            // Start would have run within these
            yield return null;
            yield return null;

            foreach( var voxelizer in gameObject.GetComponents<Voxelizer>( ) ) {
                Assert.IsFalse( voxelizer.enabled, voxelizer.GetType( ).Name );
            }

            Assert.AreEqual( 1, gameObject.GetComponents<MeshFilter>( ).Length );
            Assert.AreSame( mesh, gameObject.GetComponent<MeshFilter>( ).sharedMesh, "something else contoured" );
        }
        finally {
            Object.Destroy( gameObject );
        }
    }

}
