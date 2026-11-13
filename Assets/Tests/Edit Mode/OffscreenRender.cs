using System;
using UnityEngine;
using UnityEngine.Rendering;

using Object = UnityEngine.Object;

// a camera that sees one layer and renders it into a render texture, through the render pipeline's own render request,
// and gives back what it rendered. It is hidden and disabled, so it never renders by itself, and the render texture has
// no MSAA, so a pixel is wholly the triangle whose area holds its centre
public sealed class OffscreenRender : IDisposable {

    // a layer no scene uses, so the open scene stays out of the pictures
    public const int layer = 30;

    public readonly Camera camera;
    public readonly int    size;

    private readonly RenderTexture target;

    // a camera at z = -2 looking along +z, which shows a square of the given height at z = 0 in a square render texture
    // of the given size in pixels. With perspective, what is further away is smaller
    public OffscreenRender( int size, float height, bool perspective = false ) {
        this.size = size;

        this.target = new RenderTexture( size, size, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB ) { antiAliasing = 1, hideFlags = HideFlags.HideAndDontSave };
        this.target.Create( );

        var gameObject = new GameObject( "OffscreenRender" ) { hideFlags = HideFlags.HideAndDontSave };
        gameObject.transform.SetPositionAndRotation( new Vector3( 0.0f, 0.0f, -2.0f ), Quaternion.identity );

        this.camera = gameObject.AddComponent<Camera>( );
        this.camera.enabled          = false;
        this.camera.cullingMask      = 1 << layer;
        this.camera.clearFlags       = CameraClearFlags.SolidColor;
        this.camera.backgroundColor  = Color.black;
        this.camera.allowMSAA        = false;
        this.camera.allowHDR         = false;
        this.camera.orthographic     = !perspective;
        this.camera.orthographicSize = height * 0.5f;
        this.camera.fieldOfView      = 2.0f * Mathf.Atan( height * 0.5f / 2.0f ) * Mathf.Rad2Deg;
        this.camera.aspect           = 1.0f;
        this.camera.nearClipPlane    = 0.1f;
        this.camera.farClipPlane     = 10.0f;

        // the render request draws into its destination either way. The target texture gives WorldToScreenPoint the
        // texture's size
        this.camera.targetTexture = this.target;
    }

    // the pixels, bottom row first. RenderPipelineManager.beginCameraRendering is raised for the camera once
    public Color32[] render( ) {
        RenderPipeline.SubmitRenderRequest( this.camera, new RenderPipeline.StandardRequest { destination = this.target } );

        var previous = RenderTexture.active;
        RenderTexture.active = this.target;

        var texture = new Texture2D( this.size, this.size, TextureFormat.RGBA32, false );
        texture.ReadPixels( new Rect( 0, 0, this.size, this.size ), 0, 0 );
        texture.Apply( );

        RenderTexture.active = previous;

        var pixels = texture.GetPixels32( );
        Object.DestroyImmediate( texture );
        return pixels;
    }

    // where a point in the world lands, in pixels from the bottom left
    public Vector2 pixelOf( Vector3 position ) {
        return this.camera.WorldToScreenPoint( position );
    }

    // the pixel whose area holds the point
    public Color32 at( Color32[] pixels, Vector2 pixel ) {
        return pixels[( Mathf.FloorToInt( pixel.y ) * this.size ) + Mathf.FloorToInt( pixel.x )];
    }

    public void Dispose( ) {
        this.camera.targetTexture = null;
        Object.DestroyImmediate( this.camera.gameObject );
        Object.DestroyImmediate( this.target );
    }

}
