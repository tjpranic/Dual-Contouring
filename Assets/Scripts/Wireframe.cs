using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

using Object = UnityEngine.Object;

// draws the edges of a contour's triangles over the contour, for one camera at a time: as lines over the shaded
// surface, or as lines over a flat fill that hides the surface and the lines behind it. The contour is still drawn as
// it was, by its renderer or straight from the GPU, so picking, its materials and its shadows stay as they are, and the
// lines go over it after the opaque objects. See Documents/Plans/Wireframe Rendering.md
//
// The lines come from the buffers the contour is drawn from, so nothing is copied: a mesh's own vertex and index
// buffers, read as raw buffers, or the GPU's vertices and quads. They are drawn with three vertices for every triangle
// and no index buffer: the vertex shader reads its index itself, and knows which corner of its triangle it is. Metal
// has no geometry shaders, and a vertex shared by several triangles is a different corner of each, so this is how the
// fragment shader gets its barycentric coordinates
public sealed class Wireframe : IDisposable {

    public enum Look {
        Off,
        OverSurface,
        LinesOnly
    }

    // how the lines are drawn
    public struct Style {

        public Look look;

        // the width of the lines in pixels
        public float lineWidth;

        // the colour of the lines over the surface. Its alpha blends them
        public Color lineColor;

        // the colour under the lines with lines only, where each sub-mesh's lines take its material's colour
        public Color fillColor;

    }

    // the most sub-meshes a contour has, one for each material bit
    public const int maximumSubMeshes = 10;

    private static readonly int positionsID      = Shader.PropertyToID( "positions" );
    private static readonly int indicesID        = Shader.PropertyToID( "indices" );
    private static readonly int positionStrideID = Shader.PropertyToID( "positionStride" );
    private static readonly int positionOffsetID = Shader.PropertyToID( "positionOffset" );
    private static readonly int indices16ID      = Shader.PropertyToID( "indices16" );
    private static readonly int indexStartID     = Shader.PropertyToID( "indexStart" );
    private static readonly int baseVertexID     = Shader.PropertyToID( "baseVertex" );
    private static readonly int subMeshIndexID   = Shader.PropertyToID( "subMeshIndex" );
    private static readonly int verticesID       = Shader.PropertyToID( "vertices" );
    private static readonly int quadsID          = Shader.PropertyToID( "quads" );
    private static readonly int localToWorldID   = Shader.PropertyToID( "localToWorld" );
    private static readonly int lineWidthID      = Shader.PropertyToID( "lineWidth" );
    private static readonly int lineColorID      = Shader.PropertyToID( "lineColor" );
    private static readonly int fillColorID      = Shader.PropertyToID( "fillColor" );
    private static readonly int subMeshColorsID  = Shader.PropertyToID( "subMeshColors" );
    private static readonly int linesOnlyID      = Shader.PropertyToID( "linesOnly" );

    private readonly Material                    material;
    private readonly List<MaterialPropertyBlock> blocks        = new( );
    private readonly Vector4[]                   subMeshColors = new Vector4[maximumSubMeshes];

    // the shader's variant for the GPU's quads, made the first time they are drawn
    private Material              quadsMaterial;
    private MaterialPropertyBlock quadsBlock;

    // the mesh whose buffers are held, and the buffers. A mesh is not changed once it is shown, apart from its number
    // of sub-meshes, which leaves its buffers as they are
    private Mesh           bufferedMesh;
    private GraphicsBuffer positions;
    private GraphicsBuffer indices;

    public Wireframe( ) {
        this.material = new Material( Resources.Load<Shader>( "Shaders/RenderContourWireframe" ) ) { hideFlags = HideFlags.DontSave };
    }

    // draws the lines of the mesh, placed by localToWorld, for the camera. materials are the renderer's, which draws
    // sub-mesh n with the n-th. A sub-mesh without one is not drawn by the renderer, and gets no lines either
    public void draw( Camera camera, Mesh mesh, Matrix4x4 localToWorld, int layer, Style style, IReadOnlyList<Material> materials ) {
        if( style.look == Look.Off || mesh == null || !hasFloatPositions( mesh ) || !hasTriangles( mesh ) ) {
            return;
        }

        this.holdBuffersOf( mesh );
        this.takeColorsOf( materials );

        var stream      = mesh.GetVertexAttributeStream( VertexAttribute.Position );
        var worldBounds = transform( mesh.bounds, localToWorld );
        var subMeshes   = Math.Min( Math.Min( mesh.subMeshCount, materials.Count ), maximumSubMeshes );

        for( var subMesh = 0; subMesh < subMeshes; ++subMesh ) {
            var descriptor = mesh.GetSubMesh( subMesh );

            if( materials[subMesh] == null || descriptor.indexCount == 0 || descriptor.topology != MeshTopology.Triangles ) {
                continue;
            }

            var block = this.blockFor( subMesh );

            block.SetBuffer     ( positionsID,      this.positions );
            block.SetBuffer     ( indicesID,        this.indices );
            block.SetInteger    ( positionStrideID, mesh.GetVertexBufferStride( stream ) );
            block.SetInteger    ( positionOffsetID, mesh.GetVertexAttributeOffset( VertexAttribute.Position ) );
            block.SetInteger    ( indices16ID,      mesh.indexFormat == IndexFormat.UInt16 ? 1 : 0 );
            block.SetInteger    ( indexStartID,     descriptor.indexStart );
            block.SetInteger    ( baseVertexID,     descriptor.baseVertex );
            block.SetInteger    ( subMeshIndexID,   subMesh );

            this.setStyle( block, localToWorld, style );

            var parameters = new RenderParams( this.material ) {
                camera            = camera,
                layer             = layer,
                worldBounds       = worldBounds,
                matProps          = block,
                shadowCastingMode = ShadowCastingMode.Off,
                receiveShadows    = false
            };

            Graphics.RenderPrimitives( parameters, MeshTopology.Triangles, descriptor.indexCount );
        }
    }

    // draws the lines of a contour the GPU made and keeps, for the camera, from its buffers: vertices, and quads of six
    // indices, two triangles, and a sub-mesh each, as RenderContourDiffuse reads them. arguments are the arguments of
    // the indirect draw, which the GPU counts the indices into, and localToWorld and worldBounds are those the contour
    // itself is drawn with. Every quad gets lines, as every quad is drawn, in the colour of its sub-mesh's material with
    // lines only. Released buffers, as after the voxelizer's release, draw nothing
    public void drawQuads( Camera camera, ComputeBuffer vertices, ComputeBuffer quads, ComputeBuffer arguments, Matrix4x4 localToWorld, Bounds worldBounds, int layer, Style style, IReadOnlyList<Material> materials ) {
        var valid = vertices != null && vertices.IsValid( ) && quads != null && quads.IsValid( ) && arguments != null && arguments.IsValid( );

        if( style.look == Look.Off || !valid ) {
            return;
        }

        if( this.quadsMaterial == null ) {
            this.quadsMaterial = new Material( this.material.shader ) { hideFlags = HideFlags.DontSave };
            this.quadsMaterial.EnableKeyword( "CONTOUR_QUADS" );
            this.quadsBlock    = new MaterialPropertyBlock( );
        }

        this.takeColorsOf( materials );

        this.quadsBlock.SetBuffer( verticesID, vertices );
        this.quadsBlock.SetBuffer( quadsID,    quads );

        this.setStyle( this.quadsBlock, localToWorld, style );

        // RenderPrimitivesIndirect takes a GraphicsBuffer, and the GPU's arguments are a ComputeBuffer, as the draw of
        // the contour itself takes them
        Graphics.DrawProceduralIndirect( this.quadsMaterial, worldBounds, MeshTopology.Triangles, arguments, 0, camera, this.quadsBlock, ShadowCastingMode.Off, false, layer );
    }

    public void Dispose( ) {
        this.releaseBuffers( );

        destroy( this.material );

        if( this.quadsMaterial != null ) {
            destroy( this.quadsMaterial );
        }
    }

    private static void destroy( Object item ) {
        if( Application.isPlaying ) {
            Object.Destroy( item );
        }
        else {
            Object.DestroyImmediate( item );
        }
    }

    // the colour of each sub-mesh's material, for lines only. A sub-mesh without a material has none
    private void takeColorsOf( IReadOnlyList<Material> materials ) {
        for( var subMesh = 0; subMesh < maximumSubMeshes; ++subMesh ) {
            this.subMeshColors[subMesh] = subMesh < materials.Count && materials[subMesh] != null ? linear( materials[subMesh].color ) : Vector4.zero;
        }
    }

    private void setStyle( MaterialPropertyBlock block, Matrix4x4 localToWorld, Style style ) {
        block.SetMatrix     ( localToWorldID,  localToWorld );
        block.SetFloat      ( lineWidthID,     style.lineWidth );
        block.SetColor      ( lineColorID,     style.lineColor );
        block.SetColor      ( fillColorID,     style.fillColor );
        block.SetVectorArray( subMeshColorsID, this.subMeshColors );
        block.SetFloat      ( linesOnlyID,     style.look == Look.LinesOnly ? 1.0f : 0.0f );
    }

    // the shader reads three floats for a position
    private static bool hasFloatPositions( Mesh mesh ) {
        return mesh.HasVertexAttribute( VertexAttribute.Position )
            && mesh.GetVertexAttributeFormat( VertexAttribute.Position ) == VertexAttributeFormat.Float32
            && mesh.GetVertexAttributeDimension( VertexAttribute.Position ) == 3;
    }

    // a mesh without vertices or without indices has no buffer of them: the mesh of an empty contour, of shapes that are
    // all outside the grid, has neither
    private static bool hasTriangles( Mesh mesh ) {
        if( mesh.vertexCount == 0 ) {
            return false;
        }

        for( var subMesh = 0; subMesh < mesh.subMeshCount; ++subMesh ) {
            if( mesh.GetIndexCount( subMesh ) > 0 ) {
                return true;
            }
        }

        return false;
    }

    // the buffers of the mesh, as raw buffers. Adding the raw target rebuilds the mesh's buffers, so they are taken
    // after it
    private void holdBuffersOf( Mesh mesh ) {
        if( mesh == this.bufferedMesh && this.positions != null && this.positions.IsValid( ) && this.indices.IsValid( ) ) {
            return;
        }

        this.releaseBuffers( );

        mesh.vertexBufferTarget |= GraphicsBuffer.Target.Raw;
        mesh.indexBufferTarget  |= GraphicsBuffer.Target.Raw;

        this.positions    = mesh.GetVertexBuffer( mesh.GetVertexAttributeStream( VertexAttribute.Position ) );
        this.indices      = mesh.GetIndexBuffer( );
        this.bufferedMesh = mesh;
    }

    // the wrappers are disposed, and the mesh keeps its buffers
    private void releaseBuffers( ) {
        this.positions?.Dispose( );
        this.indices?.Dispose( );

        this.positions    = null;
        this.indices      = null;
        this.bufferedMesh = null;
    }

    // one property block for each sub-mesh, kept from draw to draw
    private MaterialPropertyBlock blockFor( int subMesh ) {
        while( this.blocks.Count <= subMesh ) {
            this.blocks.Add( new MaterialPropertyBlock( ) );
        }
        return this.blocks[subMesh];
    }

    // MaterialPropertyBlock.SetColor converts a colour to linear space when the project is in it, and SetVectorArray
    // does not, so the colours of an array are converted here
    private static Vector4 linear( Color color ) {
        return QualitySettings.activeColorSpace == ColorSpace.Linear ? color.linear : color;
    }

    private static Bounds transform( Bounds bounds, Matrix4x4 matrix ) {
        var result = new Bounds( matrix.MultiplyPoint3x4( bounds.center ), Vector3.zero );

        for( var corner = 0; corner < 8; ++corner ) {
            var direction = new Vector3( ( corner & 1 ) != 0 ? 1 : -1, ( corner & 2 ) != 0 ? 1 : -1, ( corner & 4 ) != 0 ? 1 : -1 );
            result.Encapsulate( matrix.MultiplyPoint3x4( bounds.center + Vector3.Scale( bounds.extents, direction ) ) );
        }

        return result;
    }

}
