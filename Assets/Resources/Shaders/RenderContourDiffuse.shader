Shader "Custom/RenderContourDiffuse"
{
    Properties
    {
        color0 ( "color0", Color ) = ( 0, 0, 0, 1 )
        color1 ( "color1", Color ) = ( 0, 0, 0, 1 )
        color2 ( "color2", Color ) = ( 0, 0, 0, 1 )
        color3 ( "color3", Color ) = ( 0, 0, 0, 1 )
        color4 ( "color4", Color ) = ( 0, 0, 0, 1 )
        color5 ( "color5", Color ) = ( 0, 0, 0, 1 )
        color6 ( "color6", Color ) = ( 0, 0, 0, 1 )
        color7 ( "color7", Color ) = ( 0, 0, 0, 1 )
        color8 ( "color8", Color ) = ( 0, 0, 0, 1 )
        color9 ( "color9", Color ) = ( 0, 0, 0, 1 )
    }
    SubShader
    {
        Tags { "RenderType" = "Opaque" }

        LOD 200

        CGPROGRAM

        #pragma surface surfaceShader Standard addshadow vertex:vertexShader
        #pragma target 5.0

        #include "UnityCG.cginc"

        struct Vertex {
            float4 vertex    : POSITION;
            float4 tangent   : TANGENT;
            float3 normal    : NORMAL;
            float4 texcoord0 : TEXCOORD0;
            float4 texcoord1 : TEXCOORD1;
            float4 texcoord2 : TEXCOORD2;
            float4 texcoord3 : TEXCOORD3;
            uint   vertexID  : SV_VertexID;
        };

        #ifdef SHADER_API_D3D11

        struct Quad {
            uint indices[6];
            uint subMeshIndex;
        };

        StructuredBuffer<float3> vertices;
        StructuredBuffer<float3> normals;
        StructuredBuffer<Quad>   quads;

        #endif

        float4x4 localToWorld;
        float4x4 worldToLocal;

        struct Input {
            float subMeshIndex;
        };

        void vertexShader( in out Vertex v, out Input o ) {

            UNITY_INITIALIZE_OUTPUT( Input, o );

            #ifdef SHADER_API_D3D11

            uint quadIndex = floor( v.vertexID / 6 );

            // the absolute STATE of surface shaders:
            // https://forum.unity.com/threads/output-color-in-surface-shader-has-artifacts-when-using-instanceid.926984/
            o.subMeshIndex = quads[quadIndex].subMeshIndex + 0.5f; // HUH

            uint vertexIndex = quads[quadIndex].indices[v.vertexID % 6];

            float3 vertex = vertices[vertexIndex];
            float3 normal = normals[vertexIndex];

            v.vertex = float4( vertex, 1.0f );
            v.normal = normal;

            #endif

            unity_ObjectToWorld = localToWorld;
            unity_WorldToObject = worldToLocal;
        }

        fixed4 color0;
        fixed4 color1;
        fixed4 color2;
        fixed4 color3;
        fixed4 color4;
        fixed4 color5;
        fixed4 color6;
        fixed4 color7;
        fixed4 color8;
        fixed4 color9;

        void surfaceShader( Input input, in out SurfaceOutputStandard output ) {
            switch( input.subMeshIndex ) {
                case 0:
                    output.Albedo = color0.rgb;
                    break;
                case 1:
                    output.Albedo = color1.rgb;
                    break;
                case 2:
                    output.Albedo = color2.rgb;
                    break;
                case 3:
                    output.Albedo = color3.rgb;
                    break;
                case 4:
                    output.Albedo = color4.rgb;
                    break;
                case 5:
                    output.Albedo = color5.rgb;
                    break;
                case 6:
                    output.Albedo = color6.rgb;
                    break;
                case 7:
                    output.Albedo = color7.rgb;
                    break;
                case 8:
                    output.Albedo = color8.rgb;
                    break;
                case 9:
                    output.Albedo = color9.rgb;
                    break;
                default:
                    output.Albedo = color0.rgb;
                    break;
            }
        }
        ENDCG
    }
    FallBack "Diffuse"
}