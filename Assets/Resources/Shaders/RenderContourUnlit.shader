Shader "Custom/RenderContourUnlit"
{
    Properties
    {
        [HDR] _color ( "color", Color ) = ( 0, 0, 0, 1 )
    }

    SubShader
    {

        Tags { "RenderType" = "Opaque" }

        Pass
        {
            CGPROGRAM

            #include "UnityCG.cginc"

            #pragma vertex vert
            #pragma fragment frag

            struct Quad {
                uint indices[6];
                uint subMeshIndex;
            };

            StructuredBuffer<float3> vertices;
            StructuredBuffer<float3> normals;
            StructuredBuffer<Quad>   quads;

            float4x4 localToWorld;
            float4x4 worldToLocal;

            float4 vert( uint vertexID : SV_VertexID, uint instanceID : SV_InstanceID ) : SV_POSITION {

                unity_ObjectToWorld = localToWorld;
                unity_WorldToObject = worldToLocal;

                uint quadIndex    = floor( vertexID / 6 );
                uint quadSubIndex = vertexID % 6;

                float3 position = vertices[quads[quadIndex].indices[quadSubIndex]];

                return UnityObjectToClipPos( float4( position, 1.0f ) );
            }

            fixed4 color;

            fixed4 frag( ) : SV_TARGET {
                return color;
            }

            ENDCG
        }
    }

    Fallback "VertexLit"
}