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
        Tags { "RenderType" = "Opaque" "RenderPipeline" = "UniversalPipeline" "Queue" = "Geometry" }

        LOD 200

        HLSLINCLUDE

        // structured buffers in the vertex stage
        #pragma target 4.5

        #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"

        struct Quad {
            uint indices[6];
            uint subMeshIndex;
        };

        StructuredBuffer<float3> vertices;
        StructuredBuffer<float3> normals;
        StructuredBuffer<Quad>   quads;

        // the contour is drawn procedurally, so there's no per-object transform from the renderer, the voxelizer
        // supplies its own
        float4x4 localToWorld;
        float4x4 worldToLocal;

        CBUFFER_START( UnityPerMaterial )
            half4 color0;
            half4 color1;
            half4 color2;
            half4 color3;
            half4 color4;
            half4 color5;
            half4 color6;
            half4 color7;
            half4 color8;
            half4 color9;
        CBUFFER_END

        struct ContourVertex {
            float3 positionWS;
            float3 normalWS;
            uint   subMeshIndex;
        };

        ContourVertex fetchContourVertex( uint vertexID ) {
            Quad quad        = quads[vertexID / 6];
            uint vertexIndex = quad.indices[vertexID % 6];

            ContourVertex output;
            output.positionWS   = mul( localToWorld, float4( vertices[vertexIndex], 1.0f ) ).xyz;
            // normals transform by the inverse transpose
            output.normalWS     = normalize( mul( normals[vertexIndex], ( float3x3 )worldToLocal ) );
            output.subMeshIndex = quad.subMeshIndex;
            return output;
        }

        half4 getSubMeshColor( uint subMeshIndex ) {
            switch( subMeshIndex ) {
                case 0:  return color0;
                case 1:  return color1;
                case 2:  return color2;
                case 3:  return color3;
                case 4:  return color4;
                case 5:  return color5;
                case 6:  return color6;
                case 7:  return color7;
                case 8:  return color8;
                case 9:  return color9;
                default: return color0;
            }
        }

        ENDHLSL

        Pass
        {
            Name "ForwardLit"
            Tags { "LightMode" = "UniversalForward" }

            HLSLPROGRAM

            #pragma vertex   vertexShader
            #pragma fragment fragmentShader

            #pragma multi_compile _ _MAIN_LIGHT_SHADOWS _MAIN_LIGHT_SHADOWS_CASCADE _MAIN_LIGHT_SHADOWS_SCREEN
            #pragma multi_compile _ _ADDITIONAL_LIGHTS_VERTEX _ADDITIONAL_LIGHTS
            #pragma multi_compile_fragment _ _ADDITIONAL_LIGHT_SHADOWS
            #pragma multi_compile_fragment _ _SHADOWS_SOFT _SHADOWS_SOFT_LOW _SHADOWS_SOFT_MEDIUM _SHADOWS_SOFT_HIGH
            #pragma multi_compile_fragment _ _SCREEN_SPACE_OCCLUSION
            #pragma multi_compile _ _CLUSTER_LIGHT_LOOP
            #include_with_pragmas "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Fog.hlsl"

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"

            struct Varyings {
                float4               positionCS     : SV_POSITION;
                float3               positionWS     : TEXCOORD0;
                float3               normalWS       : TEXCOORD1;
                nointerpolation uint subMeshIndex   : TEXCOORD2;
                half                 fogFactor      : TEXCOORD3;
                half3                vertexLighting : TEXCOORD4;
            };

            Varyings vertexShader( uint vertexID : SV_VertexID ) {
                ContourVertex contourVertex = fetchContourVertex( vertexID );

                Varyings output;
                output.positionWS     = contourVertex.positionWS;
                output.normalWS       = contourVertex.normalWS;
                output.subMeshIndex   = contourVertex.subMeshIndex;
                output.positionCS     = TransformWorldToHClip( contourVertex.positionWS );
                output.fogFactor      = ComputeFogFactor( output.positionCS.z );
                output.vertexLighting = VertexLighting( contourVertex.positionWS, contourVertex.normalWS );
                return output;
            }

            half4 fragmentShader( Varyings input ) : SV_TARGET {
                InputData inputData = ( InputData )0;
                inputData.positionWS              = input.positionWS;
                inputData.positionCS              = input.positionCS;
                inputData.normalWS                = NormalizeNormalPerPixel( input.normalWS );
                inputData.viewDirectionWS         = GetWorldSpaceNormalizeViewDir( input.positionWS );
                inputData.normalizedScreenSpaceUV = GetNormalizedScreenSpaceUV( input.positionCS );
                inputData.fogCoord                = InitializeInputDataFog( float4( input.positionWS, 1.0f ), input.fogFactor );
                inputData.vertexLighting          = input.vertexLighting;
                inputData.bakedGI                 = SampleSH( inputData.normalWS );
                inputData.shadowMask              = half4( 1.0h, 1.0h, 1.0h, 1.0h );

                #if defined( _MAIN_LIGHT_SHADOWS_SCREEN )
                inputData.shadowCoord = float4( inputData.normalizedScreenSpaceUV, 0.0f, 1.0f );
                #else
                inputData.shadowCoord = TransformWorldToShadowCoord( input.positionWS );
                #endif

                // matches the original surface shader's SurfaceOutputStandard: zero metallic and smoothness
                SurfaceData surfaceData = ( SurfaceData )0;
                surfaceData.albedo    = getSubMeshColor( input.subMeshIndex ).rgb;
                surfaceData.occlusion = 1.0h;
                surfaceData.alpha     = 1.0h;

                half4 color = UniversalFragmentPBR( inputData, surfaceData );
                color.rgb   = MixFog( color.rgb, inputData.fogCoord );
                return color;
            }

            ENDHLSL
        }

        Pass
        {
            Name "ShadowCaster"
            Tags { "LightMode" = "ShadowCaster" }

            ZWrite    On
            ZTest     LEqual
            ColorMask 0

            HLSLPROGRAM

            #pragma vertex   shadowVertexShader
            #pragma fragment shadowFragmentShader

            #pragma multi_compile_vertex _ _CASTING_PUNCTUAL_LIGHT_SHADOW

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Shadows.hlsl"

            // set by URP while rendering shadow maps
            float3 _LightDirection;
            float3 _LightPosition;

            float4 shadowVertexShader( uint vertexID : SV_VertexID ) : SV_POSITION {
                ContourVertex contourVertex = fetchContourVertex( vertexID );

                #if _CASTING_PUNCTUAL_LIGHT_SHADOW
                float3 lightDirectionWS = normalize( _LightPosition - contourVertex.positionWS );
                #else
                float3 lightDirectionWS = _LightDirection;
                #endif

                float4 positionCS = TransformWorldToHClip( ApplyShadowBias( contourVertex.positionWS, contourVertex.normalWS, lightDirectionWS ) );
                return ApplyShadowClamping( positionCS );
            }

            half4 shadowFragmentShader( ) : SV_TARGET {
                return 0;
            }

            ENDHLSL
        }

        Pass
        {
            Name "DepthOnly"
            Tags { "LightMode" = "DepthOnly" }

            ZWrite    On
            ColorMask R

            HLSLPROGRAM

            #pragma vertex   depthVertexShader
            #pragma fragment depthFragmentShader

            float4 depthVertexShader( uint vertexID : SV_VertexID ) : SV_POSITION {
                return TransformWorldToHClip( fetchContourVertex( vertexID ).positionWS );
            }

            half depthFragmentShader( float4 positionCS : SV_POSITION ) : SV_TARGET {
                return positionCS.z;
            }

            ENDHLSL
        }
    }
    FallBack "Hidden/Universal Render Pipeline/FallbackError"
}
