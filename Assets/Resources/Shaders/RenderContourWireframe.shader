Shader "Custom/RenderContourWireframe"
{
    // the edges of a contour's triangles, drawn over the contour after the opaque objects: lines over the shaded
    // surface, or lines over a flat fill that hides the surface and, through its depth, the lines behind it. Wireframe
    // draws it, and Documents/Plans/Wireframe Rendering.md explains it
    SubShader
    {
        Tags { "RenderType" = "Transparent" "RenderPipeline" = "UniversalPipeline" "Queue" = "Transparent" }

        Pass
        {
            Name "Wireframe"
            Tags { "LightMode" = "SRPDefaultUnlit" }

            // the renderer has written the contour's depth, which this matches: measured, without an offset nothing
            // z-fought. One unit of offset allows for a compiler that arranges the two vertex shaders differently, and
            // has no slope, which let lines behind a crease through
            ZTest  LEqual
            ZWrite Off
            Offset 0, -1
            Cull   Back
            Blend  SrcAlpha OneMinusSrcAlpha

            HLSLPROGRAM

            // raw buffers in the vertex stage
            #pragma target 4.5

            #pragma vertex   vertexShader
            #pragma fragment fragmentShader

            // where the triangles come from: a mesh, or the quads of a contour the GPU made and keeps
            #pragma multi_compile_local _ CONTOUR_QUADS

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"

            // the most sub-meshes a contour has, one for each material bit
            #define MAXIMUM_SUB_MESHES 10

            #if defined( CONTOUR_QUADS )

            // the GPU's contour, as RenderContourDiffuse reads it: six indices for the two triangles of a quad, and the
            // sub-mesh of its material. The draw has six vertices for every quad, so vertex n is corner n % 3 of its
            // triangle here too
            struct Quad {
                uint indices[6];
                uint subMeshIndex;
            };

            StructuredBuffer<float3> vertices;
            StructuredBuffer<Quad>   quads;

            #else

            // the mesh's own vertex and index buffers, read as raw buffers. The draw has three vertices for every
            // triangle of the sub-mesh and no index buffer, so vertex n is corner n % 3 of triangle n / 3, and reads
            // its index itself
            ByteAddressBuffer positions;
            ByteAddressBuffer indices;

            uint positionStride;
            uint positionOffset;
            uint indices16;
            uint indexStart;
            int  baseVertex;
            uint subMeshIndex;

            #endif

            // the contour is drawn procedurally, so there's no per-object transform from the renderer
            float4x4 localToWorld;

            float  lineWidth;
            float4 lineColor;
            float4 fillColor;
            float4 subMeshColors[MAXIMUM_SUB_MESHES];
            float  linesOnly;

            struct Varyings {
                float4               positionCS  : SV_POSITION;
                float3               barycentric : TEXCOORD0;
                nointerpolation uint subMesh     : TEXCOORD1;
            };

            #if !defined( CONTOUR_QUADS )

            // 16-bit indices are two to a 32-bit word, the first in the low half. Unity pads an odd number of them to
            // an even one, so the word of the last is always in the buffer
            uint readIndex( uint index ) {
                if( indices16 != 0 ) {
                    uint word = indices.Load( ( index * 2 ) & ~3u );
                    return ( index & 1 ) != 0 ? word >> 16 : word & 0xFFFF;
                }
                return indices.Load( index * 4 );
            }

            #endif

            Varyings vertexShader( uint vertexID : SV_VertexID ) {
                #if defined( CONTOUR_QUADS )
                Quad   quad     = quads[vertexID / 6];
                float3 position = vertices[quad.indices[vertexID % 6]];
                uint   subMesh  = quad.subMeshIndex;
                #else
                uint   vertexIndex = readIndex( indexStart + vertexID ) + baseVertex;
                float3 position    = asfloat( positions.Load3( vertexIndex * positionStride + positionOffset ) );
                uint   subMesh     = subMeshIndex;
                #endif

                uint corner = vertexID % 3;

                Varyings output;
                output.positionCS  = TransformWorldToHClip( mul( localToWorld, float4( position, 1.0f ) ).xyz );
                output.barycentric = float3( corner == 0, corner == 1, corner == 2 );
                output.subMesh     = subMesh;
                return output;
            }

            half4 fragmentShader( Varyings input ) : SV_TARGET {
                // a barycentric coordinate is zero on the edge across from its corner, and its rate of change across
                // the screen turns it into a distance in pixels. The two triangles of an edge each draw half the line
                float3 distances = input.barycentric / fwidth( input.barycentric );
                float  distance  = min( distances.x, min( distances.y, distances.z ) );
                float  coverage  = saturate( lineWidth * 0.5f + 0.5f - distance );

                if( linesOnly > 0.5f ) {
                    float4 materialColor = subMeshColors[min( input.subMesh, MAXIMUM_SUB_MESHES - 1 )];
                    return half4( lerp( fillColor.rgb, materialColor.rgb, coverage ), 1.0h );
                }

                clip( coverage - 1e-3f );
                return half4( lineColor.rgb, lineColor.a * coverage );
            }

            ENDHLSL
        }
    }
    FallBack Off
}
