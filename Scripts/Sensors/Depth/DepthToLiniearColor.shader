Shader "Custom/DepthToLinearColor"
{
    Properties
    {
        _MainTex ("Main Tex", 2D) = "grey" {}
    }

    HLSLINCLUDE

    #pragma vertex Vert
    #pragma target 4.5
    #pragma only_renderers d3d11 playstation xboxone xboxseries vulkan metal switch
    #include "Packages/com.unity.render-pipelines.high-definition/Runtime/RenderPipeline/RenderPass/CustomPass/CustomPassCommon.hlsl"

    float4 SampleBuffer(PositionInputs posInput);
    Texture2D _MainTex;
    SamplerState sampler_MainTex;

    float4 FullScreenPass(Varyings varyings) : SV_Target
    {
        UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(varyings);

        // Pixel coordinates
        int2 pixelCoords = int2(varyings.positionCS.xy);

        // ✅ Correct way to get depth in HDRP
        float depth = LoadCameraDepth(pixelCoords);

        // Convert to position input (HDRP helper)
        PositionInputs posInput = GetPositionInput(
            varyings.positionCS.xy,
            _ScreenSize.zw,
            depth,
            UNITY_MATRIX_I_VP,
            UNITY_MATRIX_V
        );

        return SampleBuffer(posInput);
    }

    ENDHLSL

    SubShader
    {
        Pass
        {
            Name "Depth"
            ZWrite On ZTest Always Blend Off Cull Off

            HLSLPROGRAM

            #pragma fragment FullScreenPass
            #include "Packages/com.unity.render-pipelines.high-definition/Runtime/Material/NormalBuffer.hlsl"

            float4 SampleBuffer(PositionInputs posInput)
            {
                // Make linearDepth normalised by remapping it with the camera near/far clip planes
                float linearDepth01 = (posInput.linearDepth - _ProjectionParams.y) / (_ProjectionParams.z - _ProjectionParams.y);
                return float4(linearDepth01, 0, 0, 1);
            }

            ENDHLSL
        }
    }
    Fallback Off
}
